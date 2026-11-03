// SPDX-License-Identifier: GPL-3.0-or-later
#include "neupan_controller/controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace neupan_controller {
namespace {

template <typename T>
T parameter(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
            const std::string& name, const T& value) {
  nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(value));
  return node->get_parameter(name).get_value<T>();
}

template <typename T>
bool updateParameter(const rclcpp::Parameter& parameter, const std::string& name,
                     const char* expected, T& value,
                     double minimum = std::numeric_limits<double>::lowest()) {
  if (name != expected) return false;
  const auto next = parameter.get_value<T>();
  if (!std::isfinite(static_cast<double>(next)))
    throw std::invalid_argument(parameter.get_name() + " must be finite");
  if constexpr (std::is_integral_v<T>) {
    if (next > std::numeric_limits<T>::max() || next < std::numeric_limits<T>::lowest())
      throw std::invalid_argument(parameter.get_name() + " is outside the integer range");
  }
  if (next < minimum)
    throw std::invalid_argument(parameter.get_name() + " must be >= " +
                                std::to_string(minimum));
  value = next;
  return true;
}

template <int N>
Eigen::Matrix<double, N, 1> vectorParameter(const rclcpp::Parameter& parameter) {
  const auto& values = parameter.as_double_array();
  if (values.size() != static_cast<size_t>(N))
    throw std::invalid_argument(parameter.get_name() + " needs " +
                                std::to_string(N) + " values");
  Eigen::Matrix<double, N, 1> result;
  for (int i = 0; i < N; ++i) result(i) = values[i];
  if (!result.allFinite() || result.minCoeff() < 0.0)
    throw std::invalid_argument(parameter.get_name() +
                                " must contain finite nonnegative values");
  return result;
}

}  // namespace

void NeupanController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  const auto node = parent.lock();
  node_ = parent;
  name_ = name;
  clock_ = node->get_clock();
  tf_ = std::move(tf);
  base_frame_ = costmap_ros->getBaseFrameID();
  transform_tolerance_ = costmap_ros->getTransformTolerance();
  control_period_ = 1.0 / node->get_parameter("controller_frequency").as_double();

  points_topic_ = parameter(node, name + ".points_topic", std::string("points"));
  params_.pointcloud_timeout = parameter(node, name + ".pointcloud_timeout", 0.5);
  params_.min_height = parameter(node, name + ".min_obstacle_height", 0.05);
  params_.max_height = parameter(node, name + ".max_obstacle_height", 1.5);
  params_.min_range = parameter(node, name + ".min_obstacle_range", 0.0);
  params_.max_range = parameter(node, name + ".max_obstacle_range", 10.0);
  params_.rotate_to_goal_gain = parameter(node, name + ".rotate_to_goal_gain", 2.0);

  neupan::NeuPANPlanner::Config cfg;
  cfg.receding = parameter(node, name + ".receding", 10);
  cfg.step_time = parameter(node, name + ".step_time", control_period_);
  params_.ref_speed = parameter(node, name + ".ref_speed", 0.5);
  cfg.ref_speed = params_.ref_speed;
  cfg.collision_threshold = parameter(node, name + ".collision_threshold", 0.1);
  const auto speed = parameter(node, name + ".max_speed", std::vector<double>{0.8, 1.0});
  const auto acce = parameter(node, name + ".max_acce", std::vector<double>{1.0, 2.0});
  cfg.max_speed = neupan::Vec2(speed.at(0), speed.at(1));
  cfg.max_acce = neupan::Vec2(acce.at(0), acce.at(1));
  const auto footprint = costmap_ros->getUnpaddedRobotFootprint();
  cfg.vertices.resize(2, footprint.size());
  for (size_t i = 0; i < footprint.size(); ++i)
    cfg.vertices.col(i) << footprint[i].x, footprint[i].y;
  cfg.dune_checkpoint = parameter(
      node, name + ".dune_checkpoint",
      ament_index_cpp::get_package_share_directory("neupan_controller") +
          "/models/diff_default.bin");
  cfg.ipath.projected_reference = true;
  // Nav2 owns arrival tolerances. Keep the core evaluating obstacle clearance
  // during final rotation instead of latching its position-only arrival flag.
  cfg.ipath.arrive_threshold = 0.0;
  cfg.ipath.interval = parameter(node, name + ".ipath.interval", 0.05);
  cfg.ipath.close_threshold = parameter(node, name + ".ipath.close_threshold", 0.1);
  cfg.ipath.ind_range = parameter(node, name + ".ipath.ind_range", 10);
  cfg.pan.iter_num = parameter(node, name + ".pan.iter_num", cfg.pan.iter_num);
  cfg.pan.dune_max_num = parameter(node, name + ".pan.dune_max_num", cfg.pan.dune_max_num);
  cfg.pan.iter_threshold = parameter(node, name + ".pan.iter_threshold", cfg.pan.iter_threshold);
  cfg.adjust.max_num = parameter(node, name + ".pan.nrmp_max_num", cfg.adjust.max_num);
  const auto q = parameter(node, name + ".adjust.q_s", std::vector<double>{1.0, 1.0, 1.0});
  cfg.adjust.q_s = neupan::Vec3(q.at(0), q.at(1), q.at(2));
  cfg.adjust.p_u = parameter(node, name + ".adjust.p_u", cfg.adjust.p_u);
  cfg.adjust.eta = parameter(node, name + ".adjust.eta", cfg.adjust.eta);
  cfg.adjust.d_max = parameter(node, name + ".adjust.d_max", cfg.adjust.d_max);
  cfg.adjust.d_min = parameter(node, name + ".adjust.d_min", cfg.adjust.d_min);
  cfg.adjust.ro_obs = parameter(node, name + ".adjust.ro_obs", cfg.adjust.ro_obs);
  cfg.adjust.bk = parameter(node, name + ".adjust.bk", cfg.adjust.bk);
  planner_ = std::make_unique<neupan::NeuPANPlanner>(cfg);
  speed_limit_ = nav2_costmap_2d::NO_SPEED_LIMIT;
  speed_limit_percentage_ = false;
  applySpeedLimit();
  parameter_callback_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        return dynamicParametersCallback(parameters);
      });
}

void NeupanController::activate() {
  std::lock_guard<std::mutex> lock(mutex_);
  planner_->reset();
  rotating_to_goal_ = false;
  cloud_.reset();
  points_sub_ = node_.lock()->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> cloud_lock(mutex_);
        cloud_ = std::move(msg);
      });
}

void NeupanController::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  points_sub_.reset();
  cloud_.reset();
  planner_->reset();
}

void NeupanController::cleanup() {
  parameter_callback_.reset();
  deactivate();
  planner_.reset();
  tf_.reset();
  path_ = nav_msgs::msg::Path{};
}

void NeupanController::setPlan(const nav_msgs::msg::Path& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = path;
  new_plan_ = true;
  rotating_to_goal_ = false;
}

neupan::Mat2X NeupanController::obstaclePoints(
    const sensor_msgs::msg::PointCloud2& cloud) const {
  neupan::Mat2X points(2, static_cast<Eigen::Index>(cloud.width) * cloud.height);
  if (points.cols() == 0) return points;

  std::array<uint32_t, 3> offsets;
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (size_t i = 0; i < names.size(); ++i) {
    const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const auto& f) { return f.name == names[i]; });
    if (field == cloud.fields.end() ||
        field->datatype != sensor_msgs::msg::PointField::FLOAT32 || cloud.is_bigendian)
      throw nav2_core::PlannerException("PointCloud2 requires little-endian FLOAT32 x/y/z");
    offsets[i] = field->offset;
  }

  const rclcpp::Time stamp(cloud.header.stamp, clock_->get_clock_type());
  const auto timeout = rclcpp::Duration::from_seconds(transform_tolerance_);
  tf2::Transform sensor_to_base, base_to_plan;
  tf2::fromMsg(tf_->lookupTransform(
      base_frame_, cloud.header.frame_id, stamp, timeout).transform, sensor_to_base);
  tf2::fromMsg(tf_->lookupTransform(
      path_.header.frame_id, base_frame_, stamp, timeout).transform, base_to_plan);

  Eigen::Index count = 0;
  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const auto* data = cloud.data.data() + row * cloud.row_step + col * cloud.point_step;
      std::array<float, 3> xyz;
      for (size_t i = 0; i < xyz.size(); ++i)
        std::memcpy(&xyz[i], data + offsets[i], sizeof(float));
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
        continue;
      const tf2::Vector3 body = sensor_to_base * tf2::Vector3(xyz[0], xyz[1], xyz[2]);
      const double range = std::hypot(body.x(), body.y());
      if (body.z() < params_.min_height || body.z() > params_.max_height ||
          range < params_.min_range || range > params_.max_range)
        continue;
      const tf2::Vector3 global = base_to_plan * body;
      points.col(count++) << global.x(), global.y();
    }
  }
  points.conservativeResize(2, count);
  return points;
}

geometry_msgs::msg::TwistStamped NeupanController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity, nav2_core::GoalChecker* goal_checker) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!cloud_) throw nav2_core::PlannerException("Waiting for PointCloud2");
    const double age = (clock_->now() -
        rclcpp::Time(cloud_->header.stamp, clock_->get_clock_type())).seconds();
    if (age > params_.pointcloud_timeout)
      throw nav2_core::PlannerException(
          "PointCloud2 is stale: age=" + std::to_string(age) +
          " s, timeout=" + std::to_string(params_.pointcloud_timeout) + " s");
    // Sensor messages and /clock travel independently. A cloud can arrive ahead
    // of the latest clock tick while its timestamped TF is already available.
    // Reuse Nav2's TF tolerance for this bounded lead; keep the acquisition stamp.
    if (age < -transform_tolerance_)
      throw nav2_core::PlannerException(
          "PointCloud2 is too far in the future: lead=" + std::to_string(-age) +
          " s, transform_tolerance=" + std::to_string(transform_tolerance_) + " s");

    const auto current = tf_->transform(
        pose, path_.header.frame_id,
        tf2::durationFromSec(transform_tolerance_));
    const neupan::Vec3 state(current.pose.position.x, current.pose.position.y,
                             tf2::getYaw(current.pose.orientation));
    if (new_plan_) {
      std::vector<neupan::InitialPath::PathPoint> path;
      path.reserve(path_.poses.size());
      for (const auto& p : path_.poses)
        path.emplace_back(p.pose.position.x, p.pose.position.y,
                          tf2::getYaw(p.pose.orientation), 1.0);
      // Nav2 can return a single goal pose when already at the goal.
      if (path.size() == 1)
        path.insert(path.begin(), neupan::InitialPath::PathPoint(
            state.x(), state.y(), state.z(), 1.0));
      // Re-densification and invalid-path checks belong to libneupan.
      planner_->replaceInitialPath(std::move(path), state);
      new_plan_ = false;
    }

    neupan::NeuPANPlanner::Command command;
    command.action << velocity.linear.x, velocity.angular.z;
    command.elapsed = control_period_;
    neupan::NeuPANPlanner::Info info;
    neupan::Vec2 action = planner_->forward(state, obstaclePoints(*cloud_), info, command);
    if (info.stop)
      throw nav2_core::PlannerException(
          "NeuPAN collision stop: distance=" + std::to_string(info.min_distance) +
          " m, threshold=" + std::to_string(planner_->config().collision_threshold) + " m");
    if (!info.solved)
      throw nav2_core::PlannerException(
          "NeuPAN solver failed, status " + std::to_string(info.solver_status));

    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!goal_checker->getTolerances(tolerance, velocity_tolerance))
      throw nav2_core::PlannerException("Goal checker does not provide tolerances");
    const auto& goal = path_.poses.back().pose;
    rotating_to_goal_ = rotating_to_goal_ ||
        std::hypot(state.x() - goal.position.x, state.y() - goal.position.y) <=
            tolerance.position.x;
    if (rotating_to_goal_) {
      const double error = neupan::wrapToPi(tf2::getYaw(goal.orientation) - state.z());
      const double max_w = planner_->config().max_speed.y();
      const double dw = planner_->config().max_acce.y() *
          std::min(control_period_, planner_->config().step_time);
      const double desired = std::abs(error) <= tf2::getYaw(tolerance.orientation)
          ? 0.0 : std::clamp(params_.rotate_to_goal_gain * error, -max_w, max_w);
      action << 0.0, std::clamp(desired, velocity.angular.z - dw, velocity.angular.z + dw);
    }

    // This is Nav2's external speed limit; the core already enforces its own box.
    if (std::abs(action.x()) > linear_speed_limit_)
      action *= linear_speed_limit_ / std::abs(action.x());
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = clock_->now();
    output.header.frame_id = base_frame_;
    output.twist.linear.x = action.x();
    output.twist.angular.z = action.y();
    return output;
  } catch (const std::exception& ex) {
    // Humble's controller_server handles PlannerException via failure_tolerance.
    throw nav2_core::PlannerException(ex.what());
  }
}

rcl_interfaces::msg::SetParametersResult NeupanController::dynamicParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  auto next = params_;
  auto cfg = planner_->config();
  cfg.adjust = planner_->pan().nrmp().params();
  bool adjust_changed = false;
  bool changed = false;
  bool rebuild = false;
  const std::string prefix = name_ + ".";
  try {
    for (const auto& parameter : parameters) {
      if (parameter.get_name().compare(0, prefix.size(), prefix) != 0) continue;
      const std::string name = parameter.get_name().substr(prefix.size());
      if (updateParameter(parameter, name, "ref_speed", next.ref_speed, 0.0) ||
          updateParameter(parameter, name, "pointcloud_timeout", next.pointcloud_timeout, 0.0) ||
          updateParameter(parameter, name, "min_obstacle_height", next.min_height) ||
          updateParameter(parameter, name, "max_obstacle_height", next.max_height) ||
          updateParameter(parameter, name, "min_obstacle_range", next.min_range, 0.0) ||
          updateParameter(parameter, name, "max_obstacle_range", next.max_range, 0.0) ||
          updateParameter(parameter, name, "rotate_to_goal_gain", next.rotate_to_goal_gain, 0.0)) {
        changed = true;
        continue;
      }

      if (updateParameter(parameter, name, "collision_threshold", cfg.collision_threshold) ||
          updateParameter(parameter, name, "ipath.close_threshold", cfg.ipath.close_threshold, 0.0) ||
          updateParameter(parameter, name, "ipath.ind_range", cfg.ipath.ind_range, 1.0) ||
          updateParameter(parameter, name, "pan.iter_num", cfg.pan.iter_num, 1.0) ||
          updateParameter(parameter, name, "pan.dune_max_num", cfg.pan.dune_max_num, 1.0) ||
          updateParameter(parameter, name, "pan.iter_threshold", cfg.pan.iter_threshold, 0.0)) {
        rebuild = true;
      } else if (updateParameter(parameter, name, "adjust.p_u", cfg.adjust.p_u, 0.0) ||
          updateParameter(parameter, name, "adjust.eta", cfg.adjust.eta, 0.0) ||
          updateParameter(parameter, name, "adjust.d_max", cfg.adjust.d_max) ||
          updateParameter(parameter, name, "adjust.d_min", cfg.adjust.d_min) ||
          updateParameter(parameter, name, "adjust.ro_obs", cfg.adjust.ro_obs, 0.0) ||
          updateParameter(parameter, name, "adjust.bk", cfg.adjust.bk)) {
        adjust_changed = true;
      } else if (name == "max_speed") {
        cfg.max_speed = vectorParameter<2>(parameter);
        rebuild = true;
      } else if (name == "max_acce") {
        cfg.max_acce = vectorParameter<2>(parameter);
        rebuild = true;
      } else if (name == "adjust.q_s") {
        cfg.adjust.q_s = vectorParameter<3>(parameter);
        adjust_changed = true;
      } else {
        throw std::invalid_argument(parameter.get_name() +
            " is an initialization parameter; cleanup/configure is required");
      }
      changed = true;
    }
    if (changed) {
      if (next.min_height > next.max_height || next.min_range > next.max_range)
        throw std::invalid_argument("Obstacle filter minimum must not exceed maximum");
      cfg.ref_speed = next.ref_speed;
      if (rebuild) {
        // Reuse core validation and construction; publish the new planner only
        // after the whole plugin parameter batch has been accepted here.
        auto replacement = std::make_unique<neupan::NeuPANPlanner>(cfg);
        planner_ = std::move(replacement);
        new_plan_ = true;  // Re-project the existing Nav2 path on the next cycle.
      } else if (adjust_changed) {
        neupan::NeuPANPlanner::validate(cfg);
        planner_->pan().nrmp().updateParams(cfg.adjust);
        planner_->pan().reset();
      }
      params_ = next;
      applySpeedLimit();
    }
    result.successful = true;
  } catch (const std::exception& ex) {
    result.successful = false;
    result.reason = ex.what();
  }
  return result;
}

void NeupanController::applySpeedLimit() {
  const auto& cfg = planner_->config();
  linear_speed_limit_ = speed_limit_ == nav2_costmap_2d::NO_SPEED_LIMIT
      ? cfg.max_speed.x()
      : std::clamp(speed_limit_percentage_ ? cfg.max_speed.x() * speed_limit_ / 100.0
                                         : speed_limit_,
                   0.0, cfg.max_speed.x());
  planner_->setReferenceSpeed(std::min(params_.ref_speed, linear_speed_limit_));
}

void NeupanController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
  std::lock_guard<std::mutex> lock(mutex_);
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
  applySpeedLimit();
}

}  // namespace neupan_controller

PLUGINLIB_EXPORT_CLASS(neupan_controller::NeupanController, nav2_core::Controller)
