// SPDX-License-Identifier: GPL-3.0-or-later
#include "neupan_controller/controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
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
  params_.yaw_kp = parameter(node, name + ".goal_yaw_pid.kp", 1.5);
  params_.yaw_ki = parameter(node, name + ".goal_yaw_pid.ki", 0.05);
  params_.yaw_kd = parameter(node, name + ".goal_yaw_pid.kd", 0.1);

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
  const auto package_share =
      ament_index_cpp::get_package_share_directory("neupan_controller");
  cfg.dune_checkpoint = parameter(
      node, name + ".dune_checkpoint", package_share + "/models/diff_default.bin");
  if (std::filesystem::path(cfg.dune_checkpoint).is_relative())
    cfg.dune_checkpoint = package_share + "/" + cfg.dune_checkpoint;
  cfg.ipath.projected_reference = true;
  // Nav2 owns arrival. Keep the core checking obstacle clearance while the
  // plugin aligns the final yaw instead of latching a position-only arrival.
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
  reference_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      name + "/reference_path", rclcpp::QoS(1));
  optimized_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      name + "/optimized_path", rclcpp::QoS(1));
  const auto timing_topic = parameter(node, name + ".timing_topic", name + "/timing");
  timing_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      timing_topic, rclcpp::SensorDataQoS().keep_last(1));
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
  reference_pub_->on_activate();
  optimized_pub_->on_activate();
  timing_pub_->on_activate();
  timing_samples_ = 0;
  timing_means_.fill(0.0);
  timing_maxima_.fill(0.0);
  rotating_to_goal_ = false;
  have_command_ = false;
  {
    std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
    cloud_.reset();
  }
  points_sub_ = node_.lock()->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
        cloud_ = std::move(msg);
      });
}

void NeupanController::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  points_sub_.reset();
  reference_pub_->on_deactivate();
  optimized_pub_->on_deactivate();
  timing_pub_->on_deactivate();
  {
    std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
    cloud_.reset();
  }
  have_command_ = false;
  planner_->reset();
}

void NeupanController::cleanup() {
  parameter_callback_.reset();
  deactivate();
  planner_.reset();
  reference_pub_.reset();
  optimized_pub_.reset();
  timing_pub_.reset();
  tf_.reset();
  path_ = nav_msgs::msg::Path{};
}

void NeupanController::setPlan(const nav_msgs::msg::Path& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Periodic replans to the same goal must not restart position tracking.
  bool same_goal = !path_.poses.empty() && !path.poses.empty() &&
      path_.header.frame_id == path.header.frame_id;
  if (same_goal) {
    const auto& old_goal = path_.poses.back().pose;
    const auto& new_goal = path.poses.back().pose;
    same_goal = std::hypot(old_goal.position.x - new_goal.position.x,
                           old_goal.position.y - new_goal.position.y) <= 1e-6 &&
        std::abs(neupan::wrapToPi(tf2::getYaw(old_goal.orientation) -
                                 tf2::getYaw(new_goal.orientation))) <= 1e-6;
  }
  if (!same_goal) rotating_to_goal_ = false;
  path_ = path;
  new_plan_ = true;
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
  const auto cycle_start = std::chrono::steady_clock::now();
  try {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
      cloud = cloud_;
    }
    if (!cloud) throw nav2_core::PlannerException("Waiting for PointCloud2");
    const double age = (clock_->now() -
        rclcpp::Time(cloud->header.stamp, clock_->get_clock_type())).seconds();
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

    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!goal_checker->getTolerances(tolerance, velocity_tolerance))
      throw nav2_core::PlannerException("Goal checker does not provide tolerances");
    const auto& goal = path_.poses.back().pose;
    const double yaw_error =
        neupan::wrapToPi(tf2::getYaw(goal.orientation) - state.z());
    if (!rotating_to_goal_ &&
        std::hypot(state.x() - goal.position.x, state.y() - goal.position.y) <=
            tolerance.position.x) {
      rotating_to_goal_ = true;
      yaw_integral_ = 0.0;
      previous_yaw_error_ = yaw_error;
    }

    // Command bounds must advance from the command stream, as in neupan_cpp_ros.
    // Re-centering on lagging odometry every cycle can pin startup at a_max / Hz.
    // Initialize from odometry only at startup or after a full prediction-horizon gap.
    const double elapsed = have_command_
        ? std::chrono::duration<double>(cycle_start - command_time_).count()
        : control_period_;
    const auto& cfg = planner_->config();
    const bool continuing = have_command_ && elapsed <= cfg.receding * cfg.step_time;
    neupan::NeuPANPlanner::Command command;
    command.action = continuing ? last_command_
        : neupan::Vec2(velocity.linear.x, velocity.angular.z);
    command.elapsed = continuing ? elapsed : control_period_;
    neupan::NeuPANPlanner::Info info;
    neupan::Vec2 action = planner_->forward(state, obstaclePoints(*cloud), info, command);
    publishTiming(info);
    // Publish the actual solver inputs/results, not another copy of the global path.
    const auto publish_trajectory = [&](const auto& publisher, const neupan::Mat3X& states) {
      if (publisher->get_subscription_count() == 0) return;
      nav_msgs::msg::Path trajectory;
      trajectory.header.frame_id = path_.header.frame_id;
      trajectory.header.stamp = current.header.stamp;
      trajectory.poses.reserve(states.cols());
      for (Eigen::Index i = 0; i < states.cols(); ++i) {
        geometry_msgs::msg::PoseStamped p;
        p.header = trajectory.header;
        p.pose.position.x = states(0, i);
        p.pose.position.y = states(1, i);
        p.pose.orientation.z = std::sin(states(2, i) / 2.0);
        p.pose.orientation.w = std::cos(states(2, i) / 2.0);
        trajectory.poses.push_back(p);
      }
      publisher->publish(trajectory);
    };
    publish_trajectory(reference_pub_, info.ref_s);
    publish_trajectory(optimized_pub_, info.opt_s);
    RCLCPP_DEBUG_THROTTLE(
        node_.lock()->get_logger(), *clock_, 1000,
        "NeuPAN tracking: frame=%s state=(%.3f, %.3f, %.3f) segment=%d "
        "ref_end=(%.3f, %.3f) goal=(%.3f, %.3f) solved=%d stop=%d v=%.3f w=%.3f",
        path_.header.frame_id.c_str(), state.x(), state.y(), state.z(),
        planner_->ipath().point_index_,
        info.ref_s(0, info.ref_s.cols() - 1), info.ref_s(1, info.ref_s.cols() - 1),
        path_.poses.back().pose.position.x, path_.poses.back().pose.position.y,
        static_cast<int>(info.solved), static_cast<int>(info.stop), action.x(), action.y());
    if (info.stop)
      throw nav2_core::PlannerException(
          "NeuPAN collision stop: distance=" + std::to_string(info.min_distance) +
          " m, threshold=" + std::to_string(planner_->config().collision_threshold) + " m");
    if (!info.solved)
      throw nav2_core::PlannerException(
          "NeuPAN solver failed, status " + std::to_string(info.solver_status));

    if (rotating_to_goal_) {
      action.x() = 0.0;
      if (std::abs(yaw_error) <= tf2::getYaw(tolerance.orientation)) {
        action.y() = 0.0;
        yaw_integral_ = 0.0;
      } else {
        const double dt = command.elapsed;
        const double max_w = cfg.max_speed.y();
        const double integral = std::clamp(
            yaw_integral_ + params_.yaw_ki * yaw_error * dt, -max_w, max_w);
        const double desired = params_.yaw_kp * yaw_error + integral +
            params_.yaw_kd * neupan::wrapToPi(yaw_error - previous_yaw_error_) / dt;
        // Avoid accumulating integral while the angular-speed limit is saturated.
        if (std::abs(desired) <= max_w || yaw_error * desired < 0.0)
          yaw_integral_ = integral;
        const double dw = cfg.max_acce.y() * std::min(dt, cfg.step_time);
        action.y() = std::clamp(std::clamp(desired, -max_w, max_w),
                                command.action.y() - dw, command.action.y() + dw);
      }
      previous_yaw_error_ = yaw_error;
    }

    // This is Nav2's external speed limit; the core already enforces its own box.
    if (std::abs(action.x()) > linear_speed_limit_)
      action *= linear_speed_limit_ / std::abs(action.x());
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = clock_->now();
    output.header.frame_id = base_frame_;
    output.twist.linear.x = action.x();
    output.twist.angular.z = action.y();
    last_command_ = action;
    command_time_ = cycle_start;
    have_command_ = true;
    return output;
  } catch (const std::exception& ex) {
    // Humble publishes zero on this failure. Do not resume from a pre-stop command.
    last_command_.setZero();
    command_time_ = cycle_start;
    have_command_ = true;
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
          updateParameter(parameter, name, "goal_yaw_pid.kp", next.yaw_kp, 0.0) ||
          updateParameter(parameter, name, "goal_yaw_pid.ki", next.yaw_ki, 0.0) ||
          updateParameter(parameter, name, "goal_yaw_pid.kd", next.yaw_kd, 0.0)) {
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
        have_command_ = false;
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

void NeupanController::publishTiming(const neupan::NeuPANPlanner::Info& info) {
  const auto& t = info.timing;
  const std::array<double, kTimingCount> values{
      t.total_ms, t.reference_ms, t.pan.total_ms, t.pan.point_flow_ms,
      t.pan.dune_ms, t.pan.coefficients_ms, t.pan.nrmp.total_ms,
      t.pan.nrmp.assembly_ms, t.pan.nrmp.setup_ms, t.pan.nrmp.solve_ms,
      t.pan.nrmp.postprocess_ms};
  static constexpr std::array<const char*, kTimingCount> names{
      "planner_total", "reference", "pan_total", "point_flow", "dune",
      "nrmp_coefficients", "nrmp_total", "qp_assembly", "qp_setup", "qp_solve",
      "nrmp_postprocess"};

  ++timing_samples_;
  for (std::size_t i = 0; i < values.size(); ++i) {
    timing_means_[i] += (values[i] - timing_means_[i]) /
        static_cast<double>(timing_samples_);
    timing_maxima_[i] = std::max(timing_maxima_[i], values[i]);
  }
  // Accumulate every completed core call, but serialize only when observed.
  if (timing_pub_->get_subscription_count() == 0) return;

  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name_ + "/timing";
  status.hardware_id = "libneupan";
  status.level = info.stop || (!info.solved && !info.arrive)
      ? diagnostic_msgs::msg::DiagnosticStatus::WARN
      : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = info.stop ? "collision_stop"
      : info.arrive ? "arrived" : info.solved ? "solved" : "solver_failed";
  status.values.reserve(3 * values.size() + 3);
  const auto add = [&](const std::string& key, const auto& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  for (std::size_t i = 0; i < values.size(); ++i) {
    add(std::string(names[i]) + "_ms", values[i]);
    add(std::string(names[i]) + "_mean_ms", timing_means_[i]);
    add(std::string(names[i]) + "_max_ms", timing_maxima_[i]);
  }
  add("samples", timing_samples_);
  add("pan_iterations", t.pan.iterations);
  add("solver_status", info.solver_status);
  message.status.push_back(std::move(status));
  timing_pub_->publish(message);
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
