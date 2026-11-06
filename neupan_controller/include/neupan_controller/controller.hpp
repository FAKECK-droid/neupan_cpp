// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_core/controller.hpp"
#include "neupan/neupan_planner.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace neupan_controller {

class NeupanController : public nav2_core::Controller {
 public:
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path& path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped& pose,
      const geometry_msgs::msg::Twist& velocity,
      nav2_core::GoalChecker* goal_checker) override;
  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

 private:
  struct RuntimeParameters {
    double ref_speed = 0.5;
    double pointcloud_timeout = 0.5;
    double min_height = 0.05;
    double max_height = 1.5;
    double min_range = 0.0;
    double max_range = 10.0;
    double yaw_kp = 1.5;
    double yaw_ki = 0.05;
    double yaw_kd = 0.1;
  };

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);
  void applySpeedLimit();
  void publishTiming(const neupan::NeuPANPlanner::Info& info);
  neupan::Mat2X obstaclePoints(const sensor_msgs::msg::PointCloud2& cloud) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<neupan::NeuPANPlanner> planner_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr optimized_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr timing_pub_;
  static constexpr std::size_t kTimingCount = 11;
  std::array<double, kTimingCount> timing_means_{};
  std::array<double, kTimingCount> timing_maxima_{};
  std::uint64_t timing_samples_ = 0;
  nav_msgs::msg::Path path_;
  std::string name_;
  std::string points_topic_;
  std::string base_frame_;
  double transform_tolerance_;
  RuntimeParameters params_;
  double control_period_;
  double linear_speed_limit_;
  double speed_limit_ = 0.0;
  bool speed_limit_percentage_ = false;
  bool new_plan_ = false;
  bool rotating_to_goal_ = false;
  double yaw_integral_ = 0.0;  // integral contribution, rad/s
  double previous_yaw_error_ = 0.0;
  neupan::Vec2 last_command_ = neupan::Vec2::Zero();
  std::chrono::steady_clock::time_point command_time_;
  bool have_command_ = false;
  // FollowPath runs on the action server's thread; subscriptions run separately.
  std::mutex mutex_;
  // Receiving the newest cloud must not wait for TF lookups or optimization.
  std::mutex cloud_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace neupan_controller
