// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "nav2_controller/controller_server.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class ControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_neupan");
    node_->declare_parameter("controller_frequency", 20.0);
    node_->declare_parameter("FollowPath.points_topic", std::string("/test_neupan_points"));
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
    costmap_->set_parameter(rclcpp::Parameter("global_frame", "odom"));
    costmap_->set_parameter(rclcpp::Parameter("robot_base_frame", "base_link"));
    costmap_->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap_->set_parameter(rclcpp::Parameter("footprint",
        "[[-0.8,-1.0],[0.8,-1.0],[0.8,1.0],[-0.8,1.0]]"));
    costmap_->configure();
    ASSERT_TRUE(costmap_->isCurrent());
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);  // Test transforms are preloaded.
    setTransform("map", "odom", 10.0);
    setTransform("map", "base_link", 10.0);
    setTransform("base_link", "sensor", 0.0, 0.0, 0.5);

    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
        "nav2_core", "nav2_core::Controller");
    controller_ = loader_->createSharedInstance("neupan_controller::NeupanController");
    controller_->configure(node_, "FollowPath", tf_, costmap_);
    goal_checker_.initialize(node_, "goal_checker", costmap_);
    controller_->activate();

    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/test_neupan_points", rclcpp::SensorDataQoS());
    executor_.add_node(node_->get_node_base_interface());
    setPath(10.0, 14.0);
  }

  void TearDown() override {
    controller_->deactivate();
    controller_->cleanup();
    controller_.reset();
    loader_.reset();
    executor_.remove_node(node_->get_node_base_interface());
    costmap_->cleanup();
  }

  void setTransform(const std::string& parent, const std::string& child,
                    double x, double y = 0.0, double z = 0.0) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.header.stamp = node_->now();
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(transform, "test", true));
  }

  void setPath(double start, double end, double yaw = 0.0) {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (double x : {start, end}) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
      path.poses.push_back(pose);
    }
    controller_->setPlan(path);
    goal_checker_.reset();
  }

  geometry_msgs::msg::TwistStamped compute(double odom_x = 0.0, double v = 0.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = node_->now();
    pose.pose.position.x = odom_x;
    pose.pose.orientation.w = 1.0;
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = v;
    return controller_->computeVelocityCommands(pose, velocity, &goal_checker_);
  }

  sensor_msgs::msg::PointCloud2 cloud(
      const std::vector<std::array<float, 3>>& points = {}) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "sensor";
    msg.header.stamp = node_->now();
    msg.width = points.size();
    msg.height = 1;
    msg.point_step = 16;  // extra intensity bytes are ignored
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(msg.row_step);
    for (uint32_t i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = std::array<std::string, 3>{"x", "y", "z"}[i];
      field.offset = 4 * i;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      msg.fields.push_back(field);
    }
    for (size_t i = 0; i < points.size(); ++i)
      std::memcpy(msg.data.data() + i * msg.point_step, points[i].data(), 12);
    return msg;
  }

  void publish(const sensor_msgs::msg::PointCloud2& msg) {
    for (int i = 0; i < 15; ++i) {
      pub_->publish(msg);
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    executor_.spin_some();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  std::shared_ptr<nav2_core::Controller> controller_;
  nav2_controller::SimpleGoalChecker goal_checker_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(ControllerTest, LoadsThroughPluginlibAndTracksPlannerPathAcrossFrames) {
  EXPECT_THROW(compute(), nav2_core::PlannerException);
  publish(cloud());
  const auto cmd = compute();
  EXPECT_EQ(cmd.header.frame_id, "base_link");
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_LE(cmd.twist.linear.x, 0.05 + 1e-3);  // measured odometry + one control period
  EXPECT_NEAR(cmd.twist.angular.z, 0.0, 1e-3);

  setPath(10.0, 6.0);
  EXPECT_NO_THROW(compute());
}

TEST_F(ControllerTest, RejectsStaleCloudAndResubscribesOnActivation) {
  auto msg = cloud();
  msg.header.stamp = node_->now() - rclcpp::Duration::from_seconds(2.0);
  publish(msg);
  EXPECT_THROW(compute(), nav2_core::PlannerException);
  publish(cloud());
  EXPECT_NO_THROW(compute());
  controller_->deactivate();
  controller_->activate();
  EXPECT_THROW(compute(), nav2_core::PlannerException);
  publish(cloud());
  EXPECT_NO_THROW(compute());
}

TEST_F(ControllerTest, ProjectsOrganizedCloudWithRowPaddingAndFiltersHeightAndNaNs) {
  auto msg = cloud({{0.0f, 0.0f, -0.5f},
                    {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}});
  msg.width = 1;
  msg.height = 2;
  msg.row_step = 24;
  msg.data.resize(48);
  // Both valid rows are clear after filtering: ground in base frame and NaN.
  const std::array<float, 3> nan_point{std::numeric_limits<float>::quiet_NaN(), 0, 0};
  std::memcpy(msg.data.data() + msg.row_step, nan_point.data(), 12);
  publish(msg);
  EXPECT_GT(compute().twist.linear.x, 0.0);
  // The second row now contains an obstacle at the robot's center.
  const std::array<float, 3> obstacle{0.0f, 0.0f, 0.0f};
  std::memcpy(msg.data.data() + msg.row_step, obstacle.data(), 12);
  msg.header.stamp = node_->now();
  publish(msg);
  EXPECT_THROW(compute(), nav2_core::PlannerException);
}

TEST_F(ControllerTest, RequiresSensorTFAndValidPointFields) {
  auto msg = cloud({{4.0f, 0.0f, 0.0f}});
  msg.header.frame_id = "missing_sensor";
  publish(msg);
  EXPECT_THROW(compute(), nav2_core::PlannerException);
  msg.header.frame_id = "sensor";
  msg.header.stamp = node_->now();
  msg.fields.back().datatype = sensor_msgs::msg::PointField::FLOAT64;
  publish(msg);
  EXPECT_THROW(compute(), nav2_core::PlannerException);
}

TEST_F(ControllerTest, AppliesAbsolutePercentageAndResetSpeedLimits) {
  publish(cloud());
  controller_->setSpeedLimit(0.1, false);
  EXPECT_LE(std::abs(compute(0.0, 0.6).twist.linear.x), 0.1 + 1e-6);
  controller_->setSpeedLimit(10.0, true);
  EXPECT_LE(std::abs(compute(0.0, 0.6).twist.linear.x), 0.08 + 1e-6);
  controller_->setSpeedLimit(0.0, false);  // Nav2 NO_SPEED_LIMIT
  EXPECT_GT(compute(0.0, 0.6).twist.linear.x, 0.1);
}

TEST_F(ControllerTest, AlignsFinalYawAndStillHonorsCollisionStop) {
  setPath(9.0, 10.0, 1.0);
  publish(cloud());
  const auto cmd = compute();
  EXPECT_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_GT(cmd.twist.angular.z, 0.0);
  EXPECT_LE(cmd.twist.angular.z, 0.1 + 1e-6);
  publish(cloud({{0.0f, 0.0f, 0.0f}}));
  EXPECT_THROW(compute(), nav2_core::PlannerException);
}

TEST_F(ControllerTest, InvalidPathAndFootprintRemainCoreErrors) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  controller_->setPlan(path);
  publish(cloud());
  EXPECT_THROW(compute(), nav2_core::PlannerException);
  controller_->cleanup();
  auto footprint = costmap_->getUnpaddedRobotFootprint();
  footprint[0].x -= 0.1;
  costmap_->setRobotFootprint(footprint);
  EXPECT_THROW(controller_->configure(node_, "FollowPath", tf_, costmap_), std::runtime_error);
  // Restore a configured instance for lifecycle teardown.
  footprint[0].x += 0.1;
  costmap_->setRobotFootprint(footprint);
  controller_->configure(node_, "FollowPath", tf_, costmap_);
}

TEST_F(ControllerTest, AcceptsNav2SinglePoseGoal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped goal;
  goal.header = path.header;
  goal.pose.position.x = 10.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 1.0);
  goal.pose.orientation = tf2::toMsg(q);
  path.poses.push_back(goal);
  controller_->setPlan(path);
  publish(cloud());
  const auto cmd = compute();
  EXPECT_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_GT(cmd.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, RejectsUnsolvedNominalInsteadOfDrivingWithoutAvoidance) {
  controller_->cleanup();
  node_->set_parameter(rclcpp::Parameter("FollowPath.pan.iter_num", 0));
  controller_->configure(node_, "FollowPath", tf_, costmap_);
  controller_->activate();
  setPath(10.0, 14.0);
  publish(cloud());
  EXPECT_THROW(compute(), nav2_core::PlannerException);
}

namespace {

rclcpp::NodeOptions serverOptions() {
  return rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("controller_plugins", std::vector<std::string>{"FollowPath"}),
      rclcpp::Parameter("FollowPath.plugin", "neupan_controller::NeupanController"),
      rclcpp::Parameter("FollowPath.points_topic", "/test_neupan_points"),
      rclcpp::Parameter("failure_tolerance", 0.3),
      rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.1),
      rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 0.1),
      rclcpp::Parameter("bond_heartbeat_period", 0.0)});
}

class TestControllerServer : public nav2_controller::ControllerServer {
 public:
  TestControllerServer() : nav2_controller::ControllerServer(serverOptions()) {
    costmap_ros_->set_parameters({
        rclcpp::Parameter("plugins", std::vector<std::string>{}),
        rclcpp::Parameter("filters", std::vector<std::string>{}),
        rclcpp::Parameter("global_frame", "odom"),
        rclcpp::Parameter("update_frequency", 1.0),
        rclcpp::Parameter("publish_frequency", 0.0),
        rclcpp::Parameter("footprint",
            "[[-0.8,-1.0],[0.8,-1.0],[0.8,1.0],[-0.8,1.0]]")});
  }

  void setPose(double x, double y, double yaw) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.header.stamp = now();
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.transform.rotation = tf2::toMsg(q);
    costmap_ros_->getTfBuffer()->setTransform(transform, "synthetic_robot", true);
  }
};

}  // namespace

TEST_F(ControllerTest, NativeServerLifecycleFollowPathAndSensorLossClosedLoop) {
  // One ROS context avoids the host's cross-process DDS network configuration.
  auto server = std::make_shared<TestControllerServer>();
  ASSERT_EQ(server->configure().id(), 2);  // inactive
  server->setPose(0.0, 0.0, 0.0);
  ASSERT_EQ(server->activate().id(), 3);  // active
  executor_.add_node(server->get_node_base_interface());

  double x = 0.0, y = 0.0, yaw = 0.0;
  bool send_cloud = true;
  geometry_msgs::msg::Twist command;
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  odom_pub->on_activate();
  auto cmd_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [&](const geometry_msgs::msg::Twist& msg) { command = msg; });
  auto previous = std::chrono::steady_clock::now();
  auto timer = node_->create_wall_timer(20ms, [&] {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count();
    previous = now;
    x += command.linear.x * std::cos(yaw) * dt;
    y += command.linear.x * std::sin(yaw) * dt;
    yaw += command.angular.z * dt;
    server->setPose(x, y, yaw);
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.header.stamp = node_->now();
    odom.child_frame_id = "base_link";
    odom.twist.twist = command;
    odom_pub->publish(odom);
    if (send_cloud) pub_->publish(cloud());
  });

  using FollowPath = nav2_msgs::action::FollowPath;
  auto client = rclcpp_action::create_client<FollowPath>(node_, "follow_path");
  ASSERT_TRUE(client->wait_for_action_server(2s));
  auto follow = [&](double goal_x, double goal_y, double goal_yaw) {
    FollowPath::Goal goal;
    goal.controller_id = "FollowPath";
    goal.goal_checker_id = "goal_checker";
    goal.path.header.frame_id = "odom";
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal_yaw);
    for (const auto& xy : {std::array<double, 2>{x, y},
                           std::array<double, 2>{goal_x, goal_y}}) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = goal.path.header;
      pose.pose.position.x = xy[0];
      pose.pose.position.y = xy[1];
      pose.pose.orientation = tf2::toMsg(q);
      goal.path.poses.push_back(pose);
    }
    auto sent = client->async_send_goal(goal);
    EXPECT_EQ(executor_.spin_until_future_complete(sent, 2s),
              rclcpp::FutureReturnCode::SUCCESS);
    return client->async_get_result(sent.get());
  };

  auto result = follow(1.0, 0.0, 0.6);
  EXPECT_EQ(executor_.spin_until_future_complete(result, 15s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_NEAR(x, 1.0, 0.12);
  EXPECT_NEAR(y, 0.0, 0.12);
  EXPECT_NEAR(yaw, 0.6, 0.12);

  auto stopped = follow(x + 1.0, y, 0.0);
  send_cloud = false;
  EXPECT_EQ(executor_.spin_until_future_complete(stopped, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(stopped.get().code, rclcpp_action::ResultCode::ABORTED);
  executor_.spin_some();
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);

  timer->cancel();
  EXPECT_EQ(server->deactivate().id(), 2);
  EXPECT_EQ(server->cleanup().id(), 1);
  executor_.remove_node(server->get_node_base_interface());
}
