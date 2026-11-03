// SPDX-License-Identifier: GPL-3.0-or-later
#include <memory>
#include <string>

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace neupan_controller {

// Keep the official simulator's LaserScan for AMCL/costmaps and provide a
// PointCloud2 copy for NeuPAN. The controller performs the timestamped TF.
class ScanToCloud : public rclcpp::Node {
 public:
  ScanToCloud() : Node("scan_to_cloud") {
    const auto input = declare_parameter<std::string>("input_topic", "scan");
    const auto output = declare_parameter<std::string>("output_topic", "scan/points");
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input, qos, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          sensor_msgs::msg::PointCloud2 cloud;
          projector_.projectLaser(*scan, cloud);
          publisher_->publish(cloud);
        });
  }

 private:
  laser_geometry::LaserProjection projector_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace neupan_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<neupan_controller::ScanToCloud>());
  rclcpp::shutdown();
  return 0;
}
