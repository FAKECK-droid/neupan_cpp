// SPDX-License-Identifier: GPL-3.0-or-later
#include <memory>
#include <string>
#include <vector>

#include "nav2_controller/controller_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace neupan_controller {

// Keep Nav2's server unchanged; only provide typed empty arrays, which Humble's
// YAML parameter parser cannot represent. No grid layers or filters are loaded.
class NeupanControllerServer : public nav2_controller::ControllerServer {
 public:
  NeupanControllerServer() {
    costmap_ros_->set_parameters({
        rclcpp::Parameter("plugins", std::vector<std::string>{}),
        rclcpp::Parameter("filters", std::vector<std::string>{})});
  }
};

}  // namespace neupan_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<neupan_controller::NeupanControllerServer>();
  RCLCPP_INFO(server->get_logger(), "NeuPAN server ready; local costmap layers disabled");
  rclcpp::spin(server->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
