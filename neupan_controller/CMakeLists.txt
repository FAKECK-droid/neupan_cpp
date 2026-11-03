cmake_minimum_required(VERSION 3.16)
project(neupan_controller VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(libneupan REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(neupan_controller SHARED src/controller.cpp)
target_compile_features(neupan_controller PUBLIC cxx_std_17)
target_include_directories(neupan_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(neupan_controller neupan::neupan)
ament_target_dependencies(neupan_controller
  ament_index_cpp nav2_core nav2_costmap_2d nav2_util nav_msgs geometry_msgs
  sensor_msgs pluginlib rcl_interfaces rclcpp rclcpp_lifecycle tf2 tf2_ros tf2_geometry_msgs)
target_compile_options(neupan_controller PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(nav2_core neupan_controller.xml)

# A thin native-server entry point supplies typed empty costmap layer arrays.
add_executable(neupan_controller_server src/controller_server.cpp)
target_compile_features(neupan_controller_server PRIVATE cxx_std_17)
ament_target_dependencies(neupan_controller_server nav2_controller rclcpp)
install(TARGETS neupan_controller_server DESTINATION lib/${PROJECT_NAME})

add_executable(scan_to_cloud src/scan_to_cloud.cpp)
target_compile_features(scan_to_cloud PRIVATE cxx_std_17)
ament_target_dependencies(scan_to_cloud laser_geometry rclcpp sensor_msgs)
target_compile_options(scan_to_cloud PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS scan_to_cloud DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/train_turtlebot3.py DESTINATION lib/${PROJECT_NAME})
# Reuse the core export tools rather than maintaining a second NPTF writer.
install(FILES
  ../libneupan/tools/export_dune_weights.py
  ../libneupan/tools/nptf.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS neupan_controller EXPORT neupan_controllerTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES ${libneupan_MODELS_DIR}/diff_default.bin
  DESTINATION share/${PROJECT_NAME}/models)
install(FILES README.md TURTLEBOT3.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(nav2_msgs REQUIRED)
  ament_add_gtest(test_controller test/test_controller.cpp)
  ament_target_dependencies(test_controller
    rclcpp_action nav2_msgs nav2_controller nav2_core nav2_costmap_2d nav2_util nav_msgs geometry_msgs sensor_msgs
    pluginlib rclcpp rclcpp_lifecycle tf2 tf2_ros tf2_geometry_msgs)
  set_tests_properties(test_controller PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=83")
endif()

ament_export_targets(neupan_controllerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(libneupan nav2_core nav2_costmap_2d nav2_util nav_msgs
  geometry_msgs sensor_msgs pluginlib rcl_interfaces rclcpp rclcpp_lifecycle tf2 tf2_ros
  tf2_geometry_msgs ament_index_cpp)
ament_package()
