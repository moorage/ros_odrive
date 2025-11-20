cmake_minimum_required(VERSION 3.8)
project(odrive_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_auto_find_build_dependencies()

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(../odrive_base/include)

ament_auto_add_library(
  odrive_ros2_control_plugin SHARED
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_hardware_interface.cpp
)

ament_target_dependencies(odrive_ros2_control_plugin
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  diagnostic_updater
  control_msgs
  std_srvs
  transmission_interface
)

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

target_compile_features(odrive_ros2_control_plugin PRIVATE cxx_std_20)

install(TARGETS odrive_ros2_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

ament_add_gtest(unit_logic_test
  test/unit_logic_test.cpp
)
target_link_libraries(unit_logic_test odrive_ros2_control_plugin)
ament_target_dependencies(unit_logic_test
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  control_msgs
  std_srvs
  diagnostic_updater
)

ament_add_gtest(integration_can_test
  test/integration_can_test.cpp
)
target_link_libraries(integration_can_test odrive_ros2_control_plugin)
ament_target_dependencies(integration_can_test
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  control_msgs
  std_srvs
  diagnostic_updater
)

ament_package()
