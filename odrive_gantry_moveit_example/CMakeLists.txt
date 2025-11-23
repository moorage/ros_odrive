cmake_minimum_required(VERSION 3.8)
project(odrive_gantry_moveit_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(odrive_ros2_control REQUIRED)

include_directories(
  include
  ../odrive_base/include
)

add_library(sim_odrive_hw SHARED
  src/simulated_odrive_hardware.cpp
)
ament_target_dependencies(sim_odrive_hw
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  odrive_ros2_control
)
target_link_libraries(sim_odrive_hw odrive_ros2_control_plugin)
target_compile_features(sim_odrive_hw PUBLIC cxx_std_20)

pluginlib_export_plugin_description_file(hardware_interface sim_odrive_hardware.xml)

install(TARGETS sim_odrive_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config description launch srdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES sim_odrive_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_launch_test(test/gantry_trajectory_integration.test.py
    TARGET gantry_trajectory_integration
    TIMEOUT 120
  )
endif()

ament_package()
