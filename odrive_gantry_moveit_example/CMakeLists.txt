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
target_include_directories(sim_odrive_hw PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../odrive_base/include
)
target_compile_features(sim_odrive_hw PUBLIC cxx_std_20)

pluginlib_export_plugin_description_file(hardware_interface sim_odrive_hardware.xml)

install(TARGETS sim_odrive_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES sim_odrive_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
  set(WORKSPACE_INSTALL_PREFIX "${WORKSPACE_ROOT}/install")
  set(TEST_LIBRARY_DIRS
    "/opt/ros/jazzy/lib"
    "/opt/ros/jazzy/lib/x86_64-linux-gnu"
    "/opt/ros/jazzy/opt/sdformat_vendor/lib"
    "/opt/ros/jazzy/opt/rviz_ogre_vendor/lib"
    "/opt/ros/jazzy/opt/gz_math_vendor/lib"
    "/opt/ros/jazzy/opt/gz_utils_vendor/lib"
    "/opt/ros/jazzy/opt/gz_tools_vendor/lib"
    "/opt/ros/jazzy/opt/gz_cmake_vendor/lib"
    "${WORKSPACE_INSTALL_PREFIX}/odrive_ros2_control/lib"
    "${WORKSPACE_INSTALL_PREFIX}/odrive_gantry_moveit_example/lib"
  )
  set(TEST_PREFIX_PATHS
    "${WORKSPACE_INSTALL_PREFIX}/odrive_gantry_moveit_example"
    "${WORKSPACE_INSTALL_PREFIX}/odrive_ros2_control"
    "/opt/ros/jazzy"
  )
  set(TEST_PYTHONPATHS
    "/opt/ros/jazzy/lib/python3.12/site-packages"
    "${WORKSPACE_INSTALL_PREFIX}/odrive_gantry_moveit_example/lib/python3.12/site-packages"
    "${WORKSPACE_INSTALL_PREFIX}/odrive_ros2_control/lib/python3.12/site-packages"
  )
  set(TEST_PATHS
    "/opt/ros/jazzy/bin"
    "${WORKSPACE_INSTALL_PREFIX}/odrive_gantry_moveit_example/bin"
    "${WORKSPACE_INSTALL_PREFIX}/odrive_ros2_control/bin"
  )
  set(TEST_ENV_VARS "")
  foreach(_path IN LISTS TEST_PREFIX_PATHS)
    list(APPEND TEST_ENV_VARS
      AMENT_PREFIX_PATH=${_path}
      CMAKE_PREFIX_PATH=${_path}
    )
  endforeach()
  # Ensure launch tests can import ROS and workspace python packages.
  string(JOIN ":" TEST_PYTHONPATH_STR ${TEST_PYTHONPATHS})
  list(APPEND TEST_ENV_VARS "PYTHONPATH=${TEST_PYTHONPATH_STR}:$ENV{PYTHONPATH}")
  string(JOIN ":" TEST_PATH_STR ${TEST_PATHS} "$ENV{PATH}")
  list(APPEND TEST_ENV_VARS "PATH=${TEST_PATH_STR}")
  string(JOIN ":" TEST_LIBRARY_DIRS_STR ${TEST_LIBRARY_DIRS})
  list(APPEND TEST_ENV_VARS "LD_LIBRARY_PATH=${TEST_LIBRARY_DIRS_STR}:$ENV{LD_LIBRARY_PATH}")

  ament_add_pytest_test(gantry_trajectory_integration
    test/gantry_trajectory_integration_test.py
    TIMEOUT 120
    APPEND_ENV ${TEST_ENV_VARS}
  )
  set_tests_properties(gantry_trajectory_integration PROPERTIES ENVIRONMENT "${TEST_ENV_VARS}")
endif()

ament_package()
