#pragma once

#include <cstdint>
#include <string>
#include <vector>

#if __has_include(<control_msgs/msg/hardware_status.hpp>)
#include <control_msgs/msg/hardware_status.hpp>
namespace odrive_ros2_control {
using HardwareStatusMsg = control_msgs::msg::HardwareStatus;
} // namespace odrive_ros2_control
#else
// Fallback stub for environments without control_msgs available at build time.
namespace control_msgs {
namespace msg {
struct HardwareStatus {
  // Status values: 0=OK,1=WARNING,2=ERROR for compatibility with AxisHealth mapping.
  uint8_t status = 0;
  uint8_t power_state = 0;
  std::string name;
  std::string message;
  struct KeyValue {
    std::string key;
    std::string value;
  };
  std::vector<KeyValue> values;
};
} // namespace msg
} // namespace control_msgs
namespace odrive_ros2_control {
using HardwareStatusMsg = control_msgs::msg::HardwareStatus;
} // namespace odrive_ros2_control
#endif

