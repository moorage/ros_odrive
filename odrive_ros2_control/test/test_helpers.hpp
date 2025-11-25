#pragma once

#include <linux/can.h>
#include <map>
#include <vector>
#include <cstring>

#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "odrive_ros2_control/odrive_system.hpp"
#include "fake_can_transport.hpp"

// Shared helpers for odrive_ros2_control tests to avoid duplication and magic numbers.
namespace odrive_ros2_control::test_support {

// Commonly used endpoint IDs for SDO interactions.
inline constexpr uint16_t kEndpointVelocityLimit = 374;
inline constexpr uint16_t kEndpointCurrentLimit = 569;
inline constexpr uint16_t kEndpointAccelerationLimit = 403;
inline constexpr uint8_t kCmdRxSdo = 0x04;
inline constexpr uint8_t kCmdTxSdo = 0x05;

inline hardware_interface::HardwareInfo make_info() {
  hardware_interface::HardwareInfo info;
  info.name = "test_hw";
  info.type = "system";
  info.hardware_parameters["skip_can_validation"] = "true";
  for (int i = 0; i < 2; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(i + 1);
    joint.type = "revolute";
    hardware_interface::InterfaceInfo pos;
    pos.name = hardware_interface::HW_IF_POSITION;
    hardware_interface::InterfaceInfo vel;
    vel.name = hardware_interface::HW_IF_VELOCITY;
    hardware_interface::InterfaceInfo eff;
    eff.name = hardware_interface::HW_IF_EFFORT;
    joint.command_interfaces = {pos, vel, eff};
    joint.state_interfaces = {pos, vel, eff};
    joint.parameters["odrive_node_id"] = std::to_string(i + 1);
    joint.parameters["odrive_axis_index"] = "0";
    info.joints.push_back(joint);
  }
  return info;
}

// S1 exposes only axis0, so CAN ID == node_id.
inline uint32_t axis_can_id(uint32_t node_id, uint32_t /*axis_index*/) {
  return node_id;
}

inline uint32_t frame_axis_id(const can_frame &frame) {
  return frame.can_id >> 5;
}

inline uint8_t frame_cmd_id(const can_frame &frame) {
  return frame.can_id & 0x1f;
}

inline bool frame_matches(const can_frame &frame, uint32_t axis_id, uint8_t cmd_id) {
  return frame_axis_id(frame) == axis_id && frame_cmd_id(frame) == cmd_id;
}

inline uint16_t frame_endpoint(const can_frame &frame) {
  return static_cast<uint16_t>(frame.data[1] | (frame.data[2] << 8));
}

inline can_frame make_heartbeat(uint32_t node_id, uint32_t axis_error, uint8_t state) {
  Heartbeat_msg_t hb{};
  hb.Axis_Error = axis_error;
  hb.Axis_State = state;
  can_frame frame{};
  frame.can_id = (node_id << 5) | Heartbeat_msg_t::cmd_id;
  frame.can_dlc = Heartbeat_msg_t::msg_length;
  hb.encode_buf(frame.data);
  return frame;
}

struct TransportOverride {
  TransportOverride() {
    odrive_ros2_control::OdriveS1CanSystem::set_transport_factory_for_tests([this]() {
      transport = std::make_shared<FakeCanTransport>();
      return transport;
    });
  }
  ~TransportOverride() { odrive_ros2_control::OdriveS1CanSystem::set_transport_factory_for_tests(nullptr); }
  std::shared_ptr<FakeCanTransport> transport;
};

template <typename MsgT>
bool decode_if_matches(const can_frame &frame, uint32_t axis_id, MsgT &msg) {
  if (!frame_matches(frame, axis_id, MsgT::cmd_id)) return false;
  msg.decode_buf(frame.data);
  return true;
}

template <typename Iter, typename MsgT>
bool decode_matching_frame(Iter begin, Iter end, uint32_t axis_id, MsgT &msg) {
  for (auto it = begin; it != end; ++it) {
    if (decode_if_matches(*it, axis_id, msg)) return true;
  }
  return false;
}

template <typename MsgT>
bool decode_matching_frame(const std::vector<can_frame> &frames, uint32_t axis_id, MsgT &msg) {
  return decode_matching_frame(frames.begin(), frames.end(), axis_id, msg);
}

inline size_t count_frames(const std::vector<can_frame> &frames, uint32_t axis_id, uint8_t cmd_id) {
  size_t count = 0;
  for (const auto &frame : frames) {
    if (frame_matches(frame, axis_id, cmd_id)) ++count;
  }
  return count;
}

inline can_frame make_sdo_response(const can_frame &request, uint16_t endpoint, float value) {
  can_frame resp{};
  resp.can_id = (request.can_id & ~0x1f) | kCmdTxSdo;
  resp.can_dlc = 8;
  resp.data[0] = 0;
  resp.data[1] = static_cast<uint8_t>(endpoint & 0xffu);
  resp.data[2] = static_cast<uint8_t>((endpoint >> 8) & 0xffu);
  resp.data[3] = 0;
  std::memcpy(&resp.data[4], &value, sizeof(float));
  return resp;
}

inline void install_sdo_responder(FakeCanTransport &transport, const std::map<uint16_t, float> &responses) {
  transport.set_send_hook([responses](const can_frame &frame, FakeCanTransport &t) {
    const uint8_t cmd = frame_cmd_id(frame);
    if (cmd != kCmdRxSdo || frame.can_dlc < 3) return;
    const auto endpoint = frame_endpoint(frame);
    const auto it = responses.find(endpoint);
    if (it == responses.end()) return;
    t.push_rx(make_sdo_response(frame, endpoint, it->second));
  });
}

} // namespace odrive_ros2_control::test_support
