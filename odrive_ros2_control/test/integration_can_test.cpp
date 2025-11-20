#include <gtest/gtest.h>
#include <memory>

#include "odrive_ros2_control/odrive_system.hpp"
#include "fake_can_transport.hpp"

using odrive_ros2_control::AxisConfig;
using odrive_ros2_control::AxisHealth;
using odrive_ros2_control::OdriveS1CanSystem;
using hardware_interface::CallbackReturn;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct OdriveSystemTestAccess {
  static std::vector<OdriveS1CanSystem::AxisCommand> &commands(OdriveS1CanSystem &sys) {
    return sys.axis_commands_;
  }
  static std::vector<AxisConfig> &configs(OdriveS1CanSystem &sys) { return sys.axis_configs_; }
  static std::vector<OdriveS1CanSystem::AxisRuntimeMetadata> &runtime(OdriveS1CanSystem &sys) {
    return sys.runtime_metadata_;
  }
  static void handle(OdriveS1CanSystem &sys, const can_frame &frame, const rclcpp::Time &t) {
    sys.handle_frame(frame, t);
  }
  static bool send_clear(OdriveS1CanSystem &sys, size_t idx) { return sys.send_clear_errors(idx); }
};

hardware_interface::HardwareInfo make_info() {
  hardware_interface::HardwareInfo info;
  info.name = "test_hw";
  info.type = "system";
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

can_frame make_heartbeat(uint32_t node_id, uint32_t axis_error, uint8_t state) {
  Heartbeat_msg_t hb{};
  hb.Axis_Error = axis_error;
  hb.Axis_State = state;
  can_frame frame{};
  frame.can_id = (node_id << 5) | Heartbeat_msg_t::cmd_id;
  frame.can_dlc = Heartbeat_msg_t::msg_length;
  hb.encode_buf(frame.data);
  return frame;
}

} // namespace

TEST(IntegrationCan, CommandRoutingSendsFramesToExpectedNodes) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  sys.perform_command_mode_switch({"joint1/position", "joint2/velocity"}, {});
  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = kPi;
  cmds[1].velocity = 1.0;

  const size_t before = fake->sent_frames.size();
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  ASSERT_GT(fake->sent_frames.size(), before);

  bool found_pos = false;
  bool found_vel = false;
  for (const auto &frame : fake->sent_frames) {
    const uint32_t node = frame.can_id >> 5;
    const uint8_t cmd = frame.can_id & 0x1f;
    if (node == 1 && cmd == Set_Input_Pos_msg_t::cmd_id) {
      Set_Input_Pos_msg_t msg;
      msg.decode_buf(frame.data);
      found_pos = true;
      EXPECT_NEAR(msg.Input_Pos, kPi / (2 * kPi), 1e-4);
    }
    if (node == 2 && cmd == Set_Input_Vel_msg_t::cmd_id) {
      Set_Input_Vel_msg_t msg;
      msg.decode_buf(frame.data);
      found_vel = true;
      EXPECT_NEAR(msg.Input_Vel, 1.0 / (2 * kPi), 1e-4);
    }
  }
  EXPECT_TRUE(found_pos);
  EXPECT_TRUE(found_vel);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, FaultedAxisStopsSendingUntilCleared) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  sys.perform_command_mode_switch({"joint1/position"}, {});

  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = 1.0;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const size_t baseline = fake->sent_frames.size();

  auto hb_fault = make_heartbeat(1, 0x1, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_fault, rclcpp::Clock().now());
  cmds[0].position = 2.0;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_EQ(fake->sent_frames.size(), baseline); // no new frames while faulted

  OdriveSystemTestAccess::send_clear(sys, 0);
  auto hb_clear = make_heartbeat(1, 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_clear, rclcpp::Clock().now());
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_GT(fake->sent_frames.size(), baseline);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}
