#include <gtest/gtest.h>
#include <memory>
#include <cstring>

#include "odrive_ros2_control/odrive_system.hpp"
#include <linux/can.h>
#include "fake_can_transport.hpp"

using odrive_ros2_control::AxisConfig;
using odrive_ros2_control::AxisControlMode;
using odrive_ros2_control::AxisHealth;
using odrive_ros2_control::OdriveS1CanSystem;
using hardware_interface::CallbackReturn;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct OdriveSystemTestAccess {
  static std::vector<OdriveS1CanSystem::AxisCommandForTests> &commands(OdriveS1CanSystem &sys) {
    return sys.test_axis_commands();
  }
  static std::vector<AxisConfig> &configs(OdriveS1CanSystem &sys) { return sys.test_axis_configs(); }
  static std::vector<OdriveS1CanSystem::AxisRuntimeMetadataForTests> &runtime(OdriveS1CanSystem &sys) {
    return sys.test_runtime_metadata();
  }
  static std::vector<odrive_ros2_control::AxisState> &states(OdriveS1CanSystem &sys) { return sys.test_axis_states(); }
  static std::vector<AxisControlMode> &modes(OdriveS1CanSystem &sys) { return sys.test_command_modes(); }
  static void handle(OdriveS1CanSystem &sys, const can_frame &frame, const rclcpp::Time &t) {
    sys.handle_frame_for_tests(frame, t);
  }
  static std::vector<odrive_ros2_control::HardwareStatusMsg> &status(OdriveS1CanSystem &sys) {
    return sys.test_status_cache();
  }
  static odrive_ros2_control::OdriveS1CanSystem::UtilizationMetricsForTests &util(OdriveS1CanSystem &sys) {
    return sys.test_utilization();
  }
  static bool send_clear(OdriveS1CanSystem &sys, size_t idx) { return sys.send_clear_errors_for_tests(idx); }
  static bool clear_and_rearm(OdriveS1CanSystem &sys, size_t idx) {
    return sys.clear_errors_and_rearm_for_tests(idx);
  }
  static bool start_homing(OdriveS1CanSystem &sys) { return sys.start_homing_for_tests(); }
};

hardware_interface::HardwareInfo make_info() {
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

uint32_t axis_can_id(uint32_t node_id, uint32_t axis_index) {
  return node_id * 2 + axis_index;
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

  auto hb1 = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  auto hb2 = make_heartbeat(axis_can_id(2, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb1, rclcpp::Clock().now());
  OdriveSystemTestAccess::handle(sys, hb2, rclcpp::Clock().now());

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
    if (node == axis_can_id(1, 0) && cmd == Set_Input_Pos_msg_t::cmd_id) {
      Set_Input_Pos_msg_t msg;
      msg.decode_buf(frame.data);
      found_pos = true;
      EXPECT_NEAR(msg.Input_Pos, kPi / (2 * kPi), 1e-4);
    }
    if (node == axis_can_id(2, 0) && cmd == Set_Input_Vel_msg_t::cmd_id) {
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

TEST(IntegrationCan, TransmissionMappingAppliedForCommandsAndState) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.transmissions.clear();
  hardware_interface::TransmissionInfo tr;
  tr.name = "t1";
  tr.type = "SimpleTransmission";
  hardware_interface::JointInfo jinfo;
  jinfo.name = "joint1";
  jinfo.mechanical_reduction = 2.0;
  tr.joints.push_back(jinfo);
  hardware_interface::ActuatorInfo ainfo;
  ainfo.name = "motor1";
  ainfo.mechanical_reduction = 2.0;
  tr.actuators.push_back(ainfo);
  info.transmissions.push_back(tr);

  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto hb_closed = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed, rclcpp::Clock().now());
  sys.perform_command_mode_switch({"joint1/position"}, {});

  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = kPi;

  const size_t before = fake->sent_frames.size();
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const size_t after = fake->sent_frames.size();
  ASSERT_GT(after, before);

  bool found_pos = false;
  for (size_t i = before; i < after; ++i) {
    const auto &frame = fake->sent_frames[i];
    const uint32_t node = frame.can_id >> 5;
    const uint8_t cmd = frame.can_id & 0x1f;
    if (node == axis_can_id(1, 0) && cmd == Set_Input_Pos_msg_t::cmd_id) {
      Set_Input_Pos_msg_t msg;
      msg.decode_buf(frame.data);
      found_pos = true;
      EXPECT_NEAR(msg.Input_Pos, 1.0, 1e-4); // reduction 2:1 -> 1 motor turn for pi rad
    }
  }
  EXPECT_TRUE(found_pos);

  // Push encoder feedback of 1 turn and ensure joint position reflects transmission mapping.
  Get_Encoder_Estimates_msg_t enc{};
  enc.Pos_Estimate = 1.0;
  enc.Vel_Estimate = 0.0;
  can_frame enc_frame{};
  enc_frame.can_id = (axis_can_id(1, 0) << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
  enc_frame.can_dlc = Get_Encoder_Estimates_msg_t::msg_length;
  enc.encode_buf(enc_frame.data);
  fake->push_rx(enc_frame);
  sys.read(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_NEAR(OdriveSystemTestAccess::states(sys)[0].pos_joint, kPi, 1e-6);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, SkipsSendingUnchangedCommandsWithinTolerance) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["command_tolerance"] = "1e-5";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto hb_closed = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed, rclcpp::Clock().now());
  sys.perform_command_mode_switch({"joint1/position"}, {});

  auto count_pos_msgs = [](const std::vector<can_frame> &frames) {
    size_t count = 0;
    for (const auto &frame : frames) {
      const uint32_t node = frame.can_id >> 5;
      const uint8_t cmd = frame.can_id & 0x1f;
      if (node == axis_can_id(1, 0) && cmd == Set_Input_Pos_msg_t::cmd_id) count++;
    }
    return count;
  };

  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = 0.5;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const size_t after_first = count_pos_msgs(fake->sent_frames);

  cmds[0].position = 0.5; // unchanged -> should not emit new Set_Input_Pos
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const size_t after_second = count_pos_msgs(fake->sent_frames);
  EXPECT_EQ(after_second, after_first);

  cmds[0].position = 0.6; // change beyond tolerance -> should emit again
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const size_t after_third = count_pos_msgs(fake->sent_frames);
  EXPECT_GT(after_third, after_second);

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

  auto hb_fault = make_heartbeat(axis_can_id(1, 0), 0x1, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_fault, rclcpp::Clock().now());
  const size_t after_fault = fake->sent_frames.size();
  ASSERT_GT(after_fault, baseline);
  bool idle_sent = false;
  for (size_t i = baseline; i < after_fault; ++i) {
    const auto &frame = fake->sent_frames[i];
    const uint32_t node = frame.can_id >> 5;
    const uint8_t cmd = frame.can_id & 0x1f;
    if (node == axis_can_id(1, 0) && cmd == Set_Axis_State_msg_t::cmd_id) {
      Set_Axis_State_msg_t msg{};
      msg.decode_buf(frame.data);
      if (msg.Axis_Requested_State == AXIS_STATE_IDLE) idle_sent = true;
    }
  }
  EXPECT_TRUE(idle_sent);

  cmds[0].position = 2.0;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_EQ(fake->sent_frames.size(), after_fault); // no new frames while faulted

  ASSERT_TRUE(OdriveSystemTestAccess::clear_and_rearm(sys, 0));
  const size_t after_rearm = fake->sent_frames.size();
  auto hb_clear = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_clear, rclcpp::Clock().now());
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_GT(fake->sent_frames.size(), after_rearm);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, HomingCommandSendsAxisState) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  sys.perform_command_mode_switch({}, {});
  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].homing = 1.0;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));

  bool found_homing = false;
  for (const auto &frame : fake->sent_frames) {
    const uint32_t node = frame.can_id >> 5;
    const uint8_t cmd = frame.can_id & 0x1f;
    if (node == axis_can_id(1, 0) && cmd == Set_Axis_State_msg_t::cmd_id) {
      Set_Axis_State_msg_t msg{};
      msg.decode_buf(frame.data);
      if (msg.Axis_Requested_State == AXIS_STATE_HOMING) {
        found_homing = true;
        break;
      }
    }
  }
  EXPECT_TRUE(found_homing);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, HardwareStatusTracksHealthChanges) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["status_publish_rate"] = "100.0";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // Healthy heartbeat.
  auto hb_ok = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_ok, rclcpp::Clock().now());
  sys.read(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  ASSERT_FALSE(OdriveSystemTestAccess::status(sys).empty());
  EXPECT_EQ(OdriveSystemTestAccess::status(sys)[0].status, 0u);

  // Inject fault and expect status to flip to ERROR.
  auto hb_fault = make_heartbeat(axis_can_id(1, 0), 0x2, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_fault, rclcpp::Clock().now());
  sys.read(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  ASSERT_FALSE(OdriveSystemTestAccess::status(sys).empty());
  EXPECT_EQ(OdriveSystemTestAccess::status(sys)[0].status, 2u);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, RespectsFrameBudgetPerCycle) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["max_frames_per_cycle"] = "2";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto hb1 = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  auto hb2 = make_heartbeat(axis_can_id(2, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb1, rclcpp::Clock().now());
  OdriveSystemTestAccess::handle(sys, hb2, rclcpp::Clock().now());
  sys.perform_command_mode_switch({"joint1/position", "joint2/position"}, {});

  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = 0.5;
  cmds[1].position = 1.0;

  const size_t before = fake->sent_frames.size();
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const size_t after = fake->sent_frames.size();
  const size_t diff = after - before;
  EXPECT_LE(diff, 2u); // budget allows only axis_state + one Set_Input_Pos

  bool joint1_pos_sent = false;
  bool joint2_pos_sent = false;
  for (size_t i = before; i < after; ++i) {
    const auto &frame = fake->sent_frames[i];
    const uint32_t node = frame.can_id >> 5;
    const uint8_t cmd = frame.can_id & 0x1f;
    if (node == axis_can_id(1, 0) && cmd == Set_Input_Pos_msg_t::cmd_id) joint1_pos_sent = true;
    if (node == axis_can_id(2, 0) && cmd == Set_Input_Pos_msg_t::cmd_id) joint2_pos_sent = true;
  }
  EXPECT_TRUE(joint1_pos_sent);
  EXPECT_FALSE(joint2_pos_sent);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, RespectsMaxFramesPerSecondBudget) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["max_frames_per_sec"] = "1.0";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  sys.perform_command_mode_switch({"joint1/position"}, {});

  fake->sent_frames.clear();
  auto &util = OdriveSystemTestAccess::util(sys);
  util.frames_in_window = 1;
  util.window_start = rclcpp::Clock().now();

  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = 0.5;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_TRUE(fake->sent_frames.empty()); // blocked by per-sec budget

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, UtilizationLimitSkipsSendsWhenOverBudget) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["can_utilization_limit"] = "0.01";
  info.hardware_parameters["max_frames_per_cycle"] = "0";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  sys.perform_command_mode_switch({"joint1/position"}, {});

  fake->sent_frames.clear();
  auto &util = OdriveSystemTestAccess::util(sys);
  util.frames_per_sec_ewma = 100000.0; // artificially high so utilization check trips
  util.window_start = rclcpp::Clock().now();

  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = 0.5;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_TRUE(fake->sent_frames.empty());

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, CommandAndFeedbackLoopUpdatesState) {
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
  auto hb_closed = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed, rclcpp::Clock().now());
  sys.perform_command_mode_switch({"joint1/velocity"}, {});

  // Push encoder and torque feedback to simulate CANSimple stream.
  Get_Encoder_Estimates_msg_t enc{};
  enc.Pos_Estimate = 1.5;
  enc.Vel_Estimate = 0.25;
  can_frame enc_frame{};
  enc_frame.can_id = (axis_can_id(1, 0) << 5) | Get_Encoder_Estimates_msg_t::cmd_id;
  enc_frame.can_dlc = Get_Encoder_Estimates_msg_t::msg_length;
  enc.encode_buf(enc_frame.data);
  fake->push_rx(enc_frame);

  Get_Torques_msg_t tq{};
  tq.Torque_Estimate = 0.4;
  can_frame tq_frame{};
  tq_frame.can_id = (axis_can_id(1, 0) << 5) | Get_Torques_msg_t::cmd_id;
  tq_frame.can_dlc = Get_Torques_msg_t::msg_length;
  tq.encode_buf(tq_frame.data);
  fake->push_rx(tq_frame);

  sys.read(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const auto &states = OdriveSystemTestAccess::states(sys);
  EXPECT_NEAR(states[0].pos_joint, 1.5 * 2 * kPi, 1e-6);
  EXPECT_NEAR(states[0].vel_joint, 0.25 * 2 * kPi, 1e-6);
  EXPECT_NEAR(states[0].effort_joint, 0.4, 1e-6);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, LimitMismatchWarnOnlyAllowsActivateAndWarns) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "WARN_ONLY";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;
  Set_Limits_msg_t lim{};
  lim.Velocity_Limit = 2.0f; // lower than expected actuator mapping
  lim.Current_Limit = 10.0f;
  can_frame lim_frame{};
  lim_frame.can_id = (axis_can_id(1, 0) << 5) | Set_Limits_msg_t::cmd_id;
  lim_frame.can_dlc = Set_Limits_msg_t::msg_length;
  lim.encode_buf(lim_frame.data);
  OdriveSystemTestAccess::handle(sys, lim_frame, rclcpp::Clock().now());

  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(OdriveSystemTestAccess::runtime(sys)[0].limit_check_result,
            odrive_ros2_control::OdriveS1CanSystem::AxisRuntimeMetadataForTests::LimitCheckResult::WARN);
  EXPECT_EQ(OdriveSystemTestAccess::states(sys)[0].health, AxisHealth::WARNING);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, LimitMismatchStrictFailsActivate) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "STRICT";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;
  Set_Limits_msg_t lim{};
  lim.Velocity_Limit = 2.0f;
  lim.Current_Limit = 10.0f;
  can_frame lim_frame{};
  lim_frame.can_id = (axis_can_id(1, 0) << 5) | Set_Limits_msg_t::cmd_id;
  lim_frame.can_dlc = Set_Limits_msg_t::msg_length;
  lim.encode_buf(lim_frame.data);
  OdriveSystemTestAccess::handle(sys, lim_frame, rclcpp::Clock().now());

  EXPECT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, LimitCheckStrictFailsWithSdoAccelMismatch) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  fake->set_send_hook([](const can_frame &frame, FakeCanTransport &t) {
    const uint8_t cmd = frame.can_id & 0x1f;
    if (cmd != 0x04) return; // RxSdo
    const uint16_t endpoint = static_cast<uint16_t>(frame.data[1] | (frame.data[2] << 8));
    float value = 0.0f;
    if (endpoint == 374) value = 20.0f;         // vel_limit OK
    if (endpoint == 569) value = 10.0f;         // effort OK
    if (endpoint == 403) value = 10.0f;         // accel too low
    can_frame resp{};
    resp.can_id = (frame.can_id & ~0x1f) | 0x05; // TxSdo
    resp.can_dlc = 8;
    resp.data[0] = 0;
    resp.data[1] = static_cast<uint8_t>(endpoint & 0xffu);
    resp.data[2] = static_cast<uint8_t>((endpoint >> 8) & 0xffu);
    resp.data[3] = 0;
    std::memcpy(&resp.data[4], &value, sizeof(float));
    t.push_rx(resp);
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "STRICT";
  info.hardware_parameters["limits_check.use_sdo"] = "true";
  info.hardware_parameters["limits_check.flat_endpoints_path"] = "test/odrive_s1_v6.11_flat_endpoints.json";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;
  cfg.limit_effort = 5.0;
  cfg.limit_acceleration = 30.0; // in joint units -> actuator turns/s^2 higher than returned

  EXPECT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, LimitCheckStrictFailsWithoutOdriveLimits) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "STRICT";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;

  EXPECT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, LimitCheckWarnOnlyAcrossAllLimitsWithSdoPrismatic) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  fake->set_send_hook([](const can_frame &frame, FakeCanTransport &t) {
    const uint8_t cmd = frame.can_id & 0x1f;
    if (cmd != 0x04) return; // RxSdo
    const uint16_t endpoint = static_cast<uint16_t>(frame.data[1] | (frame.data[2] << 8));
    float value = 0.0f;
    if (endpoint == 374) value = 18.0f;          // vel_limit (turns/s)
    if (endpoint == 569) value = 3.0f;           // current limit
    if (endpoint == 403) value = 80.0f;          // accel_limit (turns/s^2)
    can_frame resp{};
    resp.can_id = (frame.can_id & ~0x1f) | 0x05; // TxSdo
    resp.can_dlc = 8;
    resp.data[0] = 0;
    resp.data[1] = static_cast<uint8_t>(endpoint & 0xffu);
    resp.data[2] = static_cast<uint8_t>((endpoint >> 8) & 0xffu);
    resp.data[3] = 0;
    std::memcpy(&resp.data[4], &value, sizeof(float));
    t.push_rx(resp);
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.joints[0].type = "prismatic";
  info.joints[0].parameters["lead_screw_pitch"] = "0.01";
  info.hardware_parameters["limits_check.mode"] = "WARN_ONLY";
  info.hardware_parameters["limits_check.use_sdo"] = "true";
  info.hardware_parameters["limits_check.flat_endpoints_path"] = "test/odrive_s1_v6.11_flat_endpoints.json";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 0.2;     // expected actuator 20 turns/s
  cfg.limit_effort = 5.0;       // expected actuator 5 A
  cfg.limit_acceleration = 1.0; // expected actuator 100 turns/s^2

  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  const auto &meta = OdriveSystemTestAccess::runtime(sys)[0];
  EXPECT_EQ(meta.limit_check_result,
            odrive_ros2_control::OdriveS1CanSystem::AxisRuntimeMetadataForTests::LimitCheckResult::WARN);
  EXPECT_EQ(OdriveSystemTestAccess::states(sys)[0].health, AxisHealth::WARNING);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, CommandSendLatencyRecordedWithinCycle) {
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
  cmds[0].position = 0.25;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));

  ASSERT_FALSE(fake->sent_frames.empty());
  EXPECT_LT(OdriveSystemTestAccess::util(sys).last_send_latency_sec, 0.05);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, ModeSwitchWaitsForClosedLoopBeforeStreaming) {
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

  auto hb_closed = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed, rclcpp::Clock().now());
  sys.perform_command_mode_switch({"joint1/position"}, {});
  auto &cmds = OdriveSystemTestAccess::commands(sys);
  cmds[0].position = 0.5;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  const size_t pos_msgs = fake->sent_frames.size();
  EXPECT_GT(pos_msgs, 0u);

  // Switch to velocity and ensure no streaming until heartbeat acknowledges CLOSED_LOOP.
  sys.perform_command_mode_switch({"joint1/velocity"}, {"joint1/position"});
  cmds[0].velocity = 1.0;
  const size_t before_vel = fake->sent_frames.size();
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_EQ(fake->sent_frames.size(), before_vel); // gated

  auto hb_closed_vel = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed_vel, rclcpp::Clock().now());
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));
  EXPECT_GT(fake->sent_frames.size(), before_vel);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, HomingSuccessAndFailureClassification) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  OdriveSystemTestAccess::commands(sys)[0].homing = 1.0;
  sys.write(rclcpp::Clock().now(), rclcpp::Duration(0, 0));

  // Homing success: heartbeat transitions from HOMING to CLOSED_LOOP with no error.
  auto now = rclcpp::Clock().now();
  auto hb_homing = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_HOMING);
  OdriveSystemTestAccess::handle(sys, hb_homing, now);
  auto hb_closed = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed, now + rclcpp::Duration(0, 1000000));
  EXPECT_EQ(OdriveSystemTestAccess::runtime(sys)[0].last_homing_result, "success");
  EXPECT_EQ(OdriveSystemTestAccess::modes(sys)[0], AxisControlMode::IDLE);

  // Homing failure: heartbeat with error during homing.
  ASSERT_TRUE(OdriveSystemTestAccess::start_homing(sys));
  auto hb_fail = make_heartbeat(axis_can_id(1, 0), 0x2, AXIS_STATE_HOMING);
  OdriveSystemTestAccess::handle(sys, hb_fail, now + rclcpp::Duration(0, 2000000));
  EXPECT_EQ(OdriveSystemTestAccess::runtime(sys)[0].last_homing_result, "failed");
  EXPECT_EQ(OdriveSystemTestAccess::states(sys)[0].health, AxisHealth::ERROR);
  EXPECT_EQ(OdriveSystemTestAccess::modes(sys)[0], AxisControlMode::IDLE);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}

TEST(IntegrationCan, LimitCheckUsesSdoWhenEnabled) {
  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  fake->set_send_hook([](const can_frame &frame, FakeCanTransport &t) {
    const uint8_t cmd = frame.can_id & 0x1f;
    if (cmd != 0x04) return; // RxSdo
    if (frame.can_dlc < 3) return;
    const uint16_t endpoint = static_cast<uint16_t>(frame.data[1] | (frame.data[2] << 8));
    float value = 0.0f;
    if (endpoint == 374) value = 10.0f;          // vel_limit
    if (endpoint == 569) value = 3.0f;           // effective_current_lim
    if (endpoint == 403) value = 4.0f;           // accel_limit
    can_frame resp{};
    resp.can_id = (frame.can_id & ~0x1f) | 0x05; // TxSdo
    resp.can_dlc = 8;
    resp.data[0] = 0;
    resp.data[1] = static_cast<uint8_t>(endpoint & 0xffu);
    resp.data[2] = static_cast<uint8_t>((endpoint >> 8) & 0xffu);
    resp.data[3] = 0;
    std::memcpy(&resp.data[4], &value, sizeof(float));
    t.push_rx(resp);
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "WARN_ONLY";
  info.hardware_parameters["limits_check.use_sdo"] = "true";
  info.hardware_parameters["limits_check.flat_endpoints_path"] = "test/odrive_s1_v6.11_flat_endpoints.json";
  info.hardware_parameters["limits_check.sdo_timeout_sec"] = "0.1";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto hb_closed = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed, rclcpp::Clock().now());

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;
  cfg.limit_effort = 2.0;
  cfg.limit_acceleration = 3.0;

  ASSERT_EQ(sys.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  const auto &meta = OdriveSystemTestAccess::runtime(sys)[0];
  EXPECT_TRUE(meta.odrive_velocity_limit.has_value());
  EXPECT_TRUE(meta.odrive_effort_limit.has_value());
  EXPECT_TRUE(meta.odrive_accel_limit.has_value());
  EXPECT_EQ(meta.limit_check_result,
            odrive_ros2_control::OdriveS1CanSystem::AxisRuntimeMetadataForTests::LimitCheckResult::OK);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}
