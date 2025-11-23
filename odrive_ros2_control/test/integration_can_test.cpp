#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>
#include <cstring>

#include "control_msgs/msg/hardware_status.hpp"
#include "odrive_ros2_control/odrive_system.hpp"
#include "fake_can_transport.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

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
  static std::vector<odrive_ros2_control::AxisState> &states(OdriveS1CanSystem &sys) { return sys.axis_states_; }
  static rclcpp::Node::SharedPtr node(OdriveS1CanSystem &sys) { return sys.node_; }
  static void set_last_status_time(OdriveS1CanSystem &sys, const rclcpp::Time &t) {
    sys.last_status_publish_time_ = t;
  }
  static void handle(OdriveS1CanSystem &sys, const can_frame &frame, const rclcpp::Time &t) {
    sys.handle_frame(frame, t);
  }
  static bool send_clear(OdriveS1CanSystem &sys, size_t idx) { return sys.send_clear_errors(idx); }
  static bool clear_and_rearm(OdriveS1CanSystem &sys, size_t idx) { return sys.clear_errors_and_rearm(idx); }
  static bool start_homing(OdriveS1CanSystem &sys) { return sys.start_homing_for_tests(); }
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
  ASSERT_TRUE(OdriveSystemTestAccess::start_homing(sys));

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

TEST(IntegrationCan, HardwareStatusReflectsAxisErrorsAndHeartbeats) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::shared_ptr<FakeCanTransport> fake;
  OdriveS1CanSystem::set_transport_factory_for_tests([&]() {
    fake = std::make_shared<FakeCanTransport>();
    return fake;
  });

  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["status_publish_rate"] = "100.0";
  info.hardware_parameters["heartbeat_timeout"] = "0.1";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto listener = std::make_shared<rclcpp::Node>("status_listener");
  control_msgs::msg::HardwareStatus::SharedPtr received;
  auto sub = listener->create_subscription<control_msgs::msg::HardwareStatus>(
      "hardware_status", rclcpp::QoS{10},
      [&](control_msgs::msg::HardwareStatus::SharedPtr msg) { received = std::move(msg); });
  executor->add_node(listener);

  auto now = OdriveSystemTestAccess::node(sys)->get_clock()->now();
  auto hb_fault = make_heartbeat(axis_can_id(1, 0), 0x5, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_fault, now);

  auto &states = OdriveSystemTestAccess::states(sys);
  states[1].last_heartbeat = now - rclcpp::Duration(1, 0); // trigger stale heartbeat warning
  OdriveSystemTestAccess::set_last_status_time(sys, now - rclcpp::Duration(10, 0));

  sys.read(OdriveSystemTestAccess::node(sys)->get_clock()->now(), rclcpp::Duration(0, 0));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received, nullptr);
  ASSERT_EQ(received->device_status.size(), 2u);

  auto find_by_name = [&](const std::string &name) {
    for (const auto &dev : received->device_status) {
      if (dev.name == name) return &dev;
    }
    return static_cast<const control_msgs::msg::HardwareComponentStatus *>(nullptr);
  };

  const auto *joint1 = find_by_name("joint1");
  const auto *joint2 = find_by_name("joint2");
  ASSERT_NE(joint1, nullptr);
  ASSERT_NE(joint2, nullptr);
  EXPECT_EQ(joint1->status, control_msgs::msg::HardwareStatus::STATUS_ERROR);
  EXPECT_NE(joint1->error_message.find("axis_error"), std::string::npos);
  EXPECT_NE(joint1->error_message.find("requires_rearm=true"), std::string::npos);

  EXPECT_EQ(joint2->status, control_msgs::msg::HardwareStatus::STATUS_WARNING);
  EXPECT_NE(joint2->error_message.find("heartbeat_stale=true"), std::string::npos);

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
            odrive_ros2_control::OdriveS1CanSystem::AxisRuntimeMetadata::LimitCheckResult::WARN);
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
  ASSERT_TRUE(OdriveSystemTestAccess::start_homing(sys));

  // Homing success: heartbeat transitions from HOMING to CLOSED_LOOP with no error.
  auto now = rclcpp::Clock().now();
  auto hb_homing = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_HOMING);
  OdriveSystemTestAccess::handle(sys, hb_homing, now);
  auto hb_closed = make_heartbeat(axis_can_id(1, 0), 0x0, AXIS_STATE_CLOSED_LOOP_CONTROL);
  OdriveSystemTestAccess::handle(sys, hb_closed, now + rclcpp::Duration(0, 1000000));
  EXPECT_EQ(OdriveSystemTestAccess::runtime(sys)[0].last_homing_result, "success");

  // Homing failure: heartbeat with error during homing.
  ASSERT_TRUE(OdriveSystemTestAccess::start_homing(sys));
  auto hb_fail = make_heartbeat(axis_can_id(1, 0), 0x2, AXIS_STATE_HOMING);
  OdriveSystemTestAccess::handle(sys, hb_fail, now + rclcpp::Duration(0, 2000000));
  EXPECT_EQ(OdriveSystemTestAccess::runtime(sys)[0].last_homing_result, "failed");

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
            odrive_ros2_control::OdriveS1CanSystem::AxisRuntimeMetadata::LimitCheckResult::OK);

  OdriveS1CanSystem::set_transport_factory_for_tests(nullptr);
}
