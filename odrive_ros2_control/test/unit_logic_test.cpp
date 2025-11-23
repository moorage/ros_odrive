#include <gtest/gtest.h>
#include <memory>

#include "odrive_ros2_control/odrive_system.hpp"
#include "fake_can_transport.hpp"

using odrive_ros2_control::AxisConfig;
using odrive_ros2_control::AxisControlMode;
using odrive_ros2_control::AxisHealth;
using odrive_ros2_control::OdriveS1CanSystem;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct OdriveSystemTestAccess {
  using LimitCheckResult = OdriveS1CanSystem::AxisRuntimeMetadata::LimitCheckResult;
  static double joint_to_actuator_pos(OdriveS1CanSystem &sys, const AxisConfig &cfg, double pos) {
    return sys.joint_to_actuator_pos(cfg, pos);
  }
  static double actuator_to_joint_pos(OdriveS1CanSystem &sys, const AxisConfig &cfg, double turns) {
    return sys.actuator_to_joint_pos(cfg, turns);
  }
  static double joint_vel_to_actuator(OdriveS1CanSystem &sys, const AxisConfig &cfg, double vel) {
    return sys.joint_vel_to_actuator(cfg, vel);
  }
  static double actuator_vel_to_joint(OdriveS1CanSystem &sys, const AxisConfig &cfg, double vel) {
    return sys.actuator_vel_to_joint(cfg, vel);
  }
  static bool run_limit_check(OdriveS1CanSystem &sys, size_t idx) { return sys.run_limit_check(idx); }
  static void handle(OdriveS1CanSystem &sys, const can_frame &frame, const rclcpp::Time &t) {
    sys.handle_frame(frame, t);
  }
  static std::vector<OdriveS1CanSystem::AxisRuntimeMetadata> &runtime(OdriveS1CanSystem &sys) {
    return sys.runtime_metadata_;
  }
  static std::vector<AxisConfig> &configs(OdriveS1CanSystem &sys) { return sys.axis_configs_; }
  static std::vector<odrive_ros2_control::AxisState> &states(OdriveS1CanSystem &sys) { return sys.axis_states_; }
  static bool clear_and_rearm(OdriveS1CanSystem &sys, size_t idx) { return sys.clear_errors_and_rearm(idx); }
};

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

hardware_interface::HardwareInfo make_info_missing_node_id() {
  auto info = make_info();
  info.joints[0].parameters.erase("odrive_node_id");
  info.joints[0].parameters.erase("node_id");
  return info;
}

hardware_interface::HardwareInfo make_info_with_axis_index() {
  auto info = make_info();
  info.joints[0].parameters["odrive_axis_index"] = "1";
  return info;
}

hardware_interface::HardwareInfo make_info_missing_state_if() {
  auto info = make_info();
  info.joints[0].state_interfaces.clear();
  return info;
}

hardware_interface::HardwareInfo make_info_missing_command_if() {
  auto info = make_info();
  info.joints[0].command_interfaces.clear();
  return info;
}

hardware_interface::HardwareInfo make_info_three_axes_same_node() {
  auto info = make_info();
  hardware_interface::ComponentInfo joint;
  joint.name = "joint3";
  joint.type = "revolute";
  hardware_interface::InterfaceInfo pos;
  pos.name = hardware_interface::HW_IF_POSITION;
  hardware_interface::InterfaceInfo vel;
  vel.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceInfo eff;
  eff.name = hardware_interface::HW_IF_EFFORT;
  joint.command_interfaces = {pos, vel, eff};
  joint.state_interfaces = {pos, vel, eff};
  joint.parameters["odrive_node_id"] = "1"; // same as joint1 to exceed 2 axes per node
  joint.parameters["odrive_axis_index"] = "1";
  info.joints.push_back(joint);
  return info;
}

} // namespace

TEST(AxisMapping, RevoluteGearRatioConversion) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  AxisConfig cfg;
  cfg.joint_type = "revolute";
  cfg.gear_ratio = 2.0;

  const double actuator = OdriveSystemTestAccess::joint_to_actuator_pos(sys, cfg, kPi);
  EXPECT_NEAR(actuator, 0.25, 1e-6);
  EXPECT_NEAR(OdriveSystemTestAccess::actuator_to_joint_pos(sys, cfg, actuator), kPi, 1e-6);
}

TEST(AxisMapping, PrismaticLeadScrewConversion) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  AxisConfig cfg;
  cfg.joint_type = "prismatic";
  cfg.lead_screw_pitch = 0.01; // 1 cm per rev

  const double turns = OdriveSystemTestAccess::joint_to_actuator_pos(sys, cfg, 0.05);
  EXPECT_NEAR(turns, 5.0, 1e-6);
  EXPECT_NEAR(OdriveSystemTestAccess::actuator_to_joint_pos(sys, cfg, turns), 0.05, 1e-6);
}

TEST(CommandModeSwitch, RejectsMultipleInterfacesPerJoint) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  ASSERT_EQ(sys.on_init(make_info()), CallbackReturn::SUCCESS);

  const std::vector<std::string> start = {"joint1/position", "joint1/velocity"};
  EXPECT_EQ(sys.prepare_command_mode_switch(start, {}), return_type::ERROR);
}

TEST(CommandModeSwitch, AcceptsSingleInterfacePerJoint) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  ASSERT_EQ(sys.on_init(make_info()), CallbackReturn::SUCCESS);

  const std::vector<std::string> start = {"joint1/position", "joint2/velocity"};
  EXPECT_EQ(sys.prepare_command_mode_switch(start, {}), return_type::OK);
}

TEST(FaultHandling, HeartbeatErrorFlagsAxis) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  ASSERT_EQ(sys.on_init(make_info()), CallbackReturn::SUCCESS);

  Heartbeat_msg_t hb{};
  hb.Axis_State = AXIS_STATE_CLOSED_LOOP_CONTROL;
  hb.Axis_Error = 0;
  can_frame frame{};
  frame.can_id = (1u << 5) | Heartbeat_msg_t::cmd_id;
  frame.can_dlc = Heartbeat_msg_t::msg_length;
  hb.encode_buf(frame.data);
  OdriveSystemTestAccess::handle(sys, frame, rclcpp::Clock().now());
  ASSERT_EQ(sys.debug_axis_states()[0].health, AxisHealth::OK);

  hb.Axis_Error = 1;
  hb.encode_buf(frame.data);
  OdriveSystemTestAccess::handle(sys, frame, rclcpp::Clock().now());
  EXPECT_EQ(sys.debug_axis_states()[0].health, AxisHealth::ERROR);
  EXPECT_TRUE(OdriveSystemTestAccess::runtime(sys)[0].requires_rearm);
}

TEST(FaultHandling, FaultRequiresExplicitRearm) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  auto fake = std::make_shared<FakeCanTransport>();
  sys.inject_transport(fake);
  ASSERT_EQ(sys.on_init(make_info()), CallbackReturn::SUCCESS);

  Heartbeat_msg_t hb{};
  hb.Axis_State = AXIS_STATE_CLOSED_LOOP_CONTROL;
  hb.Axis_Error = 0x5;
  can_frame frame{};
  frame.can_id = (1u << 5) | Heartbeat_msg_t::cmd_id;
  frame.can_dlc = Heartbeat_msg_t::msg_length;
  hb.encode_buf(frame.data);
  OdriveSystemTestAccess::handle(sys, frame, rclcpp::Clock().now());

  ASSERT_EQ(sys.debug_axis_states()[0].health, AxisHealth::ERROR);
  ASSERT_TRUE(OdriveSystemTestAccess::runtime(sys)[0].requires_rearm);

  hb.Axis_Error = 0x0;
  hb.encode_buf(frame.data);
  OdriveSystemTestAccess::handle(sys, frame, rclcpp::Clock().now());
  EXPECT_EQ(sys.debug_axis_states()[0].health, AxisHealth::ERROR);

  EXPECT_TRUE(OdriveSystemTestAccess::clear_and_rearm(sys, 0));
  OdriveSystemTestAccess::handle(sys, frame, rclcpp::Clock().now());
  EXPECT_EQ(sys.debug_axis_states()[0].health, AxisHealth::OK);
  EXPECT_FALSE(OdriveSystemTestAccess::runtime(sys)[0].requires_rearm);
}

TEST(LimitChecks, WarnOnlyModeToleratesSmallDelta) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "WARN_ONLY";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;
  cfg.limit_effort = 2.0;
  auto &meta = OdriveSystemTestAccess::runtime(sys)[0];
  meta.odrive_velocity_limit = 4.6; // within 10%
  meta.odrive_effort_limit = 1.9;   // within 10%

  EXPECT_TRUE(OdriveSystemTestAccess::run_limit_check(sys, 0));
}

TEST(LimitChecks, WarnOnlyFlagsMismatchedLimitAndSetsWarning) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "WARN_ONLY";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;
  auto &meta = OdriveSystemTestAccess::runtime(sys)[0];
  meta.odrive_velocity_limit = 2.0; // too low

  EXPECT_TRUE(OdriveSystemTestAccess::run_limit_check(sys, 0));
  EXPECT_EQ(OdriveSystemTestAccess::runtime(sys)[0].limit_check_result,
            OdriveSystemTestAccess::LimitCheckResult::WARN);
  EXPECT_EQ(OdriveSystemTestAccess::states(sys)[0].health, AxisHealth::WARNING);
}

TEST(LimitChecks, StrictModeFailsMismatchedLimits) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["limits_check.mode"] = "STRICT";
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto &cfg = OdriveSystemTestAccess::configs(sys)[0];
  cfg.limit_velocity = 5.0;
  auto &meta = OdriveSystemTestAccess::runtime(sys)[0];
  meta.odrive_velocity_limit = 3.0; // too low

  EXPECT_FALSE(OdriveSystemTestAccess::run_limit_check(sys, 0));
}

TEST(ParameterValidation, FailsWhenNodeIdMissing) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  EXPECT_EQ(sys.on_init(make_info_missing_node_id()), CallbackReturn::ERROR);
}

TEST(ParameterValidation, RejectsMoreThanTwoAxesPerNode) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  EXPECT_EQ(sys.on_init(make_info_three_axes_same_node()), CallbackReturn::ERROR);
}

TEST(ParameterValidation, AppliesAxisIndexIntoCanId) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  auto info = make_info_with_axis_index();
  ASSERT_EQ(sys.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.debug_axis_configs().size(), 2u);
  EXPECT_EQ(sys.debug_axis_configs()[0].axis_index, 1);

  can_frame frame{};
  frame.can_id = (3u << 5) | Heartbeat_msg_t::cmd_id; // node_id=1, axis_index=1 -> can_id=3
  frame.can_dlc = Heartbeat_msg_t::msg_length;
  Heartbeat_msg_t hb{};
  hb.Axis_State = AXIS_STATE_CLOSED_LOOP_CONTROL;
  hb.Axis_Error = 0;
  hb.encode_buf(frame.data);
  OdriveSystemTestAccess::handle(sys, frame, rclcpp::Clock().now());
  EXPECT_EQ(sys.debug_axis_states()[0].health, AxisHealth::OK);
}

TEST(ParameterValidation, RequiresPositiveBitrate) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  auto info = make_info();
  info.hardware_parameters["can_bitrate"] = "0";
  EXPECT_EQ(sys.on_init(info), CallbackReturn::ERROR);
}

TEST(ParameterValidation, RequiresStateInterfaces) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  EXPECT_EQ(sys.on_init(make_info_missing_state_if()), CallbackReturn::ERROR);
}

TEST(ParameterValidation, RequiresAtLeastOneCommandInterface) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  EXPECT_EQ(sys.on_init(make_info_missing_command_if()), CallbackReturn::ERROR);
}

TEST(SystemInterface, ExportsAllStateAndCommandInterfaces) {
  TransportOverride guard;
  OdriveS1CanSystem sys;
  ASSERT_EQ(sys.on_init(make_info()), CallbackReturn::SUCCESS);
  const auto states = sys.export_state_interfaces();
  const auto cmds = sys.export_command_interfaces();
  EXPECT_EQ(states.size(), 2u * 3u); // position, velocity, effort per joint
  EXPECT_EQ(cmds.size(), 2u * 3u);
}
