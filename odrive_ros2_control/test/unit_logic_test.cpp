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
