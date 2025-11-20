#include "odrive_ros2_control/odrive_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "socket_can.hpp"
#include "epoll_event_loop.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using std::placeholders::_1;
using std::placeholders::_2;

namespace odrive_ros2_control {

namespace {

class SocketCanTransport final : public CanTransport {
public:
  bool init(const std::string &iface, std::function<void(const can_frame &)> cb) override {
    callback_ = std::move(cb);
    return can_.init(iface, &event_loop_, [this](const can_frame &frame) { callback_(frame); });
  }

  void shutdown() override { can_.deinit(); }

  bool send(const can_frame &frame) override { return can_.send_can_frame(frame); }

  void poll() override {
    while (can_.read_nonblocking()) {
      // frames are handled via callback
    }
  }

private:
  EpollEventLoop event_loop_;
  SocketCanIntf can_;
  std::function<void(const can_frame &)> callback_;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadPerTurn = 2.0 * kPi;

} // namespace

CanTransportFactory OdriveS1CanSystem::transport_factory_override_;

CanTransportFactory OdriveS1CanSystem::default_transport_factory() {
  return []() { return std::make_shared<SocketCanTransport>(); };
}

OdriveS1CanSystem::OdriveS1CanSystem() : OdriveS1CanSystem(default_transport_factory()) {}

OdriveS1CanSystem::OdriveS1CanSystem(CanTransportFactory factory) {
  transport_factory_ = factory ? factory : default_transport_factory();
}

void OdriveS1CanSystem::set_transport_factory_for_tests(const CanTransportFactory &factory) {
  transport_factory_override_ = factory;
}

AxisControlMode OdriveS1CanSystem::string_to_mode(const std::string &mode) const {
  if (mode == "position") return AxisControlMode::POSITION;
  if (mode == "velocity") return AxisControlMode::VELOCITY;
  if (mode == "effort") return AxisControlMode::EFFORT;
  if (mode == "homing") return AxisControlMode::HOMING;
  return AxisControlMode::IDLE;
}

std::string OdriveS1CanSystem::mode_to_string(AxisControlMode mode) const {
  switch (mode) {
    case AxisControlMode::POSITION: return "position";
    case AxisControlMode::VELOCITY: return "velocity";
    case AxisControlMode::EFFORT: return "effort";
    case AxisControlMode::HOMING: return "homing";
    default: return "idle";
  }
}

CallbackReturn OdriveS1CanSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "Base on_init failed");
    return CallbackReturn::ERROR;
  }

  transport_factory_ = transport_factory_override_ ? transport_factory_override_ : default_transport_factory();

  node_ = std::make_shared<rclcpp::Node>("odrive_hw_" + info_.name);
  diag_updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  diag_updater_->setHardwareID(info_.name);
  diag_updater_->add("odrive_health", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
    size_t faults = 0;
    size_t stale = 0;
    for (const auto &state : axis_states_) {
      if (state.health == AxisHealth::ERROR) faults++;
      if (state.heartbeat_stale) stale++;
    }
    if (faults > 0) {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "%zu axes faulted", faults);
    } else if (stale > 0) {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "%zu stale heartbeats", stale);
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "healthy");
    }
    stat.add("faulted_axes", static_cast<int>(faults));
    stat.add("stale_axes", static_cast<int>(stale));
  });

  auto get_param = [&](const std::string &key, const std::string &fallback, const std::string &default_val) {
    auto it = info_.hardware_parameters.find(key);
    if (it != info_.hardware_parameters.end()) return it->second;
    auto it_fallback = info_.hardware_parameters.find(fallback);
    if (it_fallback != info_.hardware_parameters.end()) return it_fallback->second;
    return default_val;
  };

  node_status_.can_interface = get_param("can_interface", "can", "can0");
  node_status_.status_publish_rate_hz = std::stod(get_param("status_publish_rate", "", "5.0"));
  node_status_.heartbeat_timeout_sec = std::stod(get_param("heartbeat_timeout", "", "0.5"));
  node_status_.command_tolerance = std::stod(get_param("command_tolerance", "", "1e-4"));
  node_status_.can_utilization_limit = std::stod(get_param("can_utilization_limit", "", "0.85"));
  node_status_.default_mode = get_param("default_mode", "", "position");

  const std::string limit_mode = get_param("limits_check.mode", "", "off");
  if (limit_mode == "WARN_ONLY" || limit_mode == "warn" || limit_mode == "warn_only") {
    node_status_.limit_check_config.mode = LimitCheckConfig::Mode::WARN_ONLY;
  } else if (limit_mode == "STRICT" || limit_mode == "strict") {
    node_status_.limit_check_config.mode = LimitCheckConfig::Mode::STRICT;
  } else {
    node_status_.limit_check_config.mode = LimitCheckConfig::Mode::OFF;
  }
  node_status_.limit_check_config.velocity_tolerance_ratio =
      std::stod(get_param("limits_check.velocity_tolerance_ratio", "", "0.1"));
  node_status_.limit_check_config.effort_tolerance_ratio =
      std::stod(get_param("limits_check.effort_tolerance_ratio", "", "0.1"));
  node_status_.limit_check_config.acceleration_tolerance_ratio =
      std::stod(get_param("limits_check.acceleration_tolerance_ratio", "", "0.2"));

  axis_configs_.clear();
  axis_states_.clear();
  axis_commands_.clear();
  command_modes_.clear();
  runtime_metadata_.clear();

  for (const auto &joint : info_.joints) {
    AxisConfig cfg;
    cfg.joint_name = joint.name;
    cfg.joint_type = joint.type;

    auto get_joint_param = [&](const std::string &key, const std::string &fallback) -> std::optional<double> {
      auto it = joint.parameters.find(key);
      if (it != joint.parameters.end()) return std::stod(it->second);
      if (!fallback.empty()) {
        auto it2 = joint.parameters.find(fallback);
        if (it2 != joint.parameters.end()) return std::stod(it2->second);
      }
      return std::nullopt;
    };

    auto node_id_it = joint.parameters.find("odrive_node_id");
    if (node_id_it == joint.parameters.end()) node_id_it = joint.parameters.find("node_id");
    if (node_id_it == joint.parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "Joint %s missing odrive_node_id", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    cfg.node_id = std::stoi(node_id_it->second);
    cfg.axis_index = static_cast<int>(get_joint_param("odrive_axis_index", "axis").value_or(0));
    if (auto gear = get_joint_param("gear_ratio", "")) cfg.gear_ratio = *gear;
    if (auto pitch = get_joint_param("lead_screw_pitch", "")) cfg.lead_screw_pitch = *pitch;
    if (auto tq = get_joint_param("torque_constant", "")) cfg.torque_constant = *tq;
    if (auto vel_lim = get_joint_param("max_velocity", "velocity_limit")) cfg.limit_velocity = *vel_lim;
    if (auto eff_lim = get_joint_param("max_effort", "effort_limit")) cfg.limit_effort = *eff_lim;
    if (auto acc_lim = get_joint_param("max_acceleration", "acceleration_limit")) cfg.limit_acceleration = *acc_lim;

    cfg.has_transmission = std::any_of(
        info_.transmissions.begin(), info_.transmissions.end(), [&](const auto &tr) {
          return std::any_of(tr.joints.begin(), tr.joints.end(), [&](const auto &j) { return j.name == joint.name; });
        });

    axis_configs_.push_back(cfg);
    axis_states_.emplace_back();
    axis_commands_.emplace_back();
    command_modes_.push_back(string_to_mode(node_status_.default_mode));
    runtime_metadata_.emplace_back();
  }

  if (!validate_parameters()) {
    return CallbackReturn::ERROR;
  }

  hw_status_pub_ = node_->create_publisher<control_msgs::msg::HardwareStatus>(
      "hardware_status", rclcpp::SystemDefaultsQoS());

  clear_errors_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "clear_errors", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        bool ok = true;
        for (size_t i = 0; i < axis_configs_.size(); ++i) {
          // Clear faults before re-entering closed loop.
          ok &= send_clear_errors(i);
          runtime_metadata_[i].sent_closed_loop = false;
        }
        resp->success = ok;
        resp->message = ok ? "errors cleared" : "failed to clear some axes";
      });

  home_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "home", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        bool ok = true;
        for (size_t i = 0; i < axis_configs_.size(); ++i) {
          ok &= send_axis_state(i, AXIS_STATE_HOMING);
          command_modes_[i] = AxisControlMode::HOMING;
        }
        resp->success = ok;
        resp->message = ok ? "homing started" : "failed to start homing";
      });

  last_status_publish_time_ = node_->get_clock()->now();
  configured_ = false;
  return CallbackReturn::SUCCESS;
}

bool OdriveS1CanSystem::validate_parameters() {
  std::unordered_map<int, int> node_usage;
  for (size_t idx = 0; idx < axis_configs_.size(); ++idx) {
    const auto &cfg = axis_configs_[idx];
    node_usage[cfg.node_id] += 1;
    if (cfg.joint_type != "revolute" && cfg.joint_type != "continuous" && cfg.joint_type != "prismatic") {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s type %s unsupported", cfg.joint_name.c_str(), cfg.joint_type.c_str());
      return false;
    }
    if (!cfg.has_transmission && !cfg.gear_ratio && !cfg.lead_screw_pitch) {
      RCLCPP_WARN(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s missing transmission/gear/pitch config, assuming ratio 1.0", cfg.joint_name.c_str());
    }
  }

  for (const auto &kv : node_usage) {
    if (kv.second > 2) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "CAN node_id %d used by %d joints; limit is 2 axes per ODrive", kv.first, kv.second);
      return false;
    }
  }

  return true;
}

CallbackReturn OdriveS1CanSystem::on_configure(const rclcpp_lifecycle::State &) {
  transport_ = transport_factory_();
  if (!transport_) {
    RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "Failed to create CAN transport");
    return CallbackReturn::ERROR;
  }

  auto cb = [this](const can_frame &frame) { handle_frame(frame, node_->get_clock()->now()); };
  if (!transport_->init(node_status_.can_interface, cb)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Failed to initialize CAN interface %s", node_status_.can_interface.c_str());
    return CallbackReturn::ERROR;
  }

  configured_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdriveS1CanSystem::on_cleanup(const rclcpp_lifecycle::State &) {
  if (transport_) {
    transport_->shutdown();
    transport_.reset();
  }
  configured_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdriveS1CanSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (!configured_) {
    RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "Cannot activate before configure");
    return CallbackReturn::ERROR;
  }
  active_ = true;
  for (size_t i = 0; i < command_modes_.size(); ++i) {
    runtime_metadata_[i].sent_closed_loop = false;
    send_control_mode(i, CONTROL_MODE_POSITION_CONTROL, command_modes_[i] != AxisControlMode::IDLE);
    if (!run_limit_check(i) && node_status_.limit_check_config.mode == LimitCheckConfig::Mode::STRICT) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Limit check failed for axis %zu during activate", i);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn OdriveS1CanSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    send_axis_state(i, AXIS_STATE_IDLE);
    runtime_metadata_[i].sent_closed_loop = false;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OdriveS1CanSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    state_interfaces.emplace_back(
        axis_configs_[i].joint_name, hardware_interface::HW_IF_POSITION, &axis_states_[i].pos_joint);
    state_interfaces.emplace_back(
        axis_configs_[i].joint_name, hardware_interface::HW_IF_VELOCITY, &axis_states_[i].vel_joint);
    state_interfaces.emplace_back(
        axis_configs_[i].joint_name, hardware_interface::HW_IF_EFFORT, &axis_states_[i].effort_joint);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OdriveS1CanSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> cmd_interfaces;
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    cmd_interfaces.emplace_back(
        axis_configs_[i].joint_name, hardware_interface::HW_IF_POSITION, &axis_commands_[i].position);
    cmd_interfaces.emplace_back(
        axis_configs_[i].joint_name, hardware_interface::HW_IF_VELOCITY, &axis_commands_[i].velocity);
    cmd_interfaces.emplace_back(
        axis_configs_[i].joint_name, hardware_interface::HW_IF_EFFORT, &axis_commands_[i].effort);
  }
  return cmd_interfaces;
}

return_type OdriveS1CanSystem::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  (void)stop_interfaces;
  std::unordered_map<std::string, int> per_joint_counts;
  for (const auto &iface : start_interfaces) {
    auto delim = iface.find('/');
    if (delim == std::string::npos) continue;
    const auto joint = iface.substr(0, delim);
    per_joint_counts[joint] += 1;
    if (per_joint_counts[joint] > 1) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Invalid mode switch: multiple interfaces started for joint %s", joint.c_str());
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type OdriveS1CanSystem::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    bool mode_changed = false;
    const std::string base = axis_configs_[i].joint_name + "/";
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_POSITION) !=
        stop_interfaces.end()) {
      if (command_modes_[i] == AxisControlMode::POSITION) {
        command_modes_[i] = AxisControlMode::IDLE;
        mode_changed = true;
      }
    }
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_VELOCITY) !=
        stop_interfaces.end()) {
      if (command_modes_[i] == AxisControlMode::VELOCITY) {
        command_modes_[i] = AxisControlMode::IDLE;
        mode_changed = true;
      }
    }
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_EFFORT) !=
        stop_interfaces.end()) {
      if (command_modes_[i] == AxisControlMode::EFFORT) {
        command_modes_[i] = AxisControlMode::IDLE;
        mode_changed = true;
      }
    }

    if (std::find(start_interfaces.begin(), start_interfaces.end(), base + hardware_interface::HW_IF_POSITION) !=
        start_interfaces.end()) {
      command_modes_[i] = AxisControlMode::POSITION;
      mode_changed = true;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), base + hardware_interface::HW_IF_VELOCITY) !=
        start_interfaces.end()) {
      command_modes_[i] = AxisControlMode::VELOCITY;
      mode_changed = true;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), base + hardware_interface::HW_IF_EFFORT) !=
        start_interfaces.end()) {
      command_modes_[i] = AxisControlMode::EFFORT;
      mode_changed = true;
    }

    if (mode_changed) {
      runtime_metadata_[i].sent_closed_loop = false;
      auto mode = command_modes_[i];
      uint8_t ctrl_mode = CONTROL_MODE_POSITION_CONTROL;
      switch (mode) {
        case AxisControlMode::POSITION: ctrl_mode = CONTROL_MODE_POSITION_CONTROL; break;
        case AxisControlMode::VELOCITY: ctrl_mode = CONTROL_MODE_VELOCITY_CONTROL; break;
        case AxisControlMode::EFFORT: ctrl_mode = CONTROL_MODE_TORQUE_CONTROL; break;
        case AxisControlMode::HOMING:
        case AxisControlMode::IDLE:
        default: break;
      }
      send_control_mode(i, ctrl_mode, mode != AxisControlMode::IDLE);
    }
  }

  return return_type::OK;
}

return_type OdriveS1CanSystem::read(const rclcpp::Time &stamp, const rclcpp::Duration &) {
  if (!transport_) return return_type::ERROR;
  transport_->poll();

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    axis_states_[i].pos_joint = actuator_to_joint_pos(axis_configs_[i], axis_states_[i].pos_actuator);
    axis_states_[i].vel_joint = actuator_vel_to_joint(axis_configs_[i], axis_states_[i].vel_actuator);
    axis_states_[i].effort_joint = actuator_effort_to_joint(axis_configs_[i], axis_states_[i].torque_actuator);
    update_health(i, false);
  }

  publish_status_if_due(stamp);
  return return_type::OK;
}

return_type OdriveS1CanSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!transport_) return return_type::ERROR;
  if (!active_) return return_type::OK;

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    if (axis_states_[i].health == AxisHealth::ERROR) continue;

    const double cmd_pos = axis_commands_[i].position;
    const double cmd_vel = axis_commands_[i].velocity;
    const double cmd_eff = axis_commands_[i].effort;

    switch (command_modes_[i]) {
      case AxisControlMode::POSITION: {
        const double turns = joint_to_actuator_pos(axis_configs_[i], cmd_pos);
        const double vel_ff = joint_vel_to_actuator(axis_configs_[i], cmd_vel);
        const double torque_ff = joint_effort_to_actuator(axis_configs_[i], cmd_eff);
        const double diff = std::fabs(turns - runtime_metadata_[i].last_cmd_pos);
        if (diff > node_status_.command_tolerance || !runtime_metadata_[i].sent_closed_loop) {
          send_position_command(i, turns, vel_ff, torque_ff);
          runtime_metadata_[i].last_cmd_pos = turns;
          runtime_metadata_[i].last_cmd_vel = vel_ff;
          runtime_metadata_[i].last_cmd_effort = torque_ff;
        }
      } break;
      case AxisControlMode::VELOCITY: {
        const double turns_per_sec = joint_vel_to_actuator(axis_configs_[i], cmd_vel);
        const double torque_ff = joint_effort_to_actuator(axis_configs_[i], cmd_eff);
        if (std::fabs(turns_per_sec - runtime_metadata_[i].last_cmd_vel) > node_status_.command_tolerance ||
            !runtime_metadata_[i].sent_closed_loop) {
          send_velocity_command(i, turns_per_sec, torque_ff);
          runtime_metadata_[i].last_cmd_vel = turns_per_sec;
          runtime_metadata_[i].last_cmd_effort = torque_ff;
        }
      } break;
      case AxisControlMode::EFFORT: {
        const double tq = joint_effort_to_actuator(axis_configs_[i], cmd_eff);
        if (std::fabs(tq - runtime_metadata_[i].last_cmd_effort) > node_status_.command_tolerance ||
            !runtime_metadata_[i].sent_closed_loop) {
          send_torque_command(i, tq);
          runtime_metadata_[i].last_cmd_effort = tq;
        }
      } break;
      case AxisControlMode::HOMING:
      case AxisControlMode::IDLE:
      default:
        // no-op
        break;
    }
  }
  return return_type::OK;
}

void OdriveS1CanSystem::handle_frame(const can_frame &frame, const rclcpp::Time &stamp) {
  const uint32_t node_id = frame.can_id >> 5;
  auto it = std::find_if(axis_configs_.begin(), axis_configs_.end(), [&](const AxisConfig &cfg) {
    return static_cast<uint32_t>(cfg.node_id) == node_id;
  });
  if (it == axis_configs_.end()) return;
  const size_t idx = std::distance(axis_configs_.begin(), it);

  const uint8_t cmd = frame.can_id & 0x1f;
  switch (cmd) {
    case Heartbeat_msg_t::cmd_id: {
      Heartbeat_msg_t msg;
      if (frame.can_dlc >= Heartbeat_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        process_heartbeat(idx, msg, stamp);
      }
    } break;
    case Get_Encoder_Estimates_msg_t::cmd_id: {
      Get_Encoder_Estimates_msg_t msg;
      if (frame.can_dlc >= Get_Encoder_Estimates_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        process_encoder_estimate(idx, msg);
      }
    } break;
    case Get_Torques_msg_t::cmd_id: {
      Get_Torques_msg_t msg;
      if (frame.can_dlc >= Get_Torques_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        process_torque_feedback(idx, msg);
      }
    } break;
    case Set_Limits_msg_t::cmd_id: {
      Set_Limits_msg_t msg;
      if (frame.can_dlc >= Set_Limits_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        runtime_metadata_[idx].odrive_velocity_limit = msg.Velocity_Limit;
        runtime_metadata_[idx].odrive_effort_limit = msg.Current_Limit;
      }
    } break;
    case Set_Traj_Accel_Limits_msg_t::cmd_id: {
      Set_Traj_Accel_Limits_msg_t msg;
      if (frame.can_dlc >= Set_Traj_Accel_Limits_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        runtime_metadata_[idx].odrive_accel_limit = msg.Traj_Accel_Limit;
      }
    } break;
    default:
      break;
  }
}

void OdriveS1CanSystem::process_heartbeat(size_t idx, const Heartbeat_msg_t &msg, const rclcpp::Time &stamp) {
  axis_states_[idx].axis_error = msg.Axis_Error;
  axis_states_[idx].axis_state = msg.Axis_State;
  axis_states_[idx].last_heartbeat = stamp;
  runtime_metadata_[idx].sent_closed_loop = runtime_metadata_[idx].sent_closed_loop ||
      (msg.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL);

  axis_states_[idx].health = (msg.Axis_Error == 0) ? AxisHealth::OK : AxisHealth::ERROR;
  axis_states_[idx].heartbeat_stale = false;
}

void OdriveS1CanSystem::process_encoder_estimate(size_t idx, const Get_Encoder_Estimates_msg_t &msg) {
  // ODrive reports position/velocity in turns; keep actuators in turns internally.
  axis_states_[idx].pos_actuator = msg.Pos_Estimate;
  axis_states_[idx].vel_actuator = msg.Vel_Estimate;
}

void OdriveS1CanSystem::process_torque_feedback(size_t idx, const Get_Torques_msg_t &msg) {
  axis_states_[idx].torque_actuator = msg.Torque_Estimate;
}

void OdriveS1CanSystem::update_health(size_t idx, bool heartbeat_refresh) {
  (void)heartbeat_refresh;
  const rclcpp::Time now = node_->get_clock()->now();
  const double age = (now - axis_states_[idx].last_heartbeat).seconds();
  if (age > node_status_.heartbeat_timeout_sec) {
    axis_states_[idx].heartbeat_stale = true;
    if (axis_states_[idx].health == AxisHealth::OK) {
      axis_states_[idx].health = AxisHealth::WARNING;
    }
  }
}

double OdriveS1CanSystem::actuator_to_joint_pos(const AxisConfig &cfg, double turns) const {
  if (cfg.has_transmission || cfg.gear_ratio) {
    const double ratio = cfg.gear_ratio.value_or(1.0);
    return turns * kRadPerTurn * ratio;
  }
  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    return turns * (*cfg.lead_screw_pitch);
  }
  return turns * kRadPerTurn;
}

double OdriveS1CanSystem::joint_to_actuator_pos(const AxisConfig &cfg, double joint_pos) const {
  if (cfg.has_transmission || cfg.gear_ratio) {
    const double ratio = cfg.gear_ratio.value_or(1.0);
    if (ratio == 0.0) throw std::runtime_error("gear ratio is zero");
    return joint_pos / (kRadPerTurn * ratio);
  }
  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    const double pitch = *cfg.lead_screw_pitch;
    if (pitch == 0.0) throw std::runtime_error("lead screw pitch is zero");
    return joint_pos / pitch;
  }
  return joint_pos / kRadPerTurn;
}

double OdriveS1CanSystem::actuator_vel_to_joint(const AxisConfig &cfg, double turns_per_sec) const {
  if (cfg.has_transmission || cfg.gear_ratio) {
    const double ratio = cfg.gear_ratio.value_or(1.0);
    return turns_per_sec * kRadPerTurn * ratio;
  }
  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    return turns_per_sec * (*cfg.lead_screw_pitch);
  }
  return turns_per_sec * kRadPerTurn;
}

double OdriveS1CanSystem::joint_vel_to_actuator(const AxisConfig &cfg, double joint_vel) const {
  if (cfg.has_transmission || cfg.gear_ratio) {
    const double ratio = cfg.gear_ratio.value_or(1.0);
    if (ratio == 0.0) throw std::runtime_error("gear ratio is zero");
    return joint_vel / (kRadPerTurn * ratio);
  }
  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    const double pitch = *cfg.lead_screw_pitch;
    if (pitch == 0.0) throw std::runtime_error("lead screw pitch is zero");
    return joint_vel / pitch;
  }
  return joint_vel / kRadPerTurn;
}

double OdriveS1CanSystem::actuator_effort_to_joint(const AxisConfig &cfg, double torque) const {
  if (cfg.torque_constant) {
    return torque * (*cfg.torque_constant);
  }
  return torque;
}

double OdriveS1CanSystem::joint_effort_to_actuator(const AxisConfig &cfg, double effort) const {
  if (cfg.torque_constant) {
    const double k = *cfg.torque_constant;
    if (k == 0.0) throw std::runtime_error("torque constant is zero");
    return effort / k;
  }
  return effort;
}

bool OdriveS1CanSystem::send_axis_state(size_t idx, uint32_t requested_state) {
  Set_Axis_State_msg_t msg{};
  msg.Axis_Requested_State = requested_state;
  can_frame frame{};
  frame.can_id = (axis_configs_[idx].node_id << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return transport_->send(frame);
}

bool OdriveS1CanSystem::send_control_mode(size_t idx, uint8_t control_mode, bool require_closed_loop) {
  Set_Controller_Mode_msg_t ctrl{};
  ctrl.Control_Mode = control_mode;
  ctrl.Input_Mode = INPUT_MODE_PASSTHROUGH;
  can_frame frame{};
  frame.can_id = (axis_configs_[idx].node_id << 5) | ctrl.cmd_id;
  frame.can_dlc = ctrl.msg_length;
  ctrl.encode_buf(frame.data);

  bool ok = transport_->send(frame);
  if (require_closed_loop) {
    ok &= send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  runtime_metadata_[idx].sent_closed_loop = require_closed_loop;
  return ok;
}

bool OdriveS1CanSystem::send_position_command(size_t idx, double turns, double vel_ff, double torque_ff) {
  if (!runtime_metadata_[idx].sent_closed_loop) {
    send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
    runtime_metadata_[idx].sent_closed_loop = true;
  }

  Set_Input_Pos_msg_t msg{};
  msg.Input_Pos = turns;
  msg.Vel_FF = vel_ff;
  msg.Torque_FF = torque_ff;
  can_frame frame{};
  frame.can_id = (axis_configs_[idx].node_id << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return transport_->send(frame);
}

bool OdriveS1CanSystem::send_velocity_command(size_t idx, double turns_per_sec, double torque_ff) {
  if (!runtime_metadata_[idx].sent_closed_loop) {
    send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
    runtime_metadata_[idx].sent_closed_loop = true;
  }
  Set_Input_Vel_msg_t msg{};
  msg.Input_Vel = turns_per_sec;
  msg.Input_Torque_FF = torque_ff;
  can_frame frame{};
  frame.can_id = (axis_configs_[idx].node_id << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return transport_->send(frame);
}

bool OdriveS1CanSystem::send_torque_command(size_t idx, double torque) {
  if (!runtime_metadata_[idx].sent_closed_loop) {
    send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
    runtime_metadata_[idx].sent_closed_loop = true;
  }
  Set_Input_Torque_msg_t msg{};
  msg.Input_Torque = torque;
  can_frame frame{};
  frame.can_id = (axis_configs_[idx].node_id << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return transport_->send(frame);
}

bool OdriveS1CanSystem::send_clear_errors(size_t idx) {
  Clear_Errors_msg_t msg{};
  msg.Identify = 0;
  can_frame frame{};
  frame.can_id = (axis_configs_[idx].node_id << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  axis_states_[idx].health = AxisHealth::OK;
  return transport_->send(frame);
}

bool OdriveS1CanSystem::run_limit_check(size_t idx) {
  if (node_status_.limit_check_config.mode == LimitCheckConfig::Mode::OFF) return true;
  const auto &cfg = axis_configs_[idx];
  auto &meta = runtime_metadata_[idx];

  bool ok = true;
  // Compare URDF-implied limits against values reported over CAN.
  if (cfg.limit_velocity && meta.odrive_velocity_limit) {
    const double expected = joint_vel_to_actuator(cfg, *cfg.limit_velocity);
    if (*meta.odrive_velocity_limit + std::fabs(expected) * node_status_.limit_check_config.velocity_tolerance_ratio <
        expected) {
      ok = false;
    }
  }
  if (cfg.limit_effort && meta.odrive_effort_limit) {
    const double expected = joint_effort_to_actuator(cfg, *cfg.limit_effort);
    if (*meta.odrive_effort_limit + std::fabs(expected) * node_status_.limit_check_config.effort_tolerance_ratio <
        expected) {
      ok = false;
    }
  }
  if (cfg.limit_acceleration && meta.odrive_accel_limit) {
    const double exp_acc = joint_vel_to_actuator(cfg, *cfg.limit_acceleration);
    if (*meta.odrive_accel_limit + std::fabs(exp_acc) * node_status_.limit_check_config.acceleration_tolerance_ratio <
        exp_acc) {
      ok = false;
    }
  }

  if (!ok && node_status_.limit_check_config.mode == LimitCheckConfig::Mode::STRICT) {
    RCLCPP_ERROR(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Limit mismatch for joint %s", cfg.joint_name.c_str());
  } else if (!ok) {
    RCLCPP_WARN(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Limit mismatch for joint %s", cfg.joint_name.c_str());
  }
  return ok || node_status_.limit_check_config.mode == LimitCheckConfig::Mode::WARN_ONLY;
}

void OdriveS1CanSystem::publish_status_if_due(const rclcpp::Time &stamp) {
  const double elapsed = (stamp - last_status_publish_time_).seconds();
  if (elapsed < (1.0 / node_status_.status_publish_rate_hz)) return;

  control_msgs::msg::HardwareStatus status_msg;
  status_msg.name = info_.name;
  status_msg.device_status.resize(axis_configs_.size());
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    auto &device = status_msg.device_status[i];
    device.name = axis_configs_[i].joint_name;
    device.status = static_cast<uint8_t>(axis_states_[i].health == AxisHealth::ERROR
                                             ? control_msgs::msg::HardwareStatus::STATUS_ERROR
                                             : (axis_states_[i].health == AxisHealth::WARNING
                                                    ? control_msgs::msg::HardwareStatus::STATUS_WARNING
                                                    : control_msgs::msg::HardwareStatus::STATUS_RUNNING));
    device.error_message = axis_states_[i].heartbeat_stale ? "heartbeat stale" : "";
    device.type = mode_to_string(command_modes_[i]);
  }
  hw_status_pub_->publish(status_msg);
  if (diag_updater_) diag_updater_->force_update();
  last_status_publish_time_ = stamp;
}

} // namespace odrive_ros2_control

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
