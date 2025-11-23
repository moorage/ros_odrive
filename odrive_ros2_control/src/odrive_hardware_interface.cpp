#include "odrive_ros2_control/odrive_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <sstream>

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

std::optional<std::string> read_file_trimmed(const std::filesystem::path &p) {
  std::ifstream f(p);
  if (!f.is_open()) return std::nullopt;
  std::string s;
  std::getline(f, s);
  s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); }), s.end());
  return s;
}

std::optional<uint32_t> read_can_bitrate(const std::string &iface) {
  std::filesystem::path p = "/sys/class/net";
  p /= iface;
  p /= "bittiming";
  p /= "bitrate";
  auto s = read_file_trimmed(p);
  if (!s) return std::nullopt;
  try {
    return static_cast<uint32_t>(std::stoul(*s));
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<std::string> read_operstate(const std::string &iface) {
  std::filesystem::path p = "/sys/class/net";
  p /= iface;
  p /= "operstate";
  return read_file_trimmed(p);
}

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

void OdriveS1CanSystem::load_flat_endpoints() {
  if (node_status_.flat_endpoints_path.empty()) return;
  std::ifstream f(node_status_.flat_endpoints_path);
  if (!f.is_open()) {
    RCLCPP_WARN(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Failed to open flat_endpoints json at %s", node_status_.flat_endpoints_path.c_str());
    return;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  flat_endpoints_json_ = ss.str();
  endpoint_vel_.resize(axis_configs_.size());
  endpoint_effort_.resize(axis_configs_.size());
  endpoint_accel_.resize(axis_configs_.size());
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    endpoint_vel_[i] = vel_limit_endpoint_for_axis(axis_configs_[i].axis_index);
    endpoint_effort_[i] = current_limit_endpoint_for_axis(axis_configs_[i].axis_index);
    endpoint_accel_[i] = accel_limit_endpoint_for_axis(axis_configs_[i].axis_index);
  }
}

std::optional<OdriveS1CanSystem::EndpointInfo> OdriveS1CanSystem::endpoint_for_axis(
    size_t axis_index, const std::string &suffix) const {
  if (flat_endpoints_json_.empty()) return std::nullopt;
  const std::string key = "axis" + std::to_string(axis_index) + "." + suffix;
  const std::string needle = "\"" + key + "\"";
  auto pos = flat_endpoints_json_.find(needle);
  if (pos == std::string::npos) return std::nullopt;

  auto find_field = [&](const std::string &field) -> std::optional<std::string> {
    auto field_pos = flat_endpoints_json_.find("\"" + field + "\"", pos);
    if (field_pos == std::string::npos) return std::nullopt;
    auto colon = flat_endpoints_json_.find(':', field_pos);
    if (colon == std::string::npos) return std::nullopt;
    auto start = flat_endpoints_json_.find_first_not_of(" \t\n\r", colon + 1);
    if (start == std::string::npos) return std::nullopt;
    auto end = flat_endpoints_json_.find_first_of(",}", start);
    if (end == std::string::npos) return std::nullopt;
    return flat_endpoints_json_.substr(start, end - start);
  };

  auto id_str = find_field("id");
  auto type_str = find_field("type");
  if (!id_str || !type_str) return std::nullopt;
  try {
    EndpointInfo ep;
    ep.id = static_cast<uint16_t>(std::stoul(*id_str));
    std::string t = *type_str;
    // Trim quotes if present.
    if (!t.empty() && t.front() == '"') t.erase(t.begin());
    if (!t.empty() && t.back() == '"') t.pop_back();
    ep.type = t;
    return ep;
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<OdriveS1CanSystem::EndpointInfo> OdriveS1CanSystem::vel_limit_endpoint_for_axis(size_t axis_index) const {
  return endpoint_for_axis(axis_index, "controller.config.vel_limit");
}

std::optional<OdriveS1CanSystem::EndpointInfo> OdriveS1CanSystem::current_limit_endpoint_for_axis(size_t axis_index) const {
  return endpoint_for_axis(axis_index, "motor.effective_current_lim");
}

std::optional<OdriveS1CanSystem::EndpointInfo> OdriveS1CanSystem::accel_limit_endpoint_for_axis(size_t axis_index) const {
  return endpoint_for_axis(axis_index, "trap_traj.config.accel_limit");
}

CallbackReturn OdriveS1CanSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "Base on_init failed");
    return CallbackReturn::ERROR;
  }

  // Initialize the CAN transport using the factory.
  // This allows for dependency injection, which is useful for testing.
  transport_factory_ = transport_factory_override_ ? transport_factory_override_ : default_transport_factory();

  // Create a ROS node for this hardware interface.
  // This node is used for logging, parameters, and publishing diagnostics.
  node_ = std::make_shared<rclcpp::Node>("odrive_hw_" + info_.name);
  diag_updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
  diag_updater_->setHardwareID(info_.name);
  
  // Configure the diagnostic updater to publish ODrive health status.
  diag_updater_->add("odrive_health", [this](diagnostic_updater::DiagnosticStatusWrapper &stat) {
    size_t faults = 0;
    size_t stale = 0;
    size_t limit_warn = 0;
    const double last_send_age = util_metrics_.last_send_time.nanoseconds() == 0
                                     ? std::numeric_limits<double>::infinity()
                                     : (node_->get_clock()->now() - util_metrics_.last_send_time).seconds();
    const bool latency_warn =
        util_metrics_.avg_write_duration_sec > node_status_.write_latency_warn_sec && util_metrics_.avg_write_duration_sec > 0.0;
    bool budget_warn = util_metrics_.budget_exceeded_last_cycle;
    for (const auto &state : axis_states_) {
      if (state.health == AxisHealth::ERROR) faults++;
      if (state.heartbeat_stale) stale++;
    }
    for (const auto &meta : runtime_metadata_) {
      if (meta.limit_check_result == AxisRuntimeMetadata::LimitCheckResult::WARN) limit_warn++;
    }
    if (faults > 0) {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "%zu axes faulted", faults);
    } else if (budget_warn) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "CAN frame budget exceeded");
    } else if (latency_warn) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "write latency high");
    } else if (limit_warn > 0) {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "%zu axes limit mismatch", limit_warn);
    } else if (stale > 0) {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::WARN, "%zu stale heartbeats", stale);
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "healthy");
    }
    stat.add("faulted_axes", static_cast<int>(faults));
    stat.add("stale_axes", static_cast<int>(stale));
    stat.add("limit_warn_axes", static_cast<int>(limit_warn));
    stat.add("frames_last_cycle", static_cast<int>(util_metrics_.frames_last_cycle));
    stat.add("frames_per_sec_ewma", util_metrics_.frames_per_sec_ewma);
    stat.add("avg_write_duration_s", util_metrics_.avg_write_duration_sec);
    stat.add("over_budget_events", static_cast<int>(util_metrics_.over_budget_events));
    stat.add("last_send_age_s", last_send_age);
    stat.add("max_frames_per_cycle", node_status_.max_frames_per_cycle);
    stat.add("max_frames_per_sec", node_status_.max_frames_per_sec);
    for (size_t i = 0; i < axis_states_.size(); ++i) {
      const auto &state = axis_states_[i];
      const double age = (node_->get_clock()->now() - state.last_heartbeat).seconds();
      stat.add("axis" + std::to_string(i) + "_error", static_cast<int>(state.axis_error));
      stat.add("axis" + std::to_string(i) + "_state", static_cast<int>(state.axis_state));
      stat.add("axis" + std::to_string(i) + "_heartbeat_age_s", age);
      stat.add("axis" + std::to_string(i) + "_requires_rearm", runtime_metadata_[i].requires_rearm ? 1 : 0);
    }
  });

  auto get_param = [&](const std::string &key, const std::string &fallback, const std::string &default_val) {
    auto it = info_.hardware_parameters.find(key);
    if (it != info_.hardware_parameters.end()) return it->second;
    auto it_fallback = info_.hardware_parameters.find(fallback);
    if (it_fallback != info_.hardware_parameters.end()) return it_fallback->second;
    return default_val;
  };

  auto to_bool = [](const std::string &val, bool default_val) {
    if (val.empty()) return default_val;
    const std::string lower = [&]() {
      std::string s = val;
      std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      return s;
    }();
    return (lower == "true" || lower == "1" || lower == "yes");
  };

  node_status_.can_interface = get_param("can_interface", "can", "can0");
  node_status_.status_publish_rate_hz = std::stod(get_param("status_publish_rate", "", "5.0"));
  node_status_.heartbeat_timeout_sec = std::stod(get_param("heartbeat_timeout", "", "0.5"));
  node_status_.command_tolerance = std::stod(get_param("command_tolerance", "", "1e-4"));
  node_status_.can_utilization_limit = std::stod(get_param("can_utilization_limit", "", "0.85"));
  node_status_.default_mode = get_param("default_mode", "", "position");
  node_status_.can_bitrate = static_cast<uint32_t>(std::stoul(get_param("can_bitrate", "", "1000000")));
  node_status_.fault_idle_on_error = to_bool(get_param("fault_idle_on_error", "", "true"), true);
  node_status_.require_rearm_after_fault = to_bool(get_param("require_rearm_after_fault", "", "true"), true);
  node_status_.max_frames_per_cycle = std::stoi(get_param("max_frames_per_cycle", "", "0"));
  node_status_.max_frames_per_sec = std::stod(get_param("max_frames_per_sec", "", "0.0"));
  node_status_.write_latency_warn_sec = std::stod(get_param("write_latency_warn_sec", "", "0.02"));
  node_status_.limits_check_use_sdo = to_bool(get_param("limits_check.use_sdo", "", "false"), false);
  node_status_.flat_endpoints_path = get_param("limits_check.flat_endpoints_path", "", "");
  node_status_.sdo_timeout_sec = std::stod(get_param("limits_check.sdo_timeout_sec", "", "0.5"));

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
  axis_can_ids_.clear();
  can_id_lookup_.clear();
  node_to_primary_axis_.clear();
  endpoint_vel_.clear();
  endpoint_effort_.clear();
  endpoint_accel_.clear();

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
    axis_can_ids_.push_back(0);
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
          ok &= send_clear_errors(i);
          runtime_metadata_[i].idle_sent_on_fault = false;
          runtime_metadata_[i].sent_closed_loop = false;
        }
        resp->success = ok;
        resp->message = ok ? "errors cleared; rearm with clear_errors_and_rearm" : "failed to clear some axes";
      });

  clear_errors_and_rearm_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "clear_errors_and_rearm", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        bool ok = true;
        for (size_t i = 0; i < axis_configs_.size(); ++i) {
          ok &= clear_errors_and_rearm(i);
        }
        resp->success = ok;
        resp->message = ok ? "axes cleared and re-armed" : "failed to re-arm some axes";
      });

  home_srv_ = node_->create_service<std_srvs::srv::Trigger>(
      "home", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        const bool ok = start_homing();
        resp->success = ok;
        resp->message = ok ? "homing started" : "failed to start homing";
      });

  for (size_t i = 0; i < axis_states_.size(); ++i) {
    diag_updater_->add("odrive_axis_" + std::to_string(i), [this, i](diagnostic_updater::DiagnosticStatusWrapper &stat) {
      const auto &state = axis_states_[i];
      const auto &meta = runtime_metadata_[i];
      const double age = (node_->get_clock()->now() - state.last_heartbeat).seconds();
      if (meta.requires_rearm || state.health == AxisHealth::ERROR) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "faulted/rearm required");
      } else if (state.health == AxisHealth::WARNING) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "warning");
      } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ok");
      }
      stat.add("axis_state", static_cast<int>(state.axis_state));
      stat.add("heartbeat_age_s", age);
      stat.addf("axis_error_hex", "0x%08x", state.axis_error);
      stat.addf("disarm_reason_hex", "0x%08x", meta.disarm_reason);
      stat.add("requires_rearm", meta.requires_rearm ? 1 : 0);
      stat.add("heartbeat_stale", state.heartbeat_stale ? 1 : 0);
      stat.add("last_homing_result", meta.last_homing_result);
      stat.add("limit_check", static_cast<int>(meta.limit_check_result));
      if (!meta.limit_check_detail.empty()) stat.add("limit_detail", meta.limit_check_detail);
    });
  }

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
    auto require_iface = [&](const std::vector<hardware_interface::InterfaceInfo> &ifaces,
                             const std::string &name) {
      return std::any_of(ifaces.begin(), ifaces.end(), [&](const auto &iface) { return iface.name == name; });
    };
    if (!require_iface(info_.joints[idx].state_interfaces, hardware_interface::HW_IF_POSITION) ||
        !require_iface(info_.joints[idx].state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
        !require_iface(info_.joints[idx].state_interfaces, hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s missing required state interfaces (position/velocity/effort)", cfg.joint_name.c_str());
      return false;
    }
    if (!require_iface(info_.joints[idx].command_interfaces, hardware_interface::HW_IF_POSITION) &&
        !require_iface(info_.joints[idx].command_interfaces, hardware_interface::HW_IF_VELOCITY) &&
        !require_iface(info_.joints[idx].command_interfaces, hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s must expose at least one command interface (position/velocity/effort)", cfg.joint_name.c_str());
      return false;
    }
    if (cfg.axis_index < 0 || cfg.axis_index > 1) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s axis_index %d invalid (must be 0 or 1)", cfg.joint_name.c_str(), cfg.axis_index);
      return false;
    }

    const uint32_t can_id = static_cast<uint32_t>(cfg.node_id * 2 + cfg.axis_index);
    axis_can_ids_[idx] = can_id;
    if (can_id_lookup_.count(can_id)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Duplicate CAN addressing for can_id %u among joints %s and %s",
          can_id, axis_configs_[can_id_lookup_[can_id]].joint_name.c_str(), cfg.joint_name.c_str());
      return false;
    }
    can_id_lookup_[can_id] = idx;
    if (cfg.axis_index == 0) {
      node_to_primary_axis_[cfg.node_id] = idx;
    }
  }

  if (node_status_.can_bitrate == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "can_bitrate must be > 0");
    return false;
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
  // Validate interface presence and state before touching CAN.
  const std::filesystem::path iface_path = std::filesystem::path("/sys/class/net") / node_status_.can_interface;
  if (!std::filesystem::exists(iface_path)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "CAN interface %s not found (expected %s)", node_status_.can_interface.c_str(), iface_path.c_str());
    return CallbackReturn::ERROR;
  }
  if (auto oper = read_operstate(node_status_.can_interface)) {
    if (*oper != "up") {
      RCLCPP_WARN(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "CAN interface %s is not up (operstate=%s)", node_status_.can_interface.c_str(), oper->c_str());
    }
  } else {
    RCLCPP_WARN(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Could not read operstate for interface %s", node_status_.can_interface.c_str());
  }
  if (node_status_.can_bitrate > 0) {
    if (auto actual = read_can_bitrate(node_status_.can_interface)) {
      if (*actual != node_status_.can_bitrate) {
        RCLCPP_WARN(
            rclcpp::get_logger("OdriveS1CanSystem"),
            "CAN bitrate mismatch for %s: configured %u vs actual %u",
            node_status_.can_interface.c_str(), node_status_.can_bitrate, *actual);
      }
    } else {
      RCLCPP_WARN(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Could not determine bitrate for %s; ensure it matches configured %u",
          node_status_.can_interface.c_str(), node_status_.can_bitrate);
    }
  }

  if (node_status_.limits_check_use_sdo && !node_status_.flat_endpoints_path.empty()) {
    load_flat_endpoints();
  }

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
  util_metrics_.window_start = node_->get_clock()->now();
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
  util_metrics_.frames_last_cycle = util_metrics_.frames_this_cycle;
  const double write_duration = (node_->get_clock()->now() - util_metrics_.last_write_start).seconds();
  util_metrics_.avg_write_duration_sec = 0.9 * util_metrics_.avg_write_duration_sec + 0.1 * write_duration;
  if (util_metrics_.budget_exceeded_last_cycle) {
    util_metrics_.over_budget_events++;
    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("OdriveS1CanSystem"), *node_->get_clock(), 5000,
        "CAN frame budget exceeded in last cycle (frames=%zu)", util_metrics_.frames_last_cycle);
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
    // Convert actuator feedback (turns, turns/s, torque) to joint units (rad, rad/s, Nm).
    axis_states_[i].pos_joint = actuator_to_joint_pos(axis_configs_[i], axis_states_[i].pos_actuator);
    axis_states_[i].vel_joint = actuator_vel_to_joint(axis_configs_[i], axis_states_[i].vel_actuator);
    axis_states_[i].effort_joint = actuator_effort_to_joint(axis_configs_[i], axis_states_[i].torque_actuator);
    
    // Update axis health status based on errors and heartbeat freshness.
    update_health(i, false);
  }

  publish_status_if_due(stamp);
  return return_type::OK;
}

return_type OdriveS1CanSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!transport_) return return_type::ERROR;
  if (!active_) return return_type::OK;

  util_metrics_.frames_this_cycle = 0;
  util_metrics_.budget_exceeded_last_cycle = false;
  util_metrics_.last_write_start = node_->get_clock()->now();

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    if (axis_states_[i].health == AxisHealth::ERROR) continue;

    const double cmd_pos = axis_commands_[i].position;
    const double cmd_vel = axis_commands_[i].velocity;
    const double cmd_eff = axis_commands_[i].effort;

    // Send commands based on the active control mode.
    // We only send commands if the value has changed significantly or if we haven't sent a closed-loop command yet.
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
        // No streaming commands while homing; allow heartbeat to advance state/health.
        continue;
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
  const uint8_t cmd = frame.can_id & 0x1f;
  
  // Handle SDO response frames (command ID 0x05).
  if (cmd == 0x05) { // TxSdo
    process_sdo_response(frame, stamp);
    return;
  }

  auto it = can_id_lookup_.find(node_id);
  if (it == can_id_lookup_.end()) return;
  const size_t idx = it->second;
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
    case Get_Error_msg_t::cmd_id: {
      Get_Error_msg_t msg;
      if (frame.can_dlc >= Get_Error_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        runtime_metadata_[idx].disarm_reason = msg.Disarm_Reason;
        axis_states_[idx].axis_error = msg.Active_Errors;
        if (msg.Active_Errors != 0) {
          latch_fault(idx, msg.Active_Errors);
        } else if (!runtime_metadata_[idx].requires_rearm) {
          axis_states_[idx].health = AxisHealth::OK;
        }
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
  const uint8_t prev_state = axis_states_[idx].axis_state;
  axis_states_[idx].axis_error = msg.Axis_Error;
  axis_states_[idx].axis_state = msg.Axis_State;
  axis_states_[idx].last_heartbeat = stamp;
  runtime_metadata_[idx].sent_closed_loop = runtime_metadata_[idx].sent_closed_loop ||
      (msg.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL);
  runtime_metadata_[idx].last_axis_state = msg.Axis_State;
  if (prev_state == AXIS_STATE_HOMING) {
    if (msg.Axis_Error != 0) {
      runtime_metadata_[idx].last_homing_result = "failed";
    } else if (msg.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL) {
      runtime_metadata_[idx].last_homing_result = "success";
    } else {
      runtime_metadata_[idx].last_homing_result = "unknown";
    }
  }

  if (msg.Axis_Error != 0) {
    latch_fault(idx, msg.Axis_Error);
  } else if (!runtime_metadata_[idx].requires_rearm) {
    axis_states_[idx].health = AxisHealth::OK;
  } else {
    axis_states_[idx].health = AxisHealth::ERROR;
  }
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

void OdriveS1CanSystem::process_sdo_response(const can_frame &frame, const rclcpp::Time &stamp) {
  if (frame.can_dlc < 5) return;
  const uint16_t endpoint = static_cast<uint16_t>(frame.data[1] | (static_cast<uint16_t>(frame.data[2]) << 8));
  uint32_t raw = 0;
  if (frame.can_dlc >= 8) {
    std::memcpy(&raw, &frame.data[4], sizeof(uint32_t));
  } else if (frame.can_dlc >= 5) {
    raw = frame.data[4];
  }
  std::lock_guard<std::mutex> lock(sdo_mutex_);
  sdo_responses_[endpoint] = RawSdoValue{raw, stamp};
}

void OdriveS1CanSystem::update_health(size_t idx, bool heartbeat_refresh) {
  (void)heartbeat_refresh;
  const rclcpp::Time now = node_->get_clock()->now();
  const double age = (now - axis_states_[idx].last_heartbeat).seconds();
  if (runtime_metadata_[idx].requires_rearm) {
    axis_states_[idx].health = AxisHealth::ERROR;
  }
  if (age > node_status_.heartbeat_timeout_sec) {
    axis_states_[idx].heartbeat_stale = true;
    if (axis_states_[idx].health == AxisHealth::OK) {
      axis_states_[idx].health = AxisHealth::WARNING;
    }
  } else {
    axis_states_[idx].heartbeat_stale = false;
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
  frame.can_id = (axis_can_id(idx) << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return send_frame(frame, true);
}

bool OdriveS1CanSystem::send_control_mode(size_t idx, uint8_t control_mode, bool require_closed_loop) {
  Set_Controller_Mode_msg_t ctrl{};
  ctrl.Control_Mode = control_mode;
  ctrl.Input_Mode = INPUT_MODE_PASSTHROUGH;
  can_frame frame{};
  frame.can_id = (axis_can_id(idx) << 5) | ctrl.cmd_id;
  frame.can_dlc = ctrl.msg_length;
  ctrl.encode_buf(frame.data);

  bool ok = send_frame(frame, true);
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
  frame.can_id = (axis_can_id(idx) << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return send_frame(frame, true);
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
  frame.can_id = (axis_can_id(idx) << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return send_frame(frame, true);
}

bool OdriveS1CanSystem::send_torque_command(size_t idx, double torque) {
  if (!runtime_metadata_[idx].sent_closed_loop) {
    send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
    runtime_metadata_[idx].sent_closed_loop = true;
  }
  Set_Input_Torque_msg_t msg{};
  msg.Input_Torque = torque;
  can_frame frame{};
  frame.can_id = (axis_can_id(idx) << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return send_frame(frame, true);
}

bool OdriveS1CanSystem::can_send_frame(const rclcpp::Time &now) {
  if (util_metrics_.window_start.nanoseconds() == 0) {
    util_metrics_.window_start = now;
  }
  const double window_age = (now - util_metrics_.window_start).seconds();
  if (window_age >= 1.0) {
    const double rate = window_age > 0.0 ? (static_cast<double>(util_metrics_.frames_in_window) / window_age) : 0.0;
    util_metrics_.frames_per_sec_ewma = 0.9 * util_metrics_.frames_per_sec_ewma + 0.1 * rate;
    util_metrics_.frames_in_window = 0;
    util_metrics_.window_start = now;
  }

  if (node_status_.max_frames_per_cycle > 0 &&
      util_metrics_.frames_this_cycle >= static_cast<size_t>(node_status_.max_frames_per_cycle)) {
    return false;
  }
  if (node_status_.max_frames_per_sec > 0.0 &&
      util_metrics_.frames_in_window >= static_cast<size_t>(std::ceil(node_status_.max_frames_per_sec))) {
    return false;
  }
  return true;
}

bool OdriveS1CanSystem::send_frame(const can_frame &frame, bool count_budget) {
  if (!transport_) return false;
  const rclcpp::Time now = node_->get_clock()->now();
  if (count_budget && !can_send_frame(now)) {
    util_metrics_.budget_exceeded_last_cycle = true;
    return false;
  }

  const bool ok = transport_->send(frame);
  if (ok) {
    if (count_budget) {
      util_metrics_.frames_this_cycle++;
      util_metrics_.frames_in_window++;
    }
    util_metrics_.last_send_time = now;
  }
  return ok;
}

bool OdriveS1CanSystem::send_clear_errors(size_t idx) {
  Clear_Errors_msg_t msg{};
  msg.Identify = 0;
  can_frame frame{};
  frame.can_id = (axis_can_id(idx) << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return send_frame(frame, true);
}

void OdriveS1CanSystem::latch_fault(size_t idx, uint32_t axis_error) {
  auto &state = axis_states_[idx];
  auto &meta = runtime_metadata_[idx];
  state.axis_error = axis_error;
  state.health = AxisHealth::ERROR;
  meta.fault_latched = true;
  meta.requires_rearm = node_status_.require_rearm_after_fault;
  meta.sent_closed_loop = false;
  if (node_status_.fault_idle_on_error && !meta.idle_sent_on_fault && transport_) {
    send_axis_state(idx, AXIS_STATE_IDLE);
    meta.idle_sent_on_fault = true;
  }
}

bool OdriveS1CanSystem::clear_errors_and_rearm(size_t idx) {
  if (!transport_) return false;
  auto &state = axis_states_[idx];
  auto &meta = runtime_metadata_[idx];
  state.heartbeat_stale = false;
  state.axis_error = 0;
  state.motor_error = 0;
  state.controller_error = 0;
  state.encoder_error = 0;
  state.disarm_reason = 0;
  meta.fault_latched = false;
  meta.requires_rearm = false;
  meta.idle_sent_on_fault = false;
  meta.sent_closed_loop = false;

  bool ok = send_clear_errors(idx);
  ok &= send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
  state.health = AxisHealth::OK;
  meta.sent_closed_loop = true;
  return ok;
}

double OdriveS1CanSystem::decode_sdo_value(const EndpointInfo &ep, uint32_t raw) const {
  if (ep.type == "float") {
    float f;
    std::memcpy(&f, &raw, sizeof(float));
    return static_cast<double>(f);
  }
  return static_cast<double>(raw);
}

std::optional<double> OdriveS1CanSystem::read_endpoint_via_sdo(int node_id, const EndpointInfo &ep) {
  if (!transport_) return std::nullopt;
  can_frame frame{};
  frame.can_id = (static_cast<uint32_t>(node_id) << 5) | 0x04; // RxSdo
  frame.can_dlc = 4;
  frame.data[0] = 0x00; // read opcode
  frame.data[1] = static_cast<uint8_t>(ep.id & 0xffu);
  frame.data[2] = static_cast<uint8_t>((ep.id >> 8) & 0xffu);
  frame.data[3] = 0;

  if (!send_frame(frame, false)) {
    return std::nullopt;
  }

  const rclcpp::Time start = node_->get_clock()->now();
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(node_status_.sdo_timeout_sec);
  while ((node_->get_clock()->now() - start) < timeout) {
    transport_->poll();
    std::optional<uint32_t> raw;
    {
      std::lock_guard<std::mutex> lock(sdo_mutex_);
      auto it = sdo_responses_.find(ep.id);
      if (it != sdo_responses_.end()) {
        raw = it->second.raw;
        sdo_responses_.erase(it);
      }
    }
    if (raw) {
      return decode_sdo_value(ep, *raw);
    }
  }
  RCLCPP_WARN(
      rclcpp::get_logger("OdriveS1CanSystem"),
      "Timeout reading endpoint %u via SDO on node %d", ep.id, node_id);
  return std::nullopt;
}

bool OdriveS1CanSystem::run_limit_check(size_t idx) {
  if (node_status_.limit_check_config.mode == LimitCheckConfig::Mode::OFF) return true;
  const auto &cfg = axis_configs_[idx];
  auto &meta = runtime_metadata_[idx];

  // Optionally fetch limits via SDO if configured and not already cached.
  if (node_status_.limits_check_use_sdo && transport_ && !node_status_.flat_endpoints_path.empty()) {
    if (!meta.odrive_velocity_limit) {
      const auto ep = (idx < endpoint_vel_.size()) ? endpoint_vel_[idx] : vel_limit_endpoint_for_axis(cfg.axis_index);
      if (ep) {
        meta.odrive_velocity_limit = read_endpoint_via_sdo(cfg.node_id, *ep);
      }
    }
    if (!meta.odrive_effort_limit) {
      const auto ep = (idx < endpoint_effort_.size()) ? endpoint_effort_[idx] : current_limit_endpoint_for_axis(cfg.axis_index);
      if (ep) {
        meta.odrive_effort_limit = read_endpoint_via_sdo(cfg.node_id, *ep);
      }
    }
    if (!meta.odrive_accel_limit) {
      const auto ep = (idx < endpoint_accel_.size()) ? endpoint_accel_[idx] : accel_limit_endpoint_for_axis(cfg.axis_index);
      if (ep) {
        meta.odrive_accel_limit = read_endpoint_via_sdo(cfg.node_id, *ep);
      }
    }
  }

  meta.limit_check_result = AxisRuntimeMetadata::LimitCheckResult::UNKNOWN;
  meta.limit_check_detail.clear();

  const bool have_velocity = cfg.limit_velocity && meta.odrive_velocity_limit;
  const bool have_effort = cfg.limit_effort && meta.odrive_effort_limit;
  const bool have_accel = cfg.limit_acceleration && meta.odrive_accel_limit;
  if (!have_velocity && !have_effort && !have_accel) {
    meta.limit_check_result = AxisRuntimeMetadata::LimitCheckResult::SKIPPED_NO_DATA;
    RCLCPP_INFO(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Limit check skipped for joint %s (no ODrive limits received)", cfg.joint_name.c_str());
    return true;
  }

  bool ok = true;
  std::ostringstream detail;
  // Compare URDF-implied limits against values reported over CAN.
  if (cfg.limit_velocity && meta.odrive_velocity_limit) {
    const double expected = joint_vel_to_actuator(cfg, *cfg.limit_velocity);
    if (*meta.odrive_velocity_limit + std::fabs(expected) * node_status_.limit_check_config.velocity_tolerance_ratio <
        expected) {
      ok = false;
      detail << "vel_mismatch expected_actuator=" << expected << " odrive=" << *meta.odrive_velocity_limit << ";";
    }
  }
  if (cfg.limit_effort && meta.odrive_effort_limit) {
    const double expected = joint_effort_to_actuator(cfg, *cfg.limit_effort);
    if (*meta.odrive_effort_limit + std::fabs(expected) * node_status_.limit_check_config.effort_tolerance_ratio <
        expected) {
      ok = false;
      detail << "effort_mismatch expected_actuator=" << expected << " odrive=" << *meta.odrive_effort_limit << ";";
    }
  }
  if (cfg.limit_acceleration && meta.odrive_accel_limit) {
    const double exp_acc = joint_vel_to_actuator(cfg, *cfg.limit_acceleration);
    if (*meta.odrive_accel_limit + std::fabs(exp_acc) * node_status_.limit_check_config.acceleration_tolerance_ratio <
        exp_acc) {
      ok = false;
      detail << "accel_mismatch expected_actuator=" << exp_acc << " odrive=" << *meta.odrive_accel_limit << ";";
    }
  }

  if (!ok && node_status_.limit_check_config.mode == LimitCheckConfig::Mode::STRICT) {
    meta.limit_check_result = AxisRuntimeMetadata::LimitCheckResult::ERROR;
    meta.limit_check_detail = detail.str();
    RCLCPP_ERROR(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Limit mismatch for joint %s", cfg.joint_name.c_str());
  } else if (!ok) {
    meta.limit_check_result = AxisRuntimeMetadata::LimitCheckResult::WARN;
    meta.limit_check_detail = detail.str();
    RCLCPP_WARN(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Limit mismatch for joint %s", cfg.joint_name.c_str());
    if (axis_states_[idx].health == AxisHealth::OK) {
      axis_states_[idx].health = AxisHealth::WARNING;
    }
  } else {
    meta.limit_check_result = AxisRuntimeMetadata::LimitCheckResult::OK;
    meta.limit_check_detail.clear();
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
    const bool requires_rearm = runtime_metadata_[i].requires_rearm;
    device.status = static_cast<uint8_t>(
        (axis_states_[i].health == AxisHealth::ERROR || requires_rearm)
            ? control_msgs::msg::HardwareStatus::STATUS_ERROR
            : (axis_states_[i].health == AxisHealth::WARNING
                   ? control_msgs::msg::HardwareStatus::STATUS_WARNING
                   : control_msgs::msg::HardwareStatus::STATUS_RUNNING));
    std::ostringstream details;
    const double age = (node_->get_clock()->now() - axis_states_[i].last_heartbeat).seconds();
    details << "mode=" << mode_to_string(command_modes_[i]);
    details << " axis_state=" << static_cast<int>(axis_states_[i].axis_state);
    const bool power_on = axis_states_[i].axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL;
    const std::string power_state = requires_rearm ? "ERROR" : (power_on ? "ON" : "OFF");
    details << " power=" << power_state;
    details << " heartbeat_age_s=" << std::fixed << std::setprecision(3) << age;
    details << " axis_error=0x" << std::hex << axis_states_[i].axis_error;
    details << " disarm_reason=0x" << std::hex << runtime_metadata_[i].disarm_reason;
    details << std::dec;
    details << " homing=" << runtime_metadata_[i].last_homing_result;
    details << " requires_rearm=" << (requires_rearm ? "true" : "false");
    if (axis_states_[i].heartbeat_stale) details << " heartbeat_stale=true";
    if (axis_states_[i].health == AxisHealth::ERROR && !requires_rearm) details << " faulted=true";
    if (runtime_metadata_[i].limit_check_result == AxisRuntimeMetadata::LimitCheckResult::WARN) {
      details << " limits=warn";
      if (!runtime_metadata_[i].limit_check_detail.empty()) details << "(" << runtime_metadata_[i].limit_check_detail << ")";
    } else if (runtime_metadata_[i].limit_check_result == AxisRuntimeMetadata::LimitCheckResult::ERROR) {
      details << " limits=error";
      if (!runtime_metadata_[i].limit_check_detail.empty()) details << "(" << runtime_metadata_[i].limit_check_detail << ")";
    } else if (runtime_metadata_[i].limit_check_result == AxisRuntimeMetadata::LimitCheckResult::SKIPPED_NO_DATA) {
      details << " limits=skipped(no_data)";
    }
    device.error_message = details.str();
    device.type = "odrive_axis";
  }
  hw_status_pub_->publish(status_msg);
  if (diag_updater_) diag_updater_->force_update();
  last_status_publish_time_ = stamp;
}

uint32_t OdriveS1CanSystem::axis_can_id(size_t idx) const {
  if (idx >= axis_can_ids_.size()) return 0;
  return axis_can_ids_[idx];
}

bool OdriveS1CanSystem::start_homing() {
  bool ok = true;
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    ok &= send_axis_state(i, AXIS_STATE_HOMING);
    command_modes_[i] = AxisControlMode::HOMING;
    runtime_metadata_[i].sent_closed_loop = false;
    runtime_metadata_[i].last_homing_result = "requested";
  }
  return ok;
}

} // namespace odrive_ros2_control

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
