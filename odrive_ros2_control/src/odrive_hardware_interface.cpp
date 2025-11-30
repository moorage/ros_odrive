#include "odrive_ros2_control/odrive_system.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <linux/can.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <sstream>
#include <type_traits>

#include "std_msgs/msg/string.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "socket_can.hpp"
#include "epoll_event_loop.hpp"

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
    size_t budget = 64;
    while (budget-- > 0 && can_.read_nonblocking()) {
      // frames are handled via callback; bounded to avoid long RT stalls.
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

template <typename Msg>
void set_status_field(Msg &msg, uint8_t status) {
  if constexpr (requires(Msg m) { m.status = status; }) {
    msg.status = status;
  }
}

template <typename Msg>
void set_power_state_field(Msg &msg, uint8_t power) {
  if constexpr (requires(Msg m) { m.power_state = power; }) {
    msg.power_state = power;
  }
}

template <typename Msg>
void set_message_field(Msg &msg, const std::string &text) {
  if constexpr (requires(Msg m) { m.message = text; }) {
    msg.message = text;
  } else if constexpr (requires(Msg m) { m.status_message = text; }) {
    msg.status_message = text;
  }
}

template <typename Msg>
void add_kv(Msg &msg, const std::string &key, const std::string &value) {
  if constexpr (requires { typename Msg::KeyValue{}; }) {
    typename Msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    if constexpr (requires(Msg m) { m.values.push_back(kv); }) {
      msg.values.push_back(kv);
    }
  }
}

const char *axis_state_to_string(uint8_t state) {
  switch (state) {
    case AXIS_STATE_UNDEFINED: return "UNDEFINED";
    case AXIS_STATE_IDLE: return "IDLE";
    case AXIS_STATE_STARTUP_SEQUENCE: return "STARTUP_SEQUENCE";
    case AXIS_STATE_FULL_CALIBRATION_SEQUENCE: return "FULL_CALIBRATION";
    case AXIS_STATE_MOTOR_CALIBRATION: return "MOTOR_CALIBRATION";
    case AXIS_STATE_ENCODER_INDEX_SEARCH: return "ENCODER_INDEX_SEARCH";
    case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: return "ENCODER_OFFSET_CALIBRATION";
    case AXIS_STATE_CLOSED_LOOP_CONTROL: return "CLOSED_LOOP_CONTROL";
    case AXIS_STATE_LOCKIN_SPIN: return "LOCKIN_SPIN";
    case AXIS_STATE_ENCODER_DIR_FIND: return "ENCODER_DIR_FIND";
    case AXIS_STATE_HOMING: return "HOMING";
    case AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION: return "ENCODER_HALL_POLARITY_CALIBRATION";
    case AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION: return "ENCODER_HALL_PHASE_CALIBRATION";
    case AXIS_STATE_ANTICOGGING_CALIBRATION: return "ANTICOGGING_CALIBRATION";
    default: return "UNKNOWN";
  }
}

} // namespace

thread_local CanTransportFactory OdriveS1CanSystem::transport_factory_override_;

CanTransportFactory OdriveS1CanSystem::default_transport_factory() {
  return []() { return std::make_shared<SocketCanTransport>(); };
}

OdriveS1CanSystem::OdriveS1CanSystem() : OdriveS1CanSystem(default_transport_factory()) {}

OdriveS1CanSystem::OdriveS1CanSystem(CanTransportFactory factory) {
  transport_factory_ = factory ? factory : default_transport_factory();
}

void OdriveS1CanSystem::set_transport_factory_for_tests(const CanTransportFactory &factory) {
  if (factory) {
    // Create one instance eagerly so test code can attach hooks before lifecycle callbacks run.
    auto first_instance = factory();
    transport_factory_override_ = [factory, first_instance]() mutable {
      if (first_instance) {
        auto out = first_instance;
        first_instance.reset();
        return out;
      }
      return factory();
    };
  } else {
    transport_factory_override_ = nullptr;
  }
}

AxisControlMode OdriveS1CanSystem::string_to_mode(const std::string &mode) const {
  if (mode == "position") return AxisControlMode::POSITION;
  if (mode == "velocity") return AxisControlMode::VELOCITY;
  if (mode == "effort") return AxisControlMode::EFFORT;
  if (mode == "homing") return AxisControlMode::HOMING;
  return AxisControlMode::IDLE;
}

const char *OdriveS1CanSystem::mode_to_string(AxisControlMode mode) const {
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
  std::filesystem::path p = node_status_.flat_endpoints_path;
  if (!p.is_absolute()) {
    const auto source_root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const auto candidate = source_root / p;
    if (std::filesystem::exists(candidate)) {
      p = candidate;
    }
  }
  std::ifstream f(p);
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
  auto logger = rclcpp::get_logger("OdriveS1CanSystem");

  // Initialize the CAN transport using the factory.
  // This allows for dependency injection, which is useful for testing.
  transport_factory_ = transport_factory_override_ ? transport_factory_override_ : default_transport_factory();
  last_control_time_ = rclcpp::Time(0, 0, steady_clock_.get_clock_type());

  // Status publisher for homing/axis state visibility.
  status_node_ = rclcpp::Node::make_shared("odrive_s1_status");
  status_pub_ = status_node_->create_publisher<std_msgs::msg::String>("odrive_s1_ros2_control/status", 10);

  auto get_param = [&](const std::string &key, const std::string &fallback, const std::string &default_val) {
    auto it = info_.hardware_parameters.find(key);
    if (it != info_.hardware_parameters.end()) return it->second;
    auto it_fallback = info_.hardware_parameters.find(fallback);
    if (it_fallback != info_.hardware_parameters.end()) return it_fallback->second;
    return default_val;
  };

  auto parse_double = [&](const std::string &text, const char *name) -> std::optional<double> {
    try {
      const double v = std::stod(text);
      if (!std::isfinite(v)) {
        RCLCPP_ERROR(logger, "Invalid %s value (non-finite): %s", name, text.c_str());
        return std::nullopt;
      }
      return v;
    } catch (...) {
      RCLCPP_ERROR(logger, "Invalid %s value: %s", name, text.c_str());
      return std::nullopt;
    }
  };

  auto parse_int = [&](const std::string &text, const char *name) -> std::optional<int> {
    try {
      return std::stoi(text);
    } catch (...) {
      RCLCPP_ERROR(logger, "Invalid %s value: %s", name, text.c_str());
      return std::nullopt;
    }
  };

  auto parse_uint32 = [&](const std::string &text, const char *name) -> std::optional<uint32_t> {
    try {
      return static_cast<uint32_t>(std::stoul(text));
    } catch (...) {
      RCLCPP_ERROR(logger, "Invalid %s value: %s", name, text.c_str());
      return std::nullopt;
    }
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
  auto status_rate = parse_double(get_param("status_publish_rate", "", "5.0"), "status_publish_rate");
  if (!status_rate) return CallbackReturn::ERROR;
  node_status_.status_publish_rate_hz = *status_rate;
  auto heartbeat_timeout = parse_double(get_param("heartbeat_timeout", "", "0.5"), "heartbeat_timeout");
  if (!heartbeat_timeout) return CallbackReturn::ERROR;
  node_status_.heartbeat_timeout_sec = *heartbeat_timeout;
  auto command_tol = parse_double(get_param("command_tolerance", "", "1e-4"), "command_tolerance");
  if (!command_tol) return CallbackReturn::ERROR;
  node_status_.command_tolerance = *command_tol;
  auto can_util_limit = parse_double(get_param("can_utilization_limit", "", "0.85"), "can_utilization_limit");
  if (!can_util_limit) return CallbackReturn::ERROR;
  node_status_.can_utilization_limit = *can_util_limit;
  node_status_.default_mode = get_param("default_mode", "", "idle");
  node_status_.homing_use_sdo = to_bool(get_param("homing.use_sdo", "", "false"), false);
  auto can_bitrate = parse_uint32(get_param("can_bitrate", "", "1000000"), "can_bitrate");
  if (!can_bitrate) return CallbackReturn::ERROR;
  node_status_.can_bitrate = *can_bitrate;
  node_status_.fault_idle_on_error = to_bool(get_param("fault_idle_on_error", "", "true"), true);
  node_status_.require_rearm_after_fault = to_bool(get_param("require_rearm_after_fault", "", "true"), true);
  node_status_.debug_log_setpoints = to_bool(get_param("debug_log_setpoints", "", "false"), false);
  node_status_.debug_log_bus_voltage = to_bool(get_param("debug_log_bus_voltage", "", "false"), false);
  auto bus_voltage_log_period = parse_double(
      get_param("debug_log_bus_voltage_period_sec", "", "10.0"), "debug_log_bus_voltage_period_sec");
  if (!bus_voltage_log_period) return CallbackReturn::ERROR;
  node_status_.bus_voltage_log_period_sec = *bus_voltage_log_period;
  node_status_.strict_bitrate = to_bool(get_param("can_bitrate_strict", "", "true"), true);
  node_status_.skip_can_validation = to_bool(get_param("skip_can_validation", "", "false"), false);
  node_status_.log_axis_config = to_bool(get_param("log_axis_config", "", "false"), false);
  auto max_frames_per_cycle = parse_int(get_param("max_frames_per_cycle", "", "0"), "max_frames_per_cycle");
  if (!max_frames_per_cycle) return CallbackReturn::ERROR;
  node_status_.max_frames_per_cycle = *max_frames_per_cycle;
  auto max_frames_per_sec = parse_double(get_param("max_frames_per_sec", "", "0.0"), "max_frames_per_sec");
  if (!max_frames_per_sec) return CallbackReturn::ERROR;
  node_status_.max_frames_per_sec = *max_frames_per_sec;
  auto write_latency_warn = parse_double(get_param("write_latency_warn_sec", "", "0.02"), "write_latency_warn_sec");
  if (!write_latency_warn) return CallbackReturn::ERROR;
  node_status_.write_latency_warn_sec = *write_latency_warn;
  node_status_.limits_check_use_sdo = to_bool(get_param("limits_check.use_sdo", "", "false"), false);
  // Accept both a generic flat_endpoints_path and the limits_check-scoped parameter for backward compatibility.
  node_status_.flat_endpoints_path = get_param("flat_endpoints_path", "", "");
  if (node_status_.flat_endpoints_path.empty()) {
    node_status_.flat_endpoints_path = get_param("limits_check.flat_endpoints_path", "", "");
  }
  auto sdo_timeout = parse_double(get_param("limits_check.sdo_timeout_sec", "", "0.5"), "limits_check.sdo_timeout_sec");
  if (!sdo_timeout) return CallbackReturn::ERROR;
  node_status_.sdo_timeout_sec = *sdo_timeout;

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
  transmissions_.clear();
  can_id_lookup_.clear();
  node_to_primary_axis_.clear();
  endpoint_vel_.clear();
  endpoint_effort_.clear();
  endpoint_accel_.clear();
  last_logged_commands_.clear();
  last_logged_modes_.clear();

  for (const auto &joint : info_.joints) {
    AxisConfig cfg;
    cfg.joint_name = joint.name;
    cfg.joint_type = joint.type;
    auto jt_param = joint.parameters.find("joint_type");
    if (jt_param != joint.parameters.end()) {
      cfg.joint_type = jt_param->second;
    }

    auto get_joint_param = [&](const std::string &key, const std::string &fallback) -> std::optional<double> {
      auto it = joint.parameters.find(key);
      if (it != joint.parameters.end()) {
        return parse_double(it->second, key.c_str());
      }
      if (!fallback.empty()) {
        auto it2 = joint.parameters.find(fallback);
        if (it2 != joint.parameters.end()) {
          return parse_double(it2->second, fallback.c_str());
        }
      }
      return std::nullopt;
    };

    auto node_id_it = joint.parameters.find("odrive_node_id");
    if (node_id_it == joint.parameters.end()) node_id_it = joint.parameters.find("node_id");
    if (node_id_it == joint.parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "Joint %s missing odrive_node_id", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    auto node_id_val = parse_int(node_id_it->second, "odrive_node_id");
    if (!node_id_val) return CallbackReturn::ERROR;
    cfg.node_id = *node_id_val;
    auto axis_idx_val = get_joint_param("odrive_axis_index", "axis");
    cfg.axis_index = static_cast<int>(axis_idx_val.value_or(0));
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
    if (cfg.has_transmission && !cfg.gear_ratio) {
      for (const auto &tr : info_.transmissions) {
        for (const auto &j : tr.joints) {
          if (j.name != joint.name) continue;
          cfg.gear_ratio = j.mechanical_reduction;
        }
      }
    }

    std::optional<TransmissionData> tx;
    if (cfg.has_transmission) {
      double joint_offset = 0.0;
      for (const auto &tr : info_.transmissions) {
        for (const auto &j : tr.joints) {
          if (j.name != joint.name) continue;
          joint_offset = j.offset;
          if (!cfg.gear_ratio) cfg.gear_ratio = j.mechanical_reduction;
        }
      }
      cfg.joint_offset = joint_offset;
      tx = TransmissionData{};
      tx->reduction = cfg.gear_ratio.value_or(1.0);
      tx->joint_offset = joint_offset;
    }

    axis_configs_.push_back(cfg);
    axis_states_.emplace_back();
    axis_commands_.emplace_back();
    command_modes_.push_back(AxisControlMode::IDLE);
    runtime_metadata_.emplace_back();
    axis_can_ids_.push_back(0);
    transmissions_.push_back(std::move(tx));
  }

  last_logged_commands_.assign(axis_configs_.size(), AxisCommand{});
  last_logged_modes_.assign(axis_configs_.size(), AxisControlMode::IDLE);
  last_bus_voltage_log_time_by_node_.clear();

  hardware_status_cache_.assign(axis_configs_.size(), HardwareStatusMsg{});
  if constexpr (requires { typename HardwareStatusMsg::KeyValue{}; }) {
    constexpr size_t kValueCount = 5;
    for (size_t i = 0; i < hardware_status_cache_.size(); ++i) {
      auto &msg = hardware_status_cache_[i];
      msg.values.resize(kValueCount);
      msg.values[0].key = "axis_state";
      msg.values[1].key = "fault_code";
      msg.values[2].key = "health_detail";
      msg.values[3].key = "heartbeat_age";
      msg.values[4].key = "homing_status";
    }
  }

  if (node_status_.log_axis_config) {
    for (size_t idx = 0; idx < axis_configs_.size(); ++idx) {
      const auto &cfg = axis_configs_[idx];
      const auto &joint = info_.joints[idx];
      auto join_ifaces = [](const auto &ifaces) {
        std::ostringstream ss;
        for (size_t i = 0; i < ifaces.size(); ++i) {
          if (i > 0) ss << ",";
          ss << ifaces[i].name;
        }
        return ss.str();
      };
      const std::string gear = cfg.gear_ratio ? std::to_string(*cfg.gear_ratio) : "n/a";
      const std::string pitch = cfg.lead_screw_pitch ? std::to_string(*cfg.lead_screw_pitch) : "n/a";
      RCLCPP_INFO(
          logger,
          "Axis %zu: joint=%s type=%s node_id=%d axis_index=%d has_tx=%s gear_ratio=%s lead_screw_pitch=%s "
          "cmd_ifaces=[%s] state_ifaces=[%s]",
          idx,
          cfg.joint_name.c_str(),
          cfg.joint_type.c_str(),
          cfg.node_id,
          cfg.axis_index,
          cfg.has_transmission ? "true" : "false",
          gear.c_str(),
          pitch.c_str(),
          join_ifaces(joint.command_interfaces).c_str(),
          join_ifaces(joint.state_interfaces).c_str());
    }
  }

  if (!validate_parameters()) {
    return CallbackReturn::ERROR;
  }

  last_status_publish_time_ = rclcpp::Time(0, 0, steady_clock_.get_clock_type());
  configured_ = false;
  return CallbackReturn::SUCCESS;
}

bool OdriveS1CanSystem::validate_parameters() {
  std::unordered_map<int, int> node_usage;
  for (const auto &cfg : axis_configs_) {
    node_usage[cfg.node_id] += 1;
  }
  for (const auto &kv : node_usage) {
    if (kv.second > 1) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "CAN node_id %d used by %d joints; limit is 1 axis per ODrive S1", kv.first, kv.second);
      return false;
    }
  }

  for (size_t idx = 0; idx < axis_configs_.size(); ++idx) {
    const auto &cfg = axis_configs_[idx];
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
    if (cfg.has_transmission) {
      if (!cfg.gear_ratio) {
        RCLCPP_ERROR(
            rclcpp::get_logger("OdriveS1CanSystem"),
            "Joint %s has a transmission but missing mechanical_reduction/gear_ratio", cfg.joint_name.c_str());
        return false;
      }
      if (!std::isfinite(*cfg.gear_ratio) || *cfg.gear_ratio <= 0.0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("OdriveS1CanSystem"),
            "Joint %s transmission gear ratio must be > 0", cfg.joint_name.c_str());
        return false;
      }
    }
    if (cfg.gear_ratio && (!std::isfinite(*cfg.gear_ratio) || *cfg.gear_ratio <= 0.0)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s gear_ratio must be > 0", cfg.joint_name.c_str());
      return false;
    }
    if (cfg.lead_screw_pitch && (!std::isfinite(*cfg.lead_screw_pitch) || *cfg.lead_screw_pitch <= 0.0)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s lead_screw_pitch must be > 0", cfg.joint_name.c_str());
      return false;
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
    // ODrive S1 only exposes axis0; reject any other index so CAN IDs map 1:1
    // with configured node_id.
    if (cfg.axis_index != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("OdriveS1CanSystem"),
          "Joint %s axis_index %d invalid (S1 supports only axis 0)", cfg.joint_name.c_str(), cfg.axis_index);
      return false;
    }

    // For single-axis S1, CAN node_id doubles as axis CAN ID.
    const uint32_t can_id = static_cast<uint32_t>(cfg.node_id);
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

  return true;
}

CallbackReturn OdriveS1CanSystem::on_configure(const rclcpp_lifecycle::State &) {
  if (!node_status_.skip_can_validation) {
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
          if (node_status_.strict_bitrate) {
            RCLCPP_ERROR(
                rclcpp::get_logger("OdriveS1CanSystem"),
                "CAN bitrate mismatch for %s: configured %u vs actual %u (strict)",
                node_status_.can_interface.c_str(), node_status_.can_bitrate, *actual);
            return CallbackReturn::ERROR;
          }
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
  }

  if ((node_status_.limits_check_use_sdo || node_status_.homing_use_sdo) &&
      !node_status_.flat_endpoints_path.empty()) {
    load_flat_endpoints();
  }

  transport_ = transport_factory_();
  if (!transport_) {
    RCLCPP_ERROR(rclcpp::get_logger("OdriveS1CanSystem"), "Failed to create CAN transport");
    return CallbackReturn::ERROR;
  }

  auto cb = [this](const can_frame &frame) { handle_frame(frame, now_for_io()); };
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
  util_metrics_.window_start = rclcpp::Time(0, 0, steady_clock_.get_clock_type());
  last_control_time_ = rclcpp::Time(0, 0, steady_clock_.get_clock_type());
  active_ = true;
  for (size_t i = 0; i < command_modes_.size(); ++i) {
    runtime_metadata_[i].sent_closed_loop = false;
    runtime_metadata_[i].awaiting_closed_loop = true;
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
  state_interfaces.reserve(info_.joints.size() * 9); // includes diagnostics

  auto add_iface = [&](size_t idx, const std::string &name, double *ptr) {
    state_interfaces.emplace_back(axis_configs_[idx].joint_name, name, ptr);
  };

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    bool has_pos = false, has_vel = false, has_eff = false;
    for (const auto &iface : info_.joints[i].state_interfaces) {
      const auto &name = iface.name;
      if (name == hardware_interface::HW_IF_POSITION) {
        has_pos = true;
        add_iface(i, name, &axis_states_[i].pos_joint);
      } else if (name == hardware_interface::HW_IF_VELOCITY) {
        has_vel = true;
        add_iface(i, name, &axis_states_[i].vel_joint);
      } else if (name == hardware_interface::HW_IF_EFFORT) {
        has_eff = true;
        add_iface(i, name, &axis_states_[i].effort_joint);
      } else {
        RCLCPP_WARN(
            rclcpp::get_logger("OdriveS1CanSystem"),
            "Ignoring unsupported state interface '%s' for joint %s",
            name.c_str(), axis_configs_[i].joint_name.c_str());
      }
    }
    if (!has_pos) add_iface(i, hardware_interface::HW_IF_POSITION, &axis_states_[i].pos_joint);
    if (!has_vel) add_iface(i, hardware_interface::HW_IF_VELOCITY, &axis_states_[i].vel_joint);
    if (!has_eff) add_iface(i, hardware_interface::HW_IF_EFFORT, &axis_states_[i].effort_joint);
    add_iface(i, "health", &axis_states_[i].health_numeric);
    add_iface(i, "axis_state", &axis_states_[i].axis_state_report);
    add_iface(i, "power_state", &axis_states_[i].power_state_numeric);
    add_iface(i, "fault_code", &axis_states_[i].fault_code);
    add_iface(i, "heartbeat_age", &axis_states_[i].heartbeat_age);
    add_iface(i, "homing_status", &axis_states_[i].homing_status);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OdriveS1CanSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> cmd_interfaces;
  cmd_interfaces.reserve(info_.joints.size() * 4); // include homing command

  auto add_iface = [&](size_t idx, const std::string &name, double *ptr) {
    cmd_interfaces.emplace_back(axis_configs_[idx].joint_name, name, ptr);
  };

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    bool has_pos = false, has_vel = false, has_eff = false, has_home = false;
    for (const auto &iface : info_.joints[i].command_interfaces) {
      const auto &name = iface.name;
      if (name == hardware_interface::HW_IF_POSITION) {
        has_pos = true;
        add_iface(i, name, &axis_commands_[i].position);
      } else if (name == hardware_interface::HW_IF_VELOCITY) {
        has_vel = true;
        add_iface(i, name, &axis_commands_[i].velocity);
      } else if (name == hardware_interface::HW_IF_EFFORT) {
        has_eff = true;
        add_iface(i, name, &axis_commands_[i].effort);
      } else if (name == "homing") {
        has_home = true;
        add_iface(i, name, &axis_commands_[i].homing);
      } else {
        RCLCPP_WARN(
            rclcpp::get_logger("OdriveS1CanSystem"),
            "Ignoring unsupported command interface '%s' for joint %s",
            name.c_str(), axis_configs_[i].joint_name.c_str());
      }
    }
    if (!has_pos) add_iface(i, hardware_interface::HW_IF_POSITION, &axis_commands_[i].position);
    if (!has_vel) add_iface(i, hardware_interface::HW_IF_VELOCITY, &axis_commands_[i].velocity);
    if (!has_eff) add_iface(i, hardware_interface::HW_IF_EFFORT, &axis_commands_[i].effort);
    if (!has_home) add_iface(i, "homing", &axis_commands_[i].homing);
  }
  return cmd_interfaces;
}

return_type OdriveS1CanSystem::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
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

  auto interface_for_mode = [](AxisControlMode mode) -> std::string {
    switch (mode) {
      case AxisControlMode::POSITION: return hardware_interface::HW_IF_POSITION;
      case AxisControlMode::VELOCITY: return hardware_interface::HW_IF_VELOCITY;
      case AxisControlMode::EFFORT: return hardware_interface::HW_IF_EFFORT;
      case AxisControlMode::HOMING: return "homing";
      default: return "";
    }
  };

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    AxisControlMode mode = command_modes_[i];
    const std::string base = axis_configs_[i].joint_name + "/";
    // Apply stops.
    auto stop_pos = std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_POSITION);
    auto stop_vel = std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_VELOCITY);
    auto stop_eff = std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_EFFORT);
    auto stop_home = std::find(stop_interfaces.begin(), stop_interfaces.end(), base + "homing");
    if ((mode == AxisControlMode::POSITION && stop_pos != stop_interfaces.end()) ||
        (mode == AxisControlMode::VELOCITY && stop_vel != stop_interfaces.end()) ||
        (mode == AxisControlMode::EFFORT && stop_eff != stop_interfaces.end()) ||
        (mode == AxisControlMode::HOMING && stop_home != stop_interfaces.end())) {
      mode = AxisControlMode::IDLE;
    }

    // Apply starts.
    AxisControlMode requested = mode;
    if (std::find(start_interfaces.begin(), start_interfaces.end(), base + hardware_interface::HW_IF_POSITION) !=
        start_interfaces.end()) {
      requested = AxisControlMode::POSITION;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), base + hardware_interface::HW_IF_VELOCITY) !=
        start_interfaces.end()) {
      requested = AxisControlMode::VELOCITY;
    }
    if (std::find(start_interfaces.begin(), start_interfaces.end(), base + hardware_interface::HW_IF_EFFORT) !=
        start_interfaces.end()) {
      requested = AxisControlMode::EFFORT;
    }

    if (mode != AxisControlMode::IDLE && requested != mode && requested != AxisControlMode::IDLE) {
      const std::string active_iface = base + interface_for_mode(mode);
      if (std::find(stop_interfaces.begin(), stop_interfaces.end(), active_iface) == stop_interfaces.end()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("OdriveS1CanSystem"),
            "Invalid mode switch: joint %s already in a different mode",
            axis_configs_[i].joint_name.c_str());
        return return_type::ERROR;
      }
    }
  }

  util_metrics_.frames_last_cycle = util_metrics_.frames_this_cycle;
  return return_type::OK;
}

return_type OdriveS1CanSystem::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    bool mode_changed = false;
    AxisControlMode previous_mode = command_modes_[i];
    const std::string base = axis_configs_[i].joint_name + "/";
    bool requested_idle = false;
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_POSITION) !=
        stop_interfaces.end()) {
      if (command_modes_[i] == AxisControlMode::POSITION) {
        command_modes_[i] = AxisControlMode::IDLE;
        mode_changed = true;
        requested_idle = true;
      }
    }
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_VELOCITY) !=
        stop_interfaces.end()) {
      if (command_modes_[i] == AxisControlMode::VELOCITY) {
        command_modes_[i] = AxisControlMode::IDLE;
        mode_changed = true;
        requested_idle = true;
      }
    }
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), base + hardware_interface::HW_IF_EFFORT) !=
        stop_interfaces.end()) {
      if (command_modes_[i] == AxisControlMode::EFFORT) {
        command_modes_[i] = AxisControlMode::IDLE;
        mode_changed = true;
        requested_idle = true;
      }
    }
    if (std::find(stop_interfaces.begin(), stop_interfaces.end(), base + "homing") !=
        stop_interfaces.end()) {
      if (command_modes_[i] == AxisControlMode::HOMING) {
        command_modes_[i] = AxisControlMode::IDLE;
        mode_changed = true;
        requested_idle = true;
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
      const bool request_idle = requested_idle && command_modes_[i] == AxisControlMode::IDLE;
      runtime_metadata_[i].pending_idle_request = request_idle;
      runtime_metadata_[i].awaiting_idle = request_idle;
      runtime_metadata_[i].pending_control_mode.reset();
      auto mode = command_modes_[i];
      if (mode == AxisControlMode::IDLE || mode == AxisControlMode::HOMING) {
        runtime_metadata_[i].awaiting_closed_loop = false;
        continue;
      }
      uint8_t ctrl_mode = CONTROL_MODE_POSITION_CONTROL;
      switch (mode) {
        case AxisControlMode::POSITION: ctrl_mode = CONTROL_MODE_POSITION_CONTROL; break;
        case AxisControlMode::VELOCITY: ctrl_mode = CONTROL_MODE_VELOCITY_CONTROL; break;
        case AxisControlMode::EFFORT: ctrl_mode = CONTROL_MODE_TORQUE_CONTROL; break;
        case AxisControlMode::HOMING:
        case AxisControlMode::IDLE:
        default: break;
      }
      runtime_metadata_[i].pending_control_mode = ctrl_mode;
      const bool switching_active_mode =
          (previous_mode != AxisControlMode::IDLE && previous_mode != AxisControlMode::HOMING &&
           mode != AxisControlMode::IDLE && mode != previous_mode);
      runtime_metadata_[i].awaiting_closed_loop =
          switching_active_mode || axis_states_[i].axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL;
    }
  }

  return return_type::OK;
}

return_type OdriveS1CanSystem::read(const rclcpp::Time &stamp, const rclcpp::Duration &) {
  if (!transport_) return return_type::ERROR;
  transport_->poll();
  rclcpp::Time control_now = stamp;
  if (control_now.nanoseconds() == 0) {
    control_now = steady_clock_.now();
  }
  last_control_time_ = control_now;

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    // Convert actuator feedback (turns, turns/s, torque) to joint units (rad, rad/s, Nm).
    axis_states_[i].pos_joint = actuator_to_joint_pos(i, axis_states_[i].pos_actuator);
    axis_states_[i].vel_joint = actuator_vel_to_joint(i, axis_states_[i].vel_actuator);
    axis_states_[i].effort_joint = actuator_effort_to_joint(i, axis_states_[i].torque_actuator);
    
    // Update axis health status based on errors and heartbeat freshness.
    update_health(i, control_now, false);
  }

  publish_status_if_due(control_now);
  for (const auto &state : axis_states_) {
    if (state.health == AxisHealth::ERROR || state.axis_error != 0 || state.disarm_reason != 0) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

return_type OdriveS1CanSystem::write(const rclcpp::Time &stamp, const rclcpp::Duration &) {
  if (!transport_) return return_type::ERROR;
  auto logger = rclcpp::get_logger("OdriveS1CanSystem");
  for (const auto &state : axis_states_) {
    if (state.health == AxisHealth::ERROR || state.axis_error != 0 || state.disarm_reason != 0) {
      return return_type::ERROR;
    }
  }
  if (!active_) {
    // Allow homing requests even when inactive so controllers can trigger a homing sequence before activation.
    for (size_t i = 0; i < axis_configs_.size(); ++i) {
      const double cmd_home = axis_commands_[i].homing;
      const double prev_cmd_home = runtime_metadata_[i].last_cmd_home;
      const bool home_rising = cmd_home > 0.5 && prev_cmd_home <= 0.5;
      runtime_metadata_[i].last_cmd_home = cmd_home;
      if (home_rising && !runtime_metadata_[i].homing_requested) {
        if (send_homing_request(i)) {
          RCLCPP_INFO(
              logger,
              "Axis %s (node %u) homing requested while controller inactive",
              axis_configs_[i].joint_name.c_str(), axis_can_id(i));
          runtime_metadata_[i].homing_requested = true;
          runtime_metadata_[i].last_homing_result = "requested";
          command_modes_[i] = AxisControlMode::HOMING;
          axis_states_[i].homing_status = 1.0;
        }
      }
    }
    return return_type::OK;
  }

  util_metrics_.frames_this_cycle = 0;
  util_metrics_.budget_exceeded_last_cycle = false;
  rclcpp::Time control_now = stamp;
  if (control_now.nanoseconds() == 0) {
    control_now = steady_clock_.now();
  }
  last_control_time_ = control_now;
  util_metrics_.last_write_start = control_now;
  bool ok = true;

  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    if (axis_states_[i].health == AxisHealth::ERROR) continue;

    const double cmd_pos = axis_commands_[i].position;
    const double cmd_vel = axis_commands_[i].velocity;
    const double cmd_eff = axis_commands_[i].effort;
    const double cmd_home = axis_commands_[i].homing;
    // Only trigger homing on a rising edge so a stuck-high command does not loop.
    const double prev_cmd_home = runtime_metadata_[i].last_cmd_home;
    const bool home_rising = cmd_home > 0.5 && prev_cmd_home <= 0.5;
    runtime_metadata_[i].last_cmd_home = cmd_home;
    const AxisControlMode mode = command_modes_[i];

    if (runtime_metadata_[i].pending_idle_request) {
      send_axis_state(i, AXIS_STATE_IDLE);
      runtime_metadata_[i].pending_idle_request = false;
      runtime_metadata_[i].pending_control_mode.reset();
      runtime_metadata_[i].awaiting_idle = true;
      continue;
    }
    if (runtime_metadata_[i].pending_control_mode) {
      if (!runtime_metadata_[i].awaiting_closed_loop) {
        send_control_mode(i, *runtime_metadata_[i].pending_control_mode, command_modes_[i] != AxisControlMode::IDLE);
        runtime_metadata_[i].pending_control_mode.reset();
      }
    }

    // Homing is edge-triggered: >0 triggers a homing request for that axis.
    if (home_rising && !runtime_metadata_[i].homing_requested) {
      if (send_homing_request(i)) {
        RCLCPP_INFO(
            logger,
            "Axis %s (node %u) homing requested",
            axis_configs_[i].joint_name.c_str(), axis_can_id(i));
        runtime_metadata_[i].homing_requested = true;
        runtime_metadata_[i].last_homing_result = "requested";
        command_modes_[i] = AxisControlMode::HOMING;
        axis_states_[i].homing_status = 1.0;
        log_setpoint(i, AxisControlMode::HOMING, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      } else {
        ok = false;
      }
      continue;
    }

    // Send commands based on the active control mode.
    // We only send commands if the value has changed significantly or if we haven't sent a closed-loop command yet.
    // Do not stream commands until the drive confirms CLOSED_LOOP via heartbeat.
    if (mode != AxisControlMode::IDLE && mode != AxisControlMode::HOMING) {
      if (runtime_metadata_[i].awaiting_closed_loop || axis_states_[i].axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        runtime_metadata_[i].awaiting_closed_loop = true;
        if (!runtime_metadata_[i].pending_control_mode) {
          send_axis_state(i, AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
        if (!runtime_metadata_[i].logged_waiting_closed_loop) {
          RCLCPP_WARN(
              logger,
              "Axis %s (node %u) awaiting CLOSED_LOOP_CONTROL (current state=%s); holding commands",
              axis_configs_[i].joint_name.c_str(),
              axis_can_id(i),
              axis_state_to_string(axis_states_[i].axis_state));
          runtime_metadata_[i].logged_waiting_closed_loop = true;
        }
        continue;
      }
      runtime_metadata_[i].awaiting_closed_loop = false;
      runtime_metadata_[i].logged_waiting_closed_loop = false;
    }

    switch (mode) {
      case AxisControlMode::POSITION: {
        const double turns = joint_to_actuator_pos(i, cmd_pos);
        const double vel_ff = joint_vel_to_actuator(i, cmd_vel);
        const double torque_ff = joint_effort_to_actuator(i, cmd_eff);
        const double diff = std::fabs(turns - runtime_metadata_[i].last_cmd_pos);
        if (diff > node_status_.command_tolerance || !runtime_metadata_[i].sent_closed_loop) {
          log_setpoint(i, mode, cmd_pos, cmd_vel, cmd_eff, turns, vel_ff, torque_ff);
          ok &= send_position_command(i, turns, vel_ff, torque_ff);
          runtime_metadata_[i].last_cmd_pos = turns;
          runtime_metadata_[i].last_cmd_vel = vel_ff;
          runtime_metadata_[i].last_cmd_effort = torque_ff;
        }
      } break;
      case AxisControlMode::VELOCITY: {
        const double turns_per_sec = joint_vel_to_actuator(i, cmd_vel);
        const double torque_ff = joint_effort_to_actuator(i, cmd_eff);
        if (std::fabs(turns_per_sec - runtime_metadata_[i].last_cmd_vel) > node_status_.command_tolerance ||
            !runtime_metadata_[i].sent_closed_loop) {
          log_setpoint(i, mode, 0.0, cmd_vel, cmd_eff, 0.0, turns_per_sec, torque_ff);
          ok &= send_velocity_command(i, turns_per_sec, torque_ff);
          runtime_metadata_[i].last_cmd_vel = turns_per_sec;
          runtime_metadata_[i].last_cmd_effort = torque_ff;
        }
      } break;
      case AxisControlMode::EFFORT: {
        const double tq = joint_effort_to_actuator(i, cmd_eff);
        if (std::fabs(tq - runtime_metadata_[i].last_cmd_effort) > node_status_.command_tolerance ||
            !runtime_metadata_[i].sent_closed_loop) {
          log_setpoint(i, mode, 0.0, 0.0, cmd_eff, 0.0, 0.0, tq);
          ok &= send_torque_command(i, tq);
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
  return ok ? return_type::OK : return_type::ERROR;
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
    case Get_Bus_Voltage_Current_msg_t::cmd_id: {
      Get_Bus_Voltage_Current_msg_t msg;
      if (frame.can_dlc >= Get_Bus_Voltage_Current_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        last_bus_voltage_ = msg.Bus_Voltage;
        last_bus_current_ = msg.Bus_Current;
        bus_voltage_valid_ = true;
        if (node_status_.debug_log_bus_voltage) {
          const auto node_id = axis_can_id(idx);
          bool should_log = true;
          auto it = last_bus_voltage_log_time_by_node_.find(static_cast<int>(node_id));
          if (it != last_bus_voltage_log_time_by_node_.end() &&
              it->second.nanoseconds() != 0 &&
              it->second.get_clock_type() == stamp.get_clock_type()) {
            const double age = (stamp - it->second).seconds();
            should_log = age >= node_status_.bus_voltage_log_period_sec;
          }
          if (should_log) {
            last_bus_voltage_log_time_by_node_[static_cast<int>(node_id)] = stamp;
            const double turns = idx < axis_states_.size() ? axis_states_[idx].pos_actuator : 0.0;
            RCLCPP_INFO(
                rclcpp::get_logger("OdriveS1CanSystem"),
                "Axis %s (node %u) bus_voltage=%.3fV bus_current=%.3fA turns=%.3f",
                axis_configs_[idx].joint_name.c_str(), node_id,
                msg.Bus_Voltage, msg.Bus_Current, turns);
          }
        }
      }
    } break;
    case Get_Error_msg_t::cmd_id: {
      Get_Error_msg_t msg;
      if (frame.can_dlc >= Get_Error_msg_t::msg_length) {
        msg.decode_buf(frame.data);
        runtime_metadata_[idx].disarm_reason = msg.Disarm_Reason;
        axis_states_[idx].disarm_reason = msg.Disarm_Reason;
        axis_states_[idx].axis_error = msg.Active_Errors;
        if (msg.Active_Errors != 0 || msg.Disarm_Reason != 0) {
          runtime_metadata_[idx].requires_rearm = node_status_.require_rearm_after_fault;
          latch_fault(idx, msg.Active_Errors | msg.Disarm_Reason);
        } else if (!runtime_metadata_[idx].requires_rearm) {
          axis_states_[idx].health = AxisHealth::OK;
          update_fault_detail(idx);
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
  auto &meta = runtime_metadata_[idx];
  const auto &axis_name = axis_configs_[idx].joint_name;
  const uint32_t node_id = axis_can_id(idx);
  auto logger = rclcpp::get_logger("OdriveS1CanSystem");

  axis_states_[idx].axis_error = msg.Axis_Error;
  axis_states_[idx].axis_state = msg.Axis_State;
  axis_states_[idx].last_heartbeat = stamp;
  runtime_metadata_[idx].sent_closed_loop =
      runtime_metadata_[idx].sent_closed_loop || (msg.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL);
  runtime_metadata_[idx].last_axis_state = msg.Axis_State;

  if (!meta.last_logged_axis_state || *meta.last_logged_axis_state != msg.Axis_State || prev_state != msg.Axis_State) {
    RCLCPP_INFO(
        logger,
        "Axis %s (node %u) state change: %s -> %s (axis_error=0x%x, disarm=0x%x)",
        axis_name.c_str(), node_id, axis_state_to_string(prev_state), axis_state_to_string(msg.Axis_State),
        msg.Axis_Error, axis_states_[idx].disarm_reason);
    meta.last_logged_axis_state = msg.Axis_State;
  }

  if (msg.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL) {
    runtime_metadata_[idx].awaiting_closed_loop = false;
    meta.post_homing_closed_loop_requested = false;
    meta.logged_waiting_closed_loop = false;
  }
  if (msg.Axis_State == AXIS_STATE_IDLE) {
    runtime_metadata_[idx].awaiting_idle = false;
  }
  if (prev_state == AXIS_STATE_HOMING || runtime_metadata_[idx].homing_requested) {
    if (msg.Axis_Error != 0) {
      runtime_metadata_[idx].last_homing_result = "failed";
      runtime_metadata_[idx].homing_requested = false;
      axis_states_[idx].homing_status = 3.0;
      meta.last_logged_homing_status = 3;
      RCLCPP_WARN(
          logger,
          "Axis %s (node %u) homing failed with axis_error=0x%x disarm=0x%x state=%s",
          axis_name.c_str(), node_id, msg.Axis_Error, axis_states_[idx].disarm_reason,
          axis_state_to_string(msg.Axis_State));
      latch_fault(idx, msg.Axis_Error);
    } else if (msg.Axis_State == AXIS_STATE_CLOSED_LOOP_CONTROL) {
      runtime_metadata_[idx].last_homing_result = "success";
      runtime_metadata_[idx].homing_requested = false;
      axis_states_[idx].homing_status = 2.0;
      meta.last_logged_homing_status = 2;
      RCLCPP_INFO(
          logger,
          "Axis %s (node %u) homing complete; entering CLOSED_LOOP_CONTROL",
          axis_name.c_str(), node_id);
      axis_states_[idx].pos_actuator = 0.0;
      axis_states_[idx].vel_actuator = 0.0;
      axis_states_[idx].pos_joint = 0.0;
      axis_states_[idx].vel_joint = 0.0;
      axis_commands_[idx].position = 0.0;
      axis_commands_[idx].velocity = 0.0;
      command_modes_[idx] = AxisControlMode::IDLE;
    } else if (msg.Axis_State == AXIS_STATE_IDLE) {
      runtime_metadata_[idx].last_homing_result = "success_idle";
      runtime_metadata_[idx].homing_requested = false;
      axis_states_[idx].homing_status = 2.0;
      meta.last_logged_homing_status = 2;
      RCLCPP_INFO(
          logger,
          "Axis %s (node %u) homing complete; drive returned to IDLE. Requesting CLOSED_LOOP_CONTROL",
          axis_name.c_str(), node_id);
      if (!meta.post_homing_closed_loop_requested) {
        send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
        runtime_metadata_[idx].awaiting_closed_loop = true;
        meta.post_homing_closed_loop_requested = true;
      }
    }
  }
  // If homing succeeded and we’re sitting in IDLE, proactively request CLOSED_LOOP so
  // trajectories can start without timing out on the first command.
  if (axis_states_[idx].homing_status == 2.0 &&
      msg.Axis_State == AXIS_STATE_IDLE &&
      !meta.post_homing_closed_loop_requested) {
    RCLCPP_INFO(
        logger,
        "Axis %s (node %u) homing done and IDLE; requesting CLOSED_LOOP_CONTROL",
        axis_name.c_str(), node_id);
    send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
    meta.post_homing_closed_loop_requested = true;
    runtime_metadata_[idx].awaiting_closed_loop = true;
  }

  if (msg.Axis_Error != 0) {
    latch_fault(idx, msg.Axis_Error);
  } else if (!runtime_metadata_[idx].requires_rearm) {
    axis_states_[idx].health = AxisHealth::OK;
  } else {
    axis_states_[idx].health = AxisHealth::ERROR;
  }

  const uint32_t combined_faults = axis_states_[idx].axis_error | axis_states_[idx].motor_error |
      axis_states_[idx].controller_error | axis_states_[idx].encoder_error | axis_states_[idx].disarm_reason;
  if (combined_faults != 0) {
    if (!meta.last_logged_fault || *meta.last_logged_fault != combined_faults) {
      RCLCPP_WARN(
          logger,
          "Axis %s (node %u) fault/disarm detected axis=0x%x motor=0x%x controller=0x%x encoder=0x%x disarm=0x%x state=%s",
          axis_name.c_str(), node_id, axis_states_[idx].axis_error, axis_states_[idx].motor_error,
          axis_states_[idx].controller_error, axis_states_[idx].encoder_error, axis_states_[idx].disarm_reason,
          axis_state_to_string(axis_states_[idx].axis_state));
      meta.last_logged_fault = combined_faults;
    }
  } else if (meta.last_logged_fault) {
    RCLCPP_INFO(
        logger,
        "Axis %s (node %u) cleared faults; state=%s",
        axis_name.c_str(), node_id, axis_state_to_string(axis_states_[idx].axis_state));
    meta.last_logged_fault.reset();
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

void OdriveS1CanSystem::update_health(size_t idx, const rclcpp::Time &now, bool heartbeat_refresh) {
  (void)heartbeat_refresh;
  double age = node_status_.heartbeat_timeout_sec + 1.0;
  if (axis_states_[idx].last_heartbeat.nanoseconds() != 0 &&
      axis_states_[idx].last_heartbeat.get_clock_type() == now.get_clock_type()) {
    try {
      age = (now - axis_states_[idx].last_heartbeat).seconds();
      if (age < 0.0) {
        age = node_status_.heartbeat_timeout_sec + 1.0;
      }
    } catch (...) {
      age = node_status_.heartbeat_timeout_sec + 1.0;
    }
  }
  if (runtime_metadata_[idx].requires_rearm) {
    axis_states_[idx].health = AxisHealth::ERROR;
  }
  const uint32_t combined_faults = axis_states_[idx].axis_error | axis_states_[idx].motor_error |
      axis_states_[idx].controller_error | axis_states_[idx].encoder_error | axis_states_[idx].disarm_reason;
  if (combined_faults != 0) {
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
  axis_states_[idx].health_numeric = static_cast<double>(health_to_int(axis_states_[idx].health));
  axis_states_[idx].power_state_numeric = static_cast<double>(power_state_to_int(derive_power_state(axis_states_[idx])));
}

void OdriveS1CanSystem::update_fault_detail(size_t idx) {
  auto &state = axis_states_[idx];
  auto &meta = runtime_metadata_[idx];
  const uint32_t fault = state.axis_error | state.motor_error | state.controller_error | state.encoder_error |
      state.disarm_reason;
  state.fault_code = static_cast<double>(fault);

  std::ostringstream ss;
  ss << std::hex;
  ss << "axis=0x" << state.axis_error
     << " motor=0x" << state.motor_error
     << " controller=0x" << state.controller_error
     << " encoder=0x" << state.encoder_error
     << " disarm=0x" << state.disarm_reason
     << " state=0x" << static_cast<int>(state.axis_state);
  if (bus_voltage_valid_) {
    ss << " vbus=" << last_bus_voltage_ << "V";
  }
  if (state.heartbeat_stale) ss << " heartbeat_stale=1";
  meta.fault_detail = ss.str();
}

double OdriveS1CanSystem::actuator_to_joint_pos(size_t idx, double turns) {
  const auto &cfg = axis_configs_[idx];
  double ratio = cfg.gear_ratio.value_or(1.0);
  double joint_offset = cfg.joint_offset.value_or(0.0);
  if (idx < transmissions_.size() && transmissions_[idx] && transmissions_[idx]->valid()) {
    ratio = transmissions_[idx]->reduction;
    joint_offset = transmissions_[idx]->joint_offset;
  }
  if (ratio == 0.0) return 0.0;

  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    return (turns / ratio) * (*cfg.lead_screw_pitch) + joint_offset;
  }
  return (turns * kRadPerTurn) / ratio + joint_offset;
}

double OdriveS1CanSystem::joint_to_actuator_pos(size_t idx, double joint_pos) {
  const auto &cfg = axis_configs_[idx];
  double ratio = cfg.gear_ratio.value_or(1.0);
  double joint_offset = cfg.joint_offset.value_or(0.0);
  if (idx < transmissions_.size() && transmissions_[idx] && transmissions_[idx]->valid()) {
    ratio = transmissions_[idx]->reduction;
    joint_offset = transmissions_[idx]->joint_offset;
  }
  if (ratio == 0.0) return 0.0;

  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    const double pitch = *cfg.lead_screw_pitch;
    if (pitch == 0.0) return 0.0;
    return ((joint_pos - joint_offset) / pitch) * ratio;
  }
  return ((joint_pos - joint_offset) * ratio) / kRadPerTurn;
}

double OdriveS1CanSystem::actuator_vel_to_joint(size_t idx, double turns_per_sec) {
  const auto &cfg = axis_configs_[idx];
  double ratio = cfg.gear_ratio.value_or(1.0);
  if (idx < transmissions_.size() && transmissions_[idx] && transmissions_[idx]->valid()) {
    ratio = transmissions_[idx]->reduction;
  }
  if (ratio == 0.0) return 0.0;

  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    return (turns_per_sec / ratio) * (*cfg.lead_screw_pitch);
  }
  return (turns_per_sec * kRadPerTurn) / ratio;
}

double OdriveS1CanSystem::joint_vel_to_actuator(size_t idx, double joint_vel) {
  const auto &cfg = axis_configs_[idx];
  double ratio = cfg.gear_ratio.value_or(1.0);
  if (idx < transmissions_.size() && transmissions_[idx] && transmissions_[idx]->valid()) {
    ratio = transmissions_[idx]->reduction;
  }
  if (ratio == 0.0) return 0.0;

  if (cfg.joint_type == "prismatic" && cfg.lead_screw_pitch) {
    const double pitch = *cfg.lead_screw_pitch;
    if (pitch == 0.0) return 0.0;
    return (joint_vel / pitch) * ratio;
  }
  return (joint_vel * ratio) / kRadPerTurn;
}

double OdriveS1CanSystem::actuator_effort_to_joint(size_t idx, double torque) {
  const auto &cfg = axis_configs_[idx];
  double ratio = cfg.gear_ratio.value_or(1.0);
  if (idx < transmissions_.size() && transmissions_[idx] && transmissions_[idx]->valid()) {
    ratio = transmissions_[idx]->reduction;
  }
  if (ratio == 0.0) return 0.0;

  double joint_torque = torque * ratio;
  if (cfg.torque_constant) {
    joint_torque *= *cfg.torque_constant;
  }
  return joint_torque;
}

double OdriveS1CanSystem::joint_effort_to_actuator(size_t idx, double effort) {
  const auto &cfg = axis_configs_[idx];
  double ratio = cfg.gear_ratio.value_or(1.0);
  if (idx < transmissions_.size() && transmissions_[idx] && transmissions_[idx]->valid()) {
    ratio = transmissions_[idx]->reduction;
  }
  if (ratio == 0.0) return 0.0;

  double actuator_effort = effort / ratio;
  if (cfg.torque_constant) {
    const double k = *cfg.torque_constant;
    if (k == 0.0) return 0.0;
    actuator_effort = (effort / k) / ratio;
  }
  return actuator_effort;
}

AxisPowerState OdriveS1CanSystem::derive_power_state(const AxisState &state) const {
  if (state.health == AxisHealth::ERROR || state.axis_error != 0 || state.disarm_reason != 0) {
    return AxisPowerState::ERROR;
  }
  if (state.heartbeat_stale) return AxisPowerState::OFF;
  return AxisPowerState::ON;
}

int OdriveS1CanSystem::health_to_int(AxisHealth health) const {
  switch (health) {
    case AxisHealth::OK: return 0;
    case AxisHealth::WARNING: return 1;
    case AxisHealth::ERROR: return 2;
    default: return 0;
  }
}

int OdriveS1CanSystem::power_state_to_int(AxisPowerState state) const {
  switch (state) {
    case AxisPowerState::ON: return 1;
    case AxisPowerState::OFF: return 2;
    case AxisPowerState::ERROR: return 3;
    case AxisPowerState::UNKNOWN:
    default: return 0;
  }
}

bool OdriveS1CanSystem::send_axis_state(size_t idx, uint32_t requested_state) {
  Set_Axis_State_msg_t msg{};
  msg.Axis_Requested_State = requested_state;
  can_frame frame{};
  frame.can_id = (axis_can_id(idx) << 5) | msg.cmd_id;
  frame.can_dlc = msg.msg_length;
  msg.encode_buf(frame.data);
  return send_frame(frame, true, now_for_io());
}

bool OdriveS1CanSystem::send_homing_request(size_t idx) {
  if (node_status_.homing_use_sdo && !node_status_.flat_endpoints_path.empty()) {
    if (auto ep = endpoint_for_axis(axis_configs_[idx].axis_index, "requested_state")) {
      const bool ok = write_endpoint_via_sdo(axis_configs_[idx].node_id, *ep, AXIS_STATE_HOMING);
      if (!ok) {
        RCLCPP_WARN(
            rclcpp::get_logger("OdriveS1CanSystem"),
            "Failed to send homing request via SDO for joint %s (node %d)",
            axis_configs_[idx].joint_name.c_str(), axis_configs_[idx].node_id);
      }
      return ok;
    }
  }
  return send_axis_state(idx, AXIS_STATE_HOMING);
}

bool OdriveS1CanSystem::send_control_mode(size_t idx, uint8_t control_mode, bool require_closed_loop) {
  Set_Controller_Mode_msg_t ctrl{};
  ctrl.Control_Mode = control_mode;
  ctrl.Input_Mode = INPUT_MODE_PASSTHROUGH;
  can_frame frame{};
  frame.can_id = (axis_can_id(idx) << 5) | ctrl.cmd_id;
  frame.can_dlc = ctrl.msg_length;
  ctrl.encode_buf(frame.data);

  bool ok = send_frame(frame, true, now_for_io());
  if (require_closed_loop && axis_states_[idx].axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
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
  return send_frame(frame, true, now_for_io());
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
  return send_frame(frame, true, now_for_io());
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
  return send_frame(frame, true, now_for_io());
}

bool OdriveS1CanSystem::should_log_command(size_t idx, AxisControlMode mode, double pos, double vel, double eff) {
  if (idx >= last_logged_commands_.size()) {
    last_logged_commands_.resize(axis_configs_.size());
  }
  if (idx >= last_logged_modes_.size()) {
    last_logged_modes_.resize(axis_configs_.size(), AxisControlMode::IDLE);
  }
  const double tol = node_status_.command_tolerance;
  const auto &prev = last_logged_commands_[idx];
  const AxisControlMode prev_mode = last_logged_modes_[idx];
  const bool pos_changed = std::fabs(pos - prev.position) > tol;
  const bool vel_changed = std::fabs(vel - prev.velocity) > tol;
  const bool eff_changed = std::fabs(eff - prev.effort) > tol;
  const bool mode_changed = mode != prev_mode;
  if (!(pos_changed || vel_changed || eff_changed || mode_changed)) return false;
  last_logged_commands_[idx].position = pos;
  last_logged_commands_[idx].velocity = vel;
  last_logged_commands_[idx].effort = eff;
  last_logged_modes_[idx] = mode;
  return true;
}

void OdriveS1CanSystem::log_setpoint(
    size_t idx, AxisControlMode mode, double pos_joint, double vel_joint, double eff_joint,
    double act_pos, double act_vel, double act_effort) {
  if (!node_status_.debug_log_setpoints) return;
  if (!should_log_command(idx, mode, pos_joint, vel_joint, eff_joint)) return;
  const auto &cfg = axis_configs_[idx];
  std::ostringstream ss;
  ss << "Setpoint for joint " << cfg.joint_name << " (node " << cfg.node_id << ", axis " << cfg.axis_index
     << ") mode=" << mode_to_string(mode);
  switch (mode) {
    case AxisControlMode::POSITION:
      ss << " pos=" << pos_joint << " vel_ff=" << vel_joint << " effort_ff=" << eff_joint
         << " -> turns=" << act_pos << " vel=" << act_vel << " torque=" << act_effort;
      break;
    case AxisControlMode::VELOCITY:
      ss << " vel=" << vel_joint << " effort_ff=" << eff_joint
         << " -> turns_per_sec=" << act_vel << " torque=" << act_effort;
      break;
    case AxisControlMode::EFFORT:
      ss << " effort=" << eff_joint << " -> torque=" << act_effort;
      break;
    case AxisControlMode::HOMING:
      ss << " homing requested";
      break;
    case AxisControlMode::IDLE:
    default:
      ss << " idle";
      break;
  }
  RCLCPP_INFO(rclcpp::get_logger("OdriveS1CanSystem"), "%s", ss.str().c_str());
  if (log_sink_for_tests_) log_sink_for_tests_(ss.str());
}

rclcpp::Time OdriveS1CanSystem::now_for_io() const {
  if (last_control_time_.nanoseconds() != 0) {
    return last_control_time_;
  }
  return steady_clock_.now();
}

bool OdriveS1CanSystem::can_send_frame(const rclcpp::Time &now) {
  if (util_metrics_.window_start.nanoseconds() == 0 ||
      util_metrics_.window_start.get_clock_type() != now.get_clock_type()) {
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
  if (node_status_.can_utilization_limit > 0.0 && node_status_.can_bitrate > 0) {
    constexpr double kApproxBitsPerFrame = 128.0; // includes overhead; conservative
    const double utilization =
        util_metrics_.frames_per_sec_ewma * kApproxBitsPerFrame / static_cast<double>(node_status_.can_bitrate);
    if (utilization > node_status_.can_utilization_limit) {
      util_metrics_.budget_exceeded_last_cycle = true;
      return false;
    }
  }
  return true;
}

bool OdriveS1CanSystem::send_frame(const can_frame &frame, bool count_budget, const rclcpp::Time &now) {
  if (!transport_) return false;
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
    if (util_metrics_.last_write_start.nanoseconds() == 0 ||
        util_metrics_.last_write_start.get_clock_type() != now.get_clock_type()) {
      util_metrics_.last_write_start = now;
    }
    util_metrics_.last_send_latency_sec = (now - util_metrics_.last_write_start).seconds();
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
  return send_frame(frame, true, now_for_io());
}

void OdriveS1CanSystem::latch_fault(size_t idx, uint32_t axis_error) {
  auto &state = axis_states_[idx];
  auto &meta = runtime_metadata_[idx];
  meta.mode_before_fault = command_modes_[idx];
  state.axis_error = axis_error;
  state.health = AxisHealth::ERROR;
  state.health_numeric = static_cast<double>(health_to_int(state.health));
  state.power_state_numeric = static_cast<double>(power_state_to_int(derive_power_state(state)));
  update_fault_detail(idx);
  meta.homing_requested = false;
  state.homing_status = 3.0;
  command_modes_[idx] = AxisControlMode::IDLE;
  meta.fault_latched = true;
  meta.requires_rearm = node_status_.require_rearm_after_fault;
  meta.sent_closed_loop = false;
  if (node_status_.fault_idle_on_error && !meta.idle_sent_on_fault && transport_) {
    send_axis_state(idx, AXIS_STATE_IDLE);
    meta.idle_sent_on_fault = true;
  }
  if (bus_voltage_valid_) {
    RCLCPP_WARN(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Axis %s (node %u) fault latched axis_error=0x%x disarm=0x%x vbus=%.3fV ibus=%.3fA",
        axis_configs_[idx].joint_name.c_str(), axis_can_id(idx), state.axis_error, state.disarm_reason,
        last_bus_voltage_, last_bus_current_);
  } else {
    RCLCPP_WARN(
        rclcpp::get_logger("OdriveS1CanSystem"),
        "Axis %s (node %u) fault latched axis_error=0x%x disarm=0x%x (bus voltage unknown)",
        axis_configs_[idx].joint_name.c_str(), axis_can_id(idx), state.axis_error, state.disarm_reason);
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
  state.fault_code = 0.0;
  meta.fault_detail.clear();
  meta.fault_latched = false;
  meta.requires_rearm = false;
  meta.idle_sent_on_fault = false;
  meta.sent_closed_loop = false;
  meta.logged_waiting_closed_loop = false;
  meta.homing_requested = false;
  meta.last_logged_fault.reset();
  meta.last_logged_axis_state.reset();
  meta.last_logged_homing_status.reset();
  command_modes_[idx] = meta.mode_before_fault;

  bool ok = send_clear_errors(idx);
  ok &= send_axis_state(idx, AXIS_STATE_CLOSED_LOOP_CONTROL);
  state.health = AxisHealth::OK;
  state.health_numeric = static_cast<double>(health_to_int(state.health));
  state.power_state_numeric = static_cast<double>(power_state_to_int(derive_power_state(state)));
  state.fault_code = 0.0;
  meta.sent_closed_loop = true;
  // Re-issue control mode after clearing faults if a non-idle mode was active.
  if (command_modes_[idx] == AxisControlMode::POSITION || command_modes_[idx] == AxisControlMode::VELOCITY ||
      command_modes_[idx] == AxisControlMode::EFFORT) {
    uint8_t ctrl_mode = CONTROL_MODE_POSITION_CONTROL;
    if (command_modes_[idx] == AxisControlMode::VELOCITY) ctrl_mode = CONTROL_MODE_VELOCITY_CONTROL;
    if (command_modes_[idx] == AxisControlMode::EFFORT) ctrl_mode = CONTROL_MODE_TORQUE_CONTROL;
    meta.pending_control_mode = ctrl_mode;
    meta.awaiting_closed_loop = true;
  }
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

  if (!send_frame(frame, false, now_for_io())) {
    return std::nullopt;
  }

  // Avoid blocking lifecycle transitions; perform a single poll and return if nothing is ready yet.
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
  return std::nullopt;
}

bool OdriveS1CanSystem::write_endpoint_via_sdo(int node_id, const EndpointInfo &ep, uint32_t raw) {
  if (!transport_) return false;
  can_frame frame{};
  frame.can_id = (static_cast<uint32_t>(node_id) << 5) | 0x04; // RxSdo
  frame.can_dlc = 8;
  frame.data[0] = 0x01; // write opcode
  frame.data[1] = static_cast<uint8_t>(ep.id & 0xffu);
  frame.data[2] = static_cast<uint8_t>((ep.id >> 8) & 0xffu);
  frame.data[3] = 0;
  std::memcpy(&frame.data[4], &raw, sizeof(uint32_t));
  return send_frame(frame, true, now_for_io());
}

bool OdriveS1CanSystem::run_limit_check(size_t idx) {
  if (node_status_.limit_check_config.mode == LimitCheckConfig::Mode::OFF) return true;
  const auto &cfg = axis_configs_[idx];
  auto &meta = runtime_metadata_[idx];

  // Optionally fetch limits via SDO if configured and not already cached.
  if (node_status_.limits_check_use_sdo && transport_) {
    auto fetch_endpoint = [&](std::optional<EndpointInfo> ep, std::function<std::optional<EndpointInfo>()> fallback) {
      if (!ep && fallback) {
        ep = fallback();
      }
      if (ep) {
        return read_endpoint_via_sdo(cfg.node_id, *ep);
      }
      return std::optional<double>();
    };

    if (!meta.odrive_velocity_limit) {
      const auto ep = (idx < endpoint_vel_.size()) ? endpoint_vel_[idx] : vel_limit_endpoint_for_axis(cfg.axis_index);
      meta.odrive_velocity_limit = fetch_endpoint(ep, [&]() { return vel_limit_endpoint_for_axis(cfg.axis_index); });
    }
    if (!meta.odrive_effort_limit) {
      const auto ep = (idx < endpoint_effort_.size()) ? endpoint_effort_[idx] : current_limit_endpoint_for_axis(cfg.axis_index);
      meta.odrive_effort_limit = fetch_endpoint(ep, [&]() { return current_limit_endpoint_for_axis(cfg.axis_index); });
    }
    if (!meta.odrive_accel_limit) {
      const auto ep = (idx < endpoint_accel_.size()) ? endpoint_accel_[idx] : accel_limit_endpoint_for_axis(cfg.axis_index);
      meta.odrive_accel_limit = fetch_endpoint(ep, [&]() { return accel_limit_endpoint_for_axis(cfg.axis_index); });
    }
  }

  meta.limit_check_result = AxisRuntimeMetadata::LimitCheckResult::UNKNOWN;
  meta.limit_check_detail.clear();

  const bool have_velocity = cfg.limit_velocity && meta.odrive_velocity_limit;
  const bool have_effort = cfg.limit_effort && meta.odrive_effort_limit;
  const bool have_accel = cfg.limit_acceleration && meta.odrive_accel_limit;
  bool ok = true;
  std::ostringstream detail;
  std::vector<std::string> missing;
  if (cfg.limit_velocity && !meta.odrive_velocity_limit) missing.push_back("velocity");
  if (cfg.limit_effort && !meta.odrive_effort_limit) missing.push_back("effort");
  if (cfg.limit_acceleration && !meta.odrive_accel_limit) missing.push_back("acceleration");
  if (!missing.empty()) {
    ok = false;
    detail << "missing_odrive_limits:";
    for (size_t i = 0; i < missing.size(); ++i) {
      detail << missing[i];
      if (i + 1 < missing.size()) detail << ",";
    }
    detail << ";";
  }

  // Compare URDF-implied limits against values reported over CAN.
  auto mismatch = [&](double expected_actuator, double odrive_actuator, double tol) {
    if (node_status_.limits_check_use_sdo &&
        node_status_.limit_check_config.mode == LimitCheckConfig::Mode::WARN_ONLY) {
      return (odrive_actuator + tol) < expected_actuator;
    }
    return std::fabs(expected_actuator - odrive_actuator) > tol;
  };
  if (cfg.limit_velocity && meta.odrive_velocity_limit) {
    const double expected = joint_vel_to_actuator(idx, *cfg.limit_velocity);
    const double tol = std::fabs(expected) * node_status_.limit_check_config.velocity_tolerance_ratio;
    if (mismatch(expected, *meta.odrive_velocity_limit, tol)) {
      ok = false;
      detail << "vel_mismatch expected_actuator=" << expected << " odrive=" << *meta.odrive_velocity_limit
             << " tol=" << tol << ";";
    }
  }
  if (cfg.limit_effort && meta.odrive_effort_limit) {
    const double expected = joint_effort_to_actuator(idx, *cfg.limit_effort);
    const double tol = std::fabs(expected) * node_status_.limit_check_config.effort_tolerance_ratio;
    if (mismatch(expected, *meta.odrive_effort_limit, tol)) {
      ok = false;
      detail << "effort_mismatch expected_actuator=" << expected << " odrive=" << *meta.odrive_effort_limit
             << " tol=" << tol << ";";
    }
  }
  if (cfg.limit_acceleration && meta.odrive_accel_limit) {
    const double exp_acc = joint_vel_to_actuator(idx, *cfg.limit_acceleration);
    const double tol = std::fabs(exp_acc) * node_status_.limit_check_config.acceleration_tolerance_ratio;
    if (mismatch(exp_acc, *meta.odrive_accel_limit, tol)) {
      ok = false;
      detail << "accel_mismatch expected_actuator=" << exp_acc << " odrive=" << *meta.odrive_accel_limit
             << " tol=" << tol << ";";
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
    axis_states_[idx].health = AxisHealth::WARNING;
  } else {
    meta.limit_check_result = AxisRuntimeMetadata::LimitCheckResult::OK;
    meta.limit_check_detail.clear();
  }
  return ok || node_status_.limit_check_config.mode == LimitCheckConfig::Mode::WARN_ONLY;
}

void OdriveS1CanSystem::publish_status_if_due(const rclcpp::Time &stamp) {
  const double rate = node_status_.status_publish_rate_hz;
  if (rate <= 0.0) return;
  last_status_publish_time_ = stamp;

  for (size_t i = 0; i < axis_states_.size(); ++i) {
    auto &state = axis_states_[i];
    if (stamp.get_clock_type() == state.last_heartbeat.get_clock_type()) {
      const double age = (stamp - state.last_heartbeat).seconds();
      state.heartbeat_age = age > 0.0 ? age : 0.0;
    } else {
      state.heartbeat_age = 0.0;
    }
    state.health_numeric = static_cast<double>(health_to_int(state.health));
    state.axis_state_report = static_cast<double>(state.axis_state);
    state.power_state_numeric = static_cast<double>(power_state_to_int(derive_power_state(state)));
    const uint32_t fault = state.axis_error | state.motor_error | state.controller_error | state.encoder_error |
        state.disarm_reason;
    state.fault_code = static_cast<double>(fault);
  }
  populate_status_messages();
  if (status_pub_) {
    for (const auto &msg : hardware_status_cache_) {
      std::string line = msg.name;
      if (!msg.values.empty()) {
        line.append(" ");
        for (const auto &kv : msg.values) {
          line.append(kv.key).append("=").append(kv.value).append(";");
        }
      }
      std_msgs::msg::String out;
      out.data = line;
      status_pub_->publish(out);
    }
  }
}

uint32_t OdriveS1CanSystem::axis_can_id(size_t idx) const {
  if (idx >= axis_can_ids_.size()) return 0;
  return axis_can_ids_[idx];
}

void OdriveS1CanSystem::populate_status_messages() {
  if (hardware_status_cache_.size() != axis_states_.size()) {
    hardware_status_cache_.assign(axis_states_.size(), HardwareStatusMsg{});
    if constexpr (requires { typename HardwareStatusMsg::KeyValue{}; }) {
      constexpr size_t kValueCount = 5;
      for (auto &msg : hardware_status_cache_) {
        msg.values.resize(kValueCount);
        msg.values[0].key = "axis_state";
        msg.values[1].key = "fault_code";
        msg.values[2].key = "health_detail";
        msg.values[3].key = "heartbeat_age";
        msg.values[4].key = "homing_status";
      }
    }
  }
  for (size_t i = 0; i < axis_states_.size(); ++i) {
    auto &msg = hardware_status_cache_[i];
    if constexpr (requires(HardwareStatusMsg m) { m.name = axis_configs_[i].joint_name; }) {
      msg.name = axis_configs_[i].joint_name;
    }
    set_status_field(msg, static_cast<uint8_t>(health_to_int(axis_states_[i].health)));
    set_power_state_field(msg, static_cast<uint8_t>(power_state_to_int(derive_power_state(axis_states_[i]))));
    std::string msg_text = "mode=";
    msg_text.append(mode_to_string(command_modes_[i]));
    set_message_field(msg, msg_text);

    if constexpr (requires { typename HardwareStatusMsg::KeyValue{}; }) {
      auto add_numeric = [&](const std::string &key, double val) {
        char buf[32];
        const int len = std::snprintf(buf, sizeof(buf), "%.6g", val);
        std::string value;
        if (len > 0) value.assign(buf, static_cast<size_t>(len));
        return value;
      };
      msg.values[0].value = add_numeric("axis_state", axis_states_[i].axis_state);
      msg.values[1].value = add_numeric("fault_code", axis_states_[i].fault_code);
      msg.values[2].value = runtime_metadata_[i].fault_detail;
      msg.values[3].value = add_numeric("heartbeat_age", axis_states_[i].heartbeat_age);
      msg.values[4].value = add_numeric("homing_status", axis_states_[i].homing_status);
    }
  }
}

bool OdriveS1CanSystem::start_homing() {
  auto logger = rclcpp::get_logger("OdriveS1CanSystem");
  bool ok = true;
  for (size_t i = 0; i < axis_configs_.size(); ++i) {
    RCLCPP_INFO(
        logger,
        "Requesting homing for axis %s (node %u)",
        axis_configs_[i].joint_name.c_str(), axis_can_id(i));
    ok &= send_homing_request(i);
    command_modes_[i] = AxisControlMode::HOMING;
    runtime_metadata_[i].sent_closed_loop = false;
    runtime_metadata_[i].awaiting_closed_loop = false;
    runtime_metadata_[i].awaiting_idle = false;
    runtime_metadata_[i].homing_requested = true;
    runtime_metadata_[i].last_homing_result = "requested";
    runtime_metadata_[i].last_logged_homing_status.reset();
    runtime_metadata_[i].last_logged_axis_state.reset();
    axis_states_[i].homing_status = 1.0;
  }
  return ok;
}

} // namespace odrive_ros2_control

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
