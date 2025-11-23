#pragma once

#include <functional>
#include <linux/can.h>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "can_simple_messages.hpp"
#include "control_msgs/msg/hardware_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace odrive_ros2_control {

/**
 * @brief Represents the health status of an axis.
 */
enum class AxisHealth { OK, WARNING, ERROR };

/**
 * @brief Represents the control mode of an axis.
 */
enum class AxisControlMode { IDLE, POSITION, VELOCITY, EFFORT, HOMING };

/**
 * @brief Configuration for a single axis.
 *
 * This struct holds static configuration parameters for an axis, typically loaded from the URDF/ros2_control XML.
 */
struct AxisConfig {
  std::string joint_name;
  std::string joint_type;
  int node_id = 0;
  int axis_index = 0;
  std::optional<double> gear_ratio;
  std::optional<double> lead_screw_pitch;
  std::optional<double> torque_constant;
  bool has_transmission = false;
  std::optional<double> limit_velocity;
  std::optional<double> limit_effort;
  std::optional<double> limit_acceleration;
};

/**
 * @brief Runtime state of a single axis.
 *
 * This struct holds the dynamic state of an axis, updated from CAN feedback.
 */
struct AxisState {
  double pos_actuator = 0.0;
  double vel_actuator = 0.0;
  double torque_actuator = 0.0;
  double pos_joint = 0.0;
  double vel_joint = 0.0;
  double effort_joint = 0.0;
  AxisHealth health = AxisHealth::OK;
  uint32_t axis_error = 0;
  uint32_t motor_error = 0;
  uint32_t controller_error = 0;
  uint32_t encoder_error = 0;
  uint32_t disarm_reason = 0;
  uint8_t axis_state = 0;
  bool heartbeat_stale = false;
  rclcpp::Time last_heartbeat;
};

/**
 * @brief Configuration for limit checking.
 */
struct LimitCheckConfig {
  enum class Mode { OFF, WARN_ONLY, STRICT };
  Mode mode = Mode::OFF;
  double velocity_tolerance_ratio = 0.1;
  double effort_tolerance_ratio = 0.1;
  double acceleration_tolerance_ratio = 0.2;
};

/**
 * @brief Abstract base class for CAN transport.
 *
 * Allows for dependency injection of the CAN transport layer, facilitating testing and different CAN implementations.
 */
class CanTransport {
public:
  virtual ~CanTransport() = default;
  /**
   * @brief Initialize the CAN interface.
   * @param iface The name of the CAN interface (e.g., "can0").
   * @param cb Callback function to handle received CAN frames.
   * @return true if initialization was successful, false otherwise.
   */
  virtual bool init(const std::string &iface, std::function<void(const can_frame &)> cb) = 0;
  /**
   * @brief Shutdown the CAN interface.
   */
  virtual void shutdown() = 0;
  /**
   * @brief Send a CAN frame.
   * @param frame The CAN frame to send.
   * @return true if the frame was sent successfully, false otherwise.
   */
  virtual bool send(const can_frame &frame) = 0;
  /**
   * @brief Poll for new CAN frames.
   *
   * This function should be called periodically to process incoming frames if the implementation requires polling.
   */
  virtual void poll() = 0;
};

using CanTransportFactory = std::function<std::shared_ptr<CanTransport>()>;

/**
 * @brief Hardware interface for ODrive S1 over CAN.
 *
 * This class implements the ros2_control SystemInterface for ODrive S1 controllers connected via CAN.
 * It handles communication, state updates, and command sending.
 */
class OdriveS1CanSystem : public hardware_interface::SystemInterface {
public:
  OdriveS1CanSystem();
  explicit OdriveS1CanSystem(CanTransportFactory factory);

  static void set_transport_factory_for_tests(const CanTransportFactory &factory);

  const std::vector<AxisState> &debug_axis_states() const { return axis_states_; }
  const std::vector<AxisConfig> &debug_axis_configs() const { return axis_configs_; }
  const std::vector<AxisControlMode> &debug_modes() const { return command_modes_; }

  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  // Test hooks
  void inject_transport(const std::shared_ptr<CanTransport> &transport) { transport_ = transport; }
  bool start_homing_for_tests() { return start_homing(); }

private:
  friend class OdriveSystemTestAccess;
  struct AxisCommand {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
  };

  struct NodeStatus {
    double status_publish_rate_hz = 5.0;
    double heartbeat_timeout_sec = 0.5;
    double can_utilization_limit = 0.85;
    double command_tolerance = 1e-4;
    int max_frames_per_cycle = 0;      // 0 = unlimited
    double max_frames_per_sec = 0.0;   // 0 = unlimited
    double write_latency_warn_sec = 0.02;
    bool fault_idle_on_error = true;
    bool require_rearm_after_fault = true;
    bool limits_check_use_sdo = false;
    std::string flat_endpoints_path;
    double sdo_timeout_sec = 0.5;
    LimitCheckConfig limit_check_config;
    std::string can_interface;
    uint32_t can_bitrate = 0;
    std::string default_mode = "position";
  };

  struct AxisRuntimeMetadata {
    double last_cmd_pos = 0.0;
    double last_cmd_vel = 0.0;
    double last_cmd_effort = 0.0;
    bool sent_closed_loop = false;
    std::optional<double> odrive_velocity_limit;
    std::optional<double> odrive_effort_limit;
    std::optional<double> odrive_accel_limit;
    bool fault_latched = false;
    bool requires_rearm = false;
    bool idle_sent_on_fault = false;
    uint32_t disarm_reason = 0;
    uint8_t last_axis_state = 0;
    std::string last_homing_result = "unknown";
    enum class LimitCheckResult { UNKNOWN, OK, WARN, ERROR, SKIPPED_NO_DATA };
    LimitCheckResult limit_check_result = LimitCheckResult::UNKNOWN;
    std::string limit_check_detail;
  };

  AxisControlMode string_to_mode(const std::string &mode) const;
  std::string mode_to_string(AxisControlMode mode) const;

  bool validate_parameters();
  void load_flat_endpoints();
  struct EndpointInfo {
    uint16_t id = 0;
    std::string type;
  };
  std::optional<EndpointInfo> endpoint_for_axis(size_t axis_index, const std::string &suffix) const;
  std::optional<EndpointInfo> vel_limit_endpoint_for_axis(size_t axis_index) const;
  std::optional<EndpointInfo> current_limit_endpoint_for_axis(size_t axis_index) const;
  std::optional<EndpointInfo> accel_limit_endpoint_for_axis(size_t axis_index) const;

  // Decode a CAN frame and update cached axis state.
  void handle_frame(const can_frame &frame, const rclcpp::Time &stamp);
  // Process heartbeat to track axis health and liveness.
  void process_heartbeat(size_t idx, const Heartbeat_msg_t &msg, const rclcpp::Time &stamp);
  void process_encoder_estimate(size_t idx, const Get_Encoder_Estimates_msg_t &msg);
  void process_torque_feedback(size_t idx, const Get_Torques_msg_t &msg);
  void process_sdo_response(const can_frame &frame, const rclcpp::Time &stamp);
  void update_health(size_t idx, bool heartbeat_refresh);
  void latch_fault(size_t idx, uint32_t axis_error);
  bool clear_errors_and_rearm(size_t idx);
  std::optional<double> read_endpoint_via_sdo(int node_id, const EndpointInfo &ep);
  double decode_sdo_value(const EndpointInfo &ep, uint32_t raw) const;

  // Unit conversion helpers between actuator turns/torque and joint units.
  double actuator_to_joint_pos(const AxisConfig &cfg, double turns) const;
  double joint_to_actuator_pos(const AxisConfig &cfg, double joint_pos) const;
  double actuator_vel_to_joint(const AxisConfig &cfg, double turns_per_sec) const;
  double joint_vel_to_actuator(const AxisConfig &cfg, double joint_vel) const;
  double actuator_effort_to_joint(const AxisConfig &cfg, double torque) const;
  double joint_effort_to_actuator(const AxisConfig &cfg, double effort) const;

  bool send_axis_state(size_t idx, uint32_t requested_state);
  bool send_control_mode(size_t idx, uint8_t control_mode, bool require_closed_loop);
  bool send_position_command(size_t idx, double turns, double vel_ff, double torque_ff);
  bool send_velocity_command(size_t idx, double turns_per_sec, double torque_ff);
  bool send_torque_command(size_t idx, double torque);
  bool send_clear_errors(size_t idx);
  bool send_frame(const can_frame &frame, bool count_budget);
  bool can_send_frame(const rclcpp::Time &now);

  // Compare URDF limits with ODrive-reported limits based on configured policy.
  bool run_limit_check(size_t idx);
  void publish_status_if_due(const rclcpp::Time &stamp);
  uint32_t axis_can_id(size_t idx) const;
  bool start_homing();

  static CanTransportFactory default_transport_factory();

  static CanTransportFactory transport_factory_override_;

  NodeStatus node_status_;
  std::vector<AxisConfig> axis_configs_;
  std::vector<AxisState> axis_states_;
  std::vector<AxisCommand> axis_commands_;
  std::vector<AxisControlMode> command_modes_;
  std::vector<AxisRuntimeMetadata> runtime_metadata_;
  std::vector<uint32_t> axis_can_ids_;
  std::unordered_map<uint32_t, size_t> can_id_lookup_;
  std::unordered_map<int, size_t> node_to_primary_axis_;
  std::string flat_endpoints_json_;
  std::vector<std::optional<EndpointInfo>> endpoint_vel_;
  std::vector<std::optional<EndpointInfo>> endpoint_effort_;
  std::vector<std::optional<EndpointInfo>> endpoint_accel_;
  struct RawSdoValue {
    uint32_t raw = 0;
    rclcpp::Time stamp;
  };
  std::unordered_map<uint16_t, RawSdoValue> sdo_responses_;
  std::mutex sdo_mutex_;

  std::shared_ptr<CanTransport> transport_;
  CanTransportFactory transport_factory_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<control_msgs::msg::HardwareStatus>::SharedPtr hw_status_pub_;
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_errors_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_errors_and_rearm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;

  rclcpp::Time last_status_publish_time_;
  bool active_ = false;
  bool configured_ = false;

  struct UtilizationMetrics {
    size_t frames_this_cycle = 0;
    size_t frames_last_cycle = 0;
    size_t frames_in_window = 0;
    size_t over_budget_events = 0;
    rclcpp::Time window_start;
    rclcpp::Time last_send_time;
    rclcpp::Time last_write_start;
    double frames_per_sec_ewma = 0.0;
    double avg_write_duration_sec = 0.0;
    bool budget_exceeded_last_cycle = false;
  } util_metrics_;
};

using ODriveHardwareInterface = OdriveS1CanSystem;

} // namespace odrive_ros2_control
