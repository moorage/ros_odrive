#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "can_simple_messages.hpp"
#include "odrive_ros2_control/odrive_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

using odrive_ros2_control::AxisControlMode;
using odrive_ros2_control::CanTransport;
using odrive_ros2_control::OdriveS1CanSystem;

namespace odrive_gantry_moveit_example {

/**
 * @brief A simulated CAN transport for ODrive.
 *
 * This class mocks the behavior of a real CAN bus connection to ODrive(s).
 * It maintains the state of simulated axes and updates their physics in the poll() loop.
 */
class SimulatedOdriveTransport final : public CanTransport {
public:
  SimulatedOdriveTransport() = default;

  /**
   * @brief Initializes the transport.
   *
   * @param interface_name The name of the CAN interface (unused in simulation).
   * @param cb Callback function to handle incoming CAN frames from the "drive".
   * @return true always.
   */
  bool init(const std::string &interface_name, std::function<void(const can_frame &)> cb) override {
    std::scoped_lock lock(mutex_);
    callback_ = std::move(cb);
    last_update_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(logger_, "Initializing simulated ODrive transport on interface '%s' (ignored in simulation).", interface_name.c_str());
    return true;
  }

  void shutdown() override {
    std::scoped_lock lock(mutex_);
    axes_.clear();
    callback_ = nullptr;
  }

  /**
   * @brief Sends a CAN frame to the simulated drive.
   *
   * Parses the frame to update the simulated axis state (control mode, targets, etc.).
   *
   * @param frame The CAN frame to send.
   * @return true if the callback is registered, false otherwise.
   */
  bool send(const can_frame &frame) override {
    std::scoped_lock lock(mutex_);
    if (!callback_) return false;
    const uint32_t axis_id = frame.can_id >> 5;
    auto &axis = axes_[axis_id];
    axis.last_update = std::chrono::steady_clock::now();

    const uint8_t cmd = frame.can_id & 0x1f;
    switch (cmd) {
      case Set_Input_Pos_msg_t::cmd_id: {
        Set_Input_Pos_msg_t msg{};
        msg.decode_buf(frame.data);
        axis.target_position = msg.Input_Pos;
        axis.target_velocity = msg.Vel_FF;
        axis.feedforward_torque = msg.Torque_FF;
        axis.mode = AxisMode::POSITION;
        axis.axis_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      } break;
      case Set_Input_Vel_msg_t::cmd_id: {
        Set_Input_Vel_msg_t msg{};
        msg.decode_buf(frame.data);
        axis.target_velocity = msg.Input_Vel;
        axis.feedforward_torque = msg.Input_Torque_FF;
        axis.mode = AxisMode::VELOCITY;
        axis.axis_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      } break;
      case Set_Input_Torque_msg_t::cmd_id: {
        Set_Input_Torque_msg_t msg{};
        msg.decode_buf(frame.data);
        axis.feedforward_torque = msg.Input_Torque;
        axis.mode = AxisMode::TORQUE;
        axis.axis_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      } break;
      case Set_Axis_State_msg_t::cmd_id: {
        Set_Axis_State_msg_t msg{};
        msg.decode_buf(frame.data);
        axis.axis_state = msg.Axis_Requested_State;
        if (axis.axis_state == AXIS_STATE_IDLE) {
          axis.target_velocity = 0.0;
        } else if (axis.axis_state == AXIS_STATE_HOMING) {
          axis.position = 0.0;
          axis.target_position = 0.0;
        }
      } break;
      case Set_Controller_Mode_msg_t::cmd_id: {
        // Keep this so the driver can switch control modes without errors.
        Set_Controller_Mode_msg_t msg{};
        msg.decode_buf(frame.data);
        axis.mode = static_cast<AxisMode>(msg.Control_Mode);
      } break;
      case Set_Limits_msg_t::cmd_id: {
        Set_Limits_msg_t msg{};
        msg.decode_buf(frame.data);
        axis.velocity_limit = msg.Velocity_Limit;
        axis.current_limit = msg.Current_Limit;
      } break;
      case Set_Traj_Accel_Limits_msg_t::cmd_id: {
        Set_Traj_Accel_Limits_msg_t msg{};
        msg.decode_buf(frame.data);
        axis.accel_limit = msg.Traj_Accel_Limit;
      } break;
      default:
        RCLCPP_WARN_THROTTLE(
            logger_, *logger_clock_, 2000,
            "Simulated transport ignoring unknown CAN command 0x%02x for axis %u.",
            cmd, axis_id);
        break;
    }
    return true;
  }

  /**
   * @brief Updates the simulation physics and sends feedback frames.
   *
   * This method should be called periodically. It calculates the new position/velocity
   * based on the control mode and limits, then invokes the callback with status messages
   * (Heartbeat, Encoder Estimates, Torques).
   */
  void poll() override {
    std::vector<can_frame> frames_to_publish;
    std::function<void(const can_frame &)> callback_copy;
    {
      std::scoped_lock lock(mutex_);
      if (!callback_) return;
      callback_copy = callback_;
      const auto now = std::chrono::steady_clock::now();

      for (auto &[axis_id, axis] : axes_) {
        const double dt = cycle_period_sec_ > 0.0
                              ? cycle_period_sec_
                              : std::chrono::duration<double>(now - axis.last_update).count();
        axis.last_update = now;
        const double velocity_limit = axis.velocity_limit > 0.0 ? axis.velocity_limit : default_velocity_limit_;
        const double accel_limit = axis.accel_limit > 0.0 ? axis.accel_limit : default_accel_limit_;

        double desired_vel = axis.target_velocity;
        if (axis.mode == AxisMode::POSITION) {
          const double error = axis.target_position - axis.position;
          // Simple critically damped position loop toward the target to mimic the ODrive internal controller.
          desired_vel = std::clamp(error * position_gain_, -velocity_limit, velocity_limit);
        }

        const double dv = desired_vel - axis.velocity;
        const double max_delta = accel_limit * dt;
        // Apply acceleration limit to velocity change
        if (std::abs(dv) > max_delta) {
          axis.velocity += (dv > 0 ? 1.0 : -1.0) * max_delta;
        } else {
          axis.velocity = desired_vel;
        }
        axis.velocity = std::clamp(axis.velocity, -velocity_limit, velocity_limit);

        // Integrate velocity to get position
        axis.position += axis.velocity * dt;

        Heartbeat_msg_t hb{};
        hb.Axis_Error = axis.axis_error;
        hb.Axis_State = axis.axis_state;
        can_frame hb_frame{};
        hb_frame.can_id = (axis_id << 5) | hb.cmd_id;
        hb_frame.can_dlc = hb.msg_length;
        hb.encode_buf(hb_frame.data);
        frames_to_publish.push_back(hb_frame);

        Get_Encoder_Estimates_msg_t enc{};
        enc.Pos_Estimate = axis.position;
        enc.Vel_Estimate = axis.velocity;
        can_frame enc_frame{};
        enc_frame.can_id = (axis_id << 5) | enc.cmd_id;
        enc_frame.can_dlc = enc.msg_length;
        enc.encode_buf(enc_frame.data);
        frames_to_publish.push_back(enc_frame);

        Get_Torques_msg_t tq{};
        tq.Iq_Measured = axis.feedforward_torque;
        can_frame tq_frame{};
        tq_frame.can_id = (axis_id << 5) | tq.cmd_id;
        tq_frame.can_dlc = tq.msg_length;
        tq.encode_buf(tq_frame.data);
        frames_to_publish.push_back(tq_frame);
      }
    }

    // Publish frames outside the lock to avoid holding the mutex while calling back into the driver.
    for (const auto &frame : frames_to_publish) {
      callback_copy(frame);
    }
  }

  void set_cycle_period(const rclcpp::Duration &period) {
    std::scoped_lock lock(mutex_);
    cycle_period_sec_ = std::max(0.0, period.seconds());
  }

private:
  enum class AxisMode { POSITION = CONTROL_MODE_POSITION_CONTROL, VELOCITY = CONTROL_MODE_VELOCITY_CONTROL, TORQUE = CONTROL_MODE_TORQUE_CONTROL };

  struct Axis {
    double position = 0.0;
    double velocity = 0.0;
    double target_position = 0.0;
    double target_velocity = 0.0;
    double feedforward_torque = 0.0;
    double velocity_limit = 5.0;
    double accel_limit = 10.0;
    double current_limit = 10.0;
    uint32_t axis_error = 0;
    uint8_t axis_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
    AxisMode mode = AxisMode::POSITION;
    std::chrono::steady_clock::time_point last_update = std::chrono::steady_clock::now();
  };

  std::function<void(const can_frame &)> callback_;
  std::unordered_map<uint32_t, Axis> axes_;
  std::chrono::steady_clock::time_point last_update_{std::chrono::steady_clock::now()};
  std::mutex mutex_;
  double cycle_period_sec_{0.0};
  const double default_velocity_limit_ = 8.0; // turns/s
  const double default_accel_limit_ = 20.0;   // turns/s^2
  const double position_gain_ = 8.0;          // simple P gain in turns/s per turn
  rclcpp::Logger logger_{rclcpp::get_logger("SimulatedOdriveTransport")};
  rclcpp::Clock::SharedPtr logger_clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
};

/**
 * @brief Hardware interface for Simulated ODrive.
 *
 * This plugin allows ros2_control to operate against a simulated ODrive
 * without requiring physical hardware or a real CAN interface.
 */
class SimulatedOdriveHardware final : public OdriveS1CanSystem {
public:
  SimulatedOdriveHardware() : OdriveS1CanSystem([this]() {
    auto transport = std::make_shared<SimulatedOdriveTransport>();
    transport_for_period_ = transport;
    return transport;
  }) {}

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    if (auto transport = transport_for_period_.lock()) {
      transport->set_cycle_period(period);
    }
    return OdriveS1CanSystem::read(time, period);
  }

  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    if (auto transport = transport_for_period_.lock()) {
      transport->set_cycle_period(period);
    }
    return OdriveS1CanSystem::write(time, period);
  }

private:
  std::weak_ptr<SimulatedOdriveTransport> transport_for_period_;
};

}  // namespace odrive_gantry_moveit_example

PLUGINLIB_EXPORT_CLASS(odrive_gantry_moveit_example::SimulatedOdriveHardware, hardware_interface::SystemInterface)
