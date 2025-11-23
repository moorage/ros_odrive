# ODrive ros2_control Plugin

This package serves as a hardware interface to control ODrives from [ros2_control](https://control.ros.org/master/index.html).

It assumes that the ODrive is already configured and calibrated (see [docs](https://docs.odriverobotics.com/v/latest/guides/getting-started.html) for details).

## Usage

Load `odrive_ros2_control_plugin/ODriveHardwareInterface` as a ros2_control `SystemInterface` plugin. Each joint maps to an ODrive S1 axis defined by `odrive_node_id` and (optional) `odrive_axis_index`.

## Highlights

- SocketCAN transport with per-axis routing and configurable interface name.
- Command interfaces: position, velocity, effort; state interfaces mirror these plus diagnostics.
- Command-mode switching with validation so only one control mode is active per joint.
- Mixed joint types (revolute/continuous/prismatic) with gear or lead-screw reduction; transmissions are detected when present.
- Fault monitoring from heartbeat/error fields, explicit clear-errors and homing services, and HardwareStatus + diagnostics output.
- Optional limit-consistency checks between URDF limits and ODrive-reported limits (STRICT / WARN_ONLY / OFF).
- CAN command de-duplication to reduce bus load and basic heartbeat timeout detection.

## Parameters

Top level:

- `can_interface` (or `can`): CAN interface device (e.g. `can0`).
- `status_publish_rate`: HardwareStatus publish rate [Hz].
- `heartbeat_timeout`: timeout for stale heartbeats [s].
- `command_tolerance`: threshold for resending unchanged commands.
- `limits_check.mode`: `OFF|WARN_ONLY|STRICT` plus tolerance ratios (`limits_check.velocity_tolerance_ratio`, `limits_check.effort_tolerance_ratio`, `limits_check.acceleration_tolerance_ratio`).

Per joint:

- `odrive_node_id` (or `node_id`)
- `odrive_axis_index` (default 0)
- Optional: `gear_ratio`, `lead_screw_pitch`, `torque_constant`, `max_velocity`, `max_effort`, `max_acceleration`.

## Command Interfaces

Controllers may claim one of:

- `position`
- `velocity`
- `effort` (torque/current)

## State Interfaces

- `position`
- `velocity`
- `effort`

## Tests

Unit and integration tests live under `test/` and are driven with `ament_cmake_gtest`, covering mapping/transmissions, mode switching, limit consistency, fault handling, and CAN routing with a simulated transport.

### Running tests locally

In a ROS 2 Humble workspace:

```bash
source /opt/ros/humble/setup.sh
mkdir -p ws/src
rsync -a . ws/src/ros_odrive
cd ws
colcon build --merge-install --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install
colcon test-result --verbose
```

### CI

GitHub Actions (`.github/workflows/ci.yml`) builds and runs the unit/integration test suite inside `ros:humble-ros-base` on every push/PR, ensuring canonical behavior stays covered.
