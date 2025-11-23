# ODrive ros2_control Plugin

This package serves as a hardware interface to control ODrives from [ros2_control](https://control.ros.org/master/index.html).

It assumes that the ODrive is already configured and calibrated (see [docs](https://docs.odriverobotics.com/v/latest/guides/getting-started.html) for details).

## Features

- **SocketCAN Transport**: Per-axis routing with configurable interface name.
- **Control Modes**: Position, Velocity, Effort (Torque/Current).
- **State Feedback**: Position, Velocity, Effort, plus diagnostics.
- **Safety**:
  - Command-mode switching validation.
  - Fault monitoring (heartbeat, errors).
  - Limit consistency checks (URDF vs ODrive).
  - CAN command de-duplication.
- **Joint Types**: Revolute, Continuous, Prismatic.
- **Transmissions**: Automatic detection of gear/lead-screw reductions.

## File Overview

- `include/odrive_ros2_control/odrive_system.hpp`: Main hardware interface class definition.
- `src/odrive_hardware_interface.cpp`: Implementation of the `SystemInterface` plugin.
- `test/unit_logic_test.cpp`: Unit tests for logic (mode switching, limits, etc.).
- `test/integration_can_test.cpp`: Integration tests using a fake CAN transport.
- `odrive_hardware_interface.xml`: Plugin export definition.

## Parameters

### Global Parameters
These are set at the top level of the hardware interface in URDF.

- `can_interface` (string): CAN interface name (e.g., `can0`).
- `status_publish_rate` (double): Rate [Hz] to publish `HardwareStatus`.
- `heartbeat_timeout` (double): Timeout [s] for stale heartbeats.
- `command_tolerance` (double): Threshold for resending unchanged commands.
- `limits_check.mode` (string): `OFF`, `WARN_ONLY`, or `STRICT`.
- `limits_check.velocity_tolerance_ratio` (double): Tolerance ratio for velocity limits.
- `limits_check.effort_tolerance_ratio` (double): Tolerance ratio for effort limits.
- `limits_check.acceleration_tolerance_ratio` (double): Tolerance ratio for acceleration limits.

### Per-Joint Parameters
These are set for each joint in the URDF.

- `odrive_node_id` (int): CAN node ID of the ODrive axis.
- `odrive_axis_index` (int, default 0): Axis index on the ODrive (0 or 1).
- `gear_ratio` (double, optional): Mechanical reduction ratio.
- `lead_screw_pitch` (double, optional): Pitch for prismatic joints.
- `torque_constant` (double, optional): Torque constant [Nm/A].
- `max_velocity` (double, optional): Max velocity limit.
- `max_effort` (double, optional): Max effort limit.
- `max_acceleration` (double, optional): Max acceleration limit.

## Usage

Load `odrive_ros2_control_plugin/ODriveHardwareInterface` as a ros2_control `SystemInterface` plugin.

### URDF Example

```xml
<ros2_control name="ODriveSystem" type="system">
  <hardware>
    <plugin>odrive_ros2_control_plugin/ODriveHardwareInterface</plugin>
    <param name="can_interface">can0</param>
    <param name="heartbeat_timeout">0.5</param>
  </hardware>
  <joint name="joint1">
    <param name="odrive_node_id">10</param>
    <param name="odrive_axis_index">0</param>
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

## Tests

Unit and integration tests are provided to ensure reliability.

### Running Tests Locally

In a ROS 2 Humble workspace:

```bash
colcon build --merge-install --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install
colcon test-result --verbose
```

### CI

GitHub Actions (`.github/workflows/ci.yml`) runs the test suite on every push/PR.
