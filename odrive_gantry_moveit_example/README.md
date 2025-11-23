# ODrive Gantry MoveIt Example

A complete MoveIt 2 example for a 2-DOF gantry + yaw wrist robot, demonstrating ODrive integration with `ros2_control`.

## Features

- **First-Class MoveIt Integration**: Treats ODrive axes as native ROS 2 joints using `hardware_interface::SystemInterface`.
- **Simulation Support**: Includes a `SimulatedOdriveHardware` plugin that mocks CAN bus physics, allowing development without hardware.
- **ros2_control**: Uses `JointTrajectoryController` for smooth path execution and `JointStateBroadcaster` for robot state reporting.
- **Integration Testing**: Automated tests verify the full control stack from MoveIt planning to simulated hardware execution.

## Key Files

- **`launch/bringup.launch.py`**: The main entry point. Loads the URDF, starts `ros2_control_node`, MoveIt `move_group`, and spawns controllers.
- **`src/simulated_odrive_hardware.cpp`**: A mock hardware interface that simulates ODrive physics (position/velocity loops) and CAN communication.
- **`config/ros2_controllers.yaml`**: Configures the `gantry_controller` (JointTrajectoryController) gains and constraints.
- **`test/gantry_trajectory_integration.test.py`**: Pytest-based integration test that launches the system and executes a trajectory.

## Parameters

### Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `use_simulated_odrive` | `true` | Set to `true` to use the internal simulator. Set to `false` for real hardware. |
| `can_interface` | `can0` | The CAN interface name (e.g., `can0`, `vcan0`) when using real hardware. |

### Controller Configuration (`config/ros2_controllers.yaml`)

- **`gantry_controller`**:
    - `joints`: `x_joint`, `y_joint`, `yaw_joint`
    - `gains`: PID gains for the simulated plant (P=20.0, D=2.0 for gantry; P=8.0, D=0.5 for yaw).
    - `constraints`: Goal tolerance and trajectory execution constraints.

## Usage

### 1. Build the Package

```bash
colcon build --symlink-install --packages-up-to odrive_gantry_moveit_example
source install/setup.bash
```

### 2. Run in Simulation (Default)

Launch the system with the simulated hardware backend:

```bash
ros2 launch odrive_gantry_moveit_example bringup.launch.py
```

### 3. Run on Real Hardware

To control real ODrive axes (Node IDs 10, 11, 12 as defined in URDF), specify the CAN interface and disable simulation:

```bash
ros2 launch odrive_gantry_moveit_example bringup.launch.py use_simulated_odrive:=false can_interface:=can0
```

## Testing

Run the integration test to verify the system:

```bash
colcon test --packages-select odrive_gantry_moveit_example
colcon test-result --verbose
```
