# ODrive Gantry MoveIt Example

Simple 2-DOF gantry + yaw wrist MoveIt2 example that treats the ODrive ros2_control plugin as a first-class MoveIt hardware backend. The package ships with:

- URDF/Xacro describing a planar gantry (`x_joint`, `y_joint`) with a yaw tool joint.
- ros2_control wiring for `joint_state_broadcaster` + `joint_trajectory_controller`, using either real ODrive hardware or the simulated CAN transport.
- SRDF + MoveIt controller config so MoveIt can plan and execute via the joint trajectory controller.
- Integration test that runs the controller stack against the simulated ODrive server and executes a short trajectory.

## Running the demo

```bash
colcon build --symlink-install --packages-up-to odrive_gantry_moveit_example
source install/setup.bash
ros2 launch odrive_gantry_moveit_example bringup.launch.py use_simulated_odrive:=true
```

Switch `use_simulated_odrive:=false` and set `can_interface` to talk to real hardware (ODrive node IDs 10/11 as defined in the Xacro).

## Running the integration test

```
colcon test --packages-select odrive_gantry_moveit_example
colcon test-result --verbose
```
