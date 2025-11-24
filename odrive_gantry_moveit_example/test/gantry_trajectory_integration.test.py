"""
Integration test for the ODrive Gantry.

This test launches the full robot stack (simulated) and verifies that the
gantry can execute a trajectory using the FollowJointTrajectory action.
"""
import os
import time

import launch
import launch_ros.actions
import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_testing.actions import ReadyToTest
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration

JOINT_NAMES = ["x_joint", "y_joint", "yaw_joint"]
TARGET_POSITIONS = [-0.1, -0.15, -0.6]


@pytest.mark.launch_test
def generate_test_description():
    """
    Generate the launch description for the integration test.

    Returns:
        tuple: (LaunchDescription, dict) containing the test launch description and context.
    """
    pkg_share = FindPackageShare("odrive_gantry_moveit_example")
    xacro_path = PathJoinSubstitution([pkg_share, "description", "urdf", "gantry.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_path,
            " pkg_share:=",
            pkg_share,
            " use_simulated_odrive:=true",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        output="both",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    traj = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delayed_traj = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb, on_exit=[traj])
    )

    ld = LaunchDescription([control_node, rsp, jsb, delayed_traj, ReadyToTest()])
    return ld, {"control_node": control_node}


class TrajectoryHarness:
    """
    Test harness for sending trajectories and verifying robot state.

    This class wraps the ActionClient for FollowJointTrajectory and the subscription
    to /joint_states to make testing easier.
    """
    def __init__(self, node: RclpyNode, executor: SingleThreadedExecutor):
        self._node = node
        self._executor = executor
        self._latest_state = None
        self._sub = node.create_subscription(JointState, "/joint_states", self._state_cb, 10)
        self._client = ActionClient(node, FollowJointTrajectory, "/gantry_controller/follow_joint_trajectory")

    def _state_cb(self, msg: JointState):
        self._latest_state = msg

    def wait_for_ready(self, timeout_sec=15.0):
        end = time.time() + timeout_sec
        while time.time() < end and not self._client.wait_for_server(timeout_sec=0.5):
            self._executor.spin_once(timeout_sec=0.1)
        return self._client.server_is_ready()

    def send_and_wait(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[0.25, 0.2, 0.4],
                velocities=[0.0, 0.0, 0.0],
                time_from_start=Duration(seconds=2, nanoseconds=500_000_000).to_msg(),
            ),
            JointTrajectoryPoint(
                positions=TARGET_POSITIONS,
                velocities=[0.0, 0.0, 0.0],
                time_from_start=Duration(seconds=5).to_msg(),
            ),
        ]
        send_future = self._client.send_goal_async(goal)
        self._executor.spin_until_future_complete(send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        assert goal_handle is not None and goal_handle.accepted, "Goal rejected by controller"

        result_future = goal_handle.get_result_async()
        self._executor.spin_until_future_complete(result_future, timeout_sec=15.0)
        assert result_future.result().status == GoalStatus.STATUS_SUCCEEDED, f"Goal finished with status {result_future.result().status}"

    def assert_final_pose(self, tolerance=0.02, timeout_sec=5.0):
        end = time.time() + timeout_sec
        while time.time() < end:
            self._executor.spin_once(timeout_sec=0.1)
            if self._latest_state is None:
                continue

            idx = {name: i for i, name in enumerate(self._latest_state.name)}
            all_ok = True
            for i, joint in enumerate(JOINT_NAMES):
                if joint not in idx:
                    all_ok = False
                    break
                pos = self._latest_state.position[idx[joint]]
                target = TARGET_POSITIONS[i]
                if abs(pos - target) >= tolerance:
                    all_ok = False
                    break
            
            if all_ok:
                return

        assert self._latest_state is not None, "No joint_states received"
        idx = {name: i for i, name in enumerate(self._latest_state.name)}
        for i, joint in enumerate(JOINT_NAMES):
            assert joint in idx, f"Joint {joint} not found in state"
            pos = self._latest_state.position[idx[joint]]
            target = TARGET_POSITIONS[i]
            assert abs(pos - target) < tolerance, f"{joint} final pos {pos} outside tolerance {tolerance}"


@pytest.mark.launch_test
def test_gantry_executes_trajectory(launch_service, control_node, proc_output):
    """
    Test that the gantry can execute a simple trajectory.

    Steps:
    1. Wait for the action server to be ready.
    2. Send a goal with two points.
    3. Wait for the goal to complete.
    4. Verify the final pose matches the target within tolerance.
    """
    rclpy.init()
    try:
        node = rclpy.create_node("gantry_traj_tester")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        harness = TrajectoryHarness(node, executor)
        assert harness.wait_for_ready(), "Action server not ready"
        harness.send_and_wait()
        harness.assert_final_pose()
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        rclpy.shutdown()
