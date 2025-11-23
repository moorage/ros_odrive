"""
Launch file for the ODrive Gantry MoveIt example.

This launch file brings up the entire robot system, including:
- Robot State Publisher (URDF)
- ros2_control node (Hardware Interface)
- MoveIt 2 (Move Group)
- Controllers (Joint State Broadcaster, Gantry Controller)

It supports both simulated hardware (default) and real ODrive hardware via CAN.
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the LaunchDescription for the ODrive Gantry.

    Returns:
        LaunchDescription: The launch description containing all nodes and arguments.
    """
    use_sim_hw = LaunchConfiguration("use_simulated_odrive")
    can_iface = LaunchConfiguration("can_interface")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_simulated_odrive",
            default_value="true",
            description="Use simulated ODrive CAN backend (no real hardware required).",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface name for real ODrive hardware.",
        ),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("odrive_gantry_moveit_example"), "description", "urdf", "gantry.urdf.xacro"]
            ),
            " use_simulated_odrive:=",
            use_sim_hw,
            " can_interface:=",
            can_iface,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    share_dir = get_package_share_directory("odrive_gantry_moveit_example")

    def load_yaml(rel_path):
        abs_path = os.path.join(share_dir, rel_path)
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("odrive_gantry_moveit_example"), "config", "ros2_controllers.yaml"]
    )

    moveit_params = {
        "robot_description_semantic": open(os.path.join(share_dir, "srdf", "gantry.srdf")).read(),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": load_yaml("config/moveit_controllers.yaml"),
        "ompl": load_yaml("config/ompl_planning.yaml"),
        "kinematics": load_yaml("config/kinematics.yaml"),
        "trajectory_execution": load_yaml("config/trajectory_execution.yaml"),
        "planning_pipelines": ["ompl"],
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[robot_description, moveit_params],
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    traj_ctrl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_controller", "--controller-manager", "/controller_manager"],
    )
    delayed_traj = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb, on_exit=[traj_ctrl])
    )

    nodes = [control_node, robot_state_pub_node, move_group, jsb, delayed_traj]
    return LaunchDescription(declared_arguments + nodes)
