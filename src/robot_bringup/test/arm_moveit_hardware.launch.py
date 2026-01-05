#!/usr/bin/env python3
"""
Arm + Gripper MoveIt Hardware Launch File
- MyActuator RMD motors via CAN (arm)
- Dynamixel via OpenCR serial (gripper)
- MoveIt motion planning
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # Package paths
    moveit_config_pkg = get_package_share_directory("mobile_manipulator_moveit_config")
    description_pkg = get_package_share_directory("robot_description")

    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("gripper_baudrate", default_value="115200"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    can_interface = LaunchConfiguration("can_interface")
    gripper_port = LaunchConfiguration("gripper_port")
    gripper_baudrate = LaunchConfiguration("gripper_baudrate")
    rviz_arg = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Robot description (using arm_gripper_hardware.xacro)
    urdf_path = os.path.join(description_pkg, "urdf", "arm_gripper_hardware.xacro")
    robot_description_content = Command([
        'xacro ', urdf_path,
        ' can_interface:=', can_interface,
        ' gripper_port:=', gripper_port,
        ' gripper_baudrate:=', gripper_baudrate,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    srdf_path = os.path.join(moveit_config_pkg, "config", "mobile_manipulator.srdf")
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Load YAML configs
    kinematics_yaml = load_yaml("mobile_manipulator_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mobile_manipulator_moveit_config", "config/joint_limits.yaml")
    moveit_controllers_yaml = load_yaml("mobile_manipulator_moveit_config", "config/moveit_controllers.yaml")

    # MoveIt parameters
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml.get("moveit_simple_controller_manager", {}),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl.planning_plugin": "ompl_interface/OMPLPlanner",
        "ompl.request_adapters": "default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints default_planner_request_adapters/AddTimeOptimalParameterization",
        "ompl.start_state_max_bounds_error": 0.1,
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.0,
        "trajectory_execution.wait_for_trajectory_completion_timeout": -1.0,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Controller config path
    ros2_controllers_path = os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml")

    # Nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    rviz_config = os.path.join(moveit_config_pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(rviz_arg),
        output="log",
    )

    # Sequencing: JSB -> arm -> gripper -> move_group
    delay_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    delay_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    delay_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            ros2_control_node,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            delay_arm,
            delay_gripper,
            delay_move_group,
            delay_rviz,
        ]
    )
