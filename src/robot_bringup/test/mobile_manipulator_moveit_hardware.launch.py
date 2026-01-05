#!/usr/bin/env python3
"""
============================================================================
Mobile Manipulator Full Hardware Launch File
============================================================================

통합 하드웨어 런치 파일
- Arm: MyActuator RMD motors via CAN
- Gripper: Dynamixel via OpenCR serial
- Mobile Base: MD 4WD motor driver via serial
- MoveIt motion planning

[사용법]
ros2 launch robot_bringup mobile_manipulator_moveit_hardware.launch.py

[파라미터]
- can_interface: CAN 인터페이스 (default: can0)
- gripper_port: 그리퍼 시리얼 포트 (default: /dev/ttyACM1)
- port_front: 앞바퀴 모터 드라이버 포트 (default: /dev/ttyUSB0)
- port_rear: 뒷바퀴 모터 드라이버 포트 (default: /dev/ttyUSB1)
- rviz: RViz 실행 여부 (default: true)

============================================================================
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

    # ================================================================
    # Launch Arguments
    # ================================================================
    declared_arguments = [
        # Arm (CAN)
        DeclareLaunchArgument("can_interface", default_value="can0",
                              description="CAN interface for arm motors"),

        # Gripper (Serial)
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyACM1",
                              description="Serial port for gripper (OpenCR)"),
        DeclareLaunchArgument("gripper_baudrate", default_value="115200",
                              description="Gripper serial baudrate"),

        # Mobile Base (Serial)
        DeclareLaunchArgument("port_front", default_value="/dev/ttyUSB0",
                              description="Serial port for front motor driver"),
        DeclareLaunchArgument("port_rear", default_value="/dev/ttyUSB1",
                              description="Serial port for rear motor driver"),
        DeclareLaunchArgument("mobile_baudrate", default_value="19200",
                              description="Mobile base serial baudrate"),

        # Robot parameters
        DeclareLaunchArgument("wheel_radius", default_value="0.098",
                              description="Wheel radius in meters"),
        DeclareLaunchArgument("wheel_separation", default_value="0.32",
                              description="Wheel separation in meters"),

        # General
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation time"),
    ]

    # Get launch configurations
    can_interface = LaunchConfiguration("can_interface")
    gripper_port = LaunchConfiguration("gripper_port")
    gripper_baudrate = LaunchConfiguration("gripper_baudrate")
    port_front = LaunchConfiguration("port_front")
    port_rear = LaunchConfiguration("port_rear")
    mobile_baudrate = LaunchConfiguration("mobile_baudrate")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    rviz_arg = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ================================================================
    # Robot Description (Full Hardware URDF)
    # ================================================================
    urdf_path = os.path.join(description_pkg, "urdf", "mobile_manipulator_full_hardware.xacro")
    robot_description_content = Command([
        'xacro ', urdf_path,
        ' can_interface:=', can_interface,
        ' gripper_port:=', gripper_port,
        ' gripper_baudrate:=', gripper_baudrate,
        ' port_front:=', port_front,
        ' port_rear:=', port_rear,
        ' mobile_baudrate:=', mobile_baudrate,
        ' wheel_radius:=', wheel_radius,
        ' wheel_separation:=', wheel_separation,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ================================================================
    # MoveIt Configuration
    # ================================================================

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
        "publish_planning_scene_topic": "/monitored_planning_scene",  # Fix for OctoMap not updating (MoveIt2 #3452)
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Octomap configuration with PointCloud sensor
    octomap_config = {
        "octomap_frame": "camera_sensor_link",
        "octomap_resolution": 0.05,
        "max_range": 2.5,
        # PointCloud Octomap Updater (flattened parameters)
        "sensors": ["pointcloud_sensor"],
        "pointcloud_sensor.sensor_plugin": "occupancy_map_monitor/PointCloudOctomapUpdater",
        "pointcloud_sensor.point_cloud_topic": "/camera/camera/depth/color/points",
        "pointcloud_sensor.max_range": 2.5,
        "pointcloud_sensor.point_subsample": 1,
        "pointcloud_sensor.padding_offset": 0.03,
        "pointcloud_sensor.padding_scale": 1.0,
        "pointcloud_sensor.max_update_rate": 5.0,
        "pointcloud_sensor.filtered_cloud_topic": "/filtered_cloud",
    }

    # ================================================================
    # Controller Config Path (통합 컨트롤러)
    # ================================================================
    ros2_controllers_path = os.path.join(
        moveit_config_pkg, "config", "mobile_manipulator_controllers.yaml"
    )

    # ================================================================
    # Nodes
    # ================================================================

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Diff Drive Controller (Mobile Base)
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # MoveIt Move Group
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
            octomap_config,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # RViz
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

    # ================================================================
    # Sequencing: JSB -> arm -> gripper -> diff_drive -> move_group
    # ================================================================

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

    delay_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    delay_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    delay_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # ================================================================
    # Launch Description
    # ================================================================
    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            ros2_control_node,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            delay_arm,
            delay_gripper,
            delay_diff_drive,
            delay_move_group,
            delay_rviz,
        ]
    )
