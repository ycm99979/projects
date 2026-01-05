#!/usr/bin/env python3
"""
============================================================================
Mobile Manipulator Visual Servo Launch File (USBCAN Version)
============================================================================

Visual Servoing을 위한 런치 파일 (USBCAN 기반)
- MoveIt Servo를 사용한 실시간 엔드이펙터 제어
- /target_point (geometry_msgs/PointStamped) 토픽으로 목표 좌표 수신
- 엔드이펙터가 목표점을 실시간 추종

[사용법]
ros2 launch robot_bringup mobile_manipulator_visual_servo.launch.py

[토픽]
- Input: /target_point (geometry_msgs/PointStamped) - 추종할 목표 좌표 (카메라 노드에서 발행)
- Output: /servo_node/delta_twist_cmds - MoveIt Servo 명령

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
        # Arm (USBCAN)
        DeclareLaunchArgument("can_channel", default_value="0",
                              description="USBCAN channel (0 or 1)"),
        DeclareLaunchArgument("baudrate", default_value="1000000",
                              description="CAN baudrate"),
        DeclareLaunchArgument("motor1_id", default_value="1",
                              description="Motor 1 CAN ID"),
        DeclareLaunchArgument("motor2_id", default_value="2",
                              description="Motor 2 CAN ID"),
        DeclareLaunchArgument("motor3_id", default_value="3",
                              description="Motor 3 CAN ID"),
        DeclareLaunchArgument("motor4_id", default_value="4",
                              description="Motor 4 CAN ID"),
        DeclareLaunchArgument("auto_home", default_value="true",
                              description="Auto home on activation"),

        # Gripper (Serial)
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyACM0",
                              description="Serial port for gripper"),
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

        # Visual Servo parameters
        DeclareLaunchArgument("servo_gain", default_value="0.5",
                              description="Proportional gain for visual servoing"),
        DeclareLaunchArgument("max_linear_vel", default_value="0.1",
                              description="Max linear velocity for servo (m/s)"),

        # General
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use simulation time"),
    ]

    # Get launch configurations
    can_channel = LaunchConfiguration("can_channel")
    baudrate = LaunchConfiguration("baudrate")
    motor1_id = LaunchConfiguration("motor1_id")
    motor2_id = LaunchConfiguration("motor2_id")
    motor3_id = LaunchConfiguration("motor3_id")
    motor4_id = LaunchConfiguration("motor4_id")
    auto_home = LaunchConfiguration("auto_home")
    gripper_port = LaunchConfiguration("gripper_port")
    gripper_baudrate = LaunchConfiguration("gripper_baudrate")
    port_front = LaunchConfiguration("port_front")
    port_rear = LaunchConfiguration("port_rear")
    mobile_baudrate = LaunchConfiguration("mobile_baudrate")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    servo_gain = LaunchConfiguration("servo_gain")
    max_linear_vel = LaunchConfiguration("max_linear_vel")
    rviz_arg = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ================================================================
    # Robot Description (USBCAN Hardware URDF)
    # ================================================================
    urdf_path = os.path.join(description_pkg, "urdf", "mobile_manipulator_usbcan_hardware.xacro")
    robot_description_content = Command([
        'xacro ', urdf_path,
        ' can_channel:=', can_channel,
        ' baudrate:=', baudrate,
        ' motor1_id:=', motor1_id,
        ' motor2_id:=', motor2_id,
        ' motor3_id:=', motor3_id,
        ' motor4_id:=', motor4_id,
        ' auto_home:=', auto_home,
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

    # Check if servo_config.yaml exists, otherwise use default
    servo_config_path = os.path.join(moveit_config_pkg, "config", "servo_config.yaml")
    if os.path.exists(servo_config_path):
        servo_yaml = load_yaml("mobile_manipulator_moveit_config", "config/servo_config.yaml")
        servo_params = {"moveit_servo": servo_yaml.get("moveit_servo", {})}
    else:
        # Default servo params
        servo_params = {"moveit_servo": {
            "use_gazebo": False,
            "status_topic": "~/status",
            "command_in_type": "speed_units",
            "scale": {"linear": 0.3, "rotational": 0.5},
            "command_out_topic": "/arm_controller/joint_trajectory",
            "command_out_type": "trajectory_msgs/JointTrajectory",
            "publish_joint_positions": True,
            "publish_joint_velocities": True,
            "publish_joint_accelerations": False,
            "planning_frame": "arm_base_link",
            "ee_frame_name": "end_effector_link",
            "robot_link_command_frame": "arm_base_link",
            "incoming_command_timeout": 0.5,
            "publish_period": 0.02,
        }}

    # MoveIt parameters
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # ================================================================
    # Controller Config Path
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

    # Diff Drive Controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            servo_params,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # Visual Servo Controller Node (target_point 추종)
    visual_servo_controller = Node(
        package="robot_bringup",
        executable="visual_servo_controller.py",
        name="visual_servo_controller",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"servo_gain": servo_gain},
            {"max_linear_vel": max_linear_vel},
            {"target_topic": "target_point"},
            {"planning_frame": "arm_base_link"},
            {"ee_frame": "end_effector_link"},
            {"auto_enable": True},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # RViz (servo용 설정 - RobotModel로 joint_states 직접 구독)
    rviz_config = os.path.join(moveit_config_pkg, "config", "servo.rviz")
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
    # Sequencing (컨트롤러 순차 로드)
    # ================================================================

    # JSB 완료 후 arm_controller 시작
    delay_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # arm_controller 완료 후 gripper_controller 시작
    delay_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # gripper_controller 완료 후 diff_drive_controller 시작
    delay_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # 모든 컨트롤러 로드 후 servo_node 시작
    delay_servo = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[servo_node],
        )
    )

    # servo_node 시작 후 visual_servo_controller 시작
    delay_visual_servo = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,  # servo_node가 완전히 시작될 때까지 대기
                    actions=[visual_servo_controller]
                )
            ],
        )
    )

    # RViz 시작
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
            delay_servo,
            delay_visual_servo,
            delay_rviz,
        ]
    )
