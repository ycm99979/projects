"""
Mobile Manipulator MoveIt Demo Launch File

RViz only demo with fake controllers for arm motion planning.
No Gazebo required - uses mock_components/GenericSystem.

Usage:
    ros2 launch mobile_manipulator_moveit_config demo.launch.py
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except:
        return None


def generate_launch_description():
    # Arguments
    declared_arguments = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    rviz_arg = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Package paths
    moveit_config_pkg = get_package_share_directory("mobile_manipulator_moveit_config")
    description_pkg = get_package_share_directory("robot_description")

    # Load URDF
    urdf_path = os.path.join(description_pkg, "urdf", "mobile_manipulator_moveit.urdf")
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {"robot_description": robot_description_content}

    # Load SRDF
    srdf_path = os.path.join(moveit_config_pkg, "config", "mobile_manipulator.srdf")
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Load configs
    kinematics_yaml = load_yaml("mobile_manipulator_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mobile_manipulator_moveit_config", "config/joint_limits.yaml")
    ompl_planning_yaml = load_yaml("mobile_manipulator_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers_yaml = load_yaml("mobile_manipulator_moveit_config", "config/moveit_controllers.yaml")
    sensors_3d_yaml = load_yaml("mobile_manipulator_moveit_config", "config/sensors_3d.yaml")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # MoveIt controller manager
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml.get("moveit_simple_controller_manager", {}),
        "moveit_controller_manager": moveit_controllers_yaml.get(
            "moveit_controller_manager", 
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }

    # Planning pipeline - explicitly use OMPL
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "response_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
            "start_state_max_bounds_error": 0.1,
        }
    }

    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning scene monitor
    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Octomap configuration
    octomap_config = {
        "octomap_frame": "arm_base_link",
        "octomap_resolution": 0.05,
        "max_range": 2.0,
    }

    # Sensor configuration for 3D perception
    sensors_config = {
        "sensors": ["depth_image_octomap_updater"],
        "depth_image_octomap_updater": {
            "sensor_plugin": "occupancy_map_monitor/DepthImageOctomapUpdater",
            "image_topic": "/camera/camera/depth/image_rect_raw",
            "queue_size": 5,
            "near_clipping_plane_distance": 0.1,
            "far_clipping_plane_distance": 2.0,
            "shadow_threshold": 0.2,
            "padding_scale": 2.0,
            "padding_offset": 0.03,
            "max_update_rate": 5.0,
            "skip_vertical_pixels": 4,
            "skip_horizontal_pixels": 4,
            "filtered_cloud_topic": "/filtered_cloud",
        }
    }

    # ros2_controllers config path
    ros2_controllers_path = os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml")

    # ========== NODES ==========

    # Robot State Publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ros2_control_node (fake hardware)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path, {"use_sim_time": use_sim_time}],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Joint State Broadcaster - use --inactive flag to avoid auto-configure issue
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Delay spawners to ensure controller_manager is ready
    delay_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    # Delay arm controller after joint_state_broadcaster
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Delay gripper controller after arm controller
    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_params,
            moveit_controllers,
            octomap_config,
            sensors_config,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Delay move_group to ensure controllers are ready
    delay_move_group = TimerAction(period=6.0, actions=[move_group_node])

    # RViz
    rviz_config_file = os.path.join(moveit_config_pkg, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(rviz_arg),
    )

    # Delay RViz
    delay_rviz = TimerAction(period=8.0, actions=[rviz_node])

    return LaunchDescription(
        declared_arguments + [
            robot_state_pub,
            ros2_control_node,
            delay_joint_state_broadcaster,
            delay_arm_controller,
            delay_gripper_controller,
            delay_move_group,
            delay_rviz,
        ]
    )
