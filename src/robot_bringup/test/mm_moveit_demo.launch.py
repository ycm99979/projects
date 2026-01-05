#!/usr/bin/env python3
"""
MoveIt Demo Launch File - Simulation Only (No Hardware)
- MoveIt motion planning
- Fake controllers (no real hardware)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    rviz_arg = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Robot description - MoveIt용 (ros2_control 태그 없음)
    urdf_path = os.path.join(description_pkg, "urdf", "mobile_manipulator_moveit.urdf.xacro")
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    srdf_path = os.path.join(moveit_config_pkg, "config", "mobile_manipulator.srdf")
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # Load YAML configs
    kinematics_yaml = load_yaml("mobile_manipulator_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mobile_manipulator_moveit_config", "config/joint_limits.yaml")
    ompl_planning_yaml = load_yaml("mobile_manipulator_moveit_config", "config/ompl_planning.yaml")


    # MoveIt parameters
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # Fake controllers for demo
    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["arm_controller", "gripper_controller"],
            "arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [
                    "link2_to_link1",
                    "link3_to_link2", 
                    "link4_to_link3",
                    "gripper_to_link4"
                ]
            },
            "gripper_controller": {
                "type": "FollowJointTrajectory", 
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": ["gripper_left_joint"]
            }
        },
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Load OMPL planning config from YAML file
    ompl_planning_config = ompl_planning_yaml

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
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

    # Nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Fake joint state publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
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
            ompl_planning_config,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            octomap_config,
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

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            joint_state_publisher,
            move_group_node,
            rviz_node,
        ]
    )