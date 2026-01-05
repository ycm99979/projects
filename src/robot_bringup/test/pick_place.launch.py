#!/usr/bin/env python3
"""
Launch file for fr_arm pick and place system.

This launch file starts the pick and place controller along with the necessary
fr_arm_moveit_config components for complete system operation.

Usage:
    ros2 launch robot_bringup pick_place.launch.py
    ros2 launch robot_bringup pick_place.launch.py can_interface:=can1 rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for pick and place system."""
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name (e.g., can0, can1)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level (debug, info, warn, error)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        )
    )
    
    # Get launch configurations
    can_interface = LaunchConfiguration('can_interface')
    rviz_config = LaunchConfiguration('rviz')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package paths
    robot_bringup_path = get_package_share_directory('robot_bringup')
    
    # Include mobile_manipulator_moveit_config demo launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mobile_manipulator_moveit_config'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': rviz_config,
            'log_level': log_level,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Configuration file path
    config_file = os.path.join(robot_bringup_path, 'config', 'pick_place_config.yaml')
    
    # Pick and place controller node
    pick_place_node = Node(
        package='robot_bringup',
        executable='topic_pick_place.py',
        name='topic_pick_place',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            # Ensure proper topic remapping if needed
            ('/target_point', '/target_point'),
            ('/pick_place_status', '/pick_place_status'),
        ]
    )
    
    # Delay pick and place controller to ensure MoveIt is ready
    delay_pick_place_controller = TimerAction(
        period=8.0,  # Wait for MoveIt to be fully initialized
        actions=[pick_place_node],
    )
    
    return LaunchDescription(
        declared_arguments + [
            moveit_launch,
            delay_pick_place_controller,
        ]
    )