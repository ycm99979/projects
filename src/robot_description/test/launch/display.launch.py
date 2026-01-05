#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package path
    pkg_path = get_package_share_directory('robot_description')
    
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gui', default_value='true'),
    ]
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    
    # Robot description
    urdf_path = os.path.join(pkg_path, 'urdf', 'mobile_manipulator_hardware.xacro')
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui),
        output='screen'
    )
    
    # Joint state publisher (fallback)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_gui),
        output='screen'
    )
    
    # RViz
    rviz_config = os.path.join(pkg_path, 'rviz', 'urdf.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            joint_state_publisher_gui,
            joint_state_publisher,
            rviz,
        ]
    )