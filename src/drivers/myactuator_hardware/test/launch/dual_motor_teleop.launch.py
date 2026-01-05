#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch arguments
    teleop_type_arg = DeclareLaunchArgument(
        'teleop_type',
        default_value='position',
        description='Type of teleop control: position or velocity'
    )
    
    # Get launch configuration
    teleop_type = LaunchConfiguration('teleop_type')
    
    # Package directory
    pkg_dir = get_package_share_directory('myactuator_hardware')
    
    # Teleop node - position control
    position_teleop_node = Node(
        package='myactuator_hardware',
        executable='dual_motor_teleop.py',
        name='dual_motor_position_teleop',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", teleop_type, "' == 'position'"])
        )
    )
    
    # Teleop node - velocity control  
    velocity_teleop_node = Node(
        package='myactuator_hardware',
        executable='dual_motor_velocity_teleop.py',
        name='dual_motor_velocity_teleop',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", teleop_type, "' == 'velocity'"])
        )
    )
    
    return LaunchDescription([
        teleop_type_arg,
        position_teleop_node,
        velocity_teleop_node,
    ])