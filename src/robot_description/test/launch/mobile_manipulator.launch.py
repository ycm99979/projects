import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")
    
    # 1. 파일 경로 확인 (실제 파일 이름으로 변경하세요)
    urdf_file = "mobile_manipulator.xacro"
    urdf_path = os.path.join(pkg_share, "urdf", urdf_file)
    rviz_config_path = os.path.join(pkg_share, "rviz", "frbot.rviz")

    # 2. URDF 파일 로딩 방식 수정 (파일 확장자에 맞게 선택)
    # .xacro 파일일 경우:
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}],
    )
        
    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )
    
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_gui': True}],
        output = 'screen'
    )

    return LaunchDescription([
        rsp_node,
        rviz_node,
        jsp_gui
    ])