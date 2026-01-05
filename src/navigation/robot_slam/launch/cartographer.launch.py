import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    package_path = get_package_share_directory('robot_slam')
    
    # 설정 파일 경로 (기본값용)
    default_cartographer_config_dir = os.path.join(package_path, 'config')
    rviz_config_file = os.path.join(package_path, 'rviz', 'cartographer.rviz')
    
    # LaunchConfiguration 정의
    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    
    # launch 시 변경할 수 있는 파라미터들
    return LaunchDescription([
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='open rviz'),

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=default_cartographer_config_dir,
            description='Full path to config file to load'),
            
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer.lua',
            description='Name of lua file for cartographer'),
            
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # 시뮬레이션 사용 시 true로 변경
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell in the published occupancy grid'),
            
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='0.5',
            description='OccupancyGrid publishing period'),      
        
        # cartographer를 사용하기 위한 Node
        # arguments에 내 .lua 파일의 위치를 넣는다
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('odom', '/diff_drive_controller/odom'),
                ('scan', '/scan'),
                ('imu', '/imu/data')],  # 슬래시 추가
        ),
        
        # cartographer의 2d occupancy_grid_map을 만드는 Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
        ),
        
        # rviz2 띄우는 Node
        Node(
            package='rviz2',
            executable='rviz2',                                                                                              
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
        )
    ])