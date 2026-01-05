import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 패키지 경로
    robot_nav2_dir = get_package_share_directory('robot_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 설정 파일 경로
    nav2_params_file = os.path.join(robot_nav2_dir, 'config', 'nav2_params.yaml')

    # Launch 인자
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default=nav2_params_file)
    slam = LaunchConfiguration('slam', default='True')
    namespace = LaunchConfiguration('namespace', default='')

    # Launch 인자 선언
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # Nav2 bringup with SLAM
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': slam,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'namespace': namespace,
        }.items()
    )

    # Launch Description 생성
    ld = LaunchDescription()

    # Launch 인자 추가
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_namespace_cmd)

    # Nav2 + SLAM 실행
    ld.add_action(nav2_bringup_launch)

    return ld
