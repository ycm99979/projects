import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_description")
    robot_bringup_pkg_share = get_package_share_directory("robot_bringup")
    gazebo_pkg_share = get_package_share_directory("robot_gazebo")

    # 1. 파일 경로 설정
    urdf_file = "mobile_manipulator.xacro"
    urdf_path = os.path.join(pkg_share, "urdf", urdf_file)
    controllers_file = os.path.join(robot_bringup_pkg_share, "config", "frbot_controllers_sim.yaml")
    world_file = os.path.join(gazebo_pkg_share, "worlds", "empty_worlds.sdf")
    bridge_config_path = os.path.join(robot_bringup_pkg_share, 'config', 'gz_ros_bridge.yaml')
    rviz_config_file = os.path.join(gazebo_pkg_share, "rviz", "frbot_sim.rviz")
    ekf_config_path = os.path.join(robot_bringup_pkg_share, 'config', 'ekf.yaml')

    # Gazebo 리소스 경로 설정 (Gazebo Fortress)
    gazebo_resource_path = os.path.join(gazebo_pkg_share, 'worlds')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = gazebo_resource_path
    os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_resource_path

    # 2. Launch 인자 설정
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='If true, use simulated clock'
    )

    # 3. Xacro 처리
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_path,
            ' config_file:=', controllers_file
        ]),
        value_type=str
    )

    # 4. Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    # 5. Gazebo (ros_gz_sim) 실행
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            # [수정] 변수(world_file)와 문자열을 '+'로 연결하거나 f-string 사용
            'gz_args': '-r -v 4 ' + world_file
        }.items()
    )

    # 6. Gazebo에 로봇 스폰 (create) - 높이 0.3m로 스폰
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'frbot', '-allow_renaming', 'true', '-z', '0.3'],
    )

    # 7. ros_gz_bridge (Clock 등)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    # 8. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # 9. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 10. robot_localization - EKF 노드
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
    )

    # 로봇이 생성된 후 컨트롤러를 실행하도록 이벤트 핸들러 설정
    delayed_controller_manager_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                arm_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        gz_sim_launch,
        bridge,
        rsp_node,
        gz_spawn_entity,
        delayed_controller_manager_spawner,
        ekf_node,
        rviz_node,
    ])