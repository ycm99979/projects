import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
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

    gazebo_world_path = os.path.join(gazebo_pkg_share, 'worlds')
    gazebo_models_path = os.path.join(gazebo_pkg_share, 'models')
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gazebo_world_path + ':' + gazebo_models_path)

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gazebo_world_path + ':' + gazebo_models_path)

    urdf_path = os.path.join(pkg_share, "urdf", "mobile_manipulator.xacro")
    controllers_file = os.path.join(robot_bringup_pkg_share, "config", "frbot_controllers_sim.yaml")
    world_file = os.path.join(gazebo_pkg_share, "worlds", "empty_worlds.sdf")
    bridge_config_path = os.path.join(robot_bringup_pkg_share, 'config', 'gz_ros_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the simulated clock if true'
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_path,
            ' config_file:=', controllers_file
        ]),
        value_type=str
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r -v 4 ' + world_file}.items()
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'frbot', '-x', '10.0', '-y', '0.0', '-z', '0.07'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

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

    # Static TF: Gazebo lidar frame -> URDF VLP16 frame
    lidar_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'VLP16', 'frbot/base_link/lidar_head'],
        output='screen'
    )

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
        ign_resource_path,
        gz_resource_path,
        gz_sim_launch,
        bridge,
        rsp_node,
        gz_spawn_entity,
        lidar_frame_publisher,
        delayed_controller_manager_spawner,
    ])
