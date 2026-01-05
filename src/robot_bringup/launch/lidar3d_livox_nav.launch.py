# Description:
#   Livox LiDAR SLAM with Navigation2 stack
#   Based on lidar3d_livox.launch.py with added Nav2 support
#
# Example:
#   $ ros2 launch robot_bringup lidar3d_livox_nav.launch.py
#
# Features:
#   - RTAB-Map SLAM with 2D occupancy grid
#   - Nav2 navigation stack (AMCL, Planner, Controller)
#   - PointCloud to LaserScan conversion
#   - RViz with full navigation visualization

import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context: LaunchContext, *args, **kwargs):

    frame_id = LaunchConfiguration('frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')

    imu_topic = LaunchConfiguration('imu_topic')
    imu_used = imu_topic.perform(context) != ''

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)

    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'true' or localization == 'True'

    rviz_enabled = LaunchConfiguration('rviz').perform(context)
    rviz_enabled = rviz_enabled == 'true' or rviz_enabled == 'True'

    nav_enabled = LaunchConfiguration('navigation').perform(context)
    nav_enabled = nav_enabled == 'true' or nav_enabled == 'True'

    # RViz config path (navigation version)
    rviz_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'rviz', 'lidar3d_livox_nav.rviz'
    )

    # Nav2 params path
    nav2_params_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'launch', 'config', 'nav2_params.yaml'
    )

    # Rule of thumb:
    max_correspondence_distance = voxel_size_value * 10.0

    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,
        'qos': LaunchConfiguration('qos'),
        'approx_sync': False,
        'wait_for_transform': 0.2,
        # RTAB-Map's internal parameters are strings:
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '10',
        'Icp/VoxelSize': str(voxel_size_value),
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
    }

    # Use external odom (from diff_drive_controller) or ICP odom
    use_external_odom = LaunchConfiguration('use_external_odom').perform(context)
    use_external_odom = use_external_odom == 'true' or use_external_odom == 'True'

    icp_odometry_parameters = {
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'deskewing': False,
        'odom_frame_id': 'icp_odom',  # Use separate frame to avoid conflict with diff_drive
        # RTAB-Map's internal parameters are strings:
        'Odom/ScanKeyFrameThr': '0.4',
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'false',
        'Icp/CorrespondenceRatio': '0.01'
    }
    if imu_used:
        icp_odometry_parameters['wait_imu_to_init'] = True

    rtabmap_parameters = {
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom_info': True,
        'subscribe_scan_cloud': True,
        'map_frame_id': 'map',
        'odom_sensor_sync': True,
        # ROS parameters for map publishing
        'map_always_update': True,
        'map_empty_ray_tracing': True,
        # RTAB-Map's internal parameters are strings:
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        # 2D Occupancy Grid generation
        'RGBD/CreateOccupancyGrid': 'true',
        # Grid parameters for 3D LiDAR -> 2D map
        'Grid/Sensor': '0',
        'Grid/3D': 'true',
        'Grid/RangeMax': '20.0',
        'Grid/RangeMin': '0.5',
        'Grid/RayTracing': 'true',
        'Grid/CellSize': '0.05',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.0',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/MinGroundHeight': '-10.0',
        'Grid/ClusterRadius': '0.1',
        'Grid/GroundIsObstacle': 'false',
        'Grid/FlatObstacleDetected': 'false',
        'Grid/NoiseFilteringRadius': '0.0',
        'Grid/NoiseFilteringMinNeighbors': '0',
        'Grid/Scan2dUnknownSpaceFilled': 'true',
        # Memory settings
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        'Reg/Strategy': '1',
        'Icp/CorrespondenceRatio': str(LaunchConfiguration('min_loop_closure_overlap').perform(context))
    }

    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        arguments.append('-d')  # Delete the previous database

    # Odometry topic selection based on use_external_odom
    if use_external_odom:
        # Use filtered odom from robot_localization EKF
        odom_topic = '/odometry/filtered'
    else:
        # Use ICP odom
        odom_topic = '/icp_odom'

    remappings = [('odom', odom_topic)]
    if imu_used:
        remappings.append(('imu', LaunchConfiguration('imu_topic')))
    else:
        remappings.append(('imu', 'imu_not_used'))

    nodes = []

    # ICP Odometry (only if not using external odom)
    if not use_external_odom:
        nodes.append(
            Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[shared_parameters, icp_odometry_parameters],
                remappings=[('odom', '/icp_odom'), ('imu', 'imu_not_used' if not imu_used else LaunchConfiguration('imu_topic')), ('scan_cloud', lidar_topic)])
        )

    # RTAB-Map SLAM
    nodes.append(
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters],
            remappings=remappings + [('scan_cloud', lidar_topic)],
            arguments=arguments)
    )

    # PointCloud to LaserScan for Nav2
    nodes.append(
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': base_frame_id,
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 1.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872664,  # ~0.5 degree
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 25.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', lidar_topic),
                ('scan', '/scan')
            ]
        )
    )

    # Static TF: VLP16 -> livox_frame
    # Robot URDF already has: base_link -> chassis_link -> lidar_mount -> VLP16
    # Livox driver publishes livox_frame, so we connect VLP16 -> livox_frame
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vlp16_to_livox_tf',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'VLP16',
                '--child-frame-id', 'livox_frame'
            ],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    # Add Nav2 nodes if navigation is enabled
    if nav_enabled:
        nav2_nodes = [
            # AMCL - Localization
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('/scan', '/scan')]
            ),

            # Controller Server
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                remappings=[('/cmd_vel', '/cmd_vel')]
            ),

            # Planner Server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),

            # Behavior Server (recovery behaviors)
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),

            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
            ),

            # Lifecycle Manager for Nav2
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'amcl',
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                    ]
                }]
            ),
        ]
        nodes.extend(nav2_nodes)

    # Add RViz if enabled
    if rviz_enabled:
        nodes.append(
            Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulated clock.'),

        DeclareLaunchArgument(
            'frame_id', default_value='livox_frame',
            description='LiDAR frame (use livox_frame if no robot).'),

        DeclareLaunchArgument(
            'base_frame_id', default_value='base_link',
            description='Base frame of the robot for Nav2.'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Localization mode.'),

        DeclareLaunchArgument(
            'lidar_topic', default_value='/livox/lidar',
            description='Name of the Livox lidar PointCloud2 topic.'),

        DeclareLaunchArgument(
            'imu_topic', default_value='',
            description='Livox IMU topic (ignored if empty).'),

        DeclareLaunchArgument(
            'expected_update_rate', default_value='12.0',
            description='Expected lidar frame rate for Livox (10Hz actual).'),

        DeclareLaunchArgument(
            'voxel_size', default_value='0.1',
            description='Voxel size (m) of the downsampled lidar point cloud.'),

        DeclareLaunchArgument(
            'min_loop_closure_overlap', default_value='0.2',
            description='Minimum scan overlap percentage to accept a loop closure.'),

        DeclareLaunchArgument(
            'qos', default_value='1',
            description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),

        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 for visualization.'),

        DeclareLaunchArgument(
            'navigation', default_value='true',
            description='Enable Nav2 navigation stack.'),

        DeclareLaunchArgument(
            'use_external_odom', default_value='true',
            description='Use external odometry from diff_drive_controller instead of ICP odom.'),

        OpaqueFunction(function=launch_setup),
    ])
