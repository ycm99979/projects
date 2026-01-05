from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    realsense_launch = os.path.join(
        FindPackageShare('realsense2_camera').find('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # D405 - Gripper camera (시리얼: 315122271488)
    # D405는 RGB 카메라가 없고 depth_module에서 color 출력
    d405_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'serial_no': "'315122271488'",
            'depth_module.depth_profile': '848x480x30',
            'depth_module.color_profile': '848x480x30',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
        }.items()
    )

    # D455 - Navigation camera (시리얼: 213622301251)
    # 3초 딜레이로 USB 충돌 방지
    d455_camera = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments={
                    'camera_name': 'd455',
                    'camera_namespace': 'd455',
                    'serial_no': "'213622301251'",
                    'depth_module.depth_profile': '848x480x30',
                    'rgb_camera.color_profile': '848x480x30',
                    'pointcloud.enable': 'true',
                    'align_depth.enable': 'true',
                }.items()
            )
        ]
    )

    # YOLO 노드 - 5초 딜레이 (카메라 초기화 후 실행)
    yolo_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='yolo_realsense',
                executable='yolo_node',
                name='yolo_visual_publisher',
                output='screen',
                parameters=[{
                    'model_path': '/home/ycm/frbot_ws/best.pt',
                    'camera_frame': 'd405_optical_frame',
                    'color_topic': '/camera/camera/color/image_rect_raw',
                    'depth_topic': '/camera/camera/depth/image_rect_raw',
                    'camera_info_topic': '/camera/camera/depth/camera_info',
                    'show_visualization': False,  # 웹에서 보므로 OpenCV 창 끔
                    'confidence_threshold': 0.5,
                }]
            )
        ]
    )

    return LaunchDescription([
        d405_camera,
        d455_camera,
        yolo_node,
    ])
