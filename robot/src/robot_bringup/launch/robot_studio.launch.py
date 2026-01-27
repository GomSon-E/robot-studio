import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def detect_cameras(max_cameras: int = 10) -> list[int]:
    """연결된 카메라 ID 목록 반환"""
    available = []
    for i in range(0, max_cameras, 2):  # 짝수만
        if os.path.exists(f'/dev/video{i}'):
            available.append(i)
    return available



def launch_setup(_context):
    nodes = []

    camera_ids = detect_cameras()
    for camera_id in camera_ids:
        nodes.append(
            Node(
                package='robot_sensors',
                executable='camera_node',
                name=f'camera_{camera_id}',
                output='screen',
                parameters=[{
                    'camera_id': camera_id,
                    'topic_name': f'/camera_{camera_id}/image_raw'
                }]
            )
        )

    nodes.append(
        Node(
            package='robot_ui',
            executable='ui',
            name='robot_ui',
            output='screen'
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
