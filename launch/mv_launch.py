import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    mv_camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        output='screen',
        parameters=[config],
        # Uncomment this line to change log level to DEBUG
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return LaunchDescription([mv_camera_node])
