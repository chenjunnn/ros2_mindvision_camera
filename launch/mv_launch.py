import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    mv_camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        arguments=['--ros-args', '-p', 'use_sensor_data_qos:=true'],
    )

    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5',
                   'camera_base_link', 'camera_optical_frame'],
    )

    return LaunchDescription([mv_camera_node, tf_publisher])
