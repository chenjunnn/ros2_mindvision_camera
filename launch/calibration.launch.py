import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')

    mv_camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file,
                    {'use_sensor_data_qos': False}],
    )

    # Using command because
    # https://github.com/ros-perception/image_pipeline/pull/597#issuecomment-1047616455
    calibrator = ExecuteProcess(
        cmd=['ros2', 'run', 'camera_calibration', 'cameracalibrator',
             '--size', LaunchConfiguration('size'),
             '--square', LaunchConfiguration('square'),
             '--no-service-check',
             'image:=/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='size',
                              default_value='7x9'),
        DeclareLaunchArgument(name='square',
                              default_value='0.20'),
        mv_camera_node,
        calibrator
    ])
