import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_simulation',
            executable='distance_sensor',
            name='distance_sensor'
        ),

        Node(
            package='robot_control',
            executable='obstacle_controller',
            name='obstacle_controller',
            parameters=[config_file]
        ),

        Node(
            package='robot_simulation',
            executable='simulated_robot',
            name='simulated_robot'
        ),
    ])
