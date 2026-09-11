import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')

    controller_config = os.path.join(
        bringup_share,
        'config',
        'controller.yaml'
    )

    simulation_config = os.path.join(
        bringup_share,
        'config',
        'simulation.yaml'
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
            parameters=[controller_config]
        ),

        Node(
            package='robot_simulation',
            executable='simulated_robot',
            name='simulated_robot',
            parameters=[simulation_config]
        ),
    ])
