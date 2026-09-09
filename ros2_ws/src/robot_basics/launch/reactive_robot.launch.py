from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_basics',
            executable='distance_sensor',
            name='distance_sensor'
        ),

        Node(
            package='robot_basics',
            executable='obstacle_controller',
            name='obstacle_controller'
        ),

        Node(
            package='robot_basics',
            executable='simulated_robot',
            name='simulated_robot'
        ),
    ])
