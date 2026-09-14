import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory(
        'robot_bringup'
    )

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

    odometry_config = os.path.join(
        bringup_share,
        'config',
        'odometry.yaml'
    )

    urdf_path = os.path.join(
        bringup_share,
        'urdf',
        'mobile_robot.urdf'
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

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

        Node(
            package='robot_simulation',
            executable='odometry_estimator',
            name='odometry_estimator',
            parameters=[odometry_config]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description
                }
            ],
            remappings=[
                ('joint_states', 'wheel_states')
            ]
        ),
    ])
