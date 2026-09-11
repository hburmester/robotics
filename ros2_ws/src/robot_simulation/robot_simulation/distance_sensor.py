import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64


class DistanceSensor(Node):

    def __init__(self):
        super().__init__('distance_sensor')

        self.distance_publisher_ = self.create_publisher(
            Float64,
            '/distance',
            10
        )

        self.bearing_publisher_ = self.create_publisher(
            Float64,
            '/obstacle_bearing',
            10
        )

        self.pose_subscription_ = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.pose_callback,
            10
        )

        # Obstacle position in the world frame [m]
        self.obstacle_x_ = 3.0
        self.obstacle_y_ = 2.0

        # Robot pose in the world frame
        self.robot_x_ = 0.0
        self.robot_y_ = 0.0
        self.robot_theta_ = 0.0

        self.timer_ = self.create_timer(
            0.1,
            self.publish_measurement
        )

        self.get_logger().info(
            '2D distance and bearing sensor started'
        )

    def pose_callback(self, msg):
        self.robot_x_ = msg.x
        self.robot_y_ = msg.y
        self.robot_theta_ = msg.theta

    def publish_measurement(self):
        delta_x = self.obstacle_x_ - self.robot_x_
        delta_y = self.obstacle_y_ - self.robot_y_

        # Euclidean distance
        true_distance = math.sqrt(
            delta_x ** 2
            + delta_y ** 2
        )

        # Direction from robot to obstacle
        # expressed in the world frame
        obstacle_heading_world = math.atan2(
            delta_y,
            delta_x
        )

        # Convert world-frame direction into
        # robot-relative bearing
        relative_bearing = (
            obstacle_heading_world
            - self.robot_theta_
        )

        # Normalize bearing to [-pi, pi]
        relative_bearing = math.atan2(
            math.sin(relative_bearing),
            math.cos(relative_bearing)
        )

        # Simulated Gaussian distance noise [m]
        distance_noise = random.gauss(
            0.0,
            0.02
        )

        measured_distance = max(
            0.0,
            true_distance + distance_noise
        )

        distance_msg = Float64()
        distance_msg.data = measured_distance

        bearing_msg = Float64()
        bearing_msg.data = relative_bearing

        self.distance_publisher_.publish(
            distance_msg
        )

        self.bearing_publisher_.publish(
            bearing_msg
        )


def main(args=None):
    rclpy.init(args=args)

    node = DistanceSensor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
