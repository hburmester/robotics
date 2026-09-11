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

        self.pose_subscription_ = self.create_subscription(
            Pose2D,
            '/robot_pose',
            self.pose_callback,
            10
        )

        self.obstacle_position_x_ = 3.0
        self.robot_x_ = 0.0

        self.timer_ = self.create_timer(
            0.1,
            self.publish_distance
        )

        self.get_logger().info('2D-compatible distance sensor started')

    def pose_callback(self, msg):
        self.robot_x_ = msg.x

    def publish_distance(self):
        true_distance = (
            self.obstacle_position_x_
            - self.robot_x_
        )

        noise = random.gauss(
            0.0,
            0.02
        )

        measured_distance = max(
            0.0,
            true_distance + noise
        )

        msg = Float64()
        msg.data = measured_distance

        self.distance_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DistanceSensor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
