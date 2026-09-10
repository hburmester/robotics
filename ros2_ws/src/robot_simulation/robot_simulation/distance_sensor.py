import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class DistanceSensor(Node):

    def __init__(self):
        super().__init__('distance_sensor')

        self.publisher_ = self.create_publisher(
            Float64,
            'distance',
            10
        )

        self.position_subscription_ = self.create_subscription(
            Float64,
            'robot_position',
            self.position_callback,
            10
        )

        self.robot_position_ = 0.0
        self.obstacle_position_ = 3.0

        self.timer_ = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info('Distance sensor started')

    def position_callback(self, msg):
        self.robot_position_ = msg.data

    def timer_callback(self):
        true_distance = self.obstacle_position_ - self.robot_position_

        noise = random.gauss(0.0, 0.02)

        measured_distance = max(
            0.0,
            true_distance + noise
        )

        msg = Float64()
        msg.data = measured_distance

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DistanceSensor()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
