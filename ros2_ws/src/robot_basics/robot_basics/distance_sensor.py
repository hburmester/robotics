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

        self.timer_ = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.true_distance_ = 2.0

        self.get_logger().info('Distance sensor started')

    def timer_callback(self):
        noise = random.gauss(0.0, 0.02)
        measured_distance = self.true_distance_ + noise

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
