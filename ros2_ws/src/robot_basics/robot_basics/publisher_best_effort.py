import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.publisher_ = self.create_publisher(
            String,
            'robot_status',
            qos_profile
        )

        self.timer_ = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.counter_ = 0

        self.get_logger().info('Simple publisher started')

    def timer_callback(self):
        msg = String()

        msg.data = f'Robot heartbeat: {self.counter_}'

        self.publisher_.publish(msg)

        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)

    node = SimplePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
