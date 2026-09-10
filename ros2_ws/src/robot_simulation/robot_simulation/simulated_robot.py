import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class SimulatedRobot(Node):

    def __init__(self):
        super().__init__('simulated_robot')

        self.subscription_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.position_publisher_ = self.create_publisher(
            Float64,
            'robot_position',
            10
        )

        self.velocity_ = 0.0
        self.position_ = 0.0

        self.dt_ = 0.1

        self.timer_ = self.create_timer(
            self.dt_,
            self.update_robot
        )

        self.get_logger().info('Simulated robot started')

    def cmd_vel_callback(self, msg):
        self.velocity_ = msg.linear.x

    def update_robot(self):
        self.position_ = self.position_ + self.velocity_ * self.dt_

        msg = Float64()
        msg.data = self.position_

        self.position_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = SimulatedRobot()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
