import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class ObstacleController(Node):

    def __init__(self):
        super().__init__('obstacle_controller')

        self.subscription_ = self.create_subscription(
            Float64,
            'distance',
            self.distance_callback,
            10
        )

        self.cmd_vel_publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.stop_distance_ = 0.5
        self.forward_speed_ = 0.3

        self.get_logger().info('Obstacle controller started')

    def distance_callback(self, msg):
        command = Twist()

        if msg.data > self.stop_distance_:
            command.linear.x = self.forward_speed_
        else:
            command.linear.x = 0.0

        command.angular.z = 0.0

        self.cmd_vel_publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
