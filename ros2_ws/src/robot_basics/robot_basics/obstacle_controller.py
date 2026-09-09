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

        self.desired_distance_ = 0.5
        self.kp_ = 0.8
        self.max_speed_ = 0.3
        self.tolerance_ = 0.03

        self.get_logger().info(
            'Proportional obstacle controller started'
        )

    def distance_callback(self, msg):
        distance = msg.data

        error = distance - self.desired_distance_

        if abs(error) <= self.tolerance_:
            velocity = 0.0
        else:
            velocity = self.kp_ * error

            velocity = max(
                0.0,
                min(velocity, self.max_speed_)
            )

        command = Twist()
        command.linear.x = velocity
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
