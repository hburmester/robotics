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
        self.resume_distance_ = 0.6
        self.forward_speed_ = 0.3

        self.state_ = 'MOVING'

        self.get_logger().info(
            f'Obstacle controller started in state: {self.state_}'
        )

    def distance_callback(self, msg):

        distance = msg.data

        if self.state_ == 'MOVING':
            if distance <= self.stop_distance_:
                self.state_ = 'STOPPED'
                self.get_logger().info(
                    f'STOPPED: obstacle at {distance:.3f} m'
                )

        elif self.state_ == 'STOPPED':
            if distance >= self.resume_distance_:
                self.state_ = 'MOVING'
                self.get_logger().info(
                    f'MOVING: obstacle at {distance:.3f} m'
                )

        command = Twist()

        if self.state_ == 'MOVING':
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
