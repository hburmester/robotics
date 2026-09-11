import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class ObstacleController(Node):

    def __init__(self):
        super().__init__('obstacle_controller')

        # Controller parameters
        self.declare_parameter('desired_distance', 0.5)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('tolerance', 0.03)

        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('max_angular_speed', 1.0)

        self.desired_distance_ = self.get_parameter(
            'desired_distance'
        ).value

        self.kp_ = self.get_parameter(
            'kp'
        ).value

        self.max_speed_ = self.get_parameter(
            'max_speed'
        ).value

        self.tolerance_ = self.get_parameter(
            'tolerance'
        ).value

        self.heading_kp_ = self.get_parameter(
            'heading_kp'
        ).value

        self.max_angular_speed_ = self.get_parameter(
            'max_angular_speed'
        ).value

        # Latest sensor measurements
        self.distance_ = None
        self.bearing_ = None

        self.distance_subscription_ = self.create_subscription(
            Float64,
            '/distance',
            self.distance_callback,
            10
        )

        self.bearing_subscription_ = self.create_subscription(
            Float64,
            '/obstacle_bearing',
            self.bearing_callback,
            10
        )

        self.cmd_vel_publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer_ = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '2D obstacle controller started'
        )

    def distance_callback(self, msg):
        self.distance_ = msg.data

    def bearing_callback(self, msg):
        self.bearing_ = msg.data

    def control_loop(self):

        # Do not control until both sensor measurements
        # have been received.
        if self.distance_ is None or self.bearing_ is None:
            return

        distance_error = (
            self.distance_
            - self.desired_distance_
        )

        # Distance controller
        if abs(distance_error) <= self.tolerance_:
            linear_velocity = 0.0
        else:
            linear_velocity = (
                self.kp_
                * distance_error
            )

            linear_velocity = max(
                0.0,
                min(
                    linear_velocity,
                    self.max_speed_
                )
            )

        # Heading controller
        angular_velocity = (
            self.heading_kp_
            * self.bearing_
        )

        angular_velocity = max(
            -self.max_angular_speed_,
            min(
                angular_velocity,
                self.max_angular_speed_
            )
        )

        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.cmd_vel_publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
