import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class ObstacleController(Node):

    def __init__(self):
        super().__init__('obstacle_controller')

        self.declare_parameter('desired_distance', 0.5)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('tolerance', 0.03)

        self.desired_distance_ = (
            self.get_parameter('desired_distance').value
        )
        self.kp_ = self.get_parameter('kp').value
        self.max_speed_ = self.get_parameter('max_speed').value
        self.tolerance_ = self.get_parameter('tolerance').value

        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

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

        self.get_logger().info(
            f'Controller started: '
            f'desired_distance={self.desired_distance_:.2f} m, '
            f'kp={self.kp_:.2f}, '
            f'max_speed={self.max_speed_:.2f} m/s, '
            f'tolerance={self.tolerance_:.2f} m'
        )

    def parameter_callback(self, params):
        for param in params:

            if param.name == 'desired_distance':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='desired_distance must be >= 0'
                    )

                self.desired_distance_ = param.value

            elif param.name == 'kp':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='kp must be >= 0'
                    )

                self.kp_ = param.value

            elif param.name == 'max_speed':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='max_speed must be >= 0'
                    )

                self.max_speed_ = param.value

            elif param.name == 'tolerance':
                if param.value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='tolerance must be >= 0'
                    )

                self.tolerance_ = param.value

        return SetParametersResult(
            successful=True
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
