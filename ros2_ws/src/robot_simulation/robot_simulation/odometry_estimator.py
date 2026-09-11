import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState


class OdometryEstimator(Node):

    def __init__(self):
        super().__init__('odometry_estimator')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.4)

        self.wheel_radius_ = self.get_parameter(
            'wheel_radius'
        ).value

        self.wheel_separation_ = self.get_parameter(
            'wheel_separation'
        ).value

        self.wheel_state_subscription_ = self.create_subscription(
            JointState,
            '/wheel_states',
            self.wheel_state_callback,
            10
        )

        self.odom_pose_publisher_ = self.create_publisher(
            Pose2D,
            '/odom_pose',
            10
        )

        # Estimated robot pose
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Previous encoder positions
        self.previous_left_wheel_angle_ = None
        self.previous_right_wheel_angle_ = None

        self.get_logger().info(
            'Wheel odometry estimator started'
        )

    def wheel_state_callback(self, msg):

        if len(msg.position) < 2:
            return

        left_wheel_angle = msg.position[0]
        right_wheel_angle = msg.position[1]

        # The first measurement initializes our reference.
        # We cannot calculate a delta yet.
        if (
            self.previous_left_wheel_angle_ is None
            or self.previous_right_wheel_angle_ is None
        ):
            self.previous_left_wheel_angle_ = (
                left_wheel_angle
            )

            self.previous_right_wheel_angle_ = (
                right_wheel_angle
            )

            return

        # Encoder angle increments [rad]
        delta_left_angle = (
            left_wheel_angle
            - self.previous_left_wheel_angle_
        )

        delta_right_angle = (
            right_wheel_angle
            - self.previous_right_wheel_angle_
        )

        # Store current encoder readings for next callback
        self.previous_left_wheel_angle_ = (
            left_wheel_angle
        )

        self.previous_right_wheel_angle_ = (
            right_wheel_angle
        )

        # Wheel travel distances [m]
        delta_left_distance = (
            self.wheel_radius_
            * delta_left_angle
        )

        delta_right_distance = (
            self.wheel_radius_
            * delta_right_angle
        )

        # Differential-drive displacement
        delta_distance = (
            delta_right_distance
            + delta_left_distance
        ) / 2.0

        delta_theta = (
            delta_right_distance
            - delta_left_distance
        ) / self.wheel_separation_

        # Integrate estimated robot pose
        self.x_ += (
            delta_distance
            * math.cos(self.theta_)
        )

        self.y_ += (
            delta_distance
            * math.sin(self.theta_)
        )

        self.theta_ += delta_theta

        odom_pose_msg = Pose2D()

        odom_pose_msg.x = self.x_
        odom_pose_msg.y = self.y_
        odom_pose_msg.theta = self.theta_

        self.odom_pose_publisher_.publish(
            odom_pose_msg
        )


def main(args=None):
    rclpy.init(args=args)

    node = OdometryEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
