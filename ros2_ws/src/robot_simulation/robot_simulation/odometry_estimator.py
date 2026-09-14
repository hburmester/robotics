import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


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

        self.odom_publisher_ = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster_ = TransformBroadcaster(
            self
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

        # Differential-drive incremental motion
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

        # Wheel angular velocity -> wheel linear velocity
        left_wheel_velocity = (
            self.wheel_radius_
            * msg.velocity[0]
        )

        right_wheel_velocity = (
            self.wheel_radius_
            * msg.velocity[1]
        )

        # Wheel velocity -> robot body velocity
        linear_velocity = (
            right_wheel_velocity
            + left_wheel_velocity
        ) / 2.0

        angular_velocity = (
            right_wheel_velocity
            - left_wheel_velocity
        ) / self.wheel_separation_

        self.publish_odometry(
            msg,
            linear_velocity,
            angular_velocity
        )

        self.publish_transform(
            msg
        )

    def publish_odometry(
        self,
        wheel_state_msg,
        linear_velocity,
        angular_velocity
    ):
        odom_msg = Odometry()

        odom_msg.header.stamp = (
            wheel_state_msg.header.stamp
        )

        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x_
        odom_msg.pose.pose.position.y = self.y_
        odom_msg.pose.pose.position.z = 0.0

        # Planar yaw -> quaternion
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0

        odom_msg.pose.pose.orientation.z = (
            math.sin(self.theta_ / 2.0)
        )

        odom_msg.pose.pose.orientation.w = (
            math.cos(self.theta_ / 2.0)
        )

        # Robot velocity
        odom_msg.twist.twist.linear.x = (
            linear_velocity
        )

        odom_msg.twist.twist.angular.z = (
            angular_velocity
        )

        self.odom_publisher_.publish(
            odom_msg
        )

    def publish_transform(
        self,
        wheel_state_msg
    ):
        transform = TransformStamped()

        transform.header.stamp = (
            wheel_state_msg.header.stamp
        )

        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x_
        transform.transform.translation.y = self.y_
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0

        transform.transform.rotation.z = (
            math.sin(self.theta_ / 2.0)
        )

        transform.transform.rotation.w = (
            math.cos(self.theta_ / 2.0)
        )

        self.tf_broadcaster_.sendTransform(
            transform
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
