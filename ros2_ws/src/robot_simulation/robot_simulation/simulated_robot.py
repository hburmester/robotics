import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist


class SimulatedRobot(Node):

    def __init__(self):
        super().__init__('simulated_robot')

        self.cmd_vel_subscription_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.pose_publisher_ = self.create_publisher(
            Pose2D,
            '/robot_pose',
            10
        )

        # Robot pose
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Differential-drive geometry
        self.wheel_radius_ = 0.05
        self.wheel_separation_ = 0.4

        # Wheel angular velocities [rad/s]
        self.left_wheel_angular_velocity_ = 0.0
        self.right_wheel_angular_velocity_ = 0.0

        # Simulation timestep [s]
        self.dt_ = 0.1

        self.timer_ = self.create_timer(
            self.dt_,
            self.update_robot
        )

        self.get_logger().info(
            'Differential-drive simulated robot started'
        )

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Inverse differential-drive kinematics:
        # body velocity -> wheel linear velocities [m/s]
        left_wheel_linear_velocity = (
            linear_velocity
            - angular_velocity * self.wheel_separation_ / 2.0
        )

        right_wheel_linear_velocity = (
            linear_velocity
            + angular_velocity * self.wheel_separation_ / 2.0
        )

        # Convert wheel linear velocity to angular velocity:
        # omega_wheel = v_wheel / r
        self.left_wheel_angular_velocity_ = (
            left_wheel_linear_velocity / self.wheel_radius_
        )

        self.right_wheel_angular_velocity_ = (
            right_wheel_linear_velocity / self.wheel_radius_
        )

    def update_robot(self):
        # Convert wheel angular velocities back to
        # tangential wheel velocities [m/s]:
        # v_wheel = r * omega_wheel
        left_wheel_linear_velocity = (
            self.wheel_radius_
            * self.left_wheel_angular_velocity_
        )

        right_wheel_linear_velocity = (
            self.wheel_radius_
            * self.right_wheel_angular_velocity_
        )

        # Forward differential-drive kinematics:
        # wheel velocities -> robot body velocity
        linear_velocity = (
            right_wheel_linear_velocity
            + left_wheel_linear_velocity
        ) / 2.0

        angular_velocity = (
            right_wheel_linear_velocity
            - left_wheel_linear_velocity
        ) / self.wheel_separation_

        # Body-frame velocity -> world-frame motion
        self.x_ += (
            linear_velocity
            * math.cos(self.theta_)
            * self.dt_
        )

        self.y_ += (
            linear_velocity
            * math.sin(self.theta_)
            * self.dt_
        )

        self.theta_ += angular_velocity * self.dt_

        pose_msg = Pose2D()

        pose_msg.x = self.x_
        pose_msg.y = self.y_
        pose_msg.theta = self.theta_

        self.pose_publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    node = SimulatedRobot()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
