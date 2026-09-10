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

        # Robot state
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Current commanded velocities
        self.linear_velocity_ = 0.0
        self.angular_velocity_ = 0.0

        # Simulation timestep
        self.dt_ = 0.1

        self.timer_ = self.create_timer(
            self.dt_,
            self.update_robot
        )

        self.get_logger().info('2D simulated robot started')

    def cmd_vel_callback(self, msg):
        self.linear_velocity_ = msg.linear.x
        self.angular_velocity_ = msg.angular.z

    def update_robot(self):
        # Differential-drive planar kinematic model
        self.x_ += (
            self.linear_velocity_
            * math.cos(self.theta_)
            * self.dt_
        )

        self.y_ += (
            self.linear_velocity_
            * math.sin(self.theta_)
            * self.dt_
        )

        self.theta_ += self.angular_velocity_ * self.dt_

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
