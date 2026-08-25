#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from math import *
import numpy as np
# Ensure compatibility with recent NumPy versions
np.float = float

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class NoisyOdometry(Node):
    def __init__(self):
        super().__init__('noisy_odometry')

        # Initialize parameters
        self.a1 = self.declare_parameter("alpha1", 0.05).value
        self.a2 = self.declare_parameter("alpha2", 15.0 * pi / 180.0).value
        self.a3 = self.declare_parameter("alpha3", 0.05).value
        self.a4 = self.declare_parameter("alpha4", 0.01).value

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value

        self.last_odom = None
        self.pose = [-0.512, 0.297,  -1.54] # x, y, theta

        # Publisher
        self.noisy_odom_publisher = self.create_publisher(Odometry, '/odom/wheel/noisy', 10)
        # Subscriber
        self.subscription = self.create_subscription(Odometry, "odom/wheel",self.odom_callback,10)


    def odom_callback(self, data):
        
        q = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]
        _, _, theta2 = euler_from_quaternion(q)

        if self.last_odom is None:
            self.last_odom = data
            self.pose[0] = data.pose.pose.position.x
            self.pose[1] = data.pose.pose.position.y
            self.pose[2] = theta2
        else:
            dx = data.pose.pose.position.x - self.last_odom.pose.pose.position.x
            dy = data.pose.pose.position.y - self.last_odom.pose.pose.position.y
            trans = sqrt(dx * dx + dy * dy)

            q = [
                self.last_odom.pose.pose.orientation.x,
                self.last_odom.pose.pose.orientation.y,
                self.last_odom.pose.pose.orientation.z,
                self.last_odom.pose.pose.orientation.w
            ]
            _, _, theta1 = euler_from_quaternion(q)

            rot1 = atan2(dy, dx) - theta1
            rot2 = theta2 - theta1 - rot1

            sd_rot1 = self.a1 * abs(rot1) + self.a2 * trans
            sd_rot2 = self.a1 * abs(rot2) + self.a2 * trans
            sd_trans = self.a3 * trans + self.a4 * (abs(rot1) + abs(rot2))

            trans += np.random.normal(0, sd_trans * sd_trans)
            rot1 += np.random.normal(0, sd_rot1 * sd_rot1)
            rot2 += np.random.normal(0, sd_rot2 * sd_rot2)

            self.pose[0] += trans * cos(theta1 + rot1)
            self.pose[1] += trans * sin(theta1 + rot1)
            self.pose[2] += rot1 + rot2
            self.last_odom = data

        # Publish noisy odometry message
        noisy_odom = Odometry()
        noisy_odom.header.stamp = data.header.stamp
        noisy_odom.header.frame_id = self.odom_frame
        noisy_odom.pose.pose.position.x = self.pose[0]
        noisy_odom.pose.pose.position.y = self.pose[1]
        noisy_odom.pose.pose.position.z = 0.0
        noisy_odom.pose.pose.orientation.x = q[0]
        noisy_odom.pose.pose.orientation.y = q[1]
        noisy_odom.pose.pose.orientation.z = q[2]
        noisy_odom.pose.pose.orientation.w = q[3]

        self.noisy_odom_publisher.publish(noisy_odom)


def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
