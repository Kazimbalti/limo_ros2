import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from nav_msgs.msg import Odometry
import pickle
import os
import sys
import termios
import tty
import threading
import time

ROUTES_DIR = 'routes'
os.makedirs(ROUTES_DIR, exist_ok=True)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 1)
        self.current_pose = None
        self.poses = []
        self.get_logger().info('Press "r" to record pose, "q" to quit and save.')

    def pose_callback(self, msg):
        self.current_pose = msg
        time.sleep(2)
        if self.current_pose:
            x = self.current_pose.pose.pose.position.x
            y = self.current_pose.pose.pose.position.y
            # Yaw from quaternion
            q = self.current_pose.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            angle = math.atan2(siny_cosp, cosy_cosp)
            self.poses.append((x, y, angle))
            self.get_logger().info(f'Recorded pose: x={x:.2f}, y={y:.2f}, angle={angle:.2f}')
        else:
            self.get_logger().warn('No pose received yet.')

  
    def save_and_exit(self):
        filename = os.path.join(ROUTES_DIR, 'route_' + str(self.get_clock().now().to_msg().sec) + '.txt')
        with open(filename, 'w') as f:
            records = []
            for idx, (x, y, angle) in enumerate(self.poses):
                # Use current time for each pose (could be improved if you want actual pose time)
                t = time.time()
                record = f"\ntp{idx+1}\na(F{t}\nF{x:.6f}\nF{y:.6f}\nF{angle:.6f}"
                records.append(record)
            line = ''.join(records)
            f.write(line + '\n')
        self.get_logger().info(f'Saved {len(self.poses)} poses to {filename}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_exit()

if __name__ == '__main__':
    main()