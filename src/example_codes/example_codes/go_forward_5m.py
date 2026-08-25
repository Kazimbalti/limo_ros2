import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class GoForward5m(Node):
    def __init__(self):
        super().__init__('goforward5m')        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.loops = 250 # 250 loops at 10 Hz, takes around 25s (250 x 0.1 HZ)

        # Go go forward at 0.2 m/s
        self.move_cmd = Twist()
        self.move_cmd.linear.x = 0.2


    def timer_callback(self):
        if self.count < self.loops:  # Continue moving forward
            self.get_logger().info("Going Straight")
            self.cmd_vel_pub.publish(self.move_cmd)
        else:  # Stop after 5 meters
            self.get_logger().info("Stopping")
            stop_cmd = Twist()  # Create a stop command
            self.cmd_vel_pub.publish(stop_cmd)
            self.timer.cancel()  # Stop the timer
        
        self.count += 1
        self.get_logger().info('count: %d' % self.count)

def main(args=None):
    rclpy.init(args=args)
    node = GoForward5m()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()