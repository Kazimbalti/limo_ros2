import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform
import math

def quaternion_to_yaw(x, y, z, w):
	# Convert quaternion to yaw (theta)
	siny_cosp = 2.0 * (w * z + x * y)
	cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
	return math.atan2(siny_cosp, cosy_cosp)

class PrintWaypointNode(Node):
	def __init__(self):
		super().__init__('print_waypoint')
		self.subscription = self.create_subscription(
			Transform,
			'/waypoint_cmd',
			self.waypoint_cb,
			10)
		self.curr_waypoint = None
		self.timer = self.create_timer(1.0, self.timer_callback)

	def waypoint_cb(self, msg):
		self.curr_waypoint = msg

	def timer_callback(self):
		if self.curr_waypoint is not None:
			x = self.curr_waypoint.translation.x
			y = self.curr_waypoint.translation.y
			q = self.curr_waypoint.rotation
			theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
			self.get_logger().info(f'Current waypoint (x,y): ({x},{y})')
			self.get_logger().info(f'Current waypoint (theta): ({theta})\n')

def main(args=None):
	rclpy.init(args=args)
	node = PrintWaypointNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
