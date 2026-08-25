import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math

class PrintPositionNode(Node):
	def __init__(self):
		super().__init__('print_position')
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.timer = self.create_timer(1.0, self.timer_callback)

	def timer_callback(self):
		try:
			now = rclpy.time.Time()
			trans: TransformStamped = self.tf_buffer.lookup_transform(
				'odom',
				'base_footprint',
				now,
				timeout=rclpy.duration.Duration(seconds=1.0)
			)
			x = trans.transform.translation.x
			y = trans.transform.translation.y
			self.get_logger().info(f'Robot position (x,y): ({x},{y})')

			q = trans.transform.rotation
			# Convert quaternion to yaw (theta)
			siny_cosp = 2 * (q.w * q.z + q.x * q.y)
			cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
			theta = math.atan2(siny_cosp, cosy_cosp)
			self.get_logger().info(f'Robot orientation (theta): ({theta})\n')
		except Exception as ex:
			self.get_logger().error(f'Could not transform: {ex}')

def main(args=None):
	rclpy.init(args=args)
	node = PrintPositionNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
