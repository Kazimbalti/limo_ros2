# file: closest_scan_marker_node.py
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Duration

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class ClosestScanMarker(Node):
    def __init__(self):
        super().__init__('closest_scan_marker')

        self.marker_pub = self.create_publisher(Marker, '/closest_obstacle_marker', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 1)

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def on_scan(self, msg: LaserScan):
        
        print(len(msg.ranges))
        
        #################  START WRITING YOUR CODE HERE #################

        # Find closest range reading and its angle
        closest_range = 0
        closest_angle = 0

        # Conver to (x,y) in scan frame
        x = 0
        y = 0
        z = 0

        ################# END OF WRITING YOUR CODE  #################

        point = PointStamped()
        point.header = msg.header  # source frame = scan's frame
        point.point.x = x
        point.point.y = y
        point.point.z = z

        target_frame = 'odom'
        source_frame = 'laser_link'

        try:
            # get the transform laser point in laser link frame to odom frame
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            #here we directly transofrm the pose into another pose for the given frame of reference
            point = do_transform_point(point, transform)
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
       
        # Build marker (in odom)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'closest_scan'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.marker_pub.publish(marker)

        self.get_logger().info('closest_scan_marker node started (publishing in odom).')


def main():
    rclpy.init()
    node = ClosestScanMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
