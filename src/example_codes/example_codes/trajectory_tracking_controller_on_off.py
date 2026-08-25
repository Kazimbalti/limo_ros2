import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform, Twist
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import math

class TrajectoryTrackingController(Node):
    def __init__(self):
        super().__init__('smart_driver')
        self.waypoint = None
        self.robot_pose = None

        self.subscription = self.create_subscription(
            Transform,
            '/waypoint_cmd',
            self.waypoint_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def waypoint_callback(self, msg):
        self.waypoint = msg

    def timer_callback(self):
        # Obtain current robot pose
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time())
            self.robot_pose = trans
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error(f"Transform error: {ex}")
            return

        # Print current robot pose
        if self.robot_pose:
            x = self.robot_pose.transform.translation.x
            y = self.robot_pose.transform.translation.y
            self.get_logger().info(f"Robot is believed to be at (x,y): ({x},{y})")

            q = self.robot_pose.transform.rotation
            theta = self.quaternion_to_yaw(q)
            self.get_logger().info(f"Robot is believed to have orientation (theta): ({theta})")

        # Print current destination
        if self.waypoint:
            wx = self.waypoint.translation.x
            wy = self.waypoint.translation.y
            self.get_logger().info(f"Current waypoint (x,y): ({wx},{wy})")

            wq = self.waypoint.rotation
            wtheta = self.quaternion_to_yaw(wq)
            self.get_logger().info(f"Current waypoint (theta): ({wtheta})")

        # DRIVE THE ROBOT HERE 
        
        
        # Compute the distance and angle error
        dx = wx - x
        dy = wy - y
        distance_error = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - theta

        # Normalize angle error to [-pi, pi]
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        # On/Off linear speed controller
        if distance_error > 0.1:
            linear_speed = 0.2 
        else: 
            linear_speed = 0.0


        # # If angle error is significant, rotate in place
        # if abs(angle_error) > 0.2:
        #     linear_speed = 0.0

        # On/Off angular speed controller
        if angle_error > 0:
            angular_speed = 0.5  
        else:
            angular_speed = -0.5


        motor_command = Twist()
        motor_command.linear.x = linear_speed  # robot expects smart values, not 0.1
        motor_command.angular.z = angular_speed # robot expects smart values, not 0.1
        self.publisher.publish(motor_command)

  
    @staticmethod
    def normalize_to_pi(angle):
        """Normalize/Wrap any angle to (-pi, pi]."""
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def quaternion_to_yaw(q):
        # Convert quaternion to yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
