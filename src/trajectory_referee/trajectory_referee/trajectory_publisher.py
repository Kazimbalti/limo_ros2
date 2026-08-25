#!/usr/bin/env python
import math

PKG_NAME = "trajectory_referee"  # Make sure this matches the actual package directory name under 'share'

XY_CLOSENESS_THRESH=0.2
COSTHETA_CLOSENESS_THRESH=0.5-0.5*math.cos(math.pi/4)
#TIMEOUT=100

#PERTURBATIONS_SIZE=1
PERTURBATIONS_SIZE=0.01
#PERTURBATIONS_SIZE=0

#import roslib
#roslib.load_manifest(PKG_NAME)
import rclpy
import geometry_msgs
import geometry_msgs.msg
import gazebo_msgs
import gazebo_msgs.srv
import gazebo_msgs.msg
import visualization_msgs
import visualization_msgs.msg
import trajectory_msgs
import trajectory_msgs.msg
#import arm_navigation_msgs
#import arm_navigation_msgs.msg
import sys
import time
#import tf
#import tf.transformations
import tf_transformations as tf
import signal
import pickle
import random
from rclpy.node import Node
import os
from builtin_interfaces.msg import Duration
import threading

#import roslib.packages
# Replace roslib.packages.get_pkg_dir with a function that finds the package directory
def get_pkg_dir(pkg_name):
  # Try to get package path from environment variable
  ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
  for path in ament_prefix_path.split(':'):
    candidate = os.path.join(path, 'share', pkg_name)
    if os.path.isdir(candidate):
      return candidate
  # Fallback: try src directory relative to this file
  this_dir = os.path.dirname(os.path.abspath(__file__))
  candidate = os.path.join(this_dir, '..', '..')
  candidate = os.path.abspath(candidate)
  if os.path.isdir(candidate):
    return candidate
  raise FileNotFoundError(f"Could not find package directory for {pkg_name}")

# Use get_pkg_dir instead of roslib.packages.get_pkg_dir
if len(sys.argv)<3:
  print ('''Please give arguments: 
            First argument: which route? One of route1, route2, route3 or route4.
            Second argument: displacement following only (dis) or displacement and orientation (dor).
            e.g.: 
            rosrun trajectory_referee route1 dis''')

if sys.argv[1] not in ["route1","route2","route3"]:
  print ("Please give one of route1, route2, route3 or route4 for first argument.")
  sys.exit(1)

if sys.argv[2] not in ["dis","dor"]:
  print ("Please give one of dis or dor for second argument.")
  sys.exit(1)

orientation_check_on = False
if sys.argv[2].lower()=="dor":orientation_check_on=True

routefname = os.path.join(get_pkg_dir(PKG_NAME), "routes", sys.argv[1] + ".pkl")
if not os.path.isfile(routefname):
    print(f"Route file not found: {routefname}")
    sys.exit(1)
with open(routefname, "rb") as f:  # Use "rb" for reading pickle files in binary mode
    route_raw = pickle.load(f)

route=trajectory_msgs.msg.MultiDOFJointTrajectory()

route.joint_names=["mobile_base"]
route.points=[]

marker_array=visualization_msgs.msg.MarkerArray()
marker_cntr=0
for ritem in route_raw:
  route_point=trajectory_msgs.msg.MultiDOFJointTrajectoryPoint()
  route_point.time_from_start = Duration(sec=int(ritem[0]), nanosec=int((ritem[0] % 1) * 1e9))

  transform=geometry_msgs.msg.Transform()
  transform.translation.x=ritem[1]
  transform.translation.y=ritem[2]
  q=tf.quaternion_from_euler(0,0,ritem[3])
  transform.rotation.z=q[2]
  transform.rotation.w=q[3]
  transform.rotation.x=q[0]
  transform.rotation.y=q[1]

  route_point.transforms.append(transform)

  route.points.append(route_point)

  marker=visualization_msgs.msg.Marker()
  marker.header.frame_id="odom"
  marker.id=marker_cntr
  marker_cntr=marker_cntr+1
  marker.type=visualization_msgs.msg.Marker.ARROW
  marker.pose.position.x=ritem[1]
  marker.pose.position.y=ritem[2]
  marker.pose.position.z=0.0
  marker.pose.orientation=transform.rotation
  marker_array.markers.append(marker)
  marker.scale.x=0.2
  marker.scale.y=0.02
  marker.scale.z=0.02
  marker.color.r=1.0
  marker.color.g=(1.0*marker_cntr)/len(route_raw)
  marker.color.b=1.0-(1.0*marker_cntr)/len(route_raw)
  marker.color.a=1.0

rclpy.init()
node = rclpy.create_node('referee')
rate_hz = 10
rate_period = 1.0 / rate_hz

route_pub = node.create_publisher(trajectory_msgs.msg.MultiDOFJointTrajectory, '/route_cmd', 1)
waypoint_pub = node.create_publisher(geometry_msgs.msg.Transform, '/waypoint_cmd', 1)
marker_pub = node.create_publisher(visualization_msgs.msg.MarkerArray, '/visualization_marker_array', 1)

get_state_client = node.create_client(gazebo_msgs.srv.GetEntityState, "/gazebo/get_entity_state")
apply_wrench_client = node.create_client(gazebo_msgs.srv.ApplyBodyWrench, "/gazebo/apply_body_wrench")

'''while not get_state_client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('Waiting for get_model_list service...')
while not apply_wrench_client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('Waiting for /gazebo/apply_link_wrench service...')
'''



print ("Publishing route...")
try:
    #rate.sleep()
    route_pub.publish(route)
    waypoint_pub.publish(route.points[0].transforms[0])
    print ("Published route. Now it's over to you (will publish it again occasionally & also publish individual waypoints).")
except KeyboardInterrupt:
    print ("ROS seems to have died - oh well too bad - probably the divil - bye.")
    sys.exit(0)


start_time=time.time()
route_cntr=0
cumulative_dur_error=0
loop_cntr=0

try:
  while route_cntr < len(route_raw) and rclpy.ok():

      loop_cntr = loop_cntr + 1
      
      marker_pub.publish(marker_array)

      # Prepare request for GetModelState
      get_state_req = gazebo_msgs.srv.GetEntityState.Request()
      get_state_req.name = "limo_gazebo"
      get_state_req.reference_frame = "world"
      future = get_state_client.call_async(get_state_req)
     
      rclpy.spin_until_future_complete(node, future)
      sget = future.result()
    
      x = sget.state.pose.position.x
      y = sget.state.pose.position.y

      o = sget.state.pose.orientation
      q = [o.x, o.y, o.z, o.w]
      ga, gb, theta = tf.euler_from_quaternion(q)

      # publish waypoints constantly
      wp = route.points[route_cntr].transforms[0]
      waypoint_pub.publish(wp)

      # publish route again every second, just in case it was missed the first time.
      if loop_cntr % 100 == 0:
          route_pub.publish(route)

      # add some fun and yet mean noise
      # if loop_cntr % 20 == 0 and PERTURBATIONS_SIZE > 0:
      #     wrench = geometry_msgs.msg.Wrench()
      #     wrench.force.x = (random.random() - 0.5) * PERTURBATIONS_SIZE
      #     wrench.force.y = (random.random() - 0.5) * PERTURBATIONS_SIZE
      #     wrench.force.z = (random.random() - 0.5) * PERTURBATIONS_SIZE
      #     wrench.torque.x = (random.random() - 0.5) * PERTURBATIONS_SIZE
      #     wrench.torque.y = (random.random() - 0.5) * PERTURBATIONS_SIZE
      #     wrench.torque.z = (random.random() - 0.5) * PERTURBATIONS_SIZE

      #     apply_wrench_req = gazebo_msgs.srv.ApplyBodyWrench.Request()
      #     apply_wrench_req.body_name = "mobile_base::base_footprint"
      #     apply_wrench_req.wrench = wrench
      #     apply_wrench_req.duration = Duration(sec=0, nanosec=int(1e9/100))
      #     future_wrench = apply_wrench_client.call_async(apply_wrench_req)
      #     #rclpy.spin_until_future_complete(node, future_wrench)

      this_point = route_raw[route_cntr]

      linear_error = math.sqrt((x - this_point[1]) ** 2 + (y - this_point[2]) ** 2)
      angle_error = 0.5 - 0.5 * math.cos((theta - this_point[3]))

      if linear_error <= XY_CLOSENESS_THRESH and (not orientation_check_on or angle_error <= COSTHETA_CLOSENESS_THRESH):

          print("ACHIEVED WAYPOINT ", route_cntr)
          print("  (linear error", linear_error, " angle_error", angle_error, ")")
          marker_array.markers[route_cntr].color.a = 0.35
          route_cntr = route_cntr + 1
      time.sleep(rate_period)
except KeyboardInterrupt:
    pass

#rclpy.shutdown()
    

