'''
The target node publishes the target global coordinates and the target path. The target is set to have straight line with constant velocity of (vx,vy) on XY gloabl axis. This velocity changes when the target reaches specified bound and the new velocity is brought from getVelocity server. The new velocity can be changes by changing response.vx, response.vy in serever.py script.
'''

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from assign2.srv import *
from geometry_msgs.msg import Point

rospy.init_node('target_node')
pub = rospy.Publisher('/target_position', Marker, queue_size = 10)
path_pub = rospy.Publisher('/target_path', Path, queue_size = 10)

rate = rospy.Rate(10)

path = Path()
path.header.frame_id = 'world'

radius = 0.1
marker = Marker()
marker.header.frame_id = "world"
marker.ns = "my_namespace"
marker.id = 0
marker.action = Marker.ADD
marker.pose.position.z = radius
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 0.3
marker.scale.x = 2*radius
marker.scale.y = 2*radius
marker.scale.z = 2*radius
marker.color.a = 1.0
marker.color.r = 0.1
marker.color.g = 0.2
marker.color.b = 0.7
marker.type = Marker.SPHERE
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0

# initial target position & velocity
x0 = 1; y0 = 0.5;
x = x0; y = y0
vx = 0.2; vy = 0.22

bound = 4

start_time = rospy.get_time()
point = Point()
def get_velocity_client():
	# request new target velocity
	rospy.wait_for_service('getVelocity')
	server = rospy.ServiceProxy('getVelocity', velocity)
	res = server()
	return res.vx, res.vy

done = False
while not rospy.is_shutdown():
	# adding current target position to target path
	pose = PoseStamped()
	pose.header.frame_id = 'world'
	pose.pose.position.x = x
	pose.pose.position.y = y
	path.poses.append(pose)
	path_pub.publish(path)
	
	# updating target position corresponding to its speed
	time = rospy.get_time() - start_time
	x = x0 + vx*time
	y = y0 + vy*time
	marker.pose.position.x = x
	marker.pose.position.y = y
	
	point.x=vx; point.y=vy
	
	# request new velocity when the target reaches bound
	if (x > bound or x <-bound or y > bound or y < -bound) and (done == False):
		x0=x
		y0=y
		start_time = rospy.get_time()
		vx,vy = get_velocity_client()
		done = True
	
	pub.publish(marker)
	rate.sleep()

