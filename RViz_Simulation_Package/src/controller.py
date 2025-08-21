'''
The controller node calculates the velocity and steering angle and updates and publishes the robot pose accordingly.
'''
import rospy 
import tf
import math 

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point

#Initial Values & Parameters
Vx  = 0  ; Vy  = 0; w  = 0; K = 0.5
x   = 0  ; y   = 0; th = 0; angle = 0
d   = 0.1; D = 2*d; Dc = D

Zt  = 1; Xt = 0.5; Ld = 0.4
dXt = 0; dZt = 0

Zt_pr = 1; Xt_pr = 0.5; L = math.sqrt(Xt**2+Zt**2);

tc = 0; tc_pr = 0

def callbacK(msg): 
	global Xt , dXt, Xt_pr, Zt, dZt, Zt_pr, tc_pr, L, angle
	Zt = msg.z
	Xt = msg.x
	# (Xt,Zt) are the cooridnates of the target in the camera axis published by camera node
	L = math.sqrt(Xt**2+Zt**2)
	angle = math.atan(Xt/Zt);
	tc = rospy.get_time();
	dtc = tc - tc_pr
	dZt = (Zt - Zt_pr)
	dXt = (Xt - Xt_pr)
	Zt_pr = Zt
	Xt_pr = Xt
	tc_pr = tc
	
rospy.init_node('controller_node') 
tc_pr = rospy.get_time()
rospy.Subscriber('/target_in_camera_axis', Point, callbacK)
pub = rospy.Publisher('/robot_pose', Pose2D, queue_size = 10)
path_pub = rospy.Publisher('/robot_path', Path, queue_size = 10)
rate = rospy.Rate(10)

robot_path = Path()
robot_path.header.frame_id = 'world'

pose = Pose2D()

last_time = rospy.get_time()
Xt_pr = Xt; Zt_pr = Zt
while not rospy.is_shutdown():
	# Adding current robot position to the robot path
	point = PoseStamped()
	point.header.frame_id = 'world'
	point.pose.position.x = x
	point.pose.position.y = y
	robot_path.poses.append(point)
	path_pub.publish(robot_path)
	
	pose.x = x; pose.y = y; pose.theta = th
	pub.publish(pose)
	
	# Update robot pose corresponding to the linear and angular velocities found by the controller
	time = rospy.get_time()
	dt = time - last_time
	last_time = time
	x = x + Vx*dt ; y = y + Vy*dt; th = th + w*dt;
	# Vtx, Vty represent estimation of target speed
	Vtx =  dZt 
	Vty =  dXt 
	
	# Vcx, Vcy are the desired velocities at camera point calculated by the controller
	Vcx=Vtx+K*(L-Ld)*math.cos(angle)
	Vcy=Vty+K*(L-Ld)*math.sin(angle)
	
	# Vgx, Vgy are the coordinates of the velocity of Center of Grabity in the local robot axis
	Vgx = Vcx
	Vgy = 0.5 * Vcy
	
	# Vx, Vy are the coordinates of the velocity of the Center of Gravity in the global axis
	Vx = Vgx*math.cos(th) - Vgy*math.sin(th)
	Vy = Vgx*math.sin(th) + Vgy*math.cos(th)
	w = Vcy / D
	
	rate.sleep() 

