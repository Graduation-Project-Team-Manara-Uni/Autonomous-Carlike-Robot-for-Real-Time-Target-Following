'''
This node simulates the image processing of the camera frame that returns the coordiates of the target relative to the camera frame axis. It uses the published target position and robot pose to calculate the xy of the target in camera frame and publish it.
'''


import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D, Point

import math 
import random 

rospy.init_node('camera_node') 
pub = rospy.Publisher('/target_in_camera_axis', Point, queue_size=10) 

target_xy = Point()

Camera_Offset = 0.1
xt=0.0; yt=0.0

xc=Camera_Offset; yc=0.0; thr=0.0

def callback1(msg):
	# recieving target coordinates
	global xt, yt 
	xt = msg.pose.position.x
	yt= msg.pose.position.y

def callback2(msg):
	# recieving robot pose and finding corresponding camera coordinates
	global thr, xc, yc
	xr = msg.x
	yr = msg.y
	thr = msg.theta
	xc = xr + Camera_Offset*math.cos(thr)
	yc = yr + Camera_Offset*math.sin(thr)


rospy.Subscriber('/target_position', Marker , callback1)
rospy.Subscriber('/robot_pose', Pose2D , callback2)
rate = rospy.Rate(10)

while not rospy.is_shutdown(): 
	# calculating the target coordinates in camera axis
	L = math.sqrt((xt-xc)**2+(yt-yc)**2)
	thc = math.atan2(yt-yc,xt-xc)
	th = thc - thr
	
	target_xy.z = L * math.cos(th)
	target_xy.x = L * math.sin(th)

	pub.publish(target_xy) 
	rate.sleep()
	
