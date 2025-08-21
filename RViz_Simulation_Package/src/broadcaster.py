'''
The broadcaster node shares the transformation of the robot relative to the origin axis
based on the pose published on topic '/robot_pos'. This allows the RViz to update robot
position during simulation.
'''
import rospy 
import tf 
from geometry_msgs.msg import Pose2D

x = 0; y = 0; th = 0

def callback(msg):
	# recieving robot pose
	global x, y, th
	x  = msg.x
	y  = msg.y
	th = msg.theta	

rospy.init_node('broadcaster_node') 
rospy.Subscriber('/robot_pose', Pose2D, callback)
br = tf.TransformBroadcaster() 
rate = rospy.Rate(10) 

while not rospy.is_shutdown():
	# updating robot visualization
	br.sendTransform((x, y, 0),tf.transformations.quaternion_from_euler(0,0,th), 
	rospy.Time.now(),"body","world")
	rate.sleep()
	
