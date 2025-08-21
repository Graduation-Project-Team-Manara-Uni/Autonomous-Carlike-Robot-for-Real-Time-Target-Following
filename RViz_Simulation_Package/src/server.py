import rospy
from assign2.srv import *

def velocity_server(request):
	# returns new velcoity vector when requested. Changing response.vx, response.vy changes target path
	response = velocityResponse()
	response.vx = 0.0
	response.vy = -0.22
	return response

rospy.init_node('SERVER_NODE')
rospy.Service('/getVelocity', velocity, velocity_server)
rospy.spin()


