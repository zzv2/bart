#!/usr/bin/env python
import rospy, sys
from math import *
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from copy import deepcopy
from arbotix_msgs import *

class Head_Controller:
	def __init__(self):

		rospy.init_node('Head_Controller')
		rospy.loginfo("Launched Head Controller")

		self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
		self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)

		self.head_pos_sub = rospy.Subscriber("bart/head_pos", Vector3, self.head_callback, queue_size=1)

		# self.tilt_set_speed_srv = rospy.ServiceProxy("/head_tilt_joint/set_speed", SetSpeed)
		# self.pan_set_speed_srv = rospy.ServiceProxy("/head_pan_joint/set_speed", SetSpeed)
		
		rospy.sleep(4)

		self.head_callback(Vector3(1,0,0))

		# spd_req = SetSpeed()
		# spd_req.speed = 10
		
		rospy.spin()

	def unit_vector(self, vector):
		return vector / np.linalg.norm(vector)

	def angle(self, v1, v2):
		v1_u = self.unit_vector(v1)
		v2_u = self.unit_vector(v2)
		cp = np.cross(v1_u,v2_u)
		return copysign(np.arccos(np.dot(v1_u, v2_u)), cp.item(2))

	def head_callback(self, pt):
		rospy.loginfo(pt)
		self.head_set_xyz(pt)

	def head_set_xyz(self, pt):
		rotx = atan2( pt.z, (pt.y**2+pt.x**2)**.5 )
		roty = atan2( pt.y * cos(rotx), pt.x )

		return self.head_set(roty,rotx)

	def head_set(self, pan, tilt):
		rospy.loginfo("pan: %f \t tilt: %f" % (pan, tilt))
		self.head_pan_pub.publish(pan)
		self.head_tilt_pub.publish(tilt)

		return True
if __name__ == "__main__":
	Head_Controller()