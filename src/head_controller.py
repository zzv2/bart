#!/usr/bin/env python
import rospy, sys
from math import *
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from copy import deepcopy

class Head_Controller:
	def __init__(self):

		rospy.init_node('Head_Controller')
		rospy.loginfo("Launched Head Controller")

		self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
		self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)

		self.head_pos_sub = rospy.Subscriber("bart/head_pos", Vector3, self.head_callback, queue_size=3)
		
		rospy.spin()

	def unit_vector(self, vector):
		return vector / np.linalg.norm(vector)

	def angle(self, v1, v2):
		v1_u = self.unit_vector(v1)
		v2_u = self.unit_vector(v2)
		cp = np.cross(v1_u,v2_u)
		return copysign(np.arccos(np.dot(v1_u, v2_u)), cp.item(2))

	def head_callback(self, pt):
		self.head_set_xyz(pt):

	def head_set_xyz(self, pt):
		rotx = atan2( pt.y, (pt.x**2+pt.z**2)**.5 )
		roty = atan2( pt.x * cos(rotx), pt.z )

		return self.head_set(roty,-rotx)

if __name__ == "__main__":
	Head_Controller()