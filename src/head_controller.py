#!/usr/bin/env python
import rospy, sys
from math import *
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from copy import deepcopy
from arbotix_python.arbotix import ArbotiX

class ArbotixROS(ArbotiX):
    
    def __init__(self):
        # pause = False

        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = int(rospy.get_param("~baud", "115200"))

        # start an arbotix driver
        ArbotiX.__init__(self, port, baud)        
        rospy.sleep(2.0)
        rospy.loginfo("Started ArbotiX connection on port " + port + ".")

class Servo:
	def __init__(self, device, name, id):
		self.device = device
		self.name = name
		self.id = id
		self.cmd_pub = rospy.Publisher("/%s_joint/command" % name, Float64, queue_size=1)
		# self.set_speed_srv = rospy.ServiceProxy("/%s_joint/set_speed" % name, SetSpeed)
		self.position = 0

	def publish(self, cmd):
		self.cmd_pub.publish(cmd)

	def rad_to_enc(self, rad):
		r = 2.62
		return ((r+rad)/(2*r))*1024

	def go_to(self, rad):
		# rospy.loginfo("%s going..." % self.name)
		self.cmd_pub.publish(rad)

		tolerance = 3
		dist = self.rad_to_enc(rad)-self.position
		rate = rospy.Rate(10)
		while abs(dist) > tolerance and not rospy.is_shutdown():
			dist = self.rad_to_enc(rad)-self.position
			# rospy.loginfo("dist: "+str(dist))
			# rospy.loginfo("rad: "+str(self.rad_to_enc(rad)))
			# rospy.loginfo("position: "+str(self.position))
			rate.sleep()
		# rospy.loginfo("%s gone." % self.name)

	def reset(self):
		self.cmd_pub.publish(0.0)

	def update(self):
		p = -1
		while p == -1:
			p = self.device.getPosition(self.id)
		self.position = p
		# self.voltage = self.device.getVoltage(self.id)
		# self.speed = self.device.getSpeed(self.id)
		# rospy.logwarn("servo %d (%s): %d" % (self.id, self.name, self.position))

class Head_Controller:
	def __init__(self):

		rospy.init_node('Head_Controller')
		rospy.loginfo("Launched Head Controller")

		self.device = ArbotixROS()

		self.head_tilt_servo = Servo(self.device, "head_tilt", 7)
		self.head_pan_servo = Servo(self.device, "head_pan", 6)

		self.head_pos_sub = rospy.Subscriber("bart/head_set", Vector3, self.head_set_callback, queue_size=1)
		self.head_pos_sub = rospy.Subscriber("bart/head_pos", Vector3, self.head_pos_callback, queue_size=1)
		self.head_action_sub = rospy.Subscriber("bart/head_action", String, self.head_action_callback, queue_size=1)

		rospy.sleep(4)

		# self.head_pos_callback(Vector3(1,0,0))

		rate = rospy.Rate(5)
		while not rospy.is_shutdown():
			self.head_tilt_servo.update()
			self.head_pan_servo.update()
			rate.sleep()

		rospy.spin()

	def head_action_callback(self, a):
		if a == "positive":
			# nod
			rospy.sleep(2)
		elif a == "confused":
			# figure 8
			rospy.sleep(2)
		elif a == "negative":
			# shake
			rospy.sleep(2)

	def unit_vector(self, vector):
		return vector / np.linalg.norm(vector)

	def angle(self, v1, v2):
		v1_u = self.unit_vector(v1)
		v2_u = self.unit_vector(v2)
		cp = np.cross(v1_u,v2_u)
		return copysign(np.arccos(np.dot(v1_u, v2_u)), cp.item(2))

	def head_pos_callback(self, pt):
		rospy.loginfo(pt)
		self.head_set_xyz(pt)

	def head_set_xyz(self, pt):
		rotx = atan2( pt.z, (pt.y**2+pt.x**2)**.5 )
		roty = atan2( pt.y * cos(rotx), pt.x )

		return self.head_set(roty,rotx)

	def head_set_callback(self, s):
		self.head_pan_servo.go_to(s.x)
		self.head_tilt_servo.go_to(s.y)

	def head_set(self, pan, tilt):
		tilt -= np.pi/2 - .1
		rospy.loginfo("pan: %f \t tilt: %f" % (pan, tilt))
		self.head_pan_servo.publish(pan)
		self.head_tilt_servo.publish(tilt)

		return True
if __name__ == "__main__":
	Head_Controller()