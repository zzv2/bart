#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from arbotix_python.arbotix import ArbotiX

OUTPUT = 255
LOW = 0
HI = 255
GREEN_PIN = 1
RED_PIN = 2
POS_HIGH = .3
POS_LOW = .1
NEG = .5
CONF_HIGH = .33
CONF_LOW = .25

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

class Light_Controller:
	def __init__(self):
		rospy.init_node('light_controller')
		rospy.loginfo("Launched Light Controller")


		self.device = ArbotixROS()
		rospy.Subscriber("bart/head_lights", String, self.lights_callback, queue_size=1)

		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)

		rospy.spin()

	def lights_callback(self, a):
		if a.data == "positive":
			self.positive()
		elif a.data == "confused":
			self.confused()
		elif a.data == "negative":
			self.negative()

	def positive(self):
		rospy.loginfo("positive...")
		for x in range(5):
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(POS_HIGH)
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(POS_LOW)
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(POS_HIGH)
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(POS_HIGH)

		rospy.loginfo("positive.")

	def negative(self):
		for x in range(5):
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(NEG)
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(NEG)
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(NEG)
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(NEG)

	def confused(self):
		for x in range(2):
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(CONF_HIGH)
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(CONF_HIGH)
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(CONF_HIGH)
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(CONF_HIGH)

		for x in range(3):
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(CONF_LOW)
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
			rospy.sleep(CONF_LOW)
			self.device.setDigital(RED_PIN,LOW,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(CONF_LOW)
			self.device.setDigital(RED_PIN,HI,OUTPUT)
			self.device.setDigital(GREEN_PIN,HI,OUTPUT)
			rospy.sleep(CONF_LOW)

	def neutral(self):
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)

if __name__ == "__main__":
	Light_Controller()