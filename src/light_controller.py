#!/usr/bin/env python

import rospy, sys
from math import *
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

        # self.rate = rospy.get_param("~rate", 100.0)
        # self.fake = rospy.get_param("~sim", False)

        # self.use_sync_read = rospy.get_param("~sync_read",True)      # use sync read?
        # self.use_sync_write = rospy.get_param("~sync_write",True)    # use sync write?

        # setup publishers
        # self.diagnostics = DiagnosticsPublisher()
        # self.joint_state_publisher = JointStatePublisher()

        # start an arbotix driver
        # if not self.fake:
        ArbotiX.__init__(self, port, baud)        
        rospy.sleep(1.0)
        rospy.loginfo("Started ArbotiX connection on port " + port + ".")
        # else:
        #     rospy.loginfo("ArbotiX being simulated.")

OUTPUT = 255
LOW = 0
HI = 255
RED_PIN = 1
GREEN_PIN = 2
POS_HIGH = .3
POS_LOW = .1
NEG = .5
CONF_HIGH = .33
CONF_LOW = .25


class Light_Controller:
	def __init__(self):
		# Give the launch a chance to catch up
		# rospy.sleep(5)

		rospy.init_node('light_controller')
		rospy.loginfo("Launched Light Controller")

		# # constants
		# self.GROUP_NAME_ARM = 'arm'
		# self.GRIPPER_FRAME = 'gripper_link'
		# self.REFERENCE_FRAME = 'base_link'
		# self.ARM_BASE_FRAME = 'arm_base_link'

		# self.done = True

		# self.test_pose_publisher = rospy.Publisher('/test_arm_pose', PoseStamped)

		# rospy.Subscriber("/arm_target_pose", PoseStamped, self.move_arm_to_pose, queue_size=1)
		# self.robot_name = "gatlin"
		# move_arm_service = createService('gatlin/move/arm', MoveRobot, self.handle_move_arm)

		self.device = ArbotixROS()

		rospy.sleep(2)

		self.positive()
			
		# self.device.setDigital(RED_PIN,HI,OUTPUT)
		# self.device.setDigital(GREEN_PIN,LOW,OUTPUT)

		rospy.spin()

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
			self.device.se5tDigital(GREEN_PIN,HI,OUTPUT)
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
			self.device.se5tDigital(GREEN_PIN,HI,OUTPUT)
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