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
		self.device.setDigital(RED_PIN,HI,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,HI,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)
		rospy.loginfo("positive.")

	def negative(self):
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,HI,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,HI,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)

	def confused(self):
		self.device.setDigital(RED_PIN,HI,OUTPUT)
		self.device.setDigital(GREEN_PIN,HI,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,HI,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,HI,OUTPUT)
		rospy.sleep(2)
		self.device.setDigital(RED_PIN,HI,OUTPUT)
		self.device.setDigital(GREEN_PIN,HI,OUTPUT)
		rospy.sleep(2)

	def neutral(self):
		self.device.setDigital(RED_PIN,LOW,OUTPUT)
		self.device.setDigital(GREEN_PIN,LOW,OUTPUT)
		rospy.sleep(2)

if __name__ == "__main__":
	Light_Controller()