#!/usr/bin/env python
import rospy, sys
from math import *
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from actionlib import *
from bart.srv import *

class Head_Controller:
	def __init__(self):

		rospy.init_node('Head_Controller')

		self.client = SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		# self.client.wait_for_server()

		self.head_pos_sub = rospy.Subscriber("bart/head_set", Vector3, self.head_set_callback, queue_size=1)
		self.head_pos_sub = rospy.Subscriber("bart/head_pos", Vector3, self.head_pos_callback, queue_size=1)
		self.head_action_sub = rospy.Subscriber("bart/head_action", String, self.head_action_callback, queue_size=1)

		self.head_move_srv = rospy.Service('bart/head_move', MoveHead, self.handle_head_move)

		rospy.loginfo("Launched Head Controller")
		rospy.sleep(1)
		
		self.tilt_offset = -np.pi/2+.3
		self.go_to(0,self.tilt_offset, 1)

		self.pan, self.tilt = 0, self.tilt_offset

		rospy.spin()

	def handle_head_move(self, req):
		pt = Vector3(req.x, req.y, req.z)
		self.head_pos_callback(pt)
		return MoveHeadResponse("moved")

	def make_goal(self):
		_goal = FollowJointTrajectoryGoal()
		_goal.goal_time_tolerance = rospy.Time(0.1)
		_goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
		_goal.trajectory.header.stamp = rospy.Time.now()
		return _goal

	def add_point(self, _goal, pan, tilt, time):
		point = JointTrajectoryPoint()
		point.positions = [pan, tilt]
		point.time_from_start = rospy.Duration(time)
		_goal.trajectory.points.append(point)

	def go_to(self, pan, tilt, time):
		_goal = self.make_goal()
		self.add_point(_goal, pan, tilt, 0)
		self.add_point(_goal, pan, tilt, time)
		self.client.send_goal(_goal)
		self.client.wait_for_result()
		res = self.client.get_result()
		if res.error_code == 0:
			self.pan, self.tilt = pan, tilt
		
	def positive(self):
		rospy.loginfo("positive")
		# nod
		a, pause = 0.2, 0.4
		pan, tilt = self.pan, self.tilt

		_goal = self.make_goal()
		self.add_point(_goal, pan, tilt+0, 0*pause)
		self.add_point(_goal, pan, tilt+a, 1*pause)
		self.add_point(_goal, pan, tilt+0, 2*pause)
		self.add_point(_goal, pan, tilt-a, 3*pause)
		self.add_point(_goal, pan, tilt+0, 4*pause)
		self.add_point(_goal, pan, tilt+a, 5*pause)
		self.add_point(_goal, pan, tilt+0, 6*pause)

		self.client.send_goal(_goal)
		self.client.wait_for_result()

	def negative(self):
		rospy.loginfo("negative")
		# shake
		a, pause = 0.2, 0.4
		pan, tilt = self.pan, self.tilt

		_goal = self.make_goal()
		self.add_point(_goal, pan+0, tilt, 0*pause)
		self.add_point(_goal, pan+a, tilt, 1*pause)
		self.add_point(_goal, pan+0, tilt, 2*pause)
		self.add_point(_goal, pan-a, tilt, 3*pause)
		self.add_point(_goal, pan+0, tilt, 4*pause)
		self.add_point(_goal, pan+a, tilt, 5*pause)
		self.add_point(_goal, pan+0, tilt, 6*pause)

		self.client.send_goal(_goal)
		self.client.wait_for_result()

	def confused(self):
		rospy.loginfo("confused")
		# figure 8
		a, pause = 0.2, 0.4
		pan, tilt = self.pan, self.tilt

		_goal = self.make_goal()
		self.add_point(_goal, pan+0, tilt+0, 0*pause)
		self.add_point(_goal, pan+a, tilt+a, 1*pause)
		self.add_point(_goal, pan+a, tilt-a, 3*pause)
		self.add_point(_goal, pan-a, tilt+a, 5*pause)
		self.add_point(_goal, pan-a, tilt-a, 7*pause)
		self.add_point(_goal, pan+0, tilt+0, 8*pause)

		self.client.send_goal(_goal)
		self.client.wait_for_result()

	def head_action_callback(self, a):
		if a.data == "positive":
			self.positive()
		elif a.data == "confused":
			self.confused()
		elif a.data == "negative":
			self.negative()

	def head_pos_callback(self, pt):
		rospy.loginfo(pt)
		self.head_set_xyz(pt)

	def head_set_xyz(self, pt):
		rotx = atan2( pt.z, (pt.y**2+pt.x**2)**.5 )
		roty = atan2( pt.y * cos(rotx), pt.x )

		return self.head_set(roty,rotx)

	def head_set_callback(self, s):
		self.go_to(s.x,s.y, 1)

	def head_set(self, pan, tilt):
		tilt += self.tilt_offset
		rospy.loginfo("pan: %f \t tilt: %f" % (pan, tilt))
		self.go_to(pan, tilt, 1)

		return True

if __name__ == "__main__":
	Head_Controller()