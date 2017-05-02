#!/usr/bin/env python
# Team: bart; Names: Zach Vinegar, 
import rospy
import scipy as sp
from scipy.integrate import quad
import numpy as np
import pylab as plt
from geometry_msgs.msg import *
from bart.msg import *
from copy import deepcopy

class Member:
	def __init__(self, name, seat, color):
		self.name = name
		self.seat = seat
		self.color = color

class Group
	def __init__(self, max_size):
		self.members = {}		
		self.max_size = max_size
		self.size = 0

	def join(self, cmd):
		rospy.loginfo("joining...")
		rospy.loginfo(cmd)
		if not cmd.seat in self.members.keys():
			self.members[cmd.seat] = Member(cmd.name, cmd.seat, cmd.color)
			rospy.loginfo(self.members[cmd.seat])
			rospy.loginfo("joined.")
			self.size += 1
		else:
			rospy.logerr("Seat %d is already taken" % cmd.seat)

	def leave(self, cmd):
		rospy.loginfo("leaving...")
		rospy.loginfo(cmd)
		if cmd.seat in self.members.keys():
			rospy.loginfo(self.members[cmd.seat])
			rospy.loginfo("left.")
			self.size -= 1
		else:
			rospy.logerr("Seat %d already has no one" % cmd.seat)

	def update(self, cmd):
		rospy.loginfo("updating...")
		rospy.loginfo(cmd)
		

class Server:
	def __init__(self):
		rospy.init_node("Bart_Server")
		rospy.loginfo("Initialized BART Server")

		rospy.Subscriber("bart/cmd", Cmd, cmdCb, queue_size=1)
		self.head_pos_pub = rospy.Publisher("bart/head_pos", Vector3, queue_size=1)

		self.max_size = 4
		self.group = Group(self.max_size)

		rospy.spin()

	def cmdCb(cmd):
		rospy.loginfo(cmd)
		if cmd.type == "J":
			self.group.join(cmd)
		elif cmd.type == "U":
			self.group.update(cmd)

if __name__ == "__main__":
	Server()
