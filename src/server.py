#!/usr/bin/env python
# Team: bart; Names: Zach Vinegar, 
import rospy
import scipy as sp
from scipy.integrate import quad
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from bart.msg import *
from bart.srv import *
from copy import deepcopy

class Member:
	def __init__(self, name, seat, color):
		self.name = name
		self.seat = seat
		self.color = color
		self.score = 0
		self.updates = 0
		self.total_score = 0
		self.total_updates = 0

		self.pos = None

	def change(self, v):
		self.score += v
		self.updates += 1
		self.total_score += v
		self.total_updates += 1

	def score_reset(self):
		self.score = 0
		self.updates = 0

class GroupC:
	def __init__(self, max_size):
		self.members = {}		
		self.max_size = max_size
		self.size = 0
		
		# rospy.wait_for_service('bart/head_move')
		self.head_move_srv = rospy.ServiceProxy('bart/head_move', MoveHead)
		self.head_get_srv = rospy.ServiceProxy('bart/head_get', GetHead)
		# self.head_pos_pub = rospy.Publisher("bart/head_pos", Vector3, queue_size=1)
		self.head_action_pub = rospy.Publisher("bart/head_action", String, queue_size=1)

		self.head_lights_pub = rospy.Publisher("bart/head_lights", String, queue_size=1)
		self.head_sound_pub = rospy.Publisher("bart/head_sound", String, queue_size=1)

		self.group_pub = rospy.Publisher("bart/group", Group, queue_size=1)

	def join(self, cmd):
		rospy.loginfo("joining...")
		rospy.loginfo(cmd)
		if not cmd.seat in self.members.keys():
			self.members[cmd.seat] = Member(cmd.name, cmd.seat, cmd.color)
			rospy.loginfo(self.members[cmd.seat])
			rospy.loginfo("joined.")
			self.size += 1
			self.refresh()
			return "joined"
		elif cmd.name == self.members[cmd.seat].name and cmd.color == self.members[cmd.seat].color:
			rospy.loginfo("rejoined.")
			self.refresh()
			return "rejoined"
		else:
			err = "Seat %d is already taken" % cmd.seat
			rospy.logerr(err)
			self.refresh()
			return err

	def leave(self, cmd):
		rospy.loginfo("leaving...")
		rospy.loginfo(cmd)
		res = self.members.pop(cmd.seat, None)
		if res != None:
			self.size -= 1
			self.refresh()
			rospy.loginfo("left.")
			return "left"
		else:
			err = "Seat %d already has no one" % cmd.seat
			rospy.logerr(err)
			self.refresh()
			return err

	def change(self, cmd, v):
		rospy.loginfo("changing...")
		if cmd.seat in self.members.keys():
			m = self.members[cmd.seat]
			m.change(v)
			rospy.loginfo("score: %d" % m.score)
			rospy.loginfo("updates: %d" % m.updates)
			rospy.loginfo("avg: %.3f" % (float(m.score)/m.updates))
			rospy.loginfo("total_score: %d" % m.total_score)
			rospy.loginfo("total_updates: %d" % m.total_updates)
			rospy.loginfo("total_avg: %.3f" % (float(m.total_score)/m.total_updates))
			rospy.loginfo("changed.")
			return "changed"
		else:
			err = "Seat %d has no one" % cmd.seat
			rospy.logerr(err)
			self.refresh()
			return err

	def score_reset(self, s):
		if s in self.members.keys():
			m = self.members[s]
			m.score = 0
			m.updates = 0

	def score_reset_all(self):
		for s,m in self.members:
			m.score = 0
			m.updates = 0

	def refresh(self):
		rospy.loginfo("updating...")
		g = Group()
		g.type = "U"
		g.size = self.max_size
		for key in self.members:
			m = self.members[key]
			# rospy.loginfo(m)
			g.name.append(m.name)
			g.seat.append(m.seat)
			g.color.append(m.color)

		rospy.loginfo(g)
		self.group_pub.publish(g)

	def get_pos(self, s):
		if s in self.members.keys():
			m = self.members[s]
			if m.pos != None:
				return m.pos
	def get_circular_pos(self, s):
		if 1 <= s and s <= self.max_size:
			n = self.max_size
			a = 2*np.pi/n
			i = int(s-1)
			x = np.cos(i*a)
			y = np.sin(i*a)
			return Vector3(x,y,0)
		else:
			rospy.logerr("Invalid Seat")
			return Vector3()

	def look_at(self, s):
		rospy.loginfo("Look At")
		pos = self.get_pos(s)
		if pos != None:
			pan, tilt = pos
			rospy.loginfo("using set pos")
			rospy.loginfo(pos)
			res = self.move_set_wait(pan, tilt, 2)
		else:
			pos = self.get_circular_pos(s)
			rospy.loginfo("using circular pos")
			rospy.loginfo(pos)
			# self.head_pos_pub.publish(pos)
			res = self.move_pos_wait(pos)
		return res

	def move_relax_wait(self, pan=False, tilt=False):
		try:
			pan_r = 0.0 if pan else 1.0
			tilt_r = 0.0 if tilt else 1.0
			resp = self.head_move_srv("R", pan_r, tilt_r, 0.0)
			return resp.resp
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return "err"

	def move_set_wait(self, pan, tilt, time):
		try:
			resp = self.head_move_srv("S", pan, tilt, time)
			return resp.resp
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return "err"

	def move_pos_wait(self, pt):
		try:
			resp = self.head_move_srv("L",pt.x,pt.y,pt.z)
			return resp.resp
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s"%e)
			return "err"

	def feedback(self, a):
		if a.seat in self.members.keys():
			# look at the seat
			rospy.loginfo("feedback for seat %d..." % a.seat)
			res = self.look_at(a.seat)
			# rospy.sleep(4)
			if res == "moved" or res == "set":
				m = self.members[a.seat]
				rospy.loginfo("m.score: %d" % m.score)

				# choose behavior based on score
				low, high = -1, 2
				if m.score < low:
					behavior = "negative"
				elif m.score < high:
					behavior = "confused"
				else:
					behavior = "positive"

				# execute behavior
				self.execute(behavior)

				# reset
				m.score = 0
				m.updates = 0
				return "feedback"
			else:
				rospy.logerr(res)
				return res
		else:
			rospy.logerr("seat invalid")
			return "seat invalid"

	def execute(self, b):
		rospy.loginfo("execute: %s" % b)
		# call all controllers
		self.head_action_pub.publish(b)
		self.head_lights_pub.publish(b)
		self.head_sound_pub.publish(b)
		rospy.sleep(4)
		return "executed"

	def relax_pan(self, a):
		if a.seat in self.members.keys():
			self.move_relax_wait(pan=True, tilt=False)
			return "relaxed"
		else:
			rospy.logerr("seat invalid")
			return "seat invalid"

	def set_pos(self, a):
		if a.seat in self.members.keys():
			m = self.members[a.seat]
			resp = self.head_get_srv("P")
			if resp.resp == "got":
				m.pos = list(resp.data)
				rospy.loginfo("Updated Pos")
				rospy.loginfo(m.pos)
				return "set"
		else:
			rospy.logerr("seat invalid")
			return "seat invalid"


class Server:
	def __init__(self):
		rospy.init_node("Bart_Server")
		rospy.loginfo("Initialized BART Server")

		rospy.Service('bart/action', Action, self.actionCb)

		self.max_size = 3
		self.group = GroupC(self.max_size)

		rospy.spin()

	def actionCb(self, action):
		rospy.loginfo(action)
		if action.type == "J":
			resp = self.group.join(action)
			return ActionResponse(resp)
		elif action.type == "I":
			resp = self.group.change(action, 1)
			return ActionResponse(resp)
		elif action.type == "D":
			resp = self.group.change(action, -1)
			return ActionResponse(resp)
		elif action.type == "L":
			resp = self.group.leave(action)
			return ActionResponse(resp)
		elif action.type == "A":
			resp = self.group.look_at(action.seat)
			return ActionResponse(resp)
		elif action.type == "F":
			resp = self.group.feedback(action)
			return ActionResponse(resp)
		elif action.type == "E":
			resp = self.group.execute(action.name)
			return ActionResponse(resp)
		elif action.type == "R":
			resp = self.group.relax_pan(action)
			return ActionResponse(resp)
		elif action.type == "S":
			resp = self.group.set_pos(action)
			return ActionResponse(resp)

if __name__ == "__main__":
	Server()
