#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Sound_Controller:
	def __init__(self):
		rospy.init_node('Sound_controller')
		rospy.loginfo("Launched Sound Controller")

		self.soundhandle = SoundClient()
		self.soundAssets = "/home/turtlebot/ros_ws/src/bart/wav/"
		rospy.sleep(1)

		self.wav_map = {
			# "beginning":	"beginning.wav",
			# "middle":		"middle.wav",
			# "end":			"end.wav",
			"positive":		"feedback_positive.wav",
			"confused":		"feedback_confused.wav",
			"negative":		"feedback_negative.wav"
		}

		rospy.Subscriber("bart/head_sound", String, self.sound_callback, queue_size=1)

		rospy.spin()

	def sound_callback(self, a):
		sound_name = self.wav_map[a.data]
		self.soundhandle.playWave(self.soundAssets + sound_name)

if __name__ == "__main__":
	Sound_Controller()