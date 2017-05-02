#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import time
import sys, select, termios, tty

# Author: Zach Zweig-Vinegar
# This ROS Node converts Joystick inputs from the joy node
# into commands for the head robot

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
	'i':(1,0),
	'o':(1,-1),
	'j':(0,1),
	'l':(0,-1),
	'u':(1,1),
	',':(-1,0),
	'.':(-1,1),
	'm':(-1,-1)
}

speedBindings={
	'q':(1.1,1.1),
	'z':(.9,.9),
	'w':(1.1,1),
	'x':(.9,1),
	'e':(1,1.1),
	'c':(1,.9),
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


class BeamKeyboard:
	def __init__(self):
		# publishing to "beam/cmd_vel" to control beam robot
		self.pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
		self.tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
		# starts the node
		rospy.init_node('HeadKeyboard')

		# publishes most recent command at 10 Hz
		rate = rospy.Rate(10)
		max_update_time = 3

		pan = 0
		tilt = 0
		speed = .01

		try:
			print msg
			while not rospy.is_shutdown():

				key = getKey()
				if key in moveBindings.keys():
					tilt += moveBindings[key][0]*speed
					pan += moveBindings[key][1]*speed
					self.update(pan,tilt)
				elif key in speedBindings.keys():
					speed *= speedBindings[key][0]
					turn *= speedBindings[key][1]
					rospy.loginfo("speed: %.2f, turn: %.2f" % (speed,turn))
				else:
					# pan = 0
					# tilt = 0
					if (key == '\x03'):
						break

				# rate.sleep()

		except Exception as e:
			print e

		finally:
			self.pan_pub.publish(0.0)
			self.tilt_pub.publish(0.0)

	    		# termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


		rospy.spin()

	def update(self, pan, tilt):
		self.last_update = time.time()
		self.pan_pub.publish(pan)
		self.tilt_pub.publish(tilt)

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	BeamKeyboard()

