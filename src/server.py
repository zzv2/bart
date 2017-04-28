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
# from config import *

time = 0

def cmdCb(cmd):
	rospy.loginfo(cmd)

def server():
	rospy.init_node("bart-server")
	rospy.loginfo("Initialized Server")

	rospy.Subscriber("bart/cmd", Cmd, cmdCb, queue_size=1)
	# plane_traj_pub = rospy.Publisher('/plane_traj', Trajectory, queue_size=1)

	rospy.spin()

if __name__ == "__main__":
	server()
