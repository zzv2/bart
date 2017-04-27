#!/usr/bin/env python
# Team: bart; Names: Zach Vinegar, 
import rospy
import scipy as sp
from scipy.integrate import quad
import numpy as np
import pylab as plt
from geometry_msgs.msg import *
# from bart.msg import *
from copy import deepcopy
# from config import *

time = 0

def pathCb(path):
	rospy.loginfo(path)

def server():
	rospy.init_node("bart-server")
	loginfo("Initialized Server")

	rospy.Subscriber("/cmd_path", Path, pathCb, queue_size=1)
	plane_traj_pub = rospy.Publisher('/plane_traj', Trajectory, queue_size=1)

	plt.show()
	# rospy.spin()

if __name__ == "__main__":
	server()
