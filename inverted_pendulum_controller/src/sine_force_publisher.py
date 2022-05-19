#!/usr/bin/env python3
from re import S
import rospy
from inverted_pendulum_sim.msg import ControlForce
import random
import math
from math import sin, cos, pi
import numpy as np

#create a new publisher. we specify the topic name, then type of message then the queue size
pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)

#we need to initialize the node
rospy.init_node('control_force_publisher', anonymous=True)

#set the loop rate
rate = rospy.Rate(1) # 1hz
#keep publishing until a Ctrl-C is pressed
f = 20
w = 2.0*np.pi * f
a = 1
i = 0
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    controlforce = ControlForce
    controlforce = a*sin(math.radians(w*i))
    rospy.loginfo("controller force input:")
    rospy.loginfo(controlforce)
    pub.publish(controlforce)
    rate.sleep()
    i=i+0.1

