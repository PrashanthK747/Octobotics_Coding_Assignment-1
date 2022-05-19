#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from inverted_pendulum_controller.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState
import random
import math as ma
import numpy as np
class ControllerForce(object):
    def __init__(self, act):
        self.act = act
        self.pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10) 

    def publish_controlvalue(self, act):
        global u
        self.pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10) 
        #self.K = [-0.01183, -1.57874, -0.02163, -0.01462]
        self.K = [-0.0096,-17.4426,-0.0332,-0.0428]
        self.ref = [0,0, ma.pi, 0]
        u = -self.K[0]*(self.act[0]-self.ref[0])-self.K[1]*(self.act[1]-self.ref[1])-self.K[2]*(self.act[2]-self.ref[2])-self.K[3]*(self.act[3]-self.ref[3])
        rospy.loginfo(u)
        self.pub.publish(u)
        return u

if __name__ == '__main__':
    rospy.init_node('controlvalue_node', anonymous=True, log_level=rospy.INFO)
    contr_obj = ControllerForce()
    try:
        contr_obj.publish_controlvalue(u)
    except rospy.ROSInterruptException:
        pass