#!/usr/bin/env python3
import rospy
import random
import math as ma
import numpy as np
from std_msgs.msg import Float32
from force_publisher import ControllerForce
from inverted_pendulum_controller.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState
 

class MySubscriber(object):
  
   def __init__(self):

        rospy.Subscriber("/inverted_pendulum/current_state", CurrentState, self.CurrentState_callback)
        
   def CurrentState_callback(self, CurrentState_message):
    #rospy.loginfo("current states received: (%.2f, %.2f, %.2f ,%.2f, %.2f, %.2f)",CurrentState_message.curr_x,CurrentState_message.curr_x_dot,CurrentState_message.curr_x_dot_dot,CurrentState_message.curr_theta,CurrentState_message.curr_theta_dot,CurrentState_message.curr_theta_dot_dot)
        global act, u
        self.x_s = CurrentState_message.curr_x
        self.x_dot = CurrentState_message.curr_x_dot
        self.theta = CurrentState_message.curr_theta
        self.theta_dot = CurrentState_message.curr_theta_dot
        self.act = [self.x_s, self.x_dot, self.theta, self.theta_dot]
        act = self.act
        print(act)
        #print(self.act)
        
        self.contr_obj = ControllerForce.publish_controlvalue(self, act)
        #print(u)
        print(self.contr_obj)
        #self.contr_obj.publish_controlvalue(self.act)    
   def loop(self):
        rospy.logwarn("Starting Loop...")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)
    my_subs = MySubscriber()
    my_subs.loop()
