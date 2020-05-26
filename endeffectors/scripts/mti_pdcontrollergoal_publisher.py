#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

"""

import rospy
import roslib
import std_msgs.msg
import panda_msgs_mti.msg
import threading
import numpy as np
import os, sys, time
import random

if __name__ == "__main__":
    rospy.init_node('mti_pdcontrollergoal_publisher', anonymous=False)
    pressure_pub = rospy.Publisher("/pdcontroller_goal", panda_msgs_mti.msg.PDControllerGoal8,queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        djoint_state = float(random.randint(1,10))/100
        cmd = panda_msgs_mti.msg.PDControllerGoal8()
        cmd.stamp = rospy.Time.now()
        cmd.torque = [0,0,0,0,0,0,0,0]
        cmd.position = [0,0,0,0,0,0,0,djoint_state]
        cmd.velocity = [0,0,0,0,0,0,0,0]
        cmd.kp = [0,0,0,0,0,0,0,1]
        cmd.kv = [0,0,0,0,0,0,0,1]
        pressure_pub.publish(cmd)
        print(cmd)
        rate.sleep()

