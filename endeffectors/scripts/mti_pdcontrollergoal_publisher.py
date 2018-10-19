#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

"""

import rospy
import roslib
import std_msgs.msg
import mti_panda_controller_msgs.msg
import threading
import numpy as np
import os, sys, time
import random

if __name__ == "__main__":
<<<<<<< Updated upstream
    rospy.init_node('mti_pdcontrollergoal_publisher', anonymous=False)
=======
    rospy.init_node('mti_pdcontrollergoal_publisher')
>>>>>>> Stashed changes
    pressure_pub = rospy.Publisher("/pdcontroller_goal", mti_panda_controller_msgs.msg.PDControllerGoal8,queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        djoint_state = float(random.randint(1,10))/100
        cmd = mti_panda_controller_msgs.msg.PDControllerGoal8()
        cmd.stamp = rospy.Time.now()
<<<<<<< Updated upstream
        cmd.torque = [0,0,0,0,0,0,0,1]
        cmd.position = [0,0,0,0,0,0,0,djoint_state]
        cmd.velocity = [0,0,0,0,0,0,0,1]
=======
        cmd.torque = [0,0,0,0,0,0,0,0]
        cmd.position = [0,0,0,0,0,0,0,djoint_state]
        cmd.velocity = [0,0,0,0,0,0,0,0]
>>>>>>> Stashed changes
        cmd.kp = [0,0,0,0,0,0,0,1]
        cmd.kv = [0,0,0,0,0,0,0,1]
        pressure_pub.publish(cmd)
        print(cmd)
        rate.sleep()

