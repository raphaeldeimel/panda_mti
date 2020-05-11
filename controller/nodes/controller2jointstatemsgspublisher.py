#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Copyright 2018 Raphael Deimel
#
#Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
@author: Raphael Deimel
@copyright: 2018-2020
@licence: 2-clause BSD licence
"""

import os
import sys
import rosparam
import rosbag
import rospy
from sensor_msgs.msg import JointState
from panda_msgs_mti.msg import RobotState, PDControllerGoal8, MechanicalStateDistribution8TorquePosVel

jointStateMsgnames = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"]

currentRobotState = None
def callbackCurrentState(data):
    global  currentRobotState
    currentRobotState = data

currentRobotGoal = None
def callbackCurrentGoal(data):
    global  currentRobotGoal
    currentRobotGoal = data

iTau = 0
iPos = 1
iVel = 2
currentRobotDesiredMSD = None
def callbackCurrentMSD(data):
    global currentRobotDesiredMSD
    currentRobotDesiredMSDMean = data


rospy.init_node('controller2jointstatemsgs')
listener = rospy.Subscriber("currentstate", RobotState, callbackCurrentState  , queue_size=1)
publisher_expected = rospy.Publisher("joint_states_expected", JointState, queue_size=3)
publisher_real = rospy.Publisher("joint_states", JointState, queue_size=3)

listener2 = rospy.Subscriber("pdcontroller_goal", PDControllerGoal8, callbackCurrentGoal  , queue_size=1)

listener2b = rospy.Subscriber("desiredmechanicalstate", MechanicalStateDistribution8TorquePosVel, callbackCurrentMSD  , queue_size=1)


listener3 = rospy.Subscriber("pdcontroller_goal_simulation", PDControllerGoal8, callbackCurrentGoal  , queue_size=1) # necessary for LabelApp2



arm_id = rospy.get_param('arm_id')
jointStateMsgnames_suffices = ["_joint1", "_joint2", "_joint3", "_joint4", "_joint5", "_joint6", "_joint7", "_finger_joint1", "_finger_joint2"]
jointStateMsgnames = [arm_id+sfx for sfx in jointStateMsgnames_suffices]

j = JointState()
j.name = jointStateMsgnames
j.position = [0]*len(jointStateMsgnames)
j.velocity = [0]*len(jointStateMsgnames)
j.effort = [0]*len(jointStateMsgnames)


rate = rospy.Rate(20)
while not rospy.is_shutdown():
    rate.sleep()
    if currentRobotState is not None:
        j.position[0:7] = currentRobotState.q[0:7]
        j.velocity[0:7] = currentRobotState.dq[0:7]
        j.effort[0:7]   = currentRobotState.tau[0:7]
        j.position[7]   = 0.5*currentRobotState.q[7]
        j.velocity[7]   = 0.5*currentRobotState.dq[7]
        j.effort[7]     =     currentRobotState.tau[7]
        j.position[8]   = j.position[7]
        j.velocity[8]   = j.velocity[7]
        j.effort[8]     = j.position[7] 
        j.header.stamp = currentRobotState.stamp
        currentRobotState = None
        publisher_real.publish(j)

    if (currentRobotGoal is None) or (currentRobotDesiredMSD is None):
        pass
    else:  
        rospy.logwarn_once("received messages for both pdcontroller_goal and desiredmechanicalstate topics - are you really sure you want this?")

    if currentRobotGoal is not None:

        j.position[0:7] = currentRobotGoal.position[0:7]
        j.velocity[0:7] = currentRobotGoal.velocity[0:7]
        j.effort[0:7]   = currentRobotGoal.torque[0:7]
        j.position[7]   = 0.5*currentRobotGoal.position[7]
        j.velocity[7]   = 0.5*currentRobotGoal.velocity[7]
        j.effort[7]     =     currentRobotGoal.torque[7]
        j.position[8]   = j.position[7]
        j.velocity[8]   = j.velocity[7]
        j.effort[8]     = j.position[7] 
        j.header.stamp = currentRobotGoal.stamp
        currentRobotGoal = None
        publisher_expected.publish(j)

    if currentRobotDesiredMSD is not None:
        j.position[0:7] = currentRobotDesiredMSD.meansMatrix[iPos, 0:7]
        j.velocity[0:7] = currentRobotDesiredMSD.meansMatrix[iVel, 0:7]
        j.effort[0:7]   = currentRobotDesiredMSD.meansMatrix[iTau,:0:7]
        j.position[7]   = 0.5*currentRobotDesiredMSD.meansMatrix[iPos,7]
        j.velocity[7]   = 0.5*currentRobotDesiredMSD.meansMatrix[iVel,7]
        j.effort[7]     =     currentRobotDesiredMSD.meansMatrix[iTau,7]
        j.position[8]   = j.position[7]
        j.velocity[8]   = j.velocity[7]
        j.effort[8]     = j.position[7] 
        j.header.stamp = currentRobotDesiredMSD.stamp
        currentRobotDesiredMSD = None
        publisher_expected.publish(j)


    
