#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  1 15:57:14 2018

@author: mti
"""

# license removed for brevity
import time
import numpy as np
import rospy

from std_msgs.msg import String
from mti_panda_controller_msgs.msg import PDControllerGoal8
from mti_panda_controller_msgs.msg import RobotState8

# global variables
pandaRobotState = RobotState8()
robotStatesAvailable = False


# update robot states
def robotHandler(states):
	robotStatesAvailable = True
	pandaRobotState = states



def followTraj():

	# ----- ROS -----
	# goal publisher
    goal_pub = rospy.Publisher('/pdcontroller_goal', PDControllerGoal8, queue_size=10)
	# publishes the motion phases
	state_pub = rospy.Publisher('/panda_state_pub', String, queue_size=10)
	# reads the joint robot states of the robot
    rospy.Subscriber("/panda/currentstate", RobotState8, robotHandler)
	#ROS main node
    rospy.init_node('panda_friction_node', anonymous=True)


	#----- INIT -----
    # motion variables
	# time to get home
    homingTime = 5.0
    # desired joint velocity
    desired_dot_q = 0.3
    desired_joint = 1 # 1 - 7

    # Goal publisher
    jtTraj = PDControllerGoal8()

    jtTraj.torque = np.zeros(8);
    jtTraj.position = np.zeros(8);
    jtTraj.velocity = np.zeros(8);
    jtTraj.kp = np.zeros(8);
    jtTraj.kd = np.zeros(8);

    phaseIndex = 0
    phaseState = [
        'homing',
        'acceleration',
        'recording',
        'deceleration',
        'stop']

    jointSafetyOffset = 0.1
    jLimits = [
        [-2.89 + jointSafetyOffset, +2.89 - jointSafetyOffset], # joint1
        [-1.76 + jointSafetyOffset, +1.76 - jointSafetyOffset],
        [-2.89 + jointSafetyOffset, +2.89 - jointSafetyOffset],
        [-3.07 + jointSafetyOffset, -0.06 - jointSafetyOffset],
        [-2.89 + jointSafetyOffset, +2.89 - jointSafetyOffset],
        [-0.01 + jointSafetyOffset, +3.75 - jointSafetyOffset],
        [-2.89 + jointSafetyOffset, +2.89 - jointSafetyOffset]] # joint7

	# nesting
	# column: joint setup for the corresponding measurement
	# rows: joint position (q1 - q7)
    jointHomePose = [
        [+2.89, +0.00, +0.00, +0.00, +0.00, +0.00, +0.00], #q1
        [+0.00, +2.89, +0.00, +1.54, +0.00, +1.54, +0.00], #q2
        [+0.00, +1.54, +2.89, +1.54, +0.00, +1.54, +0.00],
        [-0.10, -2.73, -0.10, -0.10, -0.10, -0.10, -0.10],
        [+0.00, +0.00, +0.00, +0.00, +2.89, +0.00, +0.00],
        [+3.14, +2.73, +3.14, +1.54, +3.14, +0.00, +3.14],
        [+0.00, +0.00, +0.00, +0.00, +0.00, +0.90, +2.89]] #q7

    jointKP = [
        20, # joint1
        20,
        20,
        20,
        20,
        20,
        20] # joint7

    jointKD = [
        30, # joint1
        30,
        20,
        20,
        20,
        10,
        10] # joint7

	# ----- HOMING -----
	time.sleep(1)
	# exit program if joint states are not published
	if not jointStatesAvailable:
    	quit()

	# position interpolation



    # Homing all / step trajectory
    for i in range(0,7):
        if not i == desired_joint-1:
            jtTraj.position[i] = jointStaticPose[i][desired_joint-1]
        else:
            jtTraj.position[i] = jLimits[i][0]
        jtTraj.kp[i] = jointKP[i]

    goal_pub.publish(jtTraj)
    state_pub.publish(phaseState[0]) # homing

    time.sleep(5)

    rate = rospy.Rate(25) # 25 Hz
	# main control loop
    while not rospy.is_shutdown():

        # publish new gaol for the desired joint
        goal_pub.publish(jtTraj)
        state_pub.publish(phaseState[phaseIndex])

        rate.sleep()



if __name__ == '__main__':
	followTraj()
