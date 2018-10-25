#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  1 15:57:14 2018

@author: alexander
"""

import time
import numpy as np
import rospy

from std_msgs.msg import String
from panda_msgs_mti.msg import PDControllerGoal8, RobotState8


class frictionMeasurement:
    def __init__(self, joint_d, q_d):
        rospy.init_node('panda_friction_node', anonymous=True)
        rospy.loginfo("init")
        self.controllerGoal = PDControllerGoal8()
        self.pandaRobotState = RobotState8()
        self.robotStatesAvailable = False

        self.phaseState = ['homing', 'acceleration', 'recording', 'deceleration', 'stop']

        self.homingTime = 6
        self.desired_dot_q = q_d
        self.desired_joint = joint_d
        self.trajSize = (8, 25*self.homingTime)
        self.interpDesiredTraj = np.zeros(self.trajSize)

        self.jointSafetyOffset = 0.1
        self.jLimits = [
                [-2.89 + self.jointSafetyOffset, +2.89 - self.jointSafetyOffset], # j1
                [-1.76 + self.jointSafetyOffset, +1.76 - self.jointSafetyOffset],
                [-2.89 + self.jointSafetyOffset, +2.89 - self.jointSafetyOffset],
                [-3.07 + self.jointSafetyOffset, -0.06 - self.jointSafetyOffset],
                [-2.89 + self.jointSafetyOffset, +2.89 - self.jointSafetyOffset],
                [-0.01 + self.jointSafetyOffset, +3.75 - self.jointSafetyOffset],
                [-2.89 + self.jointSafetyOffset, +2.89 - self.jointSafetyOffset]] # j7

	    # column: joint setup for the corresponding measurement
	    # rows: joint position (q1 - q7)
        self.jointHomePose = [
                [+2.89, +0.00, +0.00, +0.00, +0.00, +0.00, +0.00], # j1
                [+0.00, +2.89, +0.00, +1.54, +0.00, +1.54, +0.00],
                [+0.00, +1.54, +2.89, +1.54, +0.00, +1.54, +0.00],
                [-0.10, -2.73, -0.10, -0.10, -0.10, -0.10, -0.10],
                [+0.00, +0.00, +0.00, +0.00, +2.89, +0.00, +0.00],
                [+3.14, +2.73, +3.14, +1.54, +3.14, +0.00, +3.14],
                [+0.00, +0.00, +0.00, +0.00, +0.00, +0.90, +2.89]] # j7

        self.jointKP = [40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 0.00]
        self.jointKV = [30.0, 30.0, 20.0, 20.0, 20.0, 10.0, 10.0, 0.00]

        self.startROS()


    def robotStateHandler(self, states):
        self.robotStatesAvailable = True
        self.pandaRobotState = states


    def goHomeTraj(self):
        # interpolates joint position between actual value and desired start position
        for i in range(7):
            self.interpDesiredTraj[i] = np.linspace(self.pandaRobotState.q[i], self.jointHomePose[i][self.desired_joint], self.trajSize[1])
        # moving to start position
        self.controllerGoal.kp = self.jointKP
        self.controllerGoal.kv = np.zeros(8)


    def getRecTraj(self):
        self.controllerGoal.kp[self.desired_joint] = 0.0
        self.controllerGoal.kv = self.jointKV
        pass


    def startROS(self):
        self.goal_pub = rospy.Publisher('/panda/pdcontroller_goal', PDControllerGoal8, queue_size=10)
        self.state_pub = rospy.Publisher('/panda/friction/state_pub', String, queue_size=10)
        rospy.Subscriber("/panda/currentstate", RobotState8, self.robotStateHandler)
        rospy.init_node('panda_friction_node', anonymous=True)

        _state = 0
        _printOnce = True
        _updateTraj = True

        rospy.loginfo("waiting for pdcontroller...")
        time.sleep(1)

        if not self.robotStatesAvailable:
            print("error: no robot states available, quiting...")
            quit()



        rate = rospy.Rate(25) # 25 Hz
        localIndex = 0

        rospy.loginfo("measurement running...")
	       # main control loop
        while not rospy.is_shutdown():
            # ------------------- HOMING PHASE ---------------------------------
            if _state == 0:
                # generates pd-controller-goal for homing phase
                if _updateTraj
                    self.goHomeTraj()
                    _updateTraj = False

                self.controllerGoal.position = self.interpDesiredTraj[:,localIndex]
                localIndex += 1
                if localIndex >= len(self.interpDesiredTraj[0]):
                    _state += 1
                    _printOnce = True
                    _updateTraj = True

            # -------------- ACCELERATION/RECORDING PHASE-----------------------
            if _state == 1: # acc
                # generates pd-controller-goal for acceleration/recording phase
                if _updateTraj
                    self.getRecTraj()
                pass

            if _printOnce:
                rospy.loginfo(self.phaseState[_state])
                self.state_pub.publish(self.phaseState[_state])
                _printOnce = False

            # desired state publisher
            self.controllerGoal.stamp = rospy.Time.now()
            self.goal_pub.publish(self.controllerGoal)
            rate.sleep()



if __name__ == '__main__':
    joint = 0
    q_dot_desired = 0.1
    frictionObj = frictionMeasurement(joint, q_dot_desired)
