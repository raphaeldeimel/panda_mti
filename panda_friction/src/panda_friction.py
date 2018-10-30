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


class frictionMeasurement(object):

    """class defintion - friction measurement of a franka robot panda."""

    def __init__(self, joint_d, q_d):
        """Init function."""
        rospy.init_node('panda_friction_node', anonymous=True)
        rospy.loginfo("init")
        self.controllerGoal = PDControllerGoal8()
        self.pandaRobotState = RobotState8()
        self.robotStatesAvailable = False

        self.phaseState = [
                            'homing',
                            'acceleration',
                            'recording',
                            'deceleration',
                            'stop']

        self.homingTime = 6
        self.desired_acc_time = 1  # secs to get desired speed
        self.desired_dot_q = q_d
        self.desired_joint = joint_d-1
        self.delayAfterHoming = 2  # [sec]
        self.trajSize = (8, 25*self.homingTime)
        self.velTrajSize = (int(25*abs(self.desired_dot_q)))
        self.interpDesiredTraj = np.zeros(self.trajSize)
        self.interpAccTrajPhase = np.zeros(self.velTrajSize)
        self.interpDecelTrajPhase = np.zeros(self.velTrajSize)

        self.jtOff = 0.3
        self.jLimits = [
                        [-2.89 + self.jtOff, +2.89 - self.jtOff],  # j1
                        [-1.76 + self.jtOff, +1.76 - self.jtOff],
                        [-2.89 + self.jtOff, +2.89 - self.jtOff],
                        [-3.07 + self.jtOff, -0.06 - self.jtOff],
                        [-2.89 + self.jtOff, +2.89 - self.jtOff],
                        [-0.01 + self.jtOff, +3.75 - self.jtOff],
                        [-2.89 + self.jtOff, +2.89 - self.jtOff]]  # j7

        # column: joint setup for the corresponding measurement
        # rows: joint position (q1 - q7)
        self.jointHomePose = [
                [self.jLimits[0][1], +0.00,              +0.00,              +0.00,              +0.00,              +0.00,              +0.00],  # j1
                [+0.00,              self.jLimits[1][1], +0.00,              +1.54,              +0.00,              +1.54,              +0.00],
                [+0.00,              +1.54,              self.jLimits[2][1], +1.54,              +0.00,              +1.54,              +0.00],
                [self.jLimits[3][1], -2.73,              self.jLimits[3][1], self.jLimits[3][1], self.jLimits[3][1], self.jLimits[3][1], self.jLimits[3][1]],
                [+0.00,              +0.00,              +0.00,              +0.00,              self.jLimits[4][1], +0.00,              +0.00],
                [+3.14,              +2.73,              +3.14,              +1.54,              +3.14,              +0.00,              +3.14],
                [+0.00,              +0.00,              +0.00,              +0.00,              +0.00,              +0.90,              self.jLimits[6][1]]]  # j7

        self.jointKP = [30.0, 30.0, 30.0, 30.0, 30.0, 20.0, 20.0, 0.00]
        self.jointKV = [30.0, 30.0, 20.0, 20.0, 20.0, 10.0, 10.0, 0.00]

        self.startROS()

    def robotStateHandler(self, states):
        """Get actual robot state from ROS."""
        self.robotStatesAvailable = True
        self.pandaRobotState = states

    def goHomeTraj(self):
        """Interpolate joint positions between actual value and desired start position."""
        for i in range(7):
            self.interpDesiredTraj[i] = np.linspace(
                self.pandaRobotState.q[i],
                self.jointHomePose[i][self.desired_joint],
                self.trajSize[1],
                endpoint=True)
        self.interpDesiredTraj = np.hstack((
                                    self.interpDesiredTraj,
                                    np.tile(self.interpDesiredTraj[:, [-1]],
                                    int(25*self.delayAfterHoming))))
        # moving to start position
        self.controllerGoal.kp = self.jointKP
        self.controllerGoal.kv = np.zeros(8)

    def getRecTraj(self):
        """Generate velocity trajectory."""
        # kp value of desired joint is zero (no position control anymore)
        self.controllerGoal.kp[self.desired_joint] = 0.0
        # vel control of desired joint
        self.controllerGoal.kv[self.desired_joint] = self.jointKV[self.desired_joint]
        # generates motion traj for desired joint
        # acceleration phase
        self.interpAccTrajPhase = np.linspace(
                                                0.0,
                                                self.desired_dot_q,
                                                self.velTrajSize,
                                                endpoint=True)
        # deceleration phase
        self.interpDecelTrajPhase = self.interpAccTrajPhase[::-1]

    def decelDistanceReached(self):
        """Check if limits are reached."""
        decelDistance = 0.5*self.desired_dot_q*self.desired_acc_time
        if (self.pandaRobotState.q[self.desired_joint] < (self.jLimits[self.desired_joint][0] + decelDistance)):
            return True
        else:
            return False

    def startROS(self):
        """Start ROS Stuff."""
        self.goal_pub = rospy.Publisher(
                        '/panda/pdcontroller_goal',
                        PDControllerGoal8,
                        queue_size=10)
        self.state_pub = rospy.Publisher(
                        '/panda/friction/state_pub',
                        String,
                        queue_size=10)
        rospy.Subscriber(
                        "/panda/currentstate",
                        RobotState8,
                        self.robotStateHandler)

        rospy.init_node('panda_friction_node', anonymous=True)

        _state = 0
        _printOnce = True
        _updateTraj = True

        rospy.loginfo("waiting for pdcontroller...")
        time.sleep(1)

        if not self.robotStatesAvailable:
            print("error: no robot states available, quitting...")
            quit()

        rate = rospy.Rate(25)  # 25 Hz
        localIndex = 0

        rospy.loginfo("measurement running...")

        # main control loop
        while not rospy.is_shutdown():
            # ------------------- HOMING PHASE --------------------------------
            if _state == 0:  # homing phase
                # generates pd-controller-goal for homing phase
                if _updateTraj:
                    self.goHomeTraj()
                    _updateTraj = False

                self.controllerGoal.position = self.interpDesiredTraj[:, localIndex]
                localIndex += 1
                if localIndex >= len(self.interpDesiredTraj[0]):
                    _state += 1  # ready for acceleration
                    _printOnce = True
                    _updateTraj = True

            # -------------- ACCELERATION/RECORDING PHASE ---------------------
            if _state == 1:  # acceleration phase
                # generates pd-controller-goal for acceleration/recording phase
                if _updateTraj:
                    localIndex = 0
                    self.getRecTraj()
                    _updateTraj = False

                self.controllerGoal.position = self.interpDesiredTraj[:, -1]
                self.controllerGoal.velocity[self.desired_joint] = self.interpAccTrajPhase[localIndex]
                localIndex += 1
                if localIndex >= len(self.interpAccTrajPhase):
                    _state += 1  # acceleration done
                    _printOnce = True
                    _updateTraj = True

            # -------------- RECORDING/DECELERATION PHASE ---------------------
            if _state == 2:  # recording phase
                if _updateTraj:
                    localIndex = 0
                    _updateTraj = False

                if self.decelDistanceReached():
                    rospy.loginfo("reached")
                    self.controllerGoal.velocity[self.desired_joint] = self.interpDecelTrajPhase[localIndex]
                    localIndex += 1
                    if localIndex >= len(self.interpDecelTrajPhase):
                        _state += 1  # deceleration done
                        _printOnce = True
                        _updateTraj = True

# ---------------------------------- CONSOLE OUTPUT ---------------------------
            if _printOnce:
                rospy.loginfo(self.phaseState[_state])
                self.state_pub.publish(self.phaseState[_state])
                _printOnce = False

            # desired state publisher
            # self.controllerGoal.stamp = rospy.Time.now()
            # self.goal_pub.publish(self.controllerGoal)
            rate.sleep()


if __name__ == '__main__':
    joint = 1  # j1 - j7
    q_dot_desired = -1*0.8  # rad/s
    frictionObj = frictionMeasurement(joint, q_dot_desired)
