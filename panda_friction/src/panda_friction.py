#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  1 15:57:14 2018

@author: alexander
"""

import time

import numpy as np

import rospy
from panda_msgs_mti.msg import PDControllerGoal8, RobotState8
from std_msgs.msg import String


class frictionMeasurement(object):

    """Friction measurement of a franka robot panda.

    Be careful. The motion of the joints is not synchronized. Self collision
    is possible. Please move the robot to the approximate start position.
    q1, q3, q5, q7 have the same start position (setup 1).
    q2 have setup 2
    q4 and q6 use the setup 3

    The joint setups can be found in the jointHomePose list.
    """

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
        self.desired_decel_time = 0.5
        self.desired_dot_q = -q_d
        self.desired_joint = joint_d-1
        self.delay_after_homing = 3  # [sec]
        self.trajSize = (8, 25*self.homingTime)
        self.velTrajSize = (int(25*self.desired_acc_time))
        self.decelTrajSize = (int(25*self.desired_decel_time))
        self.interpDesiredTraj = np.zeros(self.trajSize)
        self.interpAccTrajPhase = np.zeros(self.velTrajSize)
        self.interpDecelTrajPhase = np.zeros(self.velTrajSize)

        self.jtOff = 0.3
        self.jLimits = [
                        [-2.89 + self.jtOff, +2.89 - self.jtOff],  # j1
                        [-1.76 + self.jtOff, +1.76 - self.jtOff],
                        [-2.89 + self.jtOff, +2.89 - self.jtOff],
                        [-3.07 + self.jtOff, -0.06 - self.jtOff/2],
                        [-2.89 + self.jtOff, +2.89 - self.jtOff],
                        [-0.01 + self.jtOff, +3.75 - self.jtOff],
                        [-2.89 + self.jtOff, +2.89 - self.jtOff]]  # j7

        # column: joint setup for the corresponding measurement
        # rows: joint position (q1 - q7)
        self.jointHomePose = [
                [self.jLimits[0][1], +0.00,              +0.00,              +0.00,              +0.00,              +0.00,              +0.00],  # j1
                [+0.00,              self.jLimits[1][1], +0.00,              +1.54,              +0.00,              +1.54,              +0.00],
                [+0.00,              +1.54,              self.jLimits[2][1], +1.54,              +0.00,              +1.54,              +0.00],
                [self.jLimits[3][1], -2.73,              self.jLimits[3][1], self.jLimits[3][1], self.jLimits[3][1], -1.80,              self.jLimits[3][1]],
                [+0.00,              +0.00,              +0.00,              +0.00,              self.jLimits[4][1], +0.00,              +0.00],
                [+3.14,              +2.73,              +3.14,              +1.54,              +3.14,              self.jLimits[5][1], +3.14],
                [+0.00,              +0.00,              +0.00,              +0.00,              +0.00,              +0.90,              self.jLimits[6][1]]]  # j7

        self.jointKP = [40.0, 40.0, 40.0, 40.0, 40.0, 20.0, 20.0, 0.00]
        self.jointKV = [30.0, 30.0, 20.0, 20.0, 20.0, 20.0, 10.0, 0.00]

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
                                    int(25*self.delay_after_homing))))
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
        self.interpDecelTrajPhase = np.linspace(
                                                self.desired_dot_q,
                                                0.0,
                                                self.decelTrajSize,
                                                endpoint=True)
        # self.interpDecelTrajPhase = self.interpAccTrajPhase[::-1]

    def decelDistanceReached(self):
        """Check if limits are reached."""
        decelDistance = abs(0.5*self.desired_dot_q*self.desired_decel_time)+0.5
        if self.pandaRobotState.q[self.desired_joint] < (self.jLimits[self.desired_joint][0] + decelDistance):
            return True
        else:
            return False

    def startROS(self):
        """ROS NODE/TOPICS"""
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
        self.runMeasurement()

    def statePublisher(self, phaseIndex):
        """Phase handling."""
        rospy.loginfo(self.phaseState[phaseIndex])
        self.state_pub.publish(self.phaseState[phaseIndex])

    def runMeasurement(self):
        """Measurment script."""

        rospy.loginfo("waiting for pdcontroller...")
        time.sleep(1)

        if not self.robotStatesAvailable:
            print("error: no robot states available, quitting...")
            quit()

        rate = rospy.Rate(25)  # 25 Hz
        _state = 0
        _printOnce = True
        _updateTraj = True
        _localIndex = 0

        rospy.loginfo("measurement running...")

        # main control loop
        while not rospy.is_shutdown():
            # ------------------- HOMING PHASE --------------------------------
            if _state == 0:  # homing phase
                # generates pd-controller-goal for homing phase
                if _updateTraj:
                    self.goHomeTraj()
                    self.statePublisher(_state)
                    _updateTraj = False

                self.controllerGoal.position = self.interpDesiredTraj[:, _localIndex]
                _localIndex += 1
                if _localIndex >= len(self.interpDesiredTraj[0]):
                    _state += 1  # ready for acceleration
                    _updateTraj = True

            # -------------- ACCELERATION/RECORDING PHASE ---------------------
            if _state == 1:  # acceleration phase
                # generates pd-controller-goal for acceleration/recording phase
                if _updateTraj:
                    _localIndex = 0
                    self.getRecTraj()
                    self.statePublisher(_state)
                    _updateTraj = False

                self.controllerGoal.position = self.interpDesiredTraj[:, -1]
                self.controllerGoal.velocity[self.desired_joint] = self.interpAccTrajPhase[_localIndex]
                _localIndex += 1
                if _localIndex >= len(self.interpAccTrajPhase):
                    _state += 1  # acceleration done
                    _updateTraj = True

            # -------------- RECORDING/DECELERATION PHASE ---------------------
            if _state == 2:  # recording phase
                if _updateTraj:
                    _localIndex = 0
                    self.statePublisher(_state)
                    _updateTraj = False

                if self.decelDistanceReached():
                    _state += 1
                    _updateTraj = True

            if _state == 3:  # deceleration phase
                if _updateTraj:
                    _localIndex = 0
                    self.statePublisher(_state)
                    _updateTraj = False

                self.controllerGoal.velocity[self.desired_joint] = self.interpDecelTrajPhase[_localIndex]
                _localIndex += 1
                if _localIndex >= len(self.interpDecelTrajPhase):
                    _state += 1  # deceleration done
                    _updateTraj = True

            if _state == 4:
                self.statePublisher(_state)
                quit()

            # desired state publisher
            self.controllerGoal.stamp = rospy.Time.now()
            self.goal_pub.publish(self.controllerGoal)
            rate.sleep()


if __name__ == '__main__':
    joint = 6  # j1 - j7
    q_dot_desired = 0.1  # rad/s
    frictionObj = frictionMeasurement(joint, q_dot_desired)
