#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Copyright 2020 Raphael Deimel
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
@copyright: 2020
@licence: 2-clause BSD licence
"""


import numpy as _np
_np.set_printoptions(precision=2, suppress=True, formatter={'float': '{: 0.1f}'.format})
import os
from operator import itemgetter


import rospy
from panda_msgs_mti.msg import PDControllerGoal8, RobotState



desiredControllerGoalPublisher = rospy.Publisher("pdcontroller_goal", PDControllerGoal8, queue_size=1, tcp_nodelay=True)

dofs = 8
Kp = _np.zeros((dofs, dofs))
Kp[3,0] = 10.0 #cross-joint gain
Kv = 2.0 * _np.eye(dofs)

pdcontrollergoal_msg = PDControllerGoal8()
pdcontrollergoal_msg.position = [0.0] * dofs
pdcontrollergoal_msg.velocity = [0.0] * dofs
pdcontrollergoal_msg.torque = [0.0] * dofs
pdcontrollergoal_msg.kp = Kp.ravel().tolist()
pdcontrollergoal_msg.kv = Kv.ravel().tolist()

#alternative test: only publish gains vectors:
#pdcontrollergoal_msg.kp = _np.diag(Kp).tolist()
#pdcontrollergoal_msg.kv = _np.diag(Kv).tolist()



rospy.init_node('pdcontrollergoal_testpublisher')

rate = rospy.Rate(10.0) # 10hz
while not rospy.is_shutdown():
    rate.sleep()
    pdcontrollergoal_msg.stamp = rospy.Time.now()
    desiredControllerGoalPublisher.publish(pdcontrollergoal_msg)


