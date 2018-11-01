#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  1 15:57:14 2018

@author: alexander
"""

import rosbag as rb
import matplotlib.pyplot as plt
import numpy as np
import os

bagPath = '../raw/'

topicNameState = '/panda/currentstate'
topicNamePhase = '/panda/friction/state_pub'

bagNames = os.listdir(bagPath)
bagNames.sort()

_recMode = False
_dataStack_dq_full = []
_dataStack_tau_full = []
_dataStack_dq = []
_dataStack_tau = []

currentJoint = 1

for listIndex in range(len(bagNames)):
    currentBag = bagNames[listIndex]
    jtNum = int(currentBag[1])-1
    if jtNum > 0:
        break
    bag = rb.Bag(bagPath + currentBag, 'r')

    print(currentBag)

    for (topic, msg, t) in bag.read_messages(topics=[topicNameState, topicNamePhase]):
        if topic == topicNamePhase and msg.data == "recording":
            _recMode = True

        if topic == topicNamePhase and msg.data == "deceleration":
            _dataStack_dq_full.append(_dataStack_dq)
            _dataStack_tau_full.append(_dataStack_tau)
            _recMode = False
            bag.close()
            break

        if topic == topicNameState and _recMode:
            _dataStack_dq.append(-1*msg.dq[jtNum])
            _dataStack_tau.append(-1*msg.tau[jtNum])

plt.scatter(_dataStack_dq_full,_dataStack_tau_full, s=0.1)
plt.xlabel("dq")
plt.ylabel("tau")
plt.show()
