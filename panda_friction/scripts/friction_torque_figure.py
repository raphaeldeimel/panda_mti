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

f, axarr = plt.subplots(7, sharex=True)
f.suptitle('friction measurement')

for listIndex in range(len(bagNames)):
    # get current bag name
    currentBag = bagNames[listIndex]
    # get the joint from current bag name
    jtCur = int(currentBag[1])-1
    # load bag
    bag = rb.Bag(bagPath + currentBag, 'r')
    # extract bag message
    for (topic, msg, t) in bag.read_messages(topics=[topicNameState, topicNamePhase]):
        # look for the "recording" tag
        if topic == topicNamePhase and msg.data == "recording":
            _recMode = True

        # if deceleration tag is found, stop filling the data stack
        if topic == topicNamePhase and msg.data == "deceleration":
            _dataStack_dq_full.append(_dataStack_dq)
            _dataStack_tau_full.append(_dataStack_tau)
            print(len(_dataStack_dq_full))
            _recMode = False
            _printMode = True
            bag.close()
            break

        # fill the data stack
        if topic == topicNameState and _recMode:
            _dataStack_dq.append(-1*msg.dq[jtCur])
            _dataStack_tau.append(-1*msg.tau[jtCur])

    print('bagname: ' + currentBag + '\t (%s/%s)' % (1+listIndex, len(bagNames)))
    if _printMode:
        axarr[jtCur].scatter(_dataStack_dq_full, _dataStack_tau_full, s=0.1, color='r')
        axarr[jtCur].set_title('joint ' + str(jtCur+1))
        _dataStack_dq_full = []
        _dataStack_tau_full = []
        _dataStack_dq = []
        _dataStack_tau = []
        _printMode = False

# for ax in axarr.flat:
    # ax.set(ylabel='tau [Nm]')
plt.xlabel("joint velocity dq [rad/s]")
plt.show()

