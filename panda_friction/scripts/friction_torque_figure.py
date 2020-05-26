#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script extracts the torque and velocity from rosbags. Subsequently a
friction coefficient is calculated by linear regression

@author: alexander
"""
from itertools import chain

import rosbag as rb
import matplotlib.pyplot as plt
import numpy as np
import os

bagPath = '../raw/'

topicNameState = '/panda/currentstate'
topicNamePhase = '/panda/friction/state_pub'

bagNames = os.listdir(bagPath)
bagNames.sort()

recMode = False
printMode = False
nextBag = False
jtOld = 0
dataStack_dq_full = []
dataStack_tau_full = []
dataStack_dq = []
dataStack_tau = []

f, axarr = plt.subplots(7, sharex=True)
f.suptitle('friction measurement')
dq = np.arange(0., 2.5, 0.01)
pnx, pny = [0, 0], [0, 4]

for listIndex in range(len(bagNames)+1):
    # just for the last plot -quick and dirty way
    if listIndex == len(bagNames):
        printMode = True

    # get current bag name, if condition is just for the last plot -quick and dirty way
    if not listIndex == len(bagNames):
        currentBag = bagNames[listIndex]
    # get the joint from current bag name, start number is 0
        jtCur = int(currentBag[1])-1

    # check if the bag belongs to the same velocity stack
    if not jtCur == jtOld:
        printMode = True

    if printMode:
        dataStack_dq_full = list(chain.from_iterable(dataStack_dq_full))
        dataStack_tau_full = list(chain.from_iterable(dataStack_tau_full))
        m, b = np.polyfit(dataStack_dq_full, dataStack_tau_full, 1)
        print('joint: %s\t torque: dq = 0: %s' % (jtOld, b))
        axarr[jtOld].plot(dq, m*dq+b)
        axarr[jtOld].plot(pnx, pny)
        axarr[jtOld].grid(True)
        axarr[jtOld].scatter(dataStack_dq_full, dataStack_tau_full, s=0.1, color='r')
        axarr[jtOld].set_title('joint ' + str(jtOld+1))
        dataStack_dq_full = []
        dataStack_tau_full = []
        dataStack_dq = []
        dataStack_tau = []
        printMode = False
        jtOld = jtCur

    # just for the last plot -quick and dirty way
    if listIndex == len(bagNames):
        break

    print('bagname: ' + currentBag + '\t (%s/%s)' % (1+listIndex, len(bagNames)))
    # load bag
    bag = rb.Bag(bagPath + currentBag, 'r')
    # extract bag message
    for (topic, msg, t) in bag.read_messages(topics=[topicNameState, topicNamePhase]):
        # look for the "recording" tag
        if topic == topicNamePhase and msg.data == "recording":
            recMode = True

        # if deceleration tag is found, stop filling the data stack
        if topic == topicNamePhase and msg.data == "deceleration":
            dataStack_dq_full.append(dataStack_dq)
            dataStack_tau_full.append(dataStack_tau)
            recMode = False
            bag.close()
            break

        # fill the data stack
        if topic == topicNameState and recMode:
            dataStack_dq.append(-1*msg.dq[jtCur])
            dataStack_tau.append(-1*msg.tau[jtCur])


for ax in axarr.flat:
    ax.set(ylabel='tau [Nm]')
plt.xlabel("joint velocity dq [rad/s]")
plt.show()

