#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  1 15:57:14 2018

@author: alexander
"""

import subprocess
import shlex
import yaml
import warnings
import sys

import rospy

hwName = rospy.get_param("/panda_hostname")

hwID = subprocess.check_output(shlex.split('./getHardwareID.sh ' + hwName))
if not hwID.__len__() == 18:
    print("No robot found! quitting...")
    sys.exit()

with open("../config/hardwareParam.yaml", 'r') as hwSetup:
    try:
        hardware = yaml.load(hwSetup)
    except yaml.YAMLError as err:
        print(err)

try:  # FEHLER NOCH BEHEBEN
    paramSet = hardware[hwID.rstrip()]
except warnings.warn("configuration for MAC: %s not found. Using default setup instead" % hwID):
    paramSet = hardware["DEFAULT"]

print(paramSet['offsets'])
print(paramSet['feedforward'])
