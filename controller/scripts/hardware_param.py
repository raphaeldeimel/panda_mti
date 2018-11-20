#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  1 15:57:14 2018

@author: alexander
"""

import subprocess
import shlex
import yaml
import sys
import os

import rospy
import rospkg


def robotParameterization():
    """Robot parameterization."""

    rospack = rospkg.RosPack()
    # get robot hostname or ip address
    hwName = rospy.get_param("/panda/hostname")
    # check if net-tools is installed
    basepath = os.path.join(rospack.get_path('controller_panda_mti'))
    hwID = subprocess.check_output(shlex.split('sh ' + basepath + '/scripts/getHardwareID.sh ' + hwName))
    if hwID.rstrip() == "dpkg-error":
        print("Error while executing net-tools! \n")
        print("Please make sure that net-tools is installed. \n")
        print("try: sudo apt install net-tools")
        sys.exit()
    if not hwID.__len__() == 18:
        print("No robot found! quitting...")
        sys.exit()

    with open(basepath + "/config/hardwareParam.yaml", 'r') as hwSetup:
        try:
            hwConfig = yaml.load(hwSetup)
        except yaml.YAMLError as err:
            print(err)
            sys.exit()
    # error handling hwID = "00:00:00:00:00:00"
    try:  # FEHLER NOCH BEHEBEN
        paramSet = hwConfig[hwID.rstrip()]
    except yaml.YAMLError as err:  # warn("Configuration for MAC address: %s could not be found. Using default setup instead" % hwID):
        print(err)
        paramSet = hwConfig["DEFAULT"]

    rospy.set_param('/panda/stiction_offset', paramSet['offsets'])
    rospy.set_param('/panda/stiction_feedforward', paramSet['feedforward'])


if __name__ == "__main__":
    robotParameterization()
