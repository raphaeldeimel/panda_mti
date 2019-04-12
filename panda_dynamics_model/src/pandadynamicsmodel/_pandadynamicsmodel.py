#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Copyright 2017 Raphael Deimel
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
@copyright: 2017
@licence: 2-clause BSD licence


"""

import warnings
import numpy as _np
import os
import rosbag as _rosbag
import sys
import copy


class PandaDynamicsModel():
    """
        Class for querying dynamical system properties of the Franka Panda robot
        
        Uses pyfranka to access the C++ library libfranka
    """
    def __init__(self, 
            load_mass = 0.5, 
            load_I = 0.5*_np.diag([0.01, 0.01, 0.01]), 
            load_masscenter = [0.1, 0.1, 0.0], 
            motorInertias = [0.01, 0.01, 0.01, 0.003,0.003,0.003,0.003, 0.01],
            viscuousFriction = [ 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5],
            frankaModelLibraryPath = '/usr/local/lib/libfrankamodellibrary.so',
            ):
        import pyfranka
        self.pandaDynamicsModel = pyfranka.Model(frankaModelLibraryPath)
        self.dofs = 8
        self.load_mass = load_mass
        self.load_I = list(load_I.flat)
        self.load_masscenter = load_masscenter
        self.jointSpaceMassMatrix = _np.eye(self.dofs, self.dofs)
        self.jointSpaceInvMassMatrix = _np.eye(self.dofs, self.dofs)
        self.motorInertiaMatrix = _np.diag(motorInertias) #projected inertia of the motor/gearbox
        self.viscuousFriction = _np.array(viscuousFriction)
        

    def getInertiaMatrix(self, position):
        position = _np.asarray(position)
        dofs_arm = 7
        inertias = 100*self.motorInertiaMatrix.copy()
        massInertia = self.pandaDynamicsModel.mass(list(position[:dofs_arm].flat), self.load_I,self.load_mass,self.load_masscenter)
        for j in range(dofs_arm):
            for i in range(dofs_arm):
                inertias[i,j] += massInertia[i+dofs_arm*j]
        return inertias

    def getViscuousFrictionCoefficients(self, jointState=None):
        return self.viscuousFriction



import PyKDL as _kdl
import kdl_parser_py.urdf as _kdl_parser
import rospy as _rospy
import subprocess

class PandaURDFModel():

    def __init__(self, robotDescriptionString=None):
        if robotDescriptionString is None:
            self.urdf_string = _rospy.get_param('/robot_description')
        else:
            self.urdf_string = robotDescriptionString
        self.baseLinkName = 'panda_link0'
        self.eeLinkName = 'panda_link7'
        isSuccessful, self.kdltree = _kdl_parser.treeFromString(self.urdf_string)
        if not isSuccessful:
            raise RuntimeError("could not parse '/robot_description'")
        self.ee_chain = self.kdltree.getChain(self.baseLinkName, self.eeLinkName)
        self.fk_ee = _kdl.ChainFkSolverPos_recursive(self.ee_chain)
        self.jointposition = _kdl.JntArray(7)
        self.eeFrame = _kdl.Frame()
        self.jac_ee = _kdl.ChainJntToJacSolver(self.ee_chain)
        self.jacobian = _kdl.Jacobian(7)

        #dynamics: (needs masses added to urdf!)
        self.grav_vector = _kdl.Vector(0., 0., -9.81)  
        self.dynParam = _kdl.ChainDynParam(self.ee_chain, self.grav_vector)
        self.inertiaMatrix = _kdl.JntSpaceInertiaMatrix(7)

        
    def setJointPosition(self, jointPosition):
        for i in range(7):
            self.jointposition[i] = jointPosition[i]
#        self.jointposition[7] = jointPosition[7] # I don't get what use the gripper would have here

        
    def getEELocation(self):
        self.fk_ee.JntToCart(self.jointposition, self.eeFrame)
        hTransform = _np.zeros((4,4))
        hTransform[3][3] = 1
        
        for iVec in range(0,3):
            hTransform[iVec][3] = self.eeFrame.p[iVec]
        
        for iMat in range(0,3):
            for jMat in range(0,3):
                hTransform[iMat][jMat] = self.eeFrame.M[(iMat,jMat)]
        
        return hTransform
    
    def getEEJacobian(self):
        self.jac_ee.JntToJac(self.jointposition, self.jacobian)
        
        # numpy array constructor does not work for kdl stuff.
        # There is likely to be a smarter way of doing this

        np_jac = _np.zeros([6,7])
        for row in range(6):
            for col in range(7):
                np_jac[row][col] = self.jacobian[row,col]
        return np_jac
        
    def getInertiaMatrix(self):
        self.dynParam.JntToMass(self.jointposition, self.inertiaMatrix)
        return self.inertiaMatrix



