#!/usr/bin/env python3
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
@copyright: 2018
@licence: 2-clause BSD licence
"""


import numpy as _np
_np.set_printoptions(precision=4, suppress=True, formatter={'float': '{: 0.3f}'.format})
import os


import rospy
from sensor_msgs.msg import JointState
from panda_msgs_mti.msg import PDControllerGoal8, RobotState

import pandadynamicsmodel

rospy.init_node('emulated_pdcontroller')

urdfModel = pandadynamicsmodel.PandaURDFModel()

def getJointParameters(jointNameList):
    """
    read joint parameters (i.e. limits) from the robot's urdf description
    """
    import xml
    import rosparam

    jointNameSet = set(jointNameList)
    urdfTree = xml.etree.ElementTree.fromstring(rosparam.get_param('robot_description')) 
    jointParameters = {}
    for joint in urdfTree.findall('joint'):
        name = joint.attrib['name']
        if name not in jointNameSet:
            continue
        jointparam = {
            'name': name,
            'type': joint.attrib['type'],
        }            
        limits = joint.find('limit')
        if not limits is None:
            jointparam['lower limit'] = float(limits.attrib['lower'])
            jointparam['upper limit'] = float(limits.attrib['upper'])
            jointparam['velocity limit'] = float(limits.attrib['velocity'])
            jointparam['effort limit'] = float(limits.attrib['effort'])
        else:
            jointparam['lower limit'] = 3.1415
            jointparam['upper limit'] = -3.1415
            jointparam['velocity limit'] = 3.0
            jointparam['effort limit'] = 100.0

        jointParameters[name] = jointparam
    return jointParameters



watchdogPeriod = rospy.Duration.from_sec( rospy.get_param('watchdog timeout', 0.5))
jointaccmax = rospy.get_param('max joint acceleration', 4.0) #rad²/s²
performance_margin = rospy.get_param('performance margin', 1.0)  #factor of how much of the available performance (effort, velocity) is allowed to be used


updateFrequency = rospy.get_param('frequency', 50.0)
substeps=int(rospy.get_param('substeps time integration', 10)) #do Euler integration on a smaller dt:


warnIfLimitsReached= rospy.get_param('warn limits', False)

dt = 1.0 / updateFrequency
rate = rospy.Rate(updateFrequency) 
substepdt = dt / substeps



#indices for the mechanical states dimension:
iTau = 0
iPos = 1
iVel = 2

mechanicalStateCount = 3
dofs = 8

arm_id = rospy.get_param('arm_id')
jointNameList = [ "{}_{}".format(arm_id, i) for i in ('joint1','joint2','joint3','joint4','joint5','joint6','joint7','joint8') ]


jointParameters = getJointParameters(jointNameList)
jointposmax = _np.zeros((dofs))
jointposmin = _np.zeros((dofs))
jointvelmax = _np.zeros((dofs))
jointtaumax = _np.zeros((dofs))
jointtaumin = _np.zeros((dofs))
for i in range(dofs):
    p=jointParameters[jointNameList[i]]
    jointposmax[i] = p['upper limit']
    jointposmin[i] = p['lower limit']
    jointvelmax[i]  = p['velocity limit']* performance_margin
    jointtaumax[i]  = p['effort limit'] * performance_margin
    jointtaumin[i] = - jointtaumax[i] * performance_margin


#fix finger joint limits
jointposmax[-1] = 0.2

currentJointState = _np.zeros((mechanicalStateCount, dofs))
#set start posture to the controller goal of the first state:
currentJointState[iPos,:] = rospy.get_param('initial state', 0.0)

goal = _np.array(currentJointState)
kp = _np.zeros((dofs, dofs))
kv = _np.zeros((dofs, dofs))
kd = 15.0 * _np.eye(dofs)
ktau = _np.eye(dofs)
watchDogTriggered=True
startTime = rospy.Time.now() - rospy.Duration(dt)
goalTime = startTime

currentJointStateMsg = _np.zeros((mechanicalStateCount,dofs+1))

#listener for new controller goals/gains:
def ControllerGoalCallback(data):
    global goal, kp,kv, goalTime, jointposmin, jointposmax, jointvelmax, jointaccmax, jointtaumax
    newgoalTime = data.stamp
    newgoal = _np.empty((mechanicalStateCount, dofs))
    newgoal[iTau,:] = data.torque
    newgoal[iPos,:] = data.position
    newgoal[iVel,:] = data.velocity
    if len(data.kp) == dofs:
        newkp  = _np.diagflat(data.kp)
    elif len(data.kp) == dofs*dofs:
        newkp  = _np.array(data.kp).reshape((dofs, dofs))
    else:
        rospy.logerr(f"kp has wrong length: is {len(data.kp)}, should be {dofs} or {dofs*dofs}")
    if len(data.kv) == dofs:
        newkv  = _np.diagflat(data.kv)
    elif len(data.kv) == dofs*dofs:
        newkv  = _np.array(data.kv).reshape((dofs, dofs))
    else:
        rospy.logerr(f"kv has wrong length: is {len(data.kv)}, should be {dofs} or {dofs*dofs}")
    
    #for safety: limit positions:
    posLimited = _np.clip(newgoal[iPos,:], jointposmin, jointposmax)
    limitAmount = newgoal[iPos,:] - posLimited
    if _np.any( _np.abs(limitAmount >1e-5)) and warnIfLimitsReached:
        rospy.loginfo("limiting positions by:{0}".format(limitAmount))
    newgoal[iPos,:] = posLimited

    #for safety: try to limit torques:
    tauLimited = _np.clip(newgoal[iTau,:], jointtaumin, jointtaumax)
    newgoal[iTau,:] = tauLimited

    delta = newgoal - goal
    dt  = (newgoalTime - goalTime).to_sec()
    
    #for safety: try to limit acceleration
    deltaAcc = delta[iPos,:] - goal[iVel,:]
    deltaAcc[ currentJointState[iVel,:] * deltaAcc <= 0.0 ] = 0.0  #don't limit decelerating actions
    jointaccmax = 10.0 # _np.dot(jointSpaceInvMassMatrix, jointtaumax)
    accelerationBudget = _np.min( jointaccmax*dt /  (_np.abs(deltaAcc) +1e-7) )
    if _np.any(accelerationBudget < 1.0):
        delta = delta * accelerationBudget
        if warnIfLimitsReached:
            rospy.loginfo("limiting acceleration")

    #for safety: try to limit velocity
    velocityBudget = _np.min(jointvelmax*dt /  (_np.abs(delta[iVel,:])+1e-7))
    if _np.any(velocityBudget < 1.0):
        delta = delta * velocityBudget
        if warnIfLimitsReached:
            rospy.loginfo("limiting velocity")


    #publish new goal data:
    goal = goal + delta
    goalTime = newgoalTime
    kp[...] = newkp
    kv[...] = newkv
    kd[...] = 1.0 * _np.eye(dofs)  #intrinsic damping of the robot


#publisher for the emulated robot pose:
jointMsg = JointState()
jointMsg.name=jointNameList
#desiredJointPublisher = rospy.Publisher("joint_states", JointState,  queue_size=3)
desiredControllerGoalListener = rospy.Subscriber("pdcontroller_goal", PDControllerGoal8, ControllerGoalCallback, queue_size=3)

currentStateMsg = RobotState()
currentStateMsg.dofs = dofs
currentStateMsg.q = [0]*8
currentStateMsg.dq = [0]*8
currentStateMsg.ddq = [0]*8
currentStateMsg.tau = [0]*8
currentStateMsg.tau_ext = [0]*8
currentStateMsg.qd = [0]*8
currentStateMsg.dqd = [0]*8
currentStateMsg.mode = 0
currentStateMsg.dofs = 8
currentStateMsg.dofs = 8
currentStateMsg.ee_htransform_base = [0] * 16
currentStateMsg.ee_wrench_ee = [0] * 6
currentStateMsg.ee_jacobian_ee = [0] * 42
currentStateMsg.ee_dotjacobian_ee = [0] * 42
currentStatePublisher = rospy.Publisher("currentstate", RobotState,  queue_size=3)

torques = _np.zeros((dofs))

rospy.loginfo('starting up.')
while not rospy.is_shutdown():
    rate.sleep()
    now  = rospy.Time.now()
    deltaTime = (now - startTime)

    #if no recent goal was posted, do some damage control
    if goalTime + watchdogPeriod < now:
        if watchDogTriggered == False:
            rospy.logwarn("pdcontrolleremulationnode: nobody is sending me updates!! Stopping for safety.")
        watchDogTriggered = True
        goal[iTau,:] = 0.0
        goal[iVel,:] = 0.0
        kp[...] = 1.0 * _np.eye(dofs)
        kv[...] = 0.0
        kd[...] = 1.0 * _np.eye(dofs)
    else:
        if watchDogTriggered == True:
            watchDogTriggered = False
            rospy.logwarn("Resuming control.")

##for debugging: print commanded values:    
#    rospy.logwarn("==")
#    rospy.logwarn("kp: {0}".format(kp))
#    rospy.logwarn("kd: {0}".format(kd))
#    rospy.logwarn("Tau: {0}".format(goal[iTau,:]))
#    rospy.logwarn("Pos: {0}".format(goal[iPos,:]))
#    rospy.logwarn("Vel: {0}".format(goal[iVel,:]))

    #emulate thresholded gripper:
    if goal[iPos,dofs-1] > 0.03:
        goal[iPos,dofs-1] = 0.06
    else:
        goal[iPos,dofs-1] = 0.0
    goal[iVel,dofs-1] = 0.0
    goal[iTau,dofs-1] = 0.0


    #emulate the pd control:
    #PD control law:
    for i in range(substeps):
        delta = goal - currentJointState
        torques_unfiltered = ktau @ goal[iTau,:] + kp @ delta[iPos,:] + kv @ delta[iVel,:] - kd @ currentJointState[iVel,:]
        
        torques += 0.333 * (torques_unfiltered-torques) #torque rate limited by hardware
        #compute dynamics:
        urdfModel.setJointPosition(currentJointState[iPos,:])
        
        #emulate effect:
        #integrate over the given time interval:
        jointSpaceInertiaMatrix =  urdfModel.getInertiaMatrix()
        jointSpaceInvInertiaMatrix = _np.linalg.inv(jointSpaceInertiaMatrix)
        
#        print(_np.linalg.eigvals(jointSpaceInertiaMatrix))
#        print(_np.diag(jointSpaceInvInertiaMatrix[:7,:7]))
        
        friction_torque = currentJointState[iVel,:] * urdfModel.getViscuousFrictionCoefficients()
        ddq = _np.dot(jointSpaceInvInertiaMatrix, torques - friction_torque  )
        currentJointState[iVel,:]  =  currentJointState[iVel,:] + ddq * substepdt
        currentJointState[iPos,:]  =  currentJointState[iPos,:] + currentJointState[iVel,:] * substepdt + 0.5 * ddq * substepdt*substepdt
        currentJointState[iTau,:] = torques
    currentJointState[iTau,:] = _np.clip(currentJointState[iTau,:], jointtaumin, jointtaumax)
    currentJointState[iVel,:] = _np.clip(currentJointState[iVel,:], -jointvelmax, jointvelmax)
    currentJointState[iPos,:] = _np.clip(currentJointState[iPos,:], jointposmin, jointposmax)

    currentStateMsg.q[:] = currentJointState[iPos,:] 
    currentStateMsg.dq[:] =  currentJointState[iVel,:]
    currentStateMsg.tau[:] = currentJointState[iTau,:]
    currentStateMsg.qd[:] = goal[iPos,:]  
    currentStateMsg.dqd[:] = goal[iVel,:]
    currentStateMsg.mode = 0

    # Jacobian
    jacoKDLBase = urdfModel.getJacobian()

    # hTransform 
    hTransform = urdfModel.getEELocation()

    # Calculate EE Jacobian from Base 
    hTransformI = _np.linalg.inv(hTransform)
    hTranslateSkewSym = _np.array([ [0,-hTransformI[2,3],hTransformI[1,3]],
                                    [hTransformI[2,3], 0, -hTransformI[0,3]],
                                    [-hTransformI[1,3], hTransformI[0,3], 0]])
    hTransformIAdjoint =  _np.vstack((_np.hstack((hTransformI[0:3,0:3], _np.zeros((3,3)))), _np.hstack((_np.matmul(hTranslateSkewSym,hTransformI[0:3,0:3]),hTransformI[0:3,0:3]))))
    jacoKDLEE = _np.matmul(hTransformIAdjoint,jacoKDLBase)

    # convert numpy to column major format list
    pos_count = 0
    for column in range(jacoKDLBase.shape[1]):
        for row in range(jacoKDLBase.shape[0]):
            currentStateMsg.ee_jacobian_ee[pos_count] = jacoKDLEE[row][column]
            #currentStateMsg.jacobian_base[pos_count] = jacoKDLBase[row][column]
            pos_count += 1

    # check if jacobian has right size 6x7
    if not len(currentStateMsg.ee_jacobian_ee) == 42:
        rospy.logerr("Emulated jacobian has size %d, but should have 42.", len(currentStateMsg.ee_jacobian_ee))

    # convert numpy to column major format list
    pos_count = 0
    for column in range(hTransform.shape[1]):
        for row in range(hTransform.shape[0]):
            currentStateMsg.ee_htransform_base[pos_count] = hTransform[row][column]
            pos_count += 1

    # check if hTransform has right size 4x4
    if not len(currentStateMsg.ee_htransform_base) == 16:
        rospy.logerr("Emulated hTransform has size %d, but should have 16.", len(currentStateMsg.ee_htransform_base))

    currentStateMsg.stamp = now
    currentStatePublisher.publish(currentStateMsg)


#done by controller2jointstatemsgs node:
#    # for the JointStateMsg, split pos, vel, effort for the gripper dof into two states:
#    currentJointStateMsg[:,:dofs-1] =     currentJointState[:,:dofs-1]
#    currentJointStateMsg[:, dofs-1] = 0.5*currentJointState[:, dofs-1]
#    currentJointStateMsg[:, dofs  ] = 0.5*currentJointState[:, dofs-1]
#    jointMsg.position = currentJointStateMsg[iPos,:]
#    jointMsg.velocity = currentJointStateMsg[iVel,:]
#    jointMsg.effort  =  currentJointStateMsg[iTau,:]
#    jointMsg.header.stamp = now 
#    desiredJointPublisher.publish(jointMsg)
    
    


