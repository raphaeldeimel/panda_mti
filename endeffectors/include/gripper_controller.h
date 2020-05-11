// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once


#include <Eigen/Core>


//libfranka includes
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

//ros includes
#include "ros/ros.h"
#include <ros/time.h>
#include <panda_msgs_mti/PDControllerGoal8.h> //ros message types
#include <sensor_msgs/JointState.h>


const int dof_gripper = 7;


class GripperController {


public:

    
    GripperController(franka::Gripper& gripper, ros::NodeHandle& rosnode, ros::Duration  period);

    //~GripperController();

    //Control loop callback
    void update(void);


private:

    franka::Gripper* pGripper;
    
    franka::GripperState gripper_state;

    ros::Subscriber pdcontroller_goal_listener_;
    void callbackPDControllerGoal(const panda_msgs_mti::PDControllerGoal8::ConstPtr& msg);

    double dt;
    double invDt;
    double rosGripperTaud;
    double rosGripperQd;
    double rosGripperDQd;
    double rosGripperKp;
    double rosGripperKv;
    double payload_mass;

    const double q_max = 0.12;
    const double q_min = 0.00;

    const double dq_max = 0.05;
    const double dq_min = -0.05;
    
    const double threshold = 0.05;
    

    double qd;
    double dqd;
    double q;
    double dq;


    ros::Time time;


};





