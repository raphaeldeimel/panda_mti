// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once


#include <Eigen/Core>


//libfranka includes
#include <franka/exception.h>

//ros includes
#include "ros/ros.h"
#include <ros/time.h>
#include <panda_msgs_mti/PDControllerGoal8.h> //ros message types
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>



#include <csignal>
#include <ros/rate.h>
#include <franka/exception.h>

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <numeric>



#define SOFTHANDFINGERS 6
const int dof_gripper_joint = 7; // Where do we find the gripper Joint


class GripperMapper {


public:

    
    GripperMapper(ros::NodeHandle& rosnode, ros::Duration  period);

    //~GripperMapper();

    //Control loop callback
    void update(void);


private:

   
    std::array<double,SOFTHANDFINGERS> gripper_state;
    double  inflation = 0.2;
    double  deflation = -0.1;
    std_msgs::Float64MultiArray gripper_cmd;

    ros::Subscriber pdcontroller_goal_listener_;
    ros::Subscriber softhand_pressure_listener_;
    ros::Publisher softhand_commander;


    void callbackPDControllerGoal(const panda_msgs_mti::PDControllerGoal8::ConstPtr& msg);
    void callbackSofthandPressures(const std_msgs::Float64MultiArray& msg);

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





