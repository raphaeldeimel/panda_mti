// Copyright (c) 2018 Raphael Deimel
#pragma once

#include <controllerinterface.h>
#include <pdcontroller.h>

#include <panda_msgs_mti/PDControllerGoal8.h> //ros message types

class PDControllerGripper : public PDController
{

public:
    PDControllerGripper(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode);
    ~PDControllerGripper();
    
    //void onStart();
    
    //Control loop callback
    franka::Torques update(const franka::RobotState& robot_state, const franka::Duration period);

    //callback that should be called regularly if update isn't
    void service(const franka::RobotState& robot_state, const franka::Duration period);

    

};





