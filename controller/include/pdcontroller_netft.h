// Copyright (c) 2018 Raphael Deimel
#pragma once

#include <controllerinterface.h>
#include <pdcontroller.h>

#include <panda_msgs_mti/PDControllerGoal8.h> //ros message types

class PDControllerNetft : public PDController
{
  public:
    PDControllerNetft(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode);
    ~PDControllerNetft();
    double rdt_data[6];
    void service(const franka::RobotState& robot_state, const franka::Duration period);
};





