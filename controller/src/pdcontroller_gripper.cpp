// Copyright (c) 2018 Raphael Deimel
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>


#include <pdcontroller_gripper.h>
#include <gripperbackground.h>

#include <atomic>


int main(int argc, char** argv) {
  //init rosnode
  ros::init(argc, argv, "pdcontroller_gripper_realtime");
  int exitval = mainloop<PDControllerGripper >(); //run the control main loop using PDController
  return exitval;
}




PDControllerGripper::PDControllerGripper(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode) :
     PDController(robot, hostname, rosnode)
{
    grippernonblocking::startGripperCommunicationInBackground(hostname); 
}


PDControllerGripper::~PDControllerGripper() {
    grippernonblocking::stopGripperCommunicationInBackground();
}

//void PDControllerGripper::onStart() {}

void PDControllerGripper::service(const franka::RobotState& robot_state, const franka::Duration period) 
{
    gripper_q =  grippernonblocking::getGripperPosition();
    PDController::service(robot_state,period);
    
}



franka::Torques PDControllerGripper::update(const franka::RobotState& robot_state, const franka::Duration period) 
{
    gripper_q =  grippernonblocking::getGripperPosition();
    franka::Torques t = PDController::update(robot_state, period);
    grippernonblocking::setGripperDesired(qd_[7], 10.0); // taud_[7]
    return t;
}


