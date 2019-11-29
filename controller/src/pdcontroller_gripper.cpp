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
    ROS_INFO_STREAM(myName << ": Consider updating your launch file to use pdcontroller_gripper_netft instead!" << std::endl);
    try {
      grippernonblocking::startGripperCommunicationInBackground(hostname);
    } catch (const franka::Exception& ex) {
      ROS_ERROR_STREAM(myName << ": Gripper not responding." << ex.what() << std::endl);
      return;
    }    

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
    grippernonblocking::setGripperDesired(rosGripperQd, 10.0); // taud_[7]
    return t;
}


