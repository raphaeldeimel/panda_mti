// Copyright (c) 2018 Raphael Deimel
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>


#include <pdcontroller_netft.h>

#include <atomic>


int main(int argc, char** argv) {
  //init rosnode
  ros::init(argc, argv, "pdcontroller_netft_realtime");
  int exitval = mainloop<PDControllerNetft >(); //run the control main loop using PDController
  return exitval;
}




PDControllerNetft::PDControllerNetft(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode) :
     PDController(robot, hostname, rosnode)
{
    std::string address = "192.168.11.3";
    //hier rdt com
    std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> netft;
    try
    {
      netft = std::shared_ptr<netft_rdt_driver::NetFTRDTDriver>(new netft_rdt_driver::NetFTRDTDriver(address));
    }
    catch(std::runtime_error)
    {
        std::cout << "error";
    }
}


PDControllerNetft::~PDControllerNetft() {
    //stop streaming
}

void PDControllerNetft::service(const franka::RobotState& robot_state, const franka::Duration period)
{
    //get new data here
    if (netft->waitForNewData())
    {
        netft->getData(&rdt_data[0]);
    }
    PDController::service(robot_state,period);
    
}


