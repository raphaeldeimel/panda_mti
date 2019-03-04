#include <iostream>
#include <memory>
#include <unistd.h>

#include <pdcontroller_netft.h>
#include <netft_rdt_driver.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "pdcontroller_netft_realtime");
    int exitval = mainloop<PDControllerNetft >(); //run the control main loop using PDController
    return exitval;
}

PDControllerNetft::PDControllerNetft(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode) :
    PDController(robot, hostname, rosnode)
{
    std::string netft_address; // error in header
    rosnode.getParam("/netft/hostaddress", netft_address);
    try
    {
        ptrNetft = new netft_rdt_driver::NetFTRDTDriver(netft_address);
    }
    catch(std::runtime_error)
    {
        std::cout << "netft error: connection to netft failed";
    }
}


PDControllerNetft::~PDControllerNetft()
{
    //stops rdt stream
    this->stopNetft();
}


void PDControllerNetft::service(const franka::RobotState& robot_state, const franka::Duration period)
{
    ptrNetft->getData(&PDController::rdtdata_[0]);
    PDController::service(robot_state, period);
}

void PDControllerNetft::stopNetft()
{
    ptrNetft->stopStreaming(); //stop rdt stream
}
