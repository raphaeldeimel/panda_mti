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
    try
    {
        ptrNetft = new netft_rdt_driver::NetFTRDTDriver("192.168.11.3");
    }
    catch(std::runtime_error)
    {
        std::cout << "netft error";
    }
}


PDControllerNetft::~PDControllerNetft() {}


void PDControllerNetft::service(const franka::RobotState& robot_state, const franka::Duration period)
{


    //ptrNetft->getData(&rdt_data[0]);


    PDController::service(robot_state, period);
}

void PDControllerNetft::stopNetft()
{
    //stop rdt stream
    ptrNetft->stopStreaming();
}
