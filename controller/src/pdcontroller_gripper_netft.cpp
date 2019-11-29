#include <iostream>
#include <memory>
#include <unistd.h>

#include <pdcontroller_gripper_netft.h>
#include <netft_rdt_driver.h>
#include <gripperbackground.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "pdcontroller_gripper_netft_realtime");
    int exitval = mainloop<PDControllerGripperNetft >(); //run the control main loop using PDController
    return exitval;
}

PDControllerGripperNetft::PDControllerGripperNetft(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode) :
    PDController(robot, hostname, rosnode)
{

    rosnode.param<bool>("withGripper",withGripper, true );
    rosnode.param<bool>("withNetft",withNetft, true );
    rosnode.param<std::string>("netft_address", netft_address, "netft.local" );

    ptrNetft=NULL;
    if (withNetft)   {
        try
        {
        ptrNetft = new netft_rdt_driver::NetFTRDTDriver(netft_address);
        }
        catch(std::runtime_error)
        {
        ROS_ERROR_STREAM(myName << ": connection to netft at " << netft_address <<" failed.");
        std::cout << netft_address;

        withNetft=false;
        }
    }
    if (withGripper) {
        try {
            grippernonblocking::startGripperCommunicationInBackground(hostname);
        } catch (const franka::Exception& ex) {
            ROS_ERROR_STREAM(myName << ": Gripper not responding." << ex.what() << std::endl);
            return;
        }        
    }    

    if (withGripper) {ROS_INFO_STREAM(myName << ": gripper enabled");}
    if (ptrNetft!=NULL) {ROS_INFO_STREAM(myName << ": F/T sensor at " << netft_address << " enabled");}
}


PDControllerGripperNetft::~PDControllerGripperNetft()
{
    //stops rdt stream
    if (ptrNetft != NULL)   {ptrNetft->stopStreaming(); delete ptrNetft; ptrNetft=NULL;}
    if (withGripper) {grippernonblocking::stopGripperCommunicationInBackground();}
}


void PDControllerGripperNetft::service(const franka::RobotState& robot_state, const franka::Duration period)
{
    if (ptrNetft != NULL)  {ptrNetft->getData(&PDController::rdtdata_[0]);}
    if (withGripper) {gripper_q =  grippernonblocking::getGripperPosition();}
    PDController::service(robot_state, period);
}



franka::Torques PDControllerGripperNetft::update(const franka::RobotState& robot_state, const franka::Duration period) 
{
    gripper_q =  grippernonblocking::getGripperPosition();
    franka::Torques t = PDController::update(robot_state, period);
    grippernonblocking::setGripperDesired(rosGripperQd, 10.0); // taud_[7]
    return t;
}


