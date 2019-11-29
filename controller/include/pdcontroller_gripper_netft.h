#pragma once

#include <pdcontroller.h>
#include <netft_rdt_driver.h>

class PDControllerGripperNetft : public PDController
{
  public:
    PDControllerGripperNetft(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode);
    ~PDControllerGripperNetft();

    void service(const franka::RobotState& robot_state, const franka::Duration period);

    franka::Torques update(const franka::RobotState& robot_state, const franka::Duration period);

  protected:
    netft_rdt_driver::NetFTRDTDriver* ptrNetft;
    std::string netft_address;
    bool withNetft = true;
    bool withGripper = true;
    //bool withSoftHand = False;
};





