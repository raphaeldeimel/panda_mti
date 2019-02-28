#pragma once

#include <pdcontroller.h>
#include <netft_rdt_driver.h>

class PDControllerNetft : public PDController
{
  public:
    PDControllerNetft(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode);
    ~PDControllerNetft();

    void service(const franka::RobotState& robot_state, const franka::Duration period);
    void stopNetft();

  protected:
    netft_rdt_driver::NetFTRDTDriver* ptrNetft;
};





