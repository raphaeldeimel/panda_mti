// Copyright (c) 2018 Raphael Deimel
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>


#include <statepublisher.h>
#include <gripperbackground.h>




int main(int argc, char** argv) {
  //init rosnode
  ros::init(argc, argv, "panda_stateobserver");
  int exitval = mainloop<ROSPublishingController >(); //run the control main loop using PDController
  return exitval;
}





ROSPublishingController::ROSPublishingController(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode) :
    gripper_q(0.0)
{
    pRobot = &robot;
    pModel = new franka::Model(robot.loadModel());
    current_state = robot.readOnce();
    initial_state = current_state;
    
    common_state_publisher = rosnode.advertise<mti_panda_controller_msgs::RobotState8>("/panda/currentstate", 10);

    grippernonblocking::startGripperCommunicationInBackground(hostname);
    grippernonblocking::setGripperDesired(grippernonblocking::q_max,0.0);

    ROS_INFO_STREAM("initial robot joint state: {" << initial_state.q[0] << ", " <<
    initial_state.q[1] << ", " <<
    initial_state.q[2] << ", " <<
    initial_state.q[3] << ", " <<
    initial_state.q[4] << ", " <<
    initial_state.q[5] << ", " <<
    initial_state.q[6] << "}");

}

ROSPublishingController::~ROSPublishingController() 
{
 delete pModel;
 grippernonblocking::stopGripperCommunicationInBackground();
}

void ROSPublishingController::onStart() { }


void ROSPublishingController::service(const franka::RobotState& robot_state, const franka::Duration period) {

    current_state = robot_state;

    if (state_culling_count-- <= 0) {
        state_culling_count = publisher_culling_amount;
        mti_panda_controller_msgs::RobotState8 statemsg = mti_panda_controller_msgs::RobotState8();
        statemsg.stamp = ros::Time::now();
        statemsg.mode = (int)robot_state.robot_mode;
        for (int i=0;i<dofs;i++) { statemsg.q[i]   = robot_state.q[i];}
        for (int i=0;i<dofs;i++) { statemsg.dq[i]  = robot_state.dq[i];}
//        for (int i=0;i<dofs;i++) { statemsg.tau[i] = robot_state.tau_J[i];}
        for (int i=0;i<dofs;i++) { statemsg.tau[i] = robot_state.tau_ext_hat_filtered[i];}

        gripper_q = grippernonblocking::getGripperPosition();
        
        statemsg.q[dofs] = gripper_q;
        statemsg.dq[dofs]  = 0.0;
        statemsg.tau[dofs] = 0.0;
        statemsg.qd[dofs]  = 0.0;
        statemsg.dqd[dofs] = 0.0;
        
        
        common_state_publisher.publish(statemsg);
    }
    //give the ros listener time to update its values:
    ros::spinOnce(); 

    static int k = 0;
    if (robot_state.control_command_success_rate > 0.05 && robot_state.control_command_success_rate < 0.90) {
        if (k <= 0) {
            std::cout << "Warning: " << (1.0 - robot_state.control_command_success_rate)*100 << " out of 100 loops where too late." << std::endl;
            k = 500;
        }
        k--;
    }
}



