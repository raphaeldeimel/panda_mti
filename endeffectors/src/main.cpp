/*



*/
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <gripper_controller.h>
#include <softhand_controller_mapper.h>

#include <csignal>


#include <ros/rate.h>

#include <franka/exception.h>
#include <franka/gripper.h>


//ctrl-c functionality:
volatile static bool exitrequested = false;
void exithandler(int s){
           std::cout << "Ctrl-C pressed. Will exit when control stops." << std::endl;
           exitrequested = true; 
}


void GripperController::callbackPDControllerGoal(const mti_panda_controller_msgs::PDControllerGoal8::ConstPtr& msg) 
{

    //internalize data:
    rosGripperTaud = msg->torque[dof_gripper];
    rosGripperQd = msg->position[dof_gripper];
    rosGripperDQd = msg->velocity[dof_gripper];
    rosGripperKp = msg->kp[dof_gripper];
    rosGripperKv = msg->kv[dof_gripper];

}


GripperController::GripperController(franka::Gripper& gripper, ros::NodeHandle& rosnode, ros::Duration period)
{
  pGripper  = &gripper;

  pdcontroller_goal_listener_ = rosnode.subscribe("/mixer/pdcontroller_goal", 5, &GripperController::callbackPDControllerGoal, this);
  
  dt = period.toSec();
  invDt = 1.0 / dt;

  rosGripperQd = q_max;
  pGripper->move(rosGripperQd, dq_max);
}

    //Control loop callback
void GripperController::update(void)
{
  gripper_state = pGripper->readOnce();
  q  = gripper_state.width;


//  double dqd = invDt * (rosGripperQd-qd);  
//  if (dqd > dq_max) {dqd = dq_max;}
//  if (dqd < dq_min) {dqd = dq_min;}
//  if (qd > q_max) {qd = q_max;}
//  if (qd < q_min) {qd = q_min;}
//  qd  = qd + dqd * dt;
//  std::cout << "q: "<< gripper_state.width << std::endl;
//  std::cout << "qd: "<< qd << std::endl;
//  std::cout << "dqd: "<< dqd << std::endl;


 if (rosGripperQd > threshold && dq <= threshold) {
  dq = rosGripperQd;
  std::cout << "open " << std::endl;
  bool result = pGripper->move(q_max, dq_max);
 } else if  (rosGripperQd < threshold  && dq >= threshold ) {
  dq = rosGripperQd;
  std::cout <<"grasp " <<  std::endl;
  bool result = pGripper->grasp(0.0, dq_max, std::abs(rosGripperTaud), 0.2, 0.2);
 } else {
  //std::cout <<"idle" << std::endl;
 }
}



 

int main(int argc, char** argv) {
  std::cout << "Will initialize the gripper now!" << std::endl;  
  std::cout << std::setprecision(4);
  //init rosnode
  ros::init(argc, argv, "gripper_controller");
  ros::NodeHandle rosnode;
  ros::Rate rate(5);

  //get your params from ros config/launchfiles
  std::string hostname;
  rosnode.param<std::string>("/panda/hostname",hostname, "panda" );


  std::string gripper_type;
  rosnode.param<std::string>("/panda/gripper_type",gripper_type, "softhand" );

  if(gripper_type == "franka_gripper"){
      try {
        franka::Gripper gripper(hostname);
        GripperController gripper_controller = GripperController(gripper, rosnode, rate.expectedCycleTime());

        std::signal(SIGINT, exithandler); //register AFTER franka::Robot

        //Keep running depending on the panda state, 
        do {
            rate.sleep();
            ros::spinOnce();
            gripper_controller.update();
         } while (exitrequested == false);
       } catch (const std::exception& ex) {std::cout << "Exception: " << ex.what() << std::endl;}// print exception

  }
  return 0;
}





