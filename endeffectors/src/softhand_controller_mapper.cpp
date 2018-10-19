/*



*/
#include <softhand_controller_mapper.h>


//ctrl-c functionality:
volatile static bool exitrequested = false;
void exithandler(int s){
           std::cout << "Ctrl-C pressed. Will exit when control stops." << std::endl;
           exitrequested = true; 
}



void GripperMapper::callbackPDControllerGoal(const panda_msgs_mti::PDControllerGoal8::ConstPtr& msg) 
{

    //internalize data:
    rosGripperTaud = msg->torque[dof_gripper_joint]; 
    rosGripperQd   = msg->position[dof_gripper_joint];
    rosGripperDQd  = msg->velocity[dof_gripper_joint];
    rosGripperKp   = msg->kp[dof_gripper_joint];
    rosGripperKv   = msg->kv[dof_gripper_joint];
    std::cout << "rosGripperQd: " << rosGripperQd << std::endl;
}


void GripperMapper::callbackSofthandPressures(const std_msgs::Float64MultiArray& msg){

    for(int i = 0 ; i < 6; i ++){  
        gripper_state[i] = msg.data[i];
        //std::cout << "gripper_pressure[" << i << "]: "<< gripper_state[i] << std::endl;
    }
        double gripper_pressure_mean  = std::accumulate(gripper_state.begin(), gripper_state.end(), 0.0) / gripper_state.size(); // calculates the mean pressure of
        std::cout << "gripper_pressure_mean: " << gripper_pressure_mean << std::endl;
       
}


GripperMapper::GripperMapper(ros::NodeHandle& rosnode, ros::Duration period)
{
  pdcontroller_goal_listener_ = rosnode.subscribe("/pdcontroller_goal", 5, &GripperMapper::callbackPDControllerGoal, this);
  softhand_pressure_listener_ = rosnode.subscribe("/pneumaticbox/pressures", 5,&GripperMapper::callbackSofthandPressures, this);
  softhand_commander = rosnode.advertise<std_msgs::Float64MultiArray>("/softhand/commands", 10);
  dt = period.toSec();
  invDt = 1.0 / dt;

  rosGripperQd = q_max;

  gripper_cmd = std_msgs::Float64MultiArray();  
  gripper_cmd.data.clear();
  for(int i = 0 ; i < 6; i ++){  
     gripper_cmd.data.push_back(0);
  }
  gripper_cmd.layout.data_offset = 0;
  std::cout << "Gripper_CMD: " << gripper_cmd << std::endl;
  softhand_commander.publish(gripper_cmd);
  std::cout << "pdcontroller_goal is successfully mapped to the softhand." << std::endl;    
  //pGripper->move(rosGripperQd, dq_max);
}

    //Control loop callback
 
void GripperMapper::update(void)
{
  //gripper_state = pGripper->readOnce(); --> we have that in callbackSofthandPressures now

  q  = std::accumulate(gripper_state.begin(), gripper_state.end(), 0.0) / gripper_state.size(); // calculates the mean pressure of all fingers

    
 if (rosGripperQd > threshold && dq <= threshold) {
  
  dq = rosGripperQd;
  std::cout << "open " << std::endl;   

   
  //bool result = pGripper->move(q_max, dq_max);
  gripper_cmd.data.clear();
  for(int i = 0 ; i < 6; i ++){  
     gripper_cmd.data.push_back(deflation);
  }
  gripper_cmd.layout.data_offset = 0;
  softhand_commander.publish(gripper_cmd);


 } else if  (rosGripperQd < threshold  && dq >= threshold ) {
  dq = rosGripperQd;
  std::cout <<"grasp " <<  std::endl;


  //bool result = pGripper->grasp(0.0, dq_max, std::abs(rosGripperTaud), 0.2, 0.2);
  gripper_cmd.data.clear();
  for(int i = 0 ; i < 6; i ++){  
     gripper_cmd.data.push_back(inflation);
  }
  gripper_cmd.layout.data_offset = 0;
  softhand_commander.publish(gripper_cmd);
    



 } else {
  //std::cout <<"idle" << std::endl;
 }
}

int main(int argc, char** argv) {
  std::cout << "Will initialize the Softhand now!" << std::endl;  
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

 if(gripper_type == "softhand"){
    std::cout << "Softhand is to be used!" << std::endl;  
    GripperMapper softhand = GripperMapper(rosnode, rate.expectedCycleTime());
    std::signal(SIGINT, exithandler); //register AFTER franka::Robot
    //Keep running depending on the panda state, 
    do{ 
        rate.sleep();
        ros::spinOnce();
        softhand.update();
     } while (exitrequested == false);
  }
  return 0;
}




