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
#include <atomic>
#include <csignal>
#include <time.h>

#include <controllerinterface.h>


static ControllerInterface* currentController = NULL;

ControllerInterface::~ControllerInterface() {} //c++ requires pure virtual desctructors to be declared anyway...

//ctrl-c functionality:
volatile static bool exitrequested = false;
void exithandler(int s){
           ROS_INFO_STREAM(currentController->myName << ": Ctrl-C pressed. Will exit panda controller when control stops." << std::endl);
           exitrequested = true;
}



const int statistics_maxcount = 10000;
static int statistics_remaining = statistics_maxcount; 
static int statistics_runtime_sum = 0;
static int statistics_runtime_min = 99999999;
static int statistics_runtime_max = 0;

//callback wrapper for franka control
franka::Torques callbackControl(const franka::RobotState& robot_state, franka::Duration period) 
{
    struct timespec t_start;
    clock_gettime(CLOCK_REALTIME, &t_start);

    franka::Torques commandedTorques =  currentController->update(robot_state, period); 
    
    //monitor the execution time and warn if limit is exceeded:
    struct timespec t_end;
    clock_gettime(CLOCK_REALTIME, &t_end);
    int nsecs  = t_end.tv_nsec - t_start.tv_nsec;
    if (nsecs < 0) {nsecs += 1000000000;}
    int usecs = nsecs / 1000;
    if (usecs > 300) {ROS_WARN_STREAM(currentController->myName << ": update() took "<< usecs  <<" us! (target: < 300)" << std::endl);}
    if (statistics_remaining > 0) {
        statistics_runtime_sum += usecs;
        if (usecs < statistics_runtime_min) {statistics_runtime_min = usecs;}
        if (usecs > statistics_runtime_max) {statistics_runtime_max = usecs;}
        statistics_remaining--;
    }
    if (statistics_remaining <= 0) {
        ROS_WARN_STREAM(currentController->myName << ": control loop min/avg/max us: " << statistics_runtime_min << "/" << statistics_runtime_sum / statistics_maxcount << "/" << statistics_runtime_max << std::endl);
        statistics_remaining = statistics_maxcount;
        statistics_runtime_sum = 0;
        statistics_runtime_max = 0;
        statistics_runtime_min = 99999999;
    }

    return commandedTorques;
}

//callback wrapper for franka control
bool callbackRead(const franka::RobotState& robot_state) 
{

    struct timespec t_start;
    clock_gettime(CLOCK_REALTIME, &t_start);

    franka::Duration period(1);
    currentController->service(robot_state, period); 
    
    //monitor the execution time and warn if limit is exceeded:
    struct timespec t_end;
    clock_gettime(CLOCK_REALTIME, &t_end);
    int nsecs  = t_end.tv_nsec - t_start.tv_nsec;
    if (nsecs < 0) {nsecs += 1000000000;}
    int usecs = nsecs / 1000;
    if (usecs > 300) {ROS_WARN_STREAM(currentController->myName << ": The control loop's service() took longer than "<< usecs  <<" us!" << std::endl);}
    
    if (robot_state.robot_mode == franka::RobotMode::kUserStopped) {
        return false;
    } else {
        return (not exitrequested);
    }
}


int mainloopImpl(franka::Robot& robot, ControllerInterface* cntrl) 
{
    franka::Duration idlePeriod(25);
    ros::Duration idleLoopRate(idlePeriod.toSec()); 

    currentController = cntrl;
    ros::spinOnce();

    franka::RobotState robot_state;
    franka::RobotMode robotmode = franka::RobotMode::kUserStopped;
    std::signal(SIGINT, exithandler); //register AFTER franka::Robot

    //Make sure we get the correct data in the beginning, before switching on any control:
    for (int i=10; i>0;i--) {
        currentController->service(robot_state, idlePeriod);
        ros::spinOnce();
        idleLoopRate.sleep();
    }
    
    //Keep running depending on the panda state, 
    try {
    do {
        robot_state = robot.readOnce();
        robotmode = robot_state.robot_mode;
        if (robotmode == franka::RobotMode::kIdle) {
            // start real-time control loop
            ROS_INFO_STREAM(currentController->myName << ": Starting Control"<< std::endl);
            currentController->onStart();
            /* --> HERE WE HAVE THE LIBFRANKA-Controller-LOOP that executes MTI PDController <-- */
                if (currentController->isTorqueController()) {
                    try {
                        robot.control( callbackControl );  /* control always exits via an exception*/
                    } catch (const franka::ControlException& ex) {  //when e.g. pressing the stop button, control exits via an exception
                        ;
                    }
                } else {
                    robot.read( callbackRead ); //fallback: only provide current values and no control
                }
            ROS_INFO_STREAM(currentController->myName << ": Stopping Control"<< std::endl);
        } else if (robotmode == franka::RobotMode::kGuiding) {
            ROS_INFO_STREAM(currentController->myName << ": Guiding Active"<< std::endl);
            robot.read( callbackRead );
            ROS_INFO_STREAM(currentController->myName << ": Guiding Stopped"<< std::endl);
        } else if (robotmode == franka::RobotMode::kUserStopped) {
            currentController->service(robot_state, idlePeriod);
        } else if (robotmode == franka::RobotMode::kOther) {
            std::cout << std::string(62, '#' )<<"\n#" <<std::string(60,' ') << "#\n#" << std::string(60,' ') << "#\n#" <<std::string(60,' ') << "#\n#" << std::string(21, ' ' ) << "Joints are locked!" << std::string(21, ' ' )<< "#\n#" <<std::string(60,' ')<< "#\n#" << std::string(60,' ') << "#\n#" << std::string(60,' ') << "#\n" << std::string(62,'#') << std::endl;
            robot.read( callbackRead );
            std::cout << std::string(62, '#' )<<"\n#" <<std::string(60,' ') << "#\n#" << std::string(60,' ') << "#\n#" <<std::string(60,' ') << "#\n#" << std::string(20, ' ' ) << "Joints are unlocked!" << std::string(20, ' ' )<< "#\n#" <<std::string(60,' ')<< "#\n#" << std::string(60,' ') << "#\n#" << std::string(60,' ') << "#\n" << std::string(62,'#') << std::endl;
        } else {
            break; //any other mode -> quit
        }
        idleLoopRate.sleep();
    } while (not exitrequested);

    } catch (const franka::CommandException& ex) {  //when e.g. pressing the stop button, control exits via an exception
        ROS_ERROR_STREAM(currentController->myName << ": franka::CommandException: " << ex.what() << std::endl);
        throw ex;
    }

    std::string modestr = "unknown";
    switch (robotmode) {
        case franka::RobotMode::kOther:
            modestr = "kOther";
            break;
        case franka::RobotMode::kIdle:
            modestr = "kIdle";
            ROS_INFO("kIdle");
            break;
        case franka::RobotMode::kMove:
            modestr = "kMove";
            break;
        case franka::RobotMode::kGuiding:
            modestr = "kGuiding";
            break;
        case franka::RobotMode::kReflex:
            modestr = "kReflex";

            ROS_ERROR_STREAM("Panda shut down due to Reflex!: " << robot_state.last_motion_errors);
            //ROS_INFO_STREAM( "cartesian collision flags: ");
            //for (int i=0;i<6;i++) {ROS_INFO(robot_state.cartesian_collision[i]);}
            //ROS_INFO_STREAM( std::endl << "joint collision flags: ");
            //for (int i=0;i<7;i++) {ROS_INFO(robot_state.joint_collision[i]);}
            break;
        case franka::RobotMode::kUserStopped:
            modestr = "kUserStopped";
            break;
        case franka::RobotMode::kAutomaticErrorRecovery:
            modestr = "kAutomaticErrorRecovery";
            break;
    }
    ROS_INFO_STREAM(currentController->myName << ": Leaving robot in mode: " << modestr << std::endl);
  return 0;
}






