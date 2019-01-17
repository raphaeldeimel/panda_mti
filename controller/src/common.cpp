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



ControllerInterface::~ControllerInterface() {} //c++ requires pure virtual desctructors to be declared anyway...

//ctrl-c functionality:
volatile static bool exitrequested = false;
void exithandler(int s){
           std::cout << "Ctrl-C pressed. Will exit when control stops." << std::endl;
           exitrequested = true;
}

static ControllerInterface* currentController = NULL;


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
    if (usecs > 300) {std::cout << "Warning: The control loop's update() took longer than "<< usecs  <<" us!" << std::endl;}
    if (statistics_remaining > 0) {
        statistics_runtime_sum += usecs;
        if (usecs < statistics_runtime_min) {statistics_runtime_min = usecs;}
        if (usecs > statistics_runtime_max) {statistics_runtime_max = usecs;}
        statistics_remaining--;
    }
    if (statistics_remaining <= 0) {
        std::cout << "Panda Control loop min/avg/max us: " << statistics_runtime_min << "/" << statistics_runtime_sum / statistics_maxcount << "/" << statistics_runtime_max << std::endl;
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
    if (usecs > 300) {std::cout << "Warning: The control loop's service() took longer than "<< usecs  <<" us!" << std::endl;}
    
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


    //Keep running depending on the panda state, 
    do {
        robot_state = robot.readOnce();
        robotmode = robot_state.robot_mode;
        if (robotmode == franka::RobotMode::kIdle) {
            // start real-time control loop
            std::cout << "Starting Control"<< std::endl;
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
            std::cout << "Stopping Control"<< std::endl;
        } else if (robotmode == franka::RobotMode::kGuiding) {
            std::cout << "Guiding Active"<< std::endl;
            robot.read( callbackRead );
            std::cout << "Guiding Stopped"<< std::endl;
        } else if (robotmode == franka::RobotMode::kUserStopped) {
            currentController->service(robot_state, idlePeriod);
            idleLoopRate.sleep();
        } else {
            break; //any other mode -> quit
        }
    } while (not exitrequested);
    std::cout << "Leaving robot in mode: ";
    switch (robotmode) {
        case franka::RobotMode::kOther:
            std::cout << "kOther";
            break;
        case franka::RobotMode::kIdle:
            std::cout << "kIdle";
            break;
        case franka::RobotMode::kMove:
            std::cout << "kMove";
            break;
        case franka::RobotMode::kGuiding:
            std::cout << "kGuiding";
            break;
        case franka::RobotMode::kReflex:
            std::cout << "kReflex";
            std::cout << std::endl << "cartesian collision flags: ";
            for (int i=0;i<6;i++) {std::cout << robot_state.cartesian_collision[i];}
            std::cout << std::endl << "joint collision flags: ";
            for (int i=0;i<7;i++) {std::cout << robot_state.joint_collision[i];}
            break;
        case franka::RobotMode::kUserStopped:
            std::cout << "kUserStopped";
            break;
        case franka::RobotMode::kAutomaticErrorRecovery:
            std::cout << "kAutomaticErrorRecovery";
            break;
    }
    std::cout << std::endl;
    std::cout << "Last Motion Errors: " << robot_state.last_motion_errors << std::endl;
    std::cout << "Current Errors: " << robot_state.current_errors << std::endl;
  return 0;
}






