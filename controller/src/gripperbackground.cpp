// Copyright (c) 2018 Raphael Deimel
//
//#include <array>
//#include <cmath>
#include <iostream>
#include <thread>


#include <gripperbackground.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper_state.h>
#include <franka/gripper.h>

//#include <boost/atomic/atomic.hpp>
#include <atomic>

namespace grippernonblocking {

typedef struct {
    float q;
    float tau;
} desiredValues;

std::atomic<double> currentGripperPosition(0.0);

std::atomic<desiredValues> commandedExternally({q_max, 0.0});


const struct timespec gripperPollingPeriod = {0, 50000000}; //50ms = 20Hz

static bool gripperCommunicatorExitRequested = false;



//function to run in separate thread
void gripperCommunicator(franka::Gripper* gripper)
{
    const float gripper_polling_interval=0.1;
    static float gripper_polling_interval_counter=0.0;
    static bool isGripperOpen = false;
    while (!gripperCommunicatorExitRequested) {
        if (gripper != NULL) {
            desiredValues desired = commandedExternally.load(std::memory_order_seq_cst);
            if (desired.q >= threshold_open && !isGripperOpen) {
                currentGripperPosition.store(q_max);
                gripper->move(q_max, dq_max);
                isGripperOpen  =true;
            }
            if (desired.q <= threshold_close && isGripperOpen) {
                currentGripperPosition.store(0.0);
                gripper->grasp(0.0, dq_max, desired.tau , 0.2, 0.2); 
                isGripperOpen=false;
            }
            
            franka::GripperState newState = gripper->readOnce();
            currentGripperPosition.store(newState.width);

        }
        nanosleep(&gripperPollingPeriod, NULL);
    }
}

float getGripperPosition() {
    return currentGripperPosition.load(std::memory_order_seq_cst);
}

void setGripperDesired(float q, float tau) {
    desiredValues desired({q,tau});
    commandedExternally.store(desired);
}


void startGripperCommunicationInBackground(std::string& hostname) 
{
    //connect to gripper:
    franka::Gripper* gripper = new franka::Gripper(hostname);
    //start the thread to handle gripper communication outside of realtime thread
    std::thread* GripperThread = new std::thread(gripperCommunicator, gripper); 
}
 
 
void stopGripperCommunicationInBackground() {
    gripperCommunicatorExitRequested  = true;
}


}
