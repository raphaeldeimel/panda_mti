// Copyright (c) 2018 Raphael Deimel
#pragma once

#include <controllerinterface.h>


class ROSPublishingController : public ControllerInterface {

public:
    ROSPublishingController(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode);
    ~ROSPublishingController() override;

    bool isTorqueController() const override {return false;};

    void onStart();

    //Control loop callback
    franka::Torques update(const franka::RobotState& robot_state, const franka::Duration period) override {throw "not a torque controller";};


    //callback that should be called regularly if update isn't
    void service(const franka::RobotState& robot_state, const franka::Duration period) override;


    double gripper_q;
    
private:

    franka::Robot* pRobot;
    franka::Model* pModel;

    franka::RobotState initial_state;
    franka::RobotState current_state;


    const int publisher_culling_amount  = 10;  //the factor to reduce ros publishing to avoid taxing the realtime-ness
    int state_culling_count = 0;
    ros::Publisher common_state_publisher;
    //mti_panda_controller_msgs::RobotState robotstatemsg_;
    const int publish_every_n_ = 25; //publish at a rate of 40Hz
    int publish_in_count = 0;


};





