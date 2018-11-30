// Copyright (c) 2018 Raphael Deimel
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>



#include <pdcontroller.h>
#include "netft.h"



std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& desired_values,
                                const std::array<double, 7>& last_desired_values) {
  std::array<double, 7> limited_values{};
  for (size_t i = 0; i < 7; i++) {
    double desired_difference = (desired_values[i] - last_desired_values[i]) / 1e-3;
    limited_values[i] =
        last_desired_values[i] +
        std::max(std::min(desired_difference, max_derivatives[i]), -max_derivatives[i]) * 1e-3;
  }
  return limited_values;
}



/* Callback Handler to get desired Joints from promp Motion Generator */
void PDController::callbackPDControllerGoal(const panda_msgs_mti::PDControllerGoal8::ConstPtr& msg)
{

    //internalize data:
    for (int i=0;i<dofs;i++) {  rosTaud_next(i) = msg->torque[i];}
    for (int i=0;i<dofs;i++) {  rosQd_next(i) = msg->position[i];}
    for (int i=0;i<dofs;i++) {  rosDQd_next(i) = msg->velocity[i];}
    for (int i=0;i<dofs;i++) {  rosKp_next(i) = msg->kp[i];}
    for (int i=0;i<dofs;i++) {  rosKv_next(i) = msg->kv[i];}

    //set future desired values
    rosTaud_next = rosTaud_next.min(maxkp_).max(minkp_);
    rosKp_next   = rosKp_next.min(maxkp_).max(minkp_);
    rosKv_next   = rosKv_next.min(maxkv_).max(minkv_);
    rosQd_next   = rosQd_next.min(maxjointposition_).max(minjointposition_);
    rosDQd_next  = rosDQd_next.min(maxjointvelocity_).max(-maxjointvelocity_);
    rosTaud_next = rosTaud_next.min(maxdesiredjointtorque_).max(-maxdesiredjointtorque_);


    //dofs+1 == gripper dof
    rosGripperTaud = msg->torque[dofs];
    rosGripperQd = msg->position[dofs];
    rosGripperDQd = msg->velocity[dofs];
    rosGripperKp = msg->kp[dofs];
    rosGripperKv = msg->kv[dofs];

    timeUntilNextUpdate = rosExpectedUpdatePeriod; //reset interpolation interval
    watchdog_timeout = franka_time +  watchdogPeriod; //reset watchdog
}


PDController::PDController(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode) :
     //sets up the mapped eigen vectors to access robot_state and derived properties
     robot_state_(),
     q_(robot_state_.q.data()),
     dq_(robot_state_.dq.data()),
     jacobian_array_(), jacobian(jacobian_array_.data()),
     mass_matrix_array_(), joint_mass_matrix(mass_matrix_array_.data()),
     coriolis_vector_(), tau_coriolis(coriolis_vector_.data()),
     sent_torques_array_(), tau_cmd(sent_torques_array_.data()),
     gripper_q(0.0)
{
    pRobot = &robot;
    pModel = new franka::Model(robot.loadModel());
    initial_state = robot.readOnce();

    common_state_publisher = rosnode.advertise<panda_msgs_mti::RobotState8>("/panda/currentstate", 10);

    pdcontroller_goal_listener_ = rosnode.subscribe("/panda/pdcontroller_goal", 5, &PDController::callbackPDControllerGoal, this);

    std::vector <double> v;
    rosnode.getParam("panda/torque_bias", v);
    if (v.size() != dofs) {
      ROS_WARN("panda/torque_bias length is wrong, ignoring it.");
    } else {
      for (int i=0; i<dofs; i++) {tau_bias[i] = v[i];}
    }

    rosnode.getParam("panda/torque_stiction", v);
    if (v.size() != dofs) {
      ROS_WARN("panda/torque_stiction length is wrong, ignoring it.");
    } else {
      for (int i=0; i<dofs; i++) {tau_stiction[i] = v[i];}
    }

    gravity_vector << 0.,0.,-9.81,0.,0.,0.;

    franka_time = franka::Duration(0);
    watchdog_timeout = franka_time;

    // Bias torque sensor
    gravity_array = pModel->gravity(initial_state);

    desired_force_torque.setZero();

    payload_mass = 0.0;

    //watchdogkv_ << 9,9,9,9,1,1,1;
    watchdogkv_ << 0,0,0,0,0,0,0;
    //additional virtual damping:
    //kd_ << 5,5,5,5,2,2,2,2;


    //values from franka's documentation:
    maxjointposition_           <<    2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973;
    minjointposition_           <<   -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973; //soft joint limits
    maxjointvelocity_           <<    2.1750,  2.1750,  2.1750,  2.1750,  2.6100,  2.6100,  2.6100; // soft velocity limits
    maxjointacceleration_       <<   15.0,     7.5,    10.0,    12.5,    15.0,    20.0,    20.0; // acceleration limits
    maxjointaccelerationchange_ << 7500,    3750,    5000,    6250,    7500,   10000,   10000; // acceleration change limits
    maxjointtorque_             <<   87,      87,      87,      87,      12,      12,      12; // torque limits
    //additional limits:
    maxkp_                      <<  100.,    100.,    100.,    100.,     50.,     50.,     50.;
    maxkv_                      <<  100.,    100.,    100.,    100.,     30.,     30.,     30.;
    maxdesiredjointtorque_      <<    6.,      6.,      6.,      6.,      2.,      2.,      2.;
    minkp_ << 0,0,0,0,0,0,0;
    minkv_ << 0,0,0,0,0,0,0;

    //soft borders
    zero_line_ << 0,0,0,0,0,0,0;
    border_zone_ = 0.85;
    torque_border_ = 10;
    max_border_ = maxjointposition_*border_zone_;
    min_border_ = minjointposition_*border_zone_;


    ROS_INFO_STREAM("initial robot joint state: {" << initial_state.q[0] << ", " <<
    initial_state.q[1] << ", " <<
    initial_state.q[2] << ", " <<
    initial_state.q[3] << ", " <<
    initial_state.q[4] << ", " <<
    initial_state.q[5] << ", " <<
    initial_state.q[6] << "}");

}

PDController::~PDController()
{
 delete pModel;
}


void PDController::onStart() {
        const std::array< double, 7 > maxcollisiontorque_ = {87,87,87,87,12,12,12}; //contact torque limits
        const std::array< double, 6 > maxcollisionwrench_ = {70,70,70,30,30,30};   //contact wrench limits

        const std::array< double, 7 > contactthresholdtorque_ = {87,87,87,87,12,12,12}; //contact detection threshold, torque
        const std::array< double, 6 > contactthresholdwrench_ = {70,70,70,30,30,30};  //contact detection threshold, wrench

        pRobot->setCollisionBehavior(
        contactthresholdtorque_,
        maxcollisiontorque_,
        contactthresholdwrench_,
        maxcollisionwrench_ );
}

void PDController::service(const franka::RobotState& robot_state, const franka::Duration period) {
    //gets called always, even if controller is not active
    robot_state_ = robot_state;


    //compute things from robot state:
    jacobian_array_ = pModel->zeroJacobian(franka::Frame::kEndEffector, robot_state);
    mass_matrix_array_ = pModel->mass(robot_state);
    coriolis_vector_ = pModel->coriolis(robot_state);

    if (state_culling_count-- <= 0) {
        state_culling_count = publisher_culling_amount;
        panda_msgs_mti::RobotState8 statemsg = panda_msgs_mti::RobotState8();
        statemsg.stamp = ros::Time::now();
        statemsg.mode = (int)robot_state.robot_mode;
        for (int i=0;i<dofs;i++) { statemsg.q[i]   = q_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.dq[i]  = dq_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.tau[i] = tau_cmd.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.qd[i]  = dq_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.dqd[i] = dqd_.coeff(i);}

        statemsg.q[dofs]   = gripper_q;
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



franka::Torques PDController::update(const franka::RobotState& robot_state, const franka::Duration period) {



     franka_time += period;
     qd_last_ = qd_;
     dqd_last_ = dqd_;

    service(robot_state, period); //get current values, communicate with ros



    //interpolate between previous and next desired ros values:
    int64_t dt_msec = period.toMSec();
    double i  = (double) std::min(timeUntilNextUpdate, dt_msec) / (timeUntilNextUpdate+0.00001); //compute the relative step size
    kp_   =   kp_ + i* (  rosKp_next -   kp_);
    kv_   =   kv_ + i* (  rosKv_next -   kv_);
    qd_   =   qd_ + i* (  rosQd_next -   qd_);
    dqd_  =  dqd_ + i* ( rosDQd_next -  dqd_);
    taud_ = taud_ + i* (rosTaud_next - taud_);
    timeUntilNextUpdate = std::max(int64_t(0), timeUntilNextUpdate - dt_msec); //adjust the remaining interpolation time

    /*if no recent goal was posted, do some damage control*/
    if(watchdog_timeout < franka_time){ // ToDo goal_time + 500ms too long?
        if(watchDogTriggered == false) ROS_ERROR("pdcontrollernode: Nobody is sending me updates!! Stopping for safety.");
        watchDogTriggered = true;
        /*keep the position-goal(?), but slow down*/
        kp_.setZero();
        kv_ = watchdogkv_;
        taud_.setZero();
        dqd_.setZero();
        qd_ = qd_last_; // hold position
    }


    /*=============================== sanitizes commanded positions and velocities ==========================*/
    //limit_desired_motion();

    /*=============================== get current state =================================================*/

    //wrap raw arrays into Eigen objects:
    Eigen::Matrix<double, 6,7 > jacobian(pModel->zeroJacobian(franka::Frame::kEndEffector, robot_state).data());
    //DOFVectorMapped tau_commanded(robot_state.tau_J.data()); //torque target acutally applied by the panda firmware

    //compute the wrench to compensate a payload mass (only point mass in EE frame currently)
    // Smoothly update the mass to reach the desired target value
    desired_payload_mass = payload_filter_gain * payload_mass + (1 - payload_filter_gain) * desired_payload_mass;
    desired_mass_wrench = gravity_vector * desired_payload_mass;

    /*=============================== Torque-Controller =================================================*/

    taud_ee << (jacobian.transpose() * (desired_force_torque+desired_mass_wrench).matrix()).array();
    //The control law:
    DOFVector tau_error =  kp_ * (qd_ - q_) +  kv_ * (dqd_ - dq_) - kd_ * dq_;

    DOFVector tau_border = torque_border_*(
                -1*q_.sign()*(zero_line_.max((q_ - max_border_)))
                - (-1*q_).sign()*(zero_line_.min((q_ - min_border_))));

    tau_cmd_unlimited_unfiltered_ <<
            tau_coriolis      //gravity torque is addedd by panda firmware
            - damping * dq_   //add damping
            + tau_error       //add pd-controller torques
            + taud_           //add desired torque
            + taud_ee         //add desired ee wrench force
            + tau_bias     //add friction compensation
            + tau_stiction  * (dq_.sign() + dqd_.sign())
            + tau_border      //add spring effect in border region
    ;
    tau_cmd_unlimited_ << 0.75*tau_cmd_unlimited_ + 0.25*tau_cmd_unlimited_unfiltered_; //low pass torques to avoid ringing

    /*=============================== sanitizes torque and torque rate ==========================*/
    tau_cmd_limited_ << tau_cmd_unlimited_.min(maxjointtorque_).max(-maxjointtorque_);
    // limit torque rate:
    tau_cmd  << tau_cmd_limited_;
    tau_cmd << tau_cmd + (tau_cmd_limited_ - tau_cmd).min(kDeltaTauMax).max(-kDeltaTauMax);




    return franka::Torques(sent_torques_array_);  //return the buffer underlying tau_cmd
}

bool PDController::limit_desired_motion()
{

    DOFVector delta_qd;
    DOFVector delta_dqd;

    //compute the desired change
    delta_qd = qd_ - qd_last_;
    delta_dqd = dqd_ - dqd_last_;

    /*budget is a factor to reduce the desired change during the next timestep, in order to avoid violating limits
      budget=1.0 means no reduction, budget=0.0 means we are at a limit and cannot do the change*/
    const double k = 0.25;


    DOFVector deltamax_dq1;
    DOFVector deltamax_dq2;
    DOFVector veldirection;
    DOFVector budgets_velocity;
    DOFVector deltamax_dq;
    deltamax_dq1 = ( k * (maxjointvelocity_ - dqd_last_)).max(0.0);
    deltamax_dq2 = (-k * (-maxjointvelocity_ - dqd_last_)).max(0.0);
    veldirection = delta_dqd.sign().max(0.0);
    deltamax_dq = deltamax_dq1 * veldirection + deltamax_dq2 * (1.0-veldirection);
    budgets_velocity = deltamax_dq / delta_dqd.abs().max(0.000001);


    DOFVector deltamax_q1;
    DOFVector deltamax_q2;
    DOFVector budgets_position;
    DOFVector posdirection;
    DOFVector deltamax_q;
    deltamax_q1 =  ( k * (maxjointposition_ - qd_last_)).max(0.0);
    deltamax_q2 =  (-k * (minjointposition_ - qd_last_)).max(0.0);
    posdirection = delta_qd.sign().max(0.0);
    deltamax_q = deltamax_dq1 * posdirection + deltamax_dq2 * (1.0 - posdirection);
    budgets_position = deltamax_q / delta_qd.abs().max(0.000001);


    double budget_pos = budgets_position.minCoeff();
    double budget_vel = budgets_velocity.minCoeff();
    //if (budget_pos < 0.99999) {ROS_INFO("Slowing motion to [%.4f]x to honor velocity limits",budget_pos);}
    //if (budget_vel < 0.99999) {ROS_INFO("Slowing motion to [%.4f]x to honor acceleration limits",budget_vel);}

    double budget = std::min(budget_pos, budget_vel);
    qd_ = qd_last_  + delta_qd * budget;
    dqd_ = dqd_last_  + delta_dqd * budget;

    return true;

}
