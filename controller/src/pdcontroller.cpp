// Copyright (c) 2018 Raphael Deimel
#include <array>
#include <cmath>
#include <iostream>
#include <iterator>
#include <kdl_parser/kdl_parser.hpp>
#include <pdcontroller.h>




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


    timeUntilNextUpdate = rosExpectedUpdatePeriod; //reset interpolation interval
    watchdog_timeout = franka_time +  watchdogPeriod; //reset watchdog

    //internalize data:
    for (int i=0;i<dofs;i++) {  rosTaud_next(i) = msg->torque[i];}
    for (int i=0;i<dofs;i++) {  rosQd_next(i) = msg->position[i];}
    for (int i=0;i<dofs;i++) {  rosDQd_next(i) = msg->velocity[i];}
    rosKflat_next.setZero(); 
    for (int d=0;d<dofs_ros;d++) { rosKflat_next(flattenIndex(0,d), flattenIndex(0,d)) = msg->kp[d];}
    for (int d=0;d<dofs_ros;d++) { rosKflat_next(flattenIndex(0,d), flattenIndex(1,d)) = msg->kv[d];}
    
    //set future desired values
    rosKflat_next   = rosKflat_next.min(maxK_).max(minK_);
    rosQd_next   = rosQd_next.min(maxjointposition_).max(minjointposition_);
    rosDQd_next  = rosDQd_next.min(maxjointvelocity_).max(-maxjointvelocity_);
    rosTaud_next = rosTaud_next.min(maxdesiredjointtorque_).max(-maxdesiredjointtorque_);


    //dofs+1 == gripper dof
    rosGripperTaud = msg->torque[dofs];
    rosGripperQd = msg->position[dofs];
    rosGripperDQd = msg->velocity[dofs];
    rosGripperKp = msg->kp[dofs];
    rosGripperKv = msg->kv[dofs];

    
    //ROS_DEBUG_STREAM(rosKflat_next);
}


/* Callback Handler to get desired Joints from promp Motion Generator */
void PDController::callbackDesiredMechanicalState(const panda_msgs_mti::MechanicalStateDistribution8TorquePosVel::ConstPtr& msg)
{

    //sanity check msg
    if  (desiredMeansFlat.size() !=  msg->meansMatrix.size()) {
        ROS_ERROR("Receieved means matrix has not the right size!");
        return;
    }

    //sanity check msg
    if  (desiredCovariancesFlat.size() !=  msg->covarianceTensor.size()) {
        ROS_ERROR("Receieved covariance tensor has not the right size!");
        return;
    }

    //internalize data:
    {
        double* pData = desiredMeansFlat.data();
        for (int i=0;i<desiredMeansFlat.size();i++) {pData[i] = msg->meansMatrix[i];}
        pData = desiredCovariancesFlat.data();
        for (int i=0;i<desiredCovariancesFlat.size();i++) {pData[i] = msg->covarianceTensor[i];}
    }

    timeUntilNextUpdate = rosExpectedUpdatePeriod; //reset interpolation interval
    watchdog_timeout = franka_time +  watchdogPeriod; //reset watchdog


    for (int i=0; i< dofs; i++){ rosTaud_next(i) = desiredMeansFlat(flattenIndex(MechanicalstateIndexTorque,  i));}
    for (int i=0; i< dofs; i++){ rosQd_next(i)   = desiredMeansFlat(flattenIndex(MechanicalstateIndexPosition,i));}
    for (int i=0; i< dofs; i++){ rosDQd_next(i)  = desiredMeansFlat(flattenIndex(MechanicalstateIndexVelocity,i));}
    
    //Compute the (1*dofs x 2*dofs) gains tensor:
    Eigen::Matrix<double, 2*dofs_ros, 2*dofs_ros>  CovMotionMotion = desiredCovariancesFlat.block(1*dofs_ros,1*dofs_ros,2*dofs_ros,2*dofs_ros);
    Eigen::Matrix<double, 1*dofs_ros, 2*dofs_ros>  CovTorqueMotion = desiredCovariancesFlat.block(0*dofs_ros,1*dofs_ros,1*dofs_ros,2*dofs_ros);
    rosKflat_next = -CovTorqueMotion * CovMotionMotion.inverse();

    rosKflat_next   = rosKflat_next.min(maxK_).max(minK_);
    rosQd_next   = rosQd_next.min(maxjointposition_).max(minjointposition_);
    rosDQd_next  = rosDQd_next.min(maxjointvelocity_).max(-maxjointvelocity_);
    rosTaud_next = rosTaud_next.min(maxdesiredjointtorque_).max(-maxdesiredjointtorque_);
    
    
    ROS_DEBUG_STREAM(rosKflat_next);

    //dofs+1 == gripper dof
    rosGripperTaud = desiredMeansFlat(flattenIndex(MechanicalstateIndexTorque,dofs));
    rosGripperQd =  desiredMeansFlat(flattenIndex(MechanicalstateIndexPosition, dofs));
    rosGripperDQd =  desiredMeansFlat(flattenIndex(MechanicalstateIndexVelocity, dofs));
//    rosGripperKp = msg->[dofs];
//    rosGripperKv = msg->kv[dofs];

  
//    ROS_DEBUG_STREAM("0:" << desiredCovariancesFlat(flattenIndex(1,3),flattenIndex(2,3)) << std::endl);

}


PDController::PDController(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode) :
     //sets up the mapped eigen vectors to access robot_state and derived properties
     robot_state_(),
     q_(robot_state_.q.data()),
     dq_(robot_state_.dq.data()),
     tau_ext_hat(robot_state_.dq.data()),
     ddq_(),
     tau_cmd_ext_(),
     joint_mass_matrix_emulated_(),
     jacobian_array_(), jacobian(jacobian_array_.data()),
     mass_matrix_array_(), joint_mass_matrix(mass_matrix_array_.data()), 
     q_kdl(7), joint_mass_matrix_kdl(7),
     coriolis_vector_(), tau_coriolis(coriolis_vector_.data()),
     sent_torques_array_(), tau_cmd(sent_torques_array_.data()),
     gripper_q(0.0),
     rosTaud_next(),rosQd_next(),rosDQd_next(),
     desiredMeansFlat(),desiredCovariancesFlat(),
     kd_(),K_()

{
    pRobot = &robot;
    pModel = new franka::Model(robot.loadModel());
    initial_state = robot.readOnce();
    robot_state_ =initial_state;

    myName = rosnode.getNamespace();

    common_state_publisher = rosnode.advertise<panda_msgs_mti::RobotState8>("currentstate", 10);

    ee_publisher = rosnode.advertise<panda_msgs_mti::RobotEEState>("currentEEstate", 10);

    pdcontroller_goal_listener_ = rosnode.subscribe("pdcontroller_goal", 5, &PDController::callbackPDControllerGoal, this);
    desiredmstate_listener_ = rosnode.subscribe("desiredmechanicalstate", 5, &PDController::callbackDesiredMechanicalState, this);

    std::vector <double> v;    
    rosnode.getParam("torque_bias", v);
    if (v.size() != dofs) {
      ROS_WARN_STREAM(myName << ": torque_bias length is " << v.size() << " instead of " << dofs << ", ignoring it.");
    } else {
      for (int i=0; i<dofs; i++) {tau_bias[i] = v[i];}
    }
    
    rosnode.getParam("torque_stiction", v);
    if (v.size() != dofs) {
      ROS_WARN_STREAM(myName << ": torque_stiction length is " << v.size() << " instead of " << dofs << ", ignoring it.");
    } else {
      for (int i=0; i<dofs; i++) {tau_stiction[i] = v[i];}
    }

    //kd_ << 0.1,0.1,0.1,0.1,0.1,0.1,0.5;
    rosnode.getParam("damping", v);
    if (v.size() != dofs) {
      ROS_WARN_STREAM(myName << ": damping length is " << v.size() << " instead of " << dofs << ", ignoring it.");
    } else {
      for (int i=0; i<dofs; i++) {kd_[i] = v[i];}
    }

    gravity_vector << 0., 0., -9.81, 0., 0., 0.;
    KDL::Vector gravity_vector_kdl(0.0,0.0,-9.81);

    std::string robot_desc_string;
    rosnode.param("robot_description", robot_desc_string, std::string());

    bool success;
    success = kdl_parser::treeFromString(robot_desc_string, kdl_tree);
    if (!success) {ROS_ERROR_STREAM(myName <<": Failed to construct kdl tree from parameter robot_description");
    } else {
        success = kdl_tree.getChain("panda_link0",
                                "panda_link8",
                                kdl_chain);
    }
    if (!success) {
           ROS_ERROR_STREAM(myName <<": Failed to get chain panda_link1 to panda_link8 from KDL tree");
    } else {
            chainDynParam = new KDL::ChainDynParam(kdl_chain, gravity_vector_kdl);
            urdf_has_inertias = true;
    }

    franka_time = franka::Duration(0);
    watchdog_timeout = franka_time;

    // Bias torque sensor
    gravity_array = pModel->gravity(initial_state);

    desired_force_torque.setZero();

    payload_mass = 0.0;
    
    joint_mass_matrix_emulated_ = Eigen::MatrixXd::Identity(7,7);
    //joint_masses_synthetic.diagonal() << 0,0,0,0,0,0,0;
    joint_mass_matrix_emulated_.diagonal() << 0.05,0.05,0.05,0.05,0.05,0.05,0.05;

    //watchdogkv_ << 9,9,9,9,1,1,1;
    watchdogkv_ << 0,0,0,0,0,0,0;
    //additional virtual damping:
    kd_ << 0.1,0.1,0.1,0.1,0.1,0.1,0.5;
    //kd_ << 5,5,5,5,2,2,2,2;
    dq_ << 0,0,0,0,0,0,0;
    ddq_ << 0,0,0,0,0,0,0;

    //values from franka's documentation:
    maxjointposition_           <<    2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973;
    minjointposition_           <<   -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973; //soft joint limits
    maxjointvelocity_           <<    2.1750,  2.1750,  2.1750,  2.1750,  2.6100,  2.6100,  2.6100; // soft velocity limits
    maxjointacceleration_       <<   15.0,     7.5,    10.0,    12.5,    15.0,    20.0,    20.0; // acceleration limits
    maxjointaccelerationchange_ << 7500,    3750,    5000,    6250,    7500,   10000,   10000; // acceleration change limits
    maxjointtorque_             <<   87,      87,      87,      87,      12,      12,      12; // torque limits
    //additional limits:

    for (int d=0; d<dofs_ros; d++)     {
        maxK_.row(d) <<  100.,    100.,    100.,    100.,     50.,     50.,     50., 50,
                  100.,    100.,    100.,    100.,     30.,     30.,     30., 10;
        minK_.row(d) <<    0.,      0.,      0.,      0.,      0.,      0.,      0., 0,
                    0.,      0.,      0.,      0.,      0.,      0.,      0., 0;
    }
    maxdesiredjointtorque_      <<    6.,      6.,      6.,      6.,      2.,      2.,      2.;

    //elastic joint limits - counter torque
    zero_line_ << 0,0,0,0,0,0,0;
    border_zone_ = 0.85;
    torque_border_ = 10;
    max_border_ = maxjointposition_*border_zone_;
    min_border_ = minjointposition_*border_zone_;


    ROS_INFO_STREAM(myName << ": initial robot joint state is [" << initial_state.q[0] << ", " <<
    initial_state.q[1] << ", " <<
    initial_state.q[2] << ", " <<
    initial_state.q[3] << ", " <<
    initial_state.q[4] << ", " <<
    initial_state.q[5] << ", " <<
    initial_state.q[6] << "]");
    
    qd_ = q_;
    dqd_ = dq_;
    taud_.setZero();
    qd_last_ = qd_;
    dqd_last_ = dqd_;
    
    desiredMeansFlat.setZero();
    desiredCovariancesFlat.setZero();
    K_.setZero();
    rosKflat_next.setZero();
    rosTaud_next << 0,0,0,0,0,0,0;
    rosQd_next = q_;
    rosDQd_next << 0,0,0,0,0,0,0;



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
        
        qd_ = q_;
        dqd_ = dq_;
}

void PDController::service(const franka::RobotState& robot_state, const franka::Duration period) {

    DOFVector dq_last = dq_;
    qd_last_ = qd_;
    dqd_last_ = dqd_;

    //gets called always, even if controller is not active
    robot_state_ = robot_state;

    //copmute a filtered acceleration estimate:
    double dtInv_sec = 1000.0;
    DOFVector ddq_next = dtInv_sec * (dq_ - dq_last);
    DOFVector ddq_delta = ddq_next.max(-maxjointacceleration_).min(maxjointacceleration_)  - ddq_;
    DOFVector filter_gains_asymmetry  = (0.75-0.25* ddq_.sign() * ddq_delta.sign());  //tend to decelerate / underestimate acceleration.
    ddq_ += 0.01 * filter_gains_asymmetry * ddq_delta;


    
    if (urdf_has_inertias) {
        q_kdl.data = q_; 
        chainDynParam->JntToMass( q_kdl, joint_mass_matrix_kdl);
        joint_mass_matrix = joint_mass_matrix_kdl.data;
        coriolis_vector_ = pModel->coriolis(robot_state); //TODO: not taken from URDF model yet
        jacobian_array_ = pModel->zeroJacobian(franka::Frame::kEndEffector, robot_state_);
        jacobian_array_ee_ = pModel->bodyJacobian(franka::Frame::kEndEffector, robot_state_);
    } else {
        //compute things from robot state:
        jacobian_array_ = pModel->zeroJacobian(franka::Frame::kEndEffector, robot_state_);
        jacobian_array_ee_ = pModel->bodyJacobian(franka::Frame::kEndEffector, robot_state_);
        mass_matrix_array_ = pModel->mass(robot_state);
        coriolis_vector_ = pModel->coriolis(robot_state);
    }


    if (state_culling_count-- <= 0) {
        state_culling_count = publisher_culling_amount;
        ros::Time now  = ros::Time::now();
        panda_msgs_mti::RobotState8 statemsg = panda_msgs_mti::RobotState8();
        statemsg.stamp = now;
        statemsg.mode = (int)robot_state.robot_mode;
        for (int i=0;i<dofs;i++) { statemsg.q[i]   = q_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.dq[i]  = dq_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.ddq[i]  = ddq_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.tau[i] = tau_cmd_ext_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.tau_ext[i] = tau_ext_hat.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.qd[i]  = qd_.coeff(i);}
        for (int i=0;i<dofs;i++) { statemsg.dqd[i] = dqd_.coeff(i);}

        for (int i=0;i<6;i++) {statemsg.ee_ft[i] = rdtdata_[i];}

        statemsg.q[dofs]   = gripper_q;
        statemsg.dq[dofs]  = 0.0;
        statemsg.tau[dofs] = 0.0;
        statemsg.qd[dofs]  = 0.0;
        statemsg.dqd[dofs] = 0.0;

        common_state_publisher.publish(statemsg);

        //create and publish an end effector frame:
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = "panda_link0";
        transformStamped.child_frame_id = "panda_EE";
        transformStamped.transform.translation.x = robot_state.O_T_EE[12];
        transformStamped.transform.translation.y = robot_state.O_T_EE[13];
        transformStamped.transform.translation.z = robot_state.O_T_EE[14];
        tf2::Matrix3x3 ee_rotationmatrix = tf2::Matrix3x3(
            robot_state.O_T_EE[0],robot_state.O_T_EE[4],robot_state.O_T_EE[8],
            robot_state.O_T_EE[1],robot_state.O_T_EE[5],robot_state.O_T_EE[9],
            robot_state.O_T_EE[2],robot_state.O_T_EE[6],robot_state.O_T_EE[10]
        );
        tf2::Quaternion ee_quaternion  = tf2::Quaternion();
        ee_rotationmatrix.getRotation(ee_quaternion);
        transformStamped.transform.rotation.x = ee_quaternion.x();
        transformStamped.transform.rotation.y = ee_quaternion.y();
        transformStamped.transform.rotation.z = ee_quaternion.z();
        transformStamped.transform.rotation.w = ee_quaternion.w();
     
        tf_broadcaster.sendTransform(transformStamped);
        
        //publish end effector jacobian in base frame:
        panda_msgs_mti::RobotEEState ee_state;
        ee_state.stamp = now;
        for (int i=0; i < 42; i++) {
            ee_state.jacobian_base[i] = jacobian_array_[i];
        }
        for (int i=0; i < 42; i++) {
            ee_state.jacobian_ee[i] = jacobian_array_ee_[i];
        }
        for (int i=0; i < 42; i++) {
            ee_state.jacobian_base[i] = jacobian_array_[i];
        }
        for (int i=0; i < 16; i++) {
            ee_state.htransform[i] = robot_state.O_T_EE[i];
        }
        ee_publisher.publish(ee_state);
        
        
    }
    //give the ros listener time to update its values:
    ros::spinOnce();

    static int k = 0;
    if (robot_state.control_command_success_rate > 0.05 && robot_state.control_command_success_rate < 0.90) {
        if (k <= 0) {
            ROS_WARN_STREAM(myName << ": " << (1.0 - robot_state.control_command_success_rate)*100 << " out of 100 loops where too late." << std::endl);
            k = 500;
        }
        k--;
    }


    //interpolate between previous and next desired ros values:
    int64_t dt_msec = period.toMSec();
    double i  = (double) dt_msec / (timeUntilNextUpdate+0.00001); //compute the relative step size
    i = std::min(i, 0.1); //limit how fast we are at most in following
    K_    =   K_  + i* (rosKflat_next -  K_);
    qd_   =   qd_ + i* (  rosQd_next -   qd_);
    dqd_  =  dqd_ + i* ( rosDQd_next -  dqd_);
    taud_ = taud_ + i* (rosTaud_next - taud_);
    timeUntilNextUpdate = std::max(int64_t(0), timeUntilNextUpdate - dt_msec); //adjust the remaining interpolation time
    
    
    /*if no recent goal was posted, do some damage control*/
    if(watchdog_timeout < franka_time){ // ToDo goal_time + 500ms too long?
        if(watchDogTriggered == false) ROS_ERROR_STREAM(myName << ": Nobody is sending me updates!! Stopping for safety.");
        watchDogTriggered = true;
        /*keep the position-goal(?), but slow down*/
        K_.setZero();
        for (int i=0; i<dofs; i++) {K_(flattenIndex(0,i), flattenIndex(1,i)) = watchdogkv_(i);}
        taud_.setZero();
        dqd_.setZero();
        //qd_ = q_; // hold position
    }
    
}



franka::Torques PDController::update(const franka::RobotState& robot_state, const franka::Duration period) {

     franka_time += period;

    service(robot_state, period); //get current values, communicate with ros
    
    //shotcut to submatrices, also throws away the last dof (gripper)
    auto Kp_ =  K_.block( flattenIndex(0,0), flattenIndex(0,0), dofs, dofs).matrix();
    auto Kv_ =  K_.block( flattenIndex(0,0), flattenIndex(1,0), dofs, dofs).matrix();

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
    DOFVector tau_error =  Kp_ * (qd_ - q_).matrix() +  Kv_ * (dqd_ - dq_).matrix() - (kd_ * dq_).matrix();

    //inertia compensation:
    tau_inertia_ = (joint_mass_matrix-joint_mass_matrix_emulated_) * ddq_.matrix();
    
    DOFVector tau_border = torque_border_*(
                -1*q_.sign()*(zero_line_.max((q_ - max_border_)))
                - (-1*q_).sign()*(zero_line_.min((q_ - min_border_))));

    //torques that should affect tau_ext:
    tau_cmd_ext_ <<
              tau_error       //add pd-controller torques
            + taud_           //add desired torque
            + taud_ee         //add desired ee wrench force
    ;

    // add torques to compensate robot-internal effets:
    tau_cmd_unlimited_unfiltered_ <<
            tau_cmd_ext_
            + tau_coriolis      //gravity torque is addedd by panda firmware
            + tau_inertia_     //inertial forces
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
