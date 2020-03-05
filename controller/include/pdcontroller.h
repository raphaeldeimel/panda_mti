// Copyright (c) 2018 Raphael Deimel
#pragma once

#include <Eigen/LU> 

#include <controllerinterface.h>
#include <panda_msgs_mti/PDControllerGoal8.h> //ros message types

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath//Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <kdl/chaindynparam.hpp>

class PDController : public ControllerInterface
{

public:
    PDController(franka::Robot& robot, std::string& hostname, ros::NodeHandle& rosnode);
    ~PDController() override;

    bool isTorqueController() const  override {return true;};

    void onStart();

    //Control loop callback
    franka::Torques update(const franka::RobotState& robot_state, const franka::Duration period) override;

    //callback that should be called regularly if update isn't
    void service(const franka::RobotState& robot_state, const franka::Duration period) override;

    double gripper_q;

protected:


    franka::Robot* pRobot;
    franka::Model* pModel;

    franka::RobotState initial_state;

    DOFVector initial_tau_ext;
    DOFVector tau_error_integral;
    // Bias torque sensor
    std::array<double, 7> gravity_array;
    Eigen::Vector3d initial_position;

    const std::array<double, dofs> kMaxTorqueRate{{1000, 1000, 1000, 1000, 1000, 1000, 1000}};
    Eigen::Matrix<double, 6,1> gravity_vector;

    double desired_payload_mass{0.0};
    const double payload_filter_gain{0.001};

    //Params from ROS:
    DOFVector tau_bias;
    DOFVector tau_stiction;

    //Values from ROS:
    DOFVector rosTaud_next;
    DOFVector rosQd_next;
    DOFVector rosDQd_next;
    GainsMatrix rosKflat_next;


    const int64_t rosExpectedUpdatePeriod = 25; //steps in the interpolation interval 0...1, i.e  = hz(control loop) / hz(minimum ros update rate) = 1000/40
    int64_t timeUntilNextUpdate  = 25;  //interpolation variable

    double rosGripperTaud;
    double rosGripperQd;
    double rosGripperDQd;
    double rosGripperKp;
    double rosGripperKv;
    double payload_mass;
    
    DesiredMechanicalStateFlatMeans desiredMeansFlat;
    DesiredMechanicalStateFlatCovariances desiredCovariancesFlat;

    const double kDeltaTauMax = 0.5; //Panda firmware complains if we use 1.0 Nm/ms from the example, use 0.5 instead

    const int publisher_culling_amount  = 10;  //the factor to reduce ros publishing to avoid taxing the realtime-ness
    int state_culling_count = 0;
    ros::Publisher common_state_publisher;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Publisher ee_publisher;
    ros::Subscriber pdcontroller_goal_listener_;
    ros::Subscriber desiredmstate_listener_;

    KDL::ChainDynParam* chainDynParam = nullptr;

    //ROS Message callback
    void callbackPDControllerGoal(const panda_msgs_mti::PDControllerGoal8::ConstPtr& msg);
    void callbackDesiredMechanicalState(const panda_msgs_mti::MechanicalStateDistribution8TorquePosVel::ConstPtr& msg);

    //limits rates of change of position and velocity received from ros
    bool limit_desired_motion();

    //watchdog functionality:
    DOFVector watchdogkv_;          //kv to use when watchdog is active
    bool watchDogTriggered = true; // --> Timeout
    const franka::Duration watchdogPeriod = franka::Duration(500);
    franka::Duration watchdog_timeout;

    //limits:
    DOFVector minjointposition_;
    DOFVector maxjointposition_;
    DOFVector maxjointvelocity_;
    DOFVector maxjointacceleration_;
    DOFVector maxjointaccelerationchange_;
    DOFVector maxjointtorque_;
    DOFVector maxdesiredjointtorque_;
    DOFVector maxjointtorquechange_;

    //soft borders
    DOFVector min_border_;
    DOFVector max_border_;
    DOFVector zero_line_;
    double border_zone_;
    double torque_border_;

    //netft data
    double rdtdata_[6] = {};

    GainsMatrix maxK_;
    GainsMatrix minK_;

    //controller variables / temporary values
    DOFVector tau_cmd_ext_;
    DOFVector qd_last_;
    DOFVector dqd_last_;
    DOFVector kd_;
    GainsMatrix K_;


    franka::RobotState robot_state_;
    DOFVectorMapped q_;
    DOFVectorMapped dq_;
    DOFVectorMapped tau_ext_hat;
    DOFVector ddq_;

    std::array<double, 6*dofs> jacobian_array_;
    JacobianMapped jacobian;

    std::array<double, 6*dofs> jacobian_array_ee_;

    std::array<double, dofs*dofs> mass_matrix_array_;
    MassMatrixMapped joint_mass_matrix;

    std::array<double, dofs> coriolis_vector_;
    DOFVectorMapped tau_coriolis;

    DOFVector qd_;
    DOFVector dqd_;
    Wrench desired_force_torque;
    Wrench desired_mass_wrench;
    DOFVector taud_;  // desired torque
    DOFVector taud_ee; //torques to create end effector forces
    DOFVector taud_last_; // was commanded last
    DOFVector tau_cmd_unlimited_unfiltered_;
    DOFVector tau_cmd_unlimited_;
    DOFVector tau_cmd_limited_;
    
    DOFVector tau_inertia_;
    MassMatrix joint_mass_matrix_emulated_;

    std::array<double, dofs> sent_torques_array_;
    DOFVectorMapped tau_cmd; // is to be commanded

    //panda_msgs_mti::RobotState robotstatemsg_;
    const int publish_every_n_ = 25; //publish at a rate of 40Hz
    int publish_in_count = 0;


    ros::Time time;
    franka::Duration franka_time;

};
