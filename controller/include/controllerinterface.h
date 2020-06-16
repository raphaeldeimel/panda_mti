// Copyright (c) 2018 Raphael Deimel
#pragma once


#include <Eigen/Core>


//libfranka includes
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

//ros includes
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <realtime_tools/realtime_publisher.h>
#include <panda_msgs_mti/RobotState.h>
#include <panda_msgs_mti/MechanicalStateDistribution8TorquePosVel.h> 
#include <sensor_msgs/JointState.h>


const int dofs = 7;
const int dofs_ros = 8;


typedef Eigen::Array<double,dofs,1> DOFVector;
typedef Eigen::Map<Eigen::Array<double, dofs, 1> > DOFVectorMapped;

typedef Eigen::Matrix<double, 6,1> Wrench;

typedef std::array<double, 42> JacobianBuffer;
typedef Eigen::Map<Eigen::Matrix<double, 6,7> > JacobianMapped;

typedef Eigen::Matrix<double, 7,7> MassMatrix;
typedef std::array<double, 49> MassMatrixBuffer;
typedef Eigen::Map<MassMatrix > MassMatrixMapped;

typedef Eigen::Array<double, dofs_ros, 2*dofs_ros> GainsMatrix;

//Mechanical State Distribution Declarations:
const int MechanicalstateIndexTorque = 0;
const int MechanicalstateIndexPosition = 1;
const int MechanicalstateIndexVelocity = 2;
const int MechanicalstateElementsCount = 3;
typedef Eigen::Array<double, MechanicalstateElementsCount*dofs_ros, 1> DesiredMechanicalStateFlatMeans;
typedef Eigen::Array<double, MechanicalstateElementsCount*dofs_ros, MechanicalstateElementsCount*dofs_ros> DesiredMechanicalStateFlatCovariances;
//Eigen doesn't support multiarrays. Provide convenience functions to map tensor indices to index in flattened tensors:
inline int flattenIndex(int m, int dof) {return   m * dofs_ros + dof ;}
inline int flattenIndex(int m, int dof, int m2, int dof2) {return  dofs_ros*MechanicalstateElementsCount*dofs_ros* m +  dofs_ros*MechanicalstateElementsCount* dof +  dofs_ros * m2 + dof2;}

/*
Abstract controller interface that is used in the mainloop:
*/
class ControllerInterface
{

  public:
    virtual ~ControllerInterface() = 0;

    virtual bool isTorqueController() const = 0;

    //called once right beefore control loop starts
    virtual void onStart() = 0;

    //torque control loop callback
    virtual franka::Torques update(const franka::RobotState& robot_state, const franka::Duration period) = 0;

    //callback that should be called regularly if update isn't
    virtual void service(const franka::RobotState& robot_state, const franka::Duration period) = 0;

    std::string hostname;

    std::string myName;

};



/*
Convenience Template for instantiating and running a specific controller class:
*/
int mainloopImpl(franka::Robot&, ControllerInterface* ControllerInterfaceImpl);
template < class ControllerInterfaceImpl > int mainloop()  {
  int retval=1;
  ros::NodeHandle rosnode("");
  //get your params from ros config/launchfiles
  std::string hostname;

  //delete this in the future:
  if (rosnode.param<std::string>("/panda/hostname",hostname, "panda" )) {
    ROS_WARN("Using /panda/hostname is deprecated. Please set panda_address as a local parameter instead!");
  }

  rosnode.param<std::string>("panda_address",hostname, "panda" );

  try {
    // connect to robot
    franka::Robot robot(hostname);

    //instantiate controller object, also handles ros publish/subscribe
    auto cntrl = ControllerInterfaceImpl(robot, hostname, rosnode);
    retval = mainloopImpl(robot, &cntrl);
  } catch (const std::exception& ex) {
        // print exception
        std::cout << "pdcontroller: Exception " << ex.what() << std::endl;
  }

 return retval;
}
