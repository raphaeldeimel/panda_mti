cmake_minimum_required(VERSION 3.0.2)
project(panda_msgs_mti)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)


add_message_files(DIRECTORY msg
  FILES 
  RobotState.msg
  JointTorqueComparison.msg
  RobotModeMsg.msg
  PDControllerGoal8.msg
  MechanicalStateDistribution8TorquePosVel.msg
)

generate_messages(DEPENDENCIES 
    std_msgs 
    sensor_msgs
)

catkin_package(CATKIN_DEPENDS 
  message_runtime
  std_msgs
)



