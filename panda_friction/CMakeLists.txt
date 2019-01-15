cmake_minimum_required(VERSION 2.8.3)
project(panda_friction)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# catkin_install_python(
#  PROGRAMS src/panda_friction.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

