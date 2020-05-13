cmake_minimum_required(VERSION 3.0.2)
project(controller_panda_mti)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE 1)
#make sure to use the full available cpu instruction set
#SET(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native")

#is somehow missing although its part of libstd:
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -latomic" )

find_package(catkin REQUIRED COMPONENTS
  panda_msgs_mti 
  roscpp
  rospy
  tf2_ros
  tf2
  kdl_parser
)

find_package(orocos_kdl)
find_package(Eigen3)
find_package(Franka 0.3.0)

#SET(Franka_FOUND false)

catkin_package(
  CATKIN_DEPENDS
    panda_msgs_mti
    roscpp
    tf2
    tf2_ros
)




IF( ${Franka_FOUND} AND ${Eigen3_FOUND} AND ${orocos_kdl_FOUND})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

#PD controller 
add_executable(pdcontroller 
  src/common.cpp
  src/pdcontroller.cpp
  src/pdcontroller_main.cpp
  
)
add_dependencies(pdcontroller 
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pdcontroller PUBLIC
  Franka::Franka
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(pdcontroller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(pdcontroller PUBLIC
  include
)

# pdcontroller, netft+gripper configuable at runtime
add_executable(pdcontroller_gripper_netft 
  src/common.cpp
  src/pdcontroller.cpp
  src/netft_rdt_driver.cpp
  src/gripperbackground.cpp
  src/pdcontroller_gripper_netft.cpp
)

add_dependencies(pdcontroller_gripper_netft 
    panda_msgs_mti_generate_messages_cpp
)

target_link_libraries(pdcontroller_gripper_netft PUBLIC
  Franka::Franka
  ${catkin_LIBRARIES}
)
target_include_directories(pdcontroller_gripper_netft SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(pdcontroller_gripper_netft PUBLIC
  include
)

#"Controller" that only publishes the state:
add_executable(statepublisher 
  src/common.cpp
  src/gripperbackground.cpp
  src/statepublisher.cpp
)
add_dependencies(statepublisher 
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(statepublisher PUBLIC
  Franka::Franka
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(statepublisher SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(statepublisher PUBLIC
  include
)


## Installation
install(TARGETS pdcontroller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS statepublisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

ELSE()
    MESSAGE(WARNING "Skipping Controller compilation due to missing dependencies, you may still use the emulated controller node. (libfranka:${Franka_FOUND}, libeigen3:${Eigen3_FOUND}, orocos_kdl:${orocos_kdl} )")
ENDIF()
