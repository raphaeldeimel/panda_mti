cmake_minimum_required(VERSION 2.8.3)
project(controller_panda_mti)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#make sure to use the full available cpu instruction set
SET(CMAKE_CXX_FLAGS_RELEASE  "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

find_package(catkin REQUIRED COMPONENTS
  panda_msgs_mti 
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.3.0 REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    panda_msgs_mti
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

#PD controller 
add_executable(pdcontroller 
  src/common.cpp
  src/pdcontroller.cpp
  src/pdcontroller_main.cpp
  
)
add_dependencies(pdcontroller 
    panda_msgs_mti_generate_messages_cpp
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


add_executable(pdcontroller_gripper 
  src/common.cpp
  src/pdcontroller.cpp
  src/gripperbackground.cpp
  src/pdcontroller_gripper.cpp
  
)
add_dependencies(pdcontroller_gripper 
    panda_msgs_mti_generate_messages_cpp
)
target_link_libraries(pdcontroller_gripper PUBLIC
  Franka::Franka
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(pdcontroller_gripper SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(pdcontroller_gripper PUBLIC
  include
)



#Teaching controller 
add_executable(statepublisher 
  src/common.cpp
  src/gripperbackground.cpp
  src/statepublisher.cpp
)
add_dependencies(statepublisher 
    panda_msgs_mti_generate_messages_cpp
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



