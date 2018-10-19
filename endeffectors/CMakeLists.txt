cmake_minimum_required(VERSION 2.8.3)
project(mti_panda_gripper)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  panda_msgs_mti 
  pluginlib
  message_generation
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.3.0 REQUIRED)




#catkin_python_setup() #not needed yet


catkin_package(
  LIBRARIES grippercontroller
  LIBRARIES softhandmapper    
  CATKIN_DEPENDS
    std_msgs
    panda_msgs_mti
    pluginlib
    roscpp
)



include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})





add_library(grippercontroller
   src/main.cpp
)

add_library(softhandmapper
  src/softhand_controller_mapper.cpp  
)

add_dependencies(grippercontroller 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)
add_dependencies(softhandmapper 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)





target_link_libraries(grippercontroller PUBLIC
  Franka::Franka
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(softhandmapper PUBLIC
  Franka::Franka
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(grippercontroller SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(softhandmapper SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(grippercontroller PUBLIC
  include
)
target_include_directories(softhandmapper PUBLIC
  include
)

## Installation
install(TARGETS grippercontroller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS softhandmapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/mti_pdcontrollergoal_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)

if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(grippercontroller FILES ${SOURCES} ${HEADERS})
  add_format_target(softhandmapper FILES ${SOURCES} ${HEADERS})
  add_tidy_target(grippercontroller
    FILES ${SOURCES}
    DEPENDS grippercontroller
  )
  add_tidy_target(softhandmapper
    FILES ${SOURCES}
    DEPENDS softhandmapper
  )

endif()





