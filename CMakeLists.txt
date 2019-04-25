cmake_minimum_required(VERSION 2.8.3)
project(offb_posctl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  mavros
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb_posctl
#  CATKIN_DEPENDS geometry_msgs mavros roscpp rospy
#  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        src/)

add_library(thelib
        src/PID.h
        src/PID.cpp
        src/FILTER.h
        src/FILTER.cpp
        src/PARAM.h
        src/PARAM.cpp src/DOB.h src/DOB.cpp)

add_executable(offb_posctl
        src/offb_posctl.cpp)
target_link_libraries(offb_posctl ${catkin_LIBRARIES} thelib)

