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
  message_generation
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
        cfg/offb_cfg.cfg
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
        src/DOB.h
        src/DOB.cpp)

add_library(thelib_vicon
        src/PID.h
        src/PID.cpp
        src/FILTER.h
        src/FILTER.cpp
        src/DOB.h
        src/DOB.cpp)


add_library(thelib_uam
        src/PID.h
        src/PID.cpp
        src/FILTER.h
        src/FILTER.cpp
        src/DOB.h
        src/DOB.cpp)


add_executable(offb_posctl
        src/offb_posctl.cpp)
target_link_libraries(offb_posctl ${catkin_LIBRARIES} thelib)


add_executable(offb_posctl_vicon
        src/offb_posctl_vicon.cpp)
target_link_libraries(offb_posctl_vicon ${catkin_LIBRARIES} thelib_vicon)

# for simulation, enable OFFBOARD automatic
add_executable(offb_enable src/offb_enable.cpp)
target_link_libraries(offb_enable ${catkin_LIBRARIES})

# for real fly, enable OFFBOARD by RC switch
add_executable(offb_enable_vicon src/offb_enable_vicon.cpp)
target_link_libraries(offb_enable_vicon ${catkin_LIBRARIES})

# for uam fly, via Xbee to control
add_executable(offb_posctl_uam src/offb_posctl_uam.cpp)
target_link_libraries(offb_posctl_uam ${catkin_LIBRARIES} thelib_uam)


