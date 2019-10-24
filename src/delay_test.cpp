
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <dynamic_reconfigure/server.h>
#include <offb_posctl/offb_Config.h>

#include <ros/ros.h>
#include "PID.h"
#include "FILTER.h"
#include "DOB.h"


//topic
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

mavros_msgs::State current_state;           //无人机当前状态(mode arm)
geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度
geometry_msgs::Vector3 vel_read;
sensor_msgs::Imu   imu_drone;               //读入的无人机的IMU信息 包括姿态角和线加速度


geometry_msgs::Vector3 acc_receive,acc_read;         //读入的无人机线加速度,VICON速度微分数据

float get_ros_time(ros::Time time_begin);

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

bool hasGotImu = false;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    hasGotImu = true;
    imu_drone = *msg;
    acc_receive = imu_drone.linear_acceleration;
}

bool vel_initialized = false;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    if(!vel_initialized)
    {
        vel_initialized = true;
        vel_drone = *msg;
        vel_read = vel_drone.twist.linear;
        return;
    }
    vel_drone = *msg;
    // lowpass filter 1p
    acc_read.x = (vel_drone.twist.linear.x - vel_read.x) / 0.05;
    acc_read.y = (vel_drone.twist.linear.y - vel_read.y) / 0.05;
    acc_read.z = (vel_drone.twist.linear.z - vel_read.z) / 0.05;
    vel_read = vel_drone.twist.linear;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "delay_test");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/vel", 10, vel_cb);

    ros::Publisher acc_imu_pub = nh.advertise<geometry_msgs::Vector3>("/plot/acc_imu", 10);
    ros::Publisher acc_mop_pub = nh.advertise<geometry_msgs::Vector3>("/plot/acc_mop", 10);

    ros::Rate loop_rate(20.0);

    ros::Time begin_time = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        float cur_time = get_ros_time(begin_time);  // 当前时间

        acc_imu_pub.publish(acc_receive);
        acc_mop_pub.publish(acc_read);

        loop_rate.sleep();
    }

    return 0;
}

float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}