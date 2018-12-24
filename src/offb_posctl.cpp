/*
 * position_control.cpp
 *
 * Author:mz
 *
 * Time: 2018.11.27
 *
 * 说明: mavros位置控制示例程序
 *      输入：mavros发布的位置/速度信息
 *      输出：无人机的推力和姿态信息
 *      采用位置环/速度环串级PID控制，位置环P控制，速度环PID控制
 */
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <Eigen/Core> 

#include <ros/ros.h>
#include "Parameter.h"
#include <PID.h>


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

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

using namespace Eigen;

// //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//mavros_msgs::State current_state;           //无人机当前状态

geometry_msgs::PoseStamped pos_drone;       //读入的无人机当前位置
geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度

float thrust_target;        //期望推力
geometry_msgs::Vector3 euler_target;   //发给无人机的姿态指令
mavros_msgs::AttitudeTarget att_target;

float Yaw_Locked;           //锁定的偏航角(一般锁定为0)
PID PIDVX, PIDVY, PIDVZ;    //声明PID类
Parameter param;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

float get_ros_time(ros::Time time_begin);                                            //获取ros当前时间
int pix_controller(float cur_time);                                                  //控制程序

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//void state_cb(const mavros_msgs::State::ConstPtr &msg){
//    current_state = *msg;
//
//}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pos_drone = *msg;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    // 【订阅】无人机当前状态/位置/速度信息
//    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 20, pos_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/velocity", 20, vel_cb);

    // 【发布】飞机姿态/拉力信息 坐标系:NED系
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/cmd/attitudeTarget", 20);
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    // 读取PID参数
    
    std::string paraadr("/home/zm/catkin_ws/src/offb_posctl/src/param");
    if (param.readParam(paraadr.c_str()) == 0){
        std::cout<<"read config file error!"<<std::endl;
        return 0;
    }

    // 设置速度环PID参数 比例参数 积分参数 微分参数
    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(2, 3, 0);
    PIDVY.set_sat(2, 3, 0);
    PIDVZ.set_sat(2, 5, 0);

    

//    // 等待和飞控的连接
//    while(ros::ok() && current_state.connected == 0)
//    {
//        ros::spinOnce();
//        ros::Duration(1).sleep();
//        ROS_INFO("Not Connected");
//    }
//    ROS_INFO("Connected!!");



    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        float cur_time = get_ros_time(begin_time);  // 当前时间
        pix_controller(cur_time);                   //控制程序

        att_target.body_rate = euler_target;
        att_target.thrust = thrust_target;

        att_pub.publish(att_target);

        rate.sleep();
    }

    return 0;
}

/**
 * 获取当前时间 单位：秒
 */
float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>控 制 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int pix_controller(float cur_time)
{
//位 置 环
    //计算误差
    float error_x = param.pos_x - pos_drone.pose.position.x;
    float error_y = param.pos_y - pos_drone.pose.position.y;
    float error_z = param.pos_z - pos_drone.pose.position.z;
    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    //计算指定速度误差
    float vel_xd = param.x_p * error_x;
    float vel_yd = param.y_p * error_y;
    float vel_zd = param.z_p * error_z;

//速 度 环
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
//    PIDVX.start_intergrate_flag = true;
//    PIDVY.start_intergrate_flag = true;
//    PIDVZ.start_intergrate_flag = true;
//    if(current_state.mode != "OFFBOARD"){
//        PIDVX.start_intergrate_flag = false;
//        PIDVY.start_intergrate_flag = false;
//        PIDVZ.start_intergrate_flag = false;
//    }

    PIDVX.start_intergrate_flag = false;
    PIDVY.start_intergrate_flag = false;
    PIDVZ.start_intergrate_flag = false;

    //计算误差
    float error_vx = vel_xd - vel_drone.twist.linear.x;
    float error_vy = vel_yd - vel_drone.twist.linear.y;
    float error_vz = vel_zd - vel_drone.twist.linear.z;
    //传递误差
    PIDVX.add_error(error_vx, cur_time);
    PIDVY.add_error(error_vy, cur_time);
    PIDVZ.add_error(error_vz, cur_time);
    //计算输出
    PIDVX.pid_output();
    PIDVY.pid_output();
    PIDVZ.pid_output();

    Yaw_Locked = 0;
    Matrix2f A_yaw;
    A_yaw << sin(Yaw_Locked), cos(Yaw_Locked),
            -cos(Yaw_Locked), sin(Yaw_Locked);
    Vector2f mat_temp(PIDVX.Output,PIDVY.Output);       //赋值到期望推力和姿态
    Vector2f euler_temp= 1/9.8 * A_yaw.inverse() * mat_temp;
    euler_target.x = euler_temp[0];
    euler_target.y = euler_temp[1];
    euler_target.z = Yaw_Locked;

    thrust_target = (float)(0.05 * (9.8 + PIDVZ.Output));   //目标推力值

    return 0;
}