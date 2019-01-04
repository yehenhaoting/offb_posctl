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
#include <sstream>
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
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

using namespace Eigen;

// //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

mavros_msgs::State current_state;           //无人机当前状态
geometry_msgs::PoseStamped pos_drone;       //读入的无人机当前位置
geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度
geometry_msgs::PoseStamped att_drone;       //读入的无人机姿态
sensor_msgs::Imu acc_drone;         //读入的无人机加速度

geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令
geometry_msgs::Vector3 angle_target;
geometry_msgs::Vector3 vel_target;
geometry_msgs::Vector3 angle_receive;       //读入的无人机姿态（欧拉角）
geometry_msgs::Vector3 acc_receive;         //读入的无人机线加速度

float thrust_target;        //期望推力
float Yaw_Init;
float Yaw_Locked = 0;           //锁定的偏航角(一般锁定为0)
PID PIDVX, PIDVY, PIDVZ;    //声明PID类
Parameter param;
std::ofstream logfile;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

float get_ros_time(ros::Time time_begin);                                            //获取ros当前时间
int pix_controller(float cur_time);                                                  //控制程序
void data_log(std::ofstream &logfile, float cur_time);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

void acc_cb(const sensor_msgs::Imu::ConstPtr &msg){
    acc_drone = *msg;
    acc_receive = acc_drone.linear_acceleration;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pos_drone = *msg;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}

bool hasGotAtt = false;
void att_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    att_drone = *msg;
    hasGotAtt = true;
    angle_receive = quaternion2euler(att_drone.pose.orientation.x, att_drone.pose.orientation.y, att_drone.pose.orientation.z, att_drone.pose.orientation.w);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;
//    std::ofstream logfile;

    // 【订阅】无人机当前状态/位置/速度信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20, state_cb);
    ros::Subscriber acceleration_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 20, acc_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 20, pos_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 20, vel_cb);
    ros::Subscriber attitude_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 20, att_cb);
    // 【发布】飞机姿态/拉力信息 坐标系:NED系
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>("/cmd/thrust", 20);
    ros::Publisher orientataion_pub = nh.advertise<geometry_msgs::Quaternion>("/cmd/orientation", 20);
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    // 读取PID参数
    std::string paraadr("/home/zm/catkin_ws/src/offb_posctl/src/param");
    if (param.readParam(paraadr.c_str()) == 0){
        std::cout<<"read config file error!"<<std::endl;
        return 0;
    }

    // log输出文件初始化
    logfile.open("/home/zm/catkin_ws/src/offb_posctl/src/log.csv", std::ios::out);
    if (! logfile.is_open()){
        std::cout<<"log to file error!"<<std::endl;
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

    // 等待和飞控的连接
    while(ros::ok() && current_state.connected == 0)
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected!!");

    while(ros::ok() && !hasGotAtt)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("waitting for att...");
    }
    float x = att_drone.pose.orientation.x;
    float y = att_drone.pose.orientation.y;
    float z = att_drone.pose.orientation.z;
    float w = att_drone.pose.orientation.w;
    angle_receive = quaternion2euler(x, y, z, w);
    Yaw_Init = angle_receive.z;
    ROS_INFO_STREAM("Got Yaw_Init: " << Yaw_Init);

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        float cur_time = get_ros_time(begin_time);  // 当前时间
        pix_controller(cur_time);                   //控制程序

        if(current_state.mode == "OFFBOARD"){
            data_log(logfile, cur_time);                     //log输出
        }

        std_msgs::Float32 data2pub;
        data2pub.data = thrust_target;
        thrust_pub.publish(data2pub);
        orientataion_pub.publish(orientation_target);

        rate.sleep();
    }
    logfile.close();
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
    vel_target.x = vel_xd;
    vel_target.y = vel_yd;
    vel_target.z = vel_zd;

//速 度 环
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDVX.start_intergrate_flag = true;
    PIDVY.start_intergrate_flag = true;
    PIDVZ.start_intergrate_flag = true;
    if(current_state.mode != "OFFBOARD"){
        PIDVX.start_intergrate_flag = false;
        PIDVY.start_intergrate_flag = false;
        PIDVZ.start_intergrate_flag = false;
    }
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

    Matrix2f A_yaw;
    A_yaw << sin(Yaw_Locked), cos(Yaw_Locked),
            -cos(Yaw_Locked), sin(Yaw_Locked);
    Vector2f acc_d(PIDVX.Output, PIDVY.Output);       //赋值到期望推力和姿态
    Vector2f euler_temp= 1/9.8 * A_yaw.inverse() * acc_d;
//    angle_target.x = euler_temp[0];
//    angle_target.y = euler_temp[1];
//    angle_target.z = Yaw_Locked;

    //滤波器输入
    PIDVX.filter_input(PIDVX.Output - acc_receive.x, cur_time);
    PIDVY.filter_input(PIDVY.Output - acc_receive.y, cur_time);
    PIDVZ.filter_input(PIDVZ.Output - acc_receive.z, cur_time);
    //计算滤波器输出
    PIDVX.filter_output();
    PIDVY.filter_output();
    PIDVZ.filter_output();

    Vector2f acc_error(PIDVX.Output_filter, PIDVY.Output_filter);
    Vector2f euler_DOB = 1/9.8 * A_yaw.inverse() * acc_error;
    angle_target.x = euler_temp[0] + euler_DOB[1];
    angle_target.y = euler_temp[1] + euler_DOB[1];
    angle_target.z = Yaw_Locked;

    orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
    thrust_target = (float)(0.05 * (9.8 + PIDVZ.Output + 0.1*PIDVZ.Output_filter));   //目标推力值


    return 0;
}

/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 返回Vector3的欧拉角
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    temp.y = asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */
void data_log(std::ofstream &logfile, float cur_time)
{
    logfile <<cur_time<<","<<param.pos_x <<","<<param.pos_y <<","<<param.pos_z <<","                           //set_pos
        <<pos_drone.pose.position.x <<","<<pos_drone.pose.position.y <<","<<pos_drone.pose.position.z <<","    //uav_pos
        <<vel_target.x <<","<<vel_target.y <<","<<vel_target.z <<","                                           //set_vel
        <<vel_drone.twist.linear.x <<","<<vel_drone.twist.linear.y <<","<<vel_drone.twist.linear.z <<","       //uav_vel
        <<angle_target.x  <<","<<angle_target.y  <<","<<angle_target.z  <<","                                  //set_att
        <<angle_receive.x <<","<<angle_receive.y <<","<<angle_receive.z <<","                                  //uav_att
        <<thrust_target<<std::endl;

}