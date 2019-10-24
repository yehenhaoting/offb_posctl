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
#include <dynamic_reconfigure/server.h>
#include <offb_posctl/offb_Config.h>

#include <ros/ros.h>
#include "PID.h"
#include "FILTER.h"
#include "DOB.h"
#include "BUFFER.h"


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

geometry_msgs::PoseStamped pos_ref;         //无人机参考位置

mavros_msgs::State current_state;           //无人机当前状态(mode arm)
sensor_msgs::Imu   imu_drone;               //读入的无人机的IMU信息 包括姿态角和线加速度

geometry_msgs::PoseStamped  pos_drone;      //读入的无人机当前位置
geometry_msgs::PoseStamped  pos_drone_last; //读入的无人机上一次位置

geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度

geometry_msgs::Vector3 acc_receive;         //读入的无人机线加速度
geometry_msgs::Vector3 angle_receive;       //读入的无人机姿态（欧拉角）

geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令

geometry_msgs::Vector3 angle_des;            //线性模型输出的理想值
//geometry_msgs::Vector3 angle_dis;            //DOB控制器估计的扰动值
geometry_msgs::Vector3 angle_target;            //经DOB控制器作用后的实际系统输入值
geometry_msgs::Vector3 vel_target, vel_read, vel_read2;
geometry_msgs::Vector3 pos_error;
geometry_msgs::Vector3 filter_in;
geometry_msgs::Vector3 filter_out;

// debug data
geometry_msgs::Vector3 vel_vicon;





float thrust_target;        //期望推力
float Yaw_Init, angel_init, angel_vicon, angle_deviation;
float Yaw_Locked = 0;           //锁定的偏航角(一般锁定为0)
//float alpha = 1.0;

PID PIDX, PIDY, PIDZ, PIDVX, PIDVY, PIDVZ;    //声明PID类
DOB DOBX, DOBY, DOBZ;                         //声明DOB类
FILTER FilterVX, FilterVY, FilterVZ;
BUFFER Buffer;
offb_posctl::offb_Config param;
std::ofstream logfile;
std::ofstream debugfile;

const float MAX_POSITION_MEASURE_ERROR = 0.2;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

float get_ros_time(ros::Time time_begin);                                            //获取ros当前时间
int pix_controller(float cur_time);                                                  //控制程序
float satfunc(float data, float Max, float Thres);
void data_log(std::ofstream &logfile, float cur_time);
void debug_log(std::ofstream &debugfile, float cur_time);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void param_cb(const offb_posctl::offb_Config &config, uint32_t level)
{
    param = config;
}


void ref_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pos_ref = *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

bool hasGotImu = false;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    hasGotImu = true;
    imu_drone = *msg;
    acc_receive = imu_drone.linear_acceleration;
    angle_receive = quaternion2euler(imu_drone.orientation.x, imu_drone.orientation.y, imu_drone.orientation.z, imu_drone.orientation.w);
}

bool pose_initialized = false;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    if(!pose_initialized)
    {
        pose_initialized = true;
        pos_drone = *msg;
        pos_drone_last = pos_drone;
        return;
    }
    if(fabs(pos_drone.pose.position.x - pos_drone_last.pose.position.x) < MAX_POSITION_MEASURE_ERROR &&
       fabs(pos_drone.pose.position.y - pos_drone_last.pose.position.y) < MAX_POSITION_MEASURE_ERROR &&
       fabs(pos_drone.pose.position.z - pos_drone_last.pose.position.z) < MAX_POSITION_MEASURE_ERROR)
    {
        pos_drone_last = pos_drone;
        pos_drone = *msg;
    }
//    Buffer.delay_step = param.Delay_step;
//    pos_drone.pose.position = Buffer.pos_buffer(pos_drone.pose.position);
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
//    vel_drone.twist.linear = Buffer.vel_buffer(vel_drone.twist.linear);

    // lowpass filter 1p
    vel_read.x = 0.25 * vel_drone.twist.linear.x + 0.75 * vel_read.x;
    vel_read.y = 0.25 * vel_drone.twist.linear.y + 0.75 * vel_read.y;
    vel_read.z = 0.25 * vel_drone.twist.linear.z + 0.75 * vel_read.z;

    vel_vicon = vel_drone.twist.linear;
}


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<offb_posctl::offb_Config> server;
    dynamic_reconfigure::Server<offb_posctl::offb_Config>::CallbackType ff;
    ff = boost::bind(&param_cb, _1, _2);
    server.setCallback(ff);


    // 【订阅】无人机当前状态/位置/速度信息
    ros::Subscriber pos_ref_sub  = nh.subscribe<geometry_msgs::PoseStamped>("/cmd/pos_ref", 10, ref_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
//
//    //Gazebo 仿真数据
//    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
//    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 10, vel_cb);
    //vicon 数据
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 10, pos_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/vel", 10, vel_cb);

 // 【发布】飞机姿态/拉力信息 坐标系:NED系
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>("/cmd/thrust", 10);
    ros::Publisher orientataion_pub = nh.advertise<geometry_msgs::Quaternion>("/cmd/orientation", 10);


    ros::Publisher error_pub = nh.advertise<geometry_msgs::Vector3>("/plot/pos_error", 10);
//    ros::Publisher filter_pub = nh.advertise<geometry_msgs::Vector3>("/plot/filter_out", 10);
//    ros::Publisher filter_pub2 = nh.advertise<geometry_msgs::Vector3>("/plot/filter_in", 10);

    //debug pub
    ros::Publisher viconvel_pub = nh.advertise<geometry_msgs::Vector3>("/plot/viconvel_read", 10);
    ros::Publisher filtervel_pub = nh.advertise<geometry_msgs::Vector3>("/plot/filtervel_read", 10);

    ros::Publisher acc_imu_pub = nh.advertise<geometry_msgs::Vector3>("/plot/acc_imu", 10);
    ros::Publisher acc_mop_pub = nh.advertise<geometry_msgs::Vector3>("/plot/acc_mop", 10);

//    ros::Publisher filtervel_pub2 = nh.advertise<geometry_msgs::Vector3>("/plot/filtervel2_read", 10);

    ros::Publisher targetvel_pub = nh.advertise<geometry_msgs::Vector3>("/plot/target_vel", 10);




    // 频率 [20Hz]
    ros::Rate rate(20.0);

//    // 读取PID参数
//    if (! param.readParam()){
//        ROS_ERROR("read config file error!");
//        return 0;
//    }

    // log输出文件初始化
    logfile.open("/home/ubuntu/catkin_px4_OFFBOARD/src/offb_posctl/log/log031102.csv", std::ios::out);
    if (! logfile.is_open()){
        ROS_ERROR("log to file error!");
        return 0;
    }

    debugfile.open("/home/ubuntu/catkin_px4_OFFBOARD/src/offb_posctl/log/debug031101.csv", std::ios::out);
    if (! debugfile.is_open()){
        ROS_ERROR("debug to file error!");
        return 0;
    }

//    // 设置位置环PID参数 比例参数 积分参数 微分参数
//    PIDX.setPID(param.x_p, param.x_i, param.x_d);
//    PIDY.setPID(param.y_p, param.y_i, param.y_d);
//    PIDZ.setPID(param.z_p, param.z_i, param.z_d);

    // 设置位置环积分上限 控制量最大值 误差死区
    PIDX.set_sat(0.3, 3, 0.0);
    PIDY.set_sat(0.3, 3, 0.0);
    PIDZ.set_sat(0.05, 5, 0.0);

//    // 设置速度环PID参数 比例参数 积分参数 微分参数
//    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
//    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
//    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(0.5, 1.5, 0);
    PIDVY.set_sat(0.5, 1.5, 0);
    PIDVZ.set_sat(1.5, 5.0, 0);

    angle_target.x = 0;
    angle_target.y = 0;
    thrust_target = 0.3;


//    vel_read2.x = 0.0;
//    vel_read2.y = 0.0;
//    vel_read2.z = 0.0;

    // 等待和飞控的连接
    while(ros::ok() && current_state.connected == 0)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected!!");

    while(ros::ok() && !hasGotImu)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("waitting for IMU message ...");
    }
    auto angle_receive = quaternion2euler(imu_drone.orientation.x, imu_drone.orientation.y, imu_drone.orientation.z, imu_drone.orientation.w);
    auto Yaw_Init = angle_receive.z;
    ROS_INFO_STREAM("Got Yaw_Init: " << Yaw_Init);

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        float cur_time = get_ros_time(begin_time);  // 当前时间
        pix_controller(cur_time);                   //控制程序

        if(current_state.mode == "OFFBOARD" && param.LOG_to_file){
            data_log(logfile, cur_time);                     //log输出
//            debug_log(debugfile, cur_time);
        }

        std_msgs::Float32 data2pub;
        data2pub.data = thrust_target;
        thrust_pub.publish(data2pub);
        orientataion_pub.publish(orientation_target);


        error_pub.publish(pos_error);
//        filter_pub.publish(filter_out);
//        filter_pub2.publish(filter_in);


        //debug pub
        viconvel_pub.publish(vel_vicon);
        filtervel_pub.publish(vel_read);
//        filtervel_pub2.publish(vel_read2);
        targetvel_pub.publish(vel_target);


        rate.sleep();
    }

    logfile.close();
    debugfile.close();

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

/**
 * 控制函数
 * @param cur_time
 * @return
 */
int pix_controller(float cur_time)
{


//位 置 环
    // 设置位置环PID参数 比例参数 积分参数 微分参数
    PIDX.setPID(param.MC_X_P, param.MC_X_I, param.MC_X_D);
//    PIDY.setPID(param.MC_Y_P, param.MC_Y_I, param.MC_Y_D);
    PIDY.setPID(param.MC_X_P, param.MC_X_I, param.MC_X_D);
    PIDZ.setPID(param.MC_Z_P, param.MC_Z_I, param.MC_Z_D);
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDX.start_intergrate_flag = true;
    PIDY.start_intergrate_flag = true;
    PIDZ.start_intergrate_flag = true;
    if(current_state.mode != "OFFBOARD"){
        PIDX.start_intergrate_flag = false;
        PIDY.start_intergrate_flag = false;
        PIDZ.start_intergrate_flag = false;
    }
    //计算误差
    pos_error.x = pos_ref.pose.position.x - pos_drone.pose.position.x;
    pos_error.y = pos_ref.pose.position.y - pos_drone.pose.position.y;
    pos_error.z = pos_ref.pose.position.z - pos_drone.pose.position.z;
    std::cout << "error: x：" << pos_error.x << "\ty：" << pos_error.y << "\tz：" << pos_error.z << std::endl;

    //传递误差
    PIDX.add_error(pos_error.x, cur_time);
    PIDY.add_error(pos_error.y, cur_time);
    PIDZ.add_error(pos_error.z, cur_time);
    //计算输出
    PIDX.pid_output();
    PIDY.pid_output();
    PIDZ.pid_output();
    //计算指定速度误差
    vel_target.x = PIDX.Output;
    vel_target.y = PIDY.Output;
    vel_target.z = PIDZ.Output;

//速 度 环
//    FilterVX.start_filter_flag = true;
//    FilterVY.start_filter_flag = true;
//    FilterVZ.start_filter_flag = true;
//
//    if(current_state.mode != "OFFBOARD"){
//        FilterVX.start_filter_flag = true;
//        FilterVY.start_filter_flag = true;
//        FilterVZ.start_filter_flag = true;
//    }
//    //滤波器输入
//    FilterVX.filter_input(vel_drone.twist.linear.x, cur_time);
//    FilterVY.filter_input(vel_drone.twist.linear.y, cur_time);
//    FilterVZ.filter_input(vel_drone.twist.linear.z, cur_time);
//    //计算滤波器输出
//    FilterVX.filter_output();
//    FilterVY.filter_output();
//    FilterVZ.filter_output();
//    vel_read.x = FilterVX.Output_filter;
//    vel_read.y = FilterVY.Output_filter;
//    vel_read.z = FilterVZ.Output_filter;
//
//    vel_read2.x = 0.3 * vel_drone.twist.linear.x + 0.7 * vel_read2.x;
//    vel_read2.y = 0.3 * vel_drone.twist.linear.y + 0.7 * vel_read2.y;
//    vel_read2.z = 0.3 * vel_drone.twist.linear.z + 0.7 * vel_read2.z;

//速 度 环
    // 设置位置环PID参数 比例参数 积分参数 微分参数
    PIDVX.setPID(param.MC_VX_P, param.MC_VX_I, param.MC_VX_D);
//    PIDVY.setPID(param.MC_VY_P, param.MC_VY_I, param.MC_VY_D);
    PIDVY.setPID(param.MC_VX_P, param.MC_VX_I, param.MC_VX_D);
    PIDVZ.setPID(param.MC_VZ_P, param.MC_VZ_I, param.MC_VZ_D);
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
    float error_vx = vel_target.x - vel_vicon.x;
    float error_vy = vel_target.y - vel_vicon.y;
    float error_vz = vel_target.z - vel_vicon.z;
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
    Vector2f euler_temp = 1/9.8 * A_yaw.inverse() * acc_d;
    angle_des.x = euler_temp[0];
    angle_des.y = euler_temp[1];


    // yaw 角度修正
    angel_vicon = pos_drone.pose.orientation.y;
    std::cout << "Euler_vicon: yaw：" << angel_vicon * 60 << std::endl;
    // std::cout << "Euler_vicon: roll：" << pos_drone.pose.orientation.x << "\tpitch：" << pos_drone.pose.orientation.y << "\tyaw：" << pos_drone.pose.orientation.z << std::endl;
    angle_deviation = angel_vicon - angel_init;
    if (fabs(angle_deviation) < 0.05)
    {
        angle_deviation = 0;
    }
    angle_des.z = Yaw_Init + Yaw_Locked + angle_deviation;


    //DOB 干扰观测器（利用积分平均的方法）

    Vector2f temp_angle(angle_target.x , angle_target.y);
    Vector2f acc_xy_des = 9.8 * A_yaw * temp_angle;     //理论的加速度值

    DOBX.add_data(cur_time, vel_drone.twist.linear.x, acc_xy_des[0]);
    DOBY.add_data(cur_time, vel_drone.twist.linear.y, acc_xy_des[1]);

//    DOBX.add_data(cur_time, vel_read.x, acc_xy_des[0]);
//    DOBY.add_data(cur_time, vel_read.y, acc_xy_des[1]);

    Vector2f acc_xy_dis(DOBX.dob_output(), DOBY.dob_output());
    Vector2f angle_dis = 1/9.8 * A_yaw.inverse() * acc_xy_dis;


    auto thrust_des = (float)(0.05 * (9.8 + PIDVZ.Output ));
    auto acc_z_des = (float)(9.8 - 1/0.05 * thrust_target);
//    auto acc_z_des = (float)(1/0.05 * thrust_target - 9.8);
    DOBZ.add_data(cur_time, vel_drone.twist.linear.z, acc_z_des);
    auto acc_z_dis = DOBZ.dob_output();
    auto thrust_dis = (float)(0.05 * (9.8 + acc_z_dis));


    angle_target.x = satfunc((angle_des.x + param.DOB_rate * angle_dis[0]), 0.314f, 0.0f);
    angle_target.y = satfunc((angle_des.y + param.DOB_rate * angle_dis[1]), 0.314f, 0.0f);
    angle_target.z = angle_des.z;
    orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);

    thrust_target = thrust_des + 0.0 * param.DOB_rate * thrust_dis + (param.THR_HOVER - 0.5f);  //目标推力值
//    thrust_target = satfunc((thrust_des + 0.0 * param.DOB_rate * thrust_dis + (param.THR_HOVER - 0.5f)), 0.80f, 0.0f);  //目标推力值



//// DOB 干扰观测器
//    Vector2f acc_real(acc_receive.x, acc_receive.y);
//    Vector2f euler_DOB = 1/9.8 * A_yaw.inverse() * acc_real;
//
//    FilterX.start_filter_flag = true;
//    FilterY.start_filter_flag = true;
//    if(current_state.mode != "OFFBOARD"){
//        FilterX.start_filter_flag = true;
//        FilterY.start_filter_flag = true;
//    }
//    //滤波器输入
//    filter_in.x = angle_target.x - euler_DOB[0];
//    filter_in.y = angle_target.y - euler_DOB[1];
//    FilterX.filter_input(filter_in.x, cur_time);
//    FilterY.filter_input(filter_in.y, cur_time);
////    PIDVZ.filter_input(PIDVZ.Output - acc_receive.z, cur_time);
//    //计算滤波器输出
//    FilterX.filter_output();
//    FilterY.filter_output();
//    filter_out.x = FilterX.Output_filter;
//    filter_out.y = FilterY.Output_filter;
////    PIDVZ.filter_output();
//
//    angle_dis.x = FilterX.satfunc(filter_out.x, 0.08, 0);
//    angle_dis.y = FilterY.satfunc(filter_out.y, 0.08, 0);





//    angle_target.x = angle_des.x + alpha * angle_dis.x;
//    angle_target.y = angle_des.y + alpha * angle_dis.y;
//    angle_target.z = angle_des.z;

//    orientation_target = euler2quaternion(angle_target.x + 0.15, angle_target.y - 0.15, angle_target.z);
//    thrust_target = (float)(0.05 * (9.8 + PIDVZ.Output ));   //目标推力值

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
 * 限幅函数
 * @param data 输入的数据
 * @param Max 数据的最大值
 * @param Thres 零死区
 * @return 返回输出值
 */
float satfunc(float data, float Max, float Thres)
{
    if (fabs(data)<Thres)
        return 0;
    else if(fabs(data)>Max){
        return (data>0)?Max:-Max;
    }
    else{
        return data;
    }
}


/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */
void data_log(std::ofstream &logfile, float cur_time)
{
//    logfile<<pos_drone.pose.position.x <<","<<pos_drone.pose.position.y <<","<<pos_drone.pose.position.z <<","    //uav_pos
//        <<vel_target.x <<","<<vel_target.y <<","<<vel_target.z <<","                                           //set_vel
//        <<vel_drone.twist.linear.x <<","<<vel_drone.twist.linear.y <<","<<vel_drone.twist.linear.z <<","       //uav_vel
//        <<angle_target.x  <<","<<angle_target.y  <<","<<angle_target.z  <<","                                  //set_att
//        <<angle_receive.x <<","<<angle_receive.y <<","<<angle_receive.z <<","                                  //uav_att
////        <<acc_receive.x   <<","<<acc_receive.y   <<","<<acc_receive.z   <<","                                  //uav_acc
//        <<thrust_target<<std::endl;
    logfile<<cur_time <<","<<param.DOB_rate<<","
           <<pos_drone.pose.position.x <<","<<pos_drone.pose.position.y <<","<<pos_drone.pose.position.z <<","    //uav_pos
           <<pos_ref.pose.position.x <<","<<pos_ref.pose.position.y <<","<<pos_ref.pose.position.z <<","          //ref_pos
           <<vel_target.x <<","<<vel_target.y <<","<<vel_target.z <<","                                           //set_vel
           <<vel_drone.twist.linear.x <<","<<vel_drone.twist.linear.y <<","<<vel_drone.twist.linear.z <<","       //uav_vel
           <<std::endl;

}

/**
 * 记录滤波器输入输出数据等
 * @param debugfile
 * @param cur_time
 */
void debug_log(std::ofstream &debugfile, float cur_time)
{
    debugfile <<cur_time <<"," <<filter_in.x <<","<<filter_in.y <<","<<filter_out.x <<","<<filter_out.y <<std::endl;
}