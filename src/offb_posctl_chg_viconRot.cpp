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
#include "Parameter_viconRot.h"
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
sensor_msgs::Imu att_drone;                 //读入的无人机姿态
geometry_msgs::PoseStamped pos_drone;       //读入的无人机当前位置
geometry_msgs::PoseStamped pos_drone_last;       //读入的无人机last位置

geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度
// geometry_msgs::PoseStamped att_drone;       //读入的无人机姿态
geometry_msgs::Vector3 angle_receive;       //读入的无人机姿态（欧拉角）

geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令
geometry_msgs::Vector3 angle_target;
geometry_msgs::Vector3 vel_target;

float thrust_target;        //期望推力
float Yaw_Init, angel_init, angel_vicon, angle_deviation;
float Yaw_Locked = 0;           //锁定的偏航角(一般锁定为0)
PID PIDX, PIDY, PIDZ, PIDVX, PIDVY, PIDVZ;    //声明PID类
Parameter_viconRot param;
std::ofstream logfile;

const float MAX_POSITION_MEASURE_ERROR = 0.2;

std::vector<geometry_msgs::PoseStamped> pose_stack;
float average_delay = 0.f;
bool if_pose_initialized = false;
const int pose_stack_max_size = 10;   // Change this value to store more past position msgs. Larger makes algorithm slower.
const float planning_time_interval = 0.05; // s

geometry_msgs::Vector3 target_p;
geometry_msgs::Vector3 current_p;
geometry_msgs::Vector3 target_v;
geometry_msgs::Vector3 current_v;
geometry_msgs::Vector3 target_p_last; 
ros::Publisher target_p_pub, target_v_pub, current_p_pub, current_v_pub;

Eigen::MatrixXd traj_p;
Eigen::MatrixXd traj_v; 
Eigen::MatrixXd traj_a; 
Eigen::VectorXd traj_t;

const bool simulation_model = false;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

float get_ros_time(ros::Time time_begin);                                            //获取ros当前时间
int pix_controller(float cur_time);                                                  //控制程序
void data_log(float cur_time);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;

}

bool pose_initialized = false;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    
    if(!pose_initialized)
    {
        pose_initialized = true;
        pos_drone_last = pos_drone;
        pos_drone = *msg;
        pos_drone.header.stamp = ros::Time::now(); // !!! note chg
        return;
    }

    if(fabs(msg-> pose.position.x - pos_drone.pose.position.x) < MAX_POSITION_MEASURE_ERROR &&
        fabs(msg-> pose.position.y - pos_drone.pose.position.y) < MAX_POSITION_MEASURE_ERROR &&
        fabs(msg-> pose.position.z - pos_drone.pose.position.z) < MAX_POSITION_MEASURE_ERROR)
    {
        pos_drone_last = pos_drone;
        pos_drone = *msg;
        pos_drone.header.stamp = ros::Time::now(); // !!! note chg

        if(pose_stack.size() < pose_stack_max_size)
        {
            pose_stack.push_back(pos_drone);
        }
        else  //delete the first one and add the current position
        {
            if_pose_initialized = true;
            std::vector<geometry_msgs::PoseStamped>::iterator iter=pose_stack.begin(); 
            pose_stack.erase(iter);
            pose_stack.push_back(pos_drone);

             // Calculate time interval
            float time_interval_this = get_ros_time(pos_drone_last.header.stamp); 
            average_delay = time_interval_this * 0.4f + average_delay * 0.6f;
            std::cout << "average_delay:" << average_delay << std::endl;
        }
    }
    
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}

bool hasGotAtt = false;
void att_cb(const sensor_msgs::Imu::ConstPtr &msg){
    att_drone = *msg;
    hasGotAtt = true;
    angle_receive = quaternion2euler(att_drone.orientation.x, att_drone.orientation.y, att_drone.orientation.z, att_drone.orientation.w);
}

void get_last_pose(float time_duration_s, geometry_msgs::PoseStamped &last_pose)
{
    int seq_to_get = pose_stack_max_size - 1 -(int)(time_duration_s / average_delay);
    std::cout<<"seq_to_get "<<seq_to_get<<std::endl;
    if(seq_to_get < 0)
    {
        seq_to_get = 0;
        std::cout << "pose_stack_max_size might be too small" << std::endl; 
    }
    
    last_pose = pose_stack.at(seq_to_get);
}

void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0, double yaw0, Eigen::Vector3d goal, double d, double v_max, double delt_t,
                       Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t)
{
    Eigen::Vector3d pf = goal;
    Eigen::Vector3d vf;
    vf << 0.0, 0.0, 0.0; // % Note: 0 maybe better, for the p curve wont go down to meet the vf

    Eigen::Vector3d af = Eigen::Vector3d::Zero();
    
    double j_limit = 3;
    double a_limit = 2;
    double v_limit = 0.8;

    double T1 = fabs(af(0)-a0(0))/j_limit > fabs(af(1)-a0(1))/j_limit ? fabs(af(0)-a0(0))/j_limit : fabs(af(1)-a0(1))/j_limit;
    T1 = T1 > fabs(af(2)-a0(2))/j_limit ? T1 : fabs(af(2)-a0(2))/j_limit;
    double T2 = fabs(vf(0)-v0(0))/a_limit > fabs(vf(1)-v0(1))/a_limit ? fabs(vf(0)-v0(0))/a_limit : fabs(vf(1)-v0(1))/a_limit;
    T2 = T2 > fabs(vf(2)-v0(2))/a_limit ? T2 : fabs(vf(2)-v0(2))/a_limit;
    double T3 = fabs(pf(0)-p0(0))/v_limit > fabs(pf(1)-p0(1))/v_limit ? fabs(pf(0)-p0(0))/v_limit : fabs(pf(1)-p0(1))/v_limit;
    T3 = T3 > fabs(pf(2)-p0(2))/v_limit ? T3 : fabs(pf(2)-p0(2))/v_limit;

    double T = T1 > T2 ? T1 : T2;
    T = T > T3 ? T : T3;
    T = T < 0.5 ? 0.5 : T;

    int times = T / delt_t;

    p = Eigen::MatrixXd::Zero(times, 3);
    v = Eigen::MatrixXd::Zero(times, 3);
    a = Eigen::MatrixXd::Zero(times, 3);
    t = Eigen::VectorXd::Zero(times);

    // % calculate optimal jerk controls by Mark W. Miller
    for(int ii=0; ii<3; ii++)
    {
        double delt_a = af(ii) - a0(ii);
        double delt_v = vf(ii) - v0(ii) - a0(ii)*T;
        double delt_p = pf(ii) - p0(ii) - v0(ii)*T - 0.5*a0(ii)*T*T;

        //%  if vf is not free
        double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
        double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
        double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);

        for(int jj=0; jj<times; jj++)
        {
            double tt = (jj + 1)*delt_t;
            t(jj) = tt;
            p(jj,ii) = alpha/120*pow(tt,5) + beta/24*pow(tt,4) + gamma/6*pow(tt,3) + a0(ii)/2*pow(tt,2) + v0(ii)*tt + p0(ii);
            v(jj,ii) = alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0(ii)*tt + v0(ii);
            a(jj,ii) = alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0(ii);
        }
    }
}


void trajectoryTimerCallback(const ros::TimerEvent& event)
{
    const int wait_times_1 = 40;
    const int wait_times_2 = 200;
    static int calculation_times = 0;
    static int send_times = 0;
    static double rise_z_delt = param.pos_z / (double)(wait_times_2 - wait_times_1);
    static double rise_vz = rise_z_delt / planning_time_interval;


    if(!simulation_model)
    {
        if(current_state.mode != "OFFBOARD"){
            return;
        }
    }
    

    target_p_last = target_p; // store last target position

    if(calculation_times < wait_times_1)
    {
        calculation_times ++;
        target_p.x = param.pos_x;
        target_p.y = param.pos_y;
        target_p.z = 0.0;

        target_v.x = 0.0;
        target_v.y = 0.0;
        target_v.z = 0.0;
    }
    else if(calculation_times < wait_times_2)
    {
        calculation_times ++;
        target_p.x = param.pos_x;
        target_p.y = param.pos_y;
        target_p.z += rise_z_delt;

        target_v.x = 0.0;
        target_v.y = 0.0;
        target_v.z = rise_vz;
    }
    else if(calculation_times == wait_times_2) // Calculate once
    {
        Eigen::Vector3d p0;
        p0 << pos_drone.pose.position.x, pos_drone.pose.position.y, param.pos_z;//pos_drone.pose.position.z;
        Eigen::Vector3d v0;
        v0 << vel_drone.twist.linear.x, vel_drone.twist.linear.y, vel_drone.twist.linear.z;
        Eigen::Vector3d a0;
        a0 << 0.0, 0.0, 0.0;

        Eigen::Vector3d pT;
        pT << 1.0, 1.0, 1.5;

        motion_primitives(p0, v0, a0, Yaw_Locked, pT, 1.4, 1.0, 0.05, traj_p, traj_v, traj_a, traj_t);
        calculation_times ++;
    }
    else //send from the calculated trajectory
    {
        send_times ++;

        if(send_times < traj_t.rows())
        {
            target_p.x = traj_p(send_times, 0);
            target_p.y = traj_p(send_times, 1);
            target_p.z = traj_p(send_times, 2);

            target_v.x = traj_v(send_times, 0);
            target_v.y = traj_v(send_times, 1);
            target_v.z = traj_v(send_times, 2);
        }
    }
    
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    // 【订阅】无人机当前状态/位置/速度信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 20, pos_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/vel", 20, vel_cb);
    ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 20, att_cb);
    // 【发布】飞机姿态/拉力信息 坐标系:NED系
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float32>("/cmd/thrust", 20);
    ros::Publisher orientataion_pub = nh.advertise<geometry_msgs::Quaternion>("/cmd/orientation", 20);

    target_p_pub = nh.advertise<geometry_msgs::Vector3>("/target_p", 1);
    target_v_pub = nh.advertise<geometry_msgs::Vector3>("/target_v", 1);
    current_p_pub = nh.advertise<geometry_msgs::Vector3>("/current_p", 1);
    current_v_pub = nh.advertise<geometry_msgs::Vector3>("/current_v", 1);

    // 频率 [20Hz]
    ros::Rate rate(20.0);

    // 读取PID参数
    std::string paraadr("/home/ubuntu/catkin_mz/src/offb_posctl/src/param150_viconRot_tracking");
    if (param.readParam(paraadr.c_str()) == 0){
        std::cout<<"read config file error!"<<std::endl;
        return 0;
    }

    // log输出文件初始化
    logfile.open("/home/ubuntu/catkin_mz/src/offb_posctl/src/log.csv", std::ios::out);
    if (! logfile.is_open()){
        std::cout<<"log to file error!"<<std::endl;
        return 0;
    }


    // 设置POSITION环PID参数 比例参数 积分参数 微分参数
    PIDX.setPID(param.x_p, param.x_i, param.x_d);
    PIDY.setPID(param.y_p, param.y_i, param.y_d);
    PIDZ.setPID(param.z_p, param.z_i, param.z_d);
    std::cout << "param:x_p" << param.x_p << "param:x_i" << param.x_i <<std::endl;


    // 设置POSITION环积分上限 控制量最大值 误差死区
    PIDX.set_sat(2, 5, 0.01); //3
    PIDY.set_sat(2, 5, 0.01); //3
    PIDZ.set_sat(2, 5, 0.01);



    // 设置速度环PID参数 比例参数 积分参数 微分参数
    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(2, 4, 0); //3
    PIDVY.set_sat(2, 4, 0); //3
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

     while(ros::ok() && !if_pose_initialized)
    {
        ros::Duration(1).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("waitting for position initialization...");
    }
    ROS_INFO("pose_initialized!!");

    float x = att_drone.orientation.x;
    float y = att_drone.orientation.y;
    float z = att_drone.orientation.z;
    float w = att_drone.orientation.w;
    angle_receive = quaternion2euler(x, y, z, w);
    Yaw_Init = angle_receive.z;
    ROS_INFO_STREAM("Got Yaw_Init: " << Yaw_Init);

    angel_init = pos_drone.pose.orientation.y;

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

      // CHG trajectory
    target_p.x = param.pos_x;
    target_p.y = param.pos_y;
    target_p.z = 0.0;

    target_v.x = 0.0;
    target_v.y = 0.0;
    target_v.z = 0.0;

    ros::Timer timer = nh.createTimer(ros::Duration(planning_time_interval), trajectoryTimerCallback);


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();

        float cur_time = get_ros_time(begin_time);  // 当前时间
        pix_controller(cur_time);                   //控制程序

        if(!simulation_model)
        {
            if(current_state.mode == "OFFBOARD"){
                data_log(cur_time);                     //log输出
            }
        }

        std_msgs::Float32 data2pub;
        data2pub.data = thrust_target;
        thrust_pub.publish(data2pub);
        orientataion_pub.publish(orientation_target);

        // to show in rqt_plot
        target_p_pub.publish(target_p);
        target_v_pub.publish(target_v);

        current_p.x = pos_drone.pose.position.x;
        current_p.y = pos_drone.pose.position.y;
        current_p.z = pos_drone.pose.position.z;
        current_p_pub.publish(current_p);

        current_v.x = vel_drone.twist.linear.x;
        current_v.y = vel_drone.twist.linear.y;
        current_v.z = vel_drone.twist.linear.z;
        current_v_pub.publish(current_v);

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
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDX.start_intergrate_flag = true;
    PIDY.start_intergrate_flag = true;
    PIDZ.start_intergrate_flag = true;

    if(!simulation_model)
    {
        if(current_state.mode != "OFFBOARD"){
            PIDX.start_intergrate_flag = false;
            PIDY.start_intergrate_flag = false;
            PIDZ.start_intergrate_flag = false;
        }
    }
    
    //计算误差

    geometry_msgs::PoseStamped last_pose;
    get_last_pose(planning_time_interval, last_pose);

    float error_x = target_p_last.x - last_pose.pose.position.x;
    float error_y = target_p_last.y - last_pose.pose.position.y;
    float error_z = target_p_last.z - last_pose.pose.position.z;
    // float error_x = target_p.x - pos_drone.pose.position.x;
    // float error_y = target_p.y - pos_drone.pose.position.y;
    // float error_z = target_p.z - pos_drone.pose.position.z;
    std::cout << "target_p: x：" << target_p.x << "\ty：" << target_p.y << "\tz：" << target_p.z << std::endl;
    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;

    //传递误差
    PIDX.add_error(error_x, cur_time);
    PIDY.add_error(error_y, cur_time);
    PIDZ.add_error(error_z, cur_time);
    //计算输出
    PIDX.pid_output();
    PIDY.pid_output();
    PIDZ.pid_output();
    //计算指定速度误差
    // float vel_xd = PIDX.Output;
    // float vel_yd = PIDY.Output;
    // float vel_zd = PIDZ.Output;

    float vel_xd = PIDX.Output + target_v.x * 0.5;  //chg
    float vel_yd = PIDY.Output + target_v.y * 0.5;  //chg
    float vel_zd = PIDZ.Output + target_v.z * 0.4;  //chg

    vel_target.x = vel_xd;
    vel_target.y = vel_yd;
    vel_target.z = vel_zd;


    // //计算误差
    // float error_x = param.pos_x - pos_drone.pose.position.x;
    // float error_y = param.pos_y - pos_drone.pose.position.y;
    // float error_z = param.pos_z - pos_drone.pose.position.z;
    // std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    // //计算指定速度误差
    // float vel_xd = param.x_p * error_x;
    // float vel_yd = param.y_p * error_y;
    // float vel_zd = param.z_p * error_z;
    // vel_target.x = vel_xd;
    // vel_target.y = vel_yd;
    // vel_target.z = vel_zd;

//速 度 环
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDVX.start_intergrate_flag = true;
    PIDVY.start_intergrate_flag = true;
    PIDVZ.start_intergrate_flag = true;

    if(!simulation_model)
    {
        if(current_state.mode != "OFFBOARD"){
            PIDVX.start_intergrate_flag = false;
            PIDVY.start_intergrate_flag = false;
            PIDVZ.start_intergrate_flag = false;
        }
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
    Vector2f mat_temp(PIDVX.Output,PIDVY.Output);       //赋值到期望推力和姿态
    Vector2f euler_temp= 1/9.8 * A_yaw.inverse() * mat_temp;
    angle_target.x = euler_temp[0];
    angle_target.y = euler_temp[1];
    // angle_target.z = Yaw_Locked + Yaw_Init;


    angel_vicon = pos_drone.pose.orientation.y;
    std::cout << "Euler_vicon: yaw：" << angel_vicon * 60 << std::endl;
    // std::cout << "Euler_vicon: roll：" << pos_drone.pose.orientation.x << "\tpitch：" << pos_drone.pose.orientation.y << "\tyaw：" << pos_drone.pose.orientation.z << std::endl;
    angle_deviation = angel_vicon - angel_init;
    if (fabs(angle_deviation) < 0.05)
    {
        angle_deviation = 0;
    }


    angle_target.z = Yaw_Init + Yaw_Locked + angle_deviation;
    orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
    thrust_target = (float)(-0.0 + 0.05 * (9.8 + PIDVZ.Output));   //目标推力值

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
void data_log(float cur_time)
{
    logfile <<cur_time<<","<<param.pos_x <<","<<param.pos_y <<","<<param.pos_z <<","                           //set_pos
        <<pos_drone.pose.position.x <<","<<pos_drone.pose.position.y <<","<<pos_drone.pose.position.z <<","    //uav_pos
        <<vel_target.x <<","<<vel_target.y <<","<<vel_target.z <<","                                           //set_vel
        <<vel_drone.twist.linear.x <<","<<vel_drone.twist.linear.y <<","<<vel_drone.twist.linear.z <<","       //uav_vel
        <<angle_target.x  <<","<<angle_target.y  <<","<<angle_target.z  <<","                                  //set_att
        <<angle_receive.x <<","<<angle_receive.y <<","<<angle_receive.z <<","                                  //uav_att
        <<thrust_target<<std::endl;

}