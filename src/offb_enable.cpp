//
// Created by zm on 19-4-8.
// To enable px4 offboard control.
// input: Target Attitude & thrust
// output: setpoint_raw attitude
//

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

/*The mavros_msgs package contains all of the custom messages required to
 * operate services and topics provided by the MAVROS package.
 * All services and topics as well as their corresponding message types
 * are documented in the mavros wiki.*/
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

mavros_msgs::State current_state;
mavros_msgs::AttitudeTarget targetattitude;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void thrust_cb(const std_msgs::Float32::ConstPtr& msg){
    targetattitude.thrust = msg->data;      //将接收到的油门信息放入targetattitude中
}
void orientation_cb(const geometry_msgs::Quaternion::ConstPtr& msg){
    targetattitude.orientation = *msg;      //将接收到的姿态信息放入targetattitude中
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_enable");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber thrust_sub = nh.subscribe<std_msgs::Float32>
            ("/cmd/thrust", 10, thrust_cb);
    ros::Subscriber orientation_sub = nh.subscribe<geometry_msgs::Quaternion>
            ("/cmd/orientation", 10, orientation_cb);

    ros::Publisher attitude_set_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);

    /*通过参数服务器的方式进行解锁、控制进行offboard模式*/
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

//    the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

//     wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not connected for sim");
    }


    targetattitude.orientation.x = 0;
    targetattitude.orientation.y = 0;
    targetattitude.orientation.z = 0;
    targetattitude.orientation.w = 1;
    targetattitude.thrust = 0.1;
    //send a few setpoints before starting
    for(int i = 5; ros::ok() && i > 0; --i){
        attitude_set_pub.publish(targetattitude);
        ros::spinOnce();
        rate.sleep();
    }

    //set the custom mode to OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //执行解锁和设置offboard控制模式
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){          //or offb_set_mode.response.success
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //发送目标姿态
        attitude_set_pub.publish(targetattitude);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}