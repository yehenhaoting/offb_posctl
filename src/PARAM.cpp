//
// Created by zm on 19-4-14.
//

#include "PARAM.h"
#include <iostream>
#include <ros/ros.h>


bool PARAM::readParam() {



    if( !(
        ros::param::get("offb_posctl/POS_X", pos_x) &&
        ros::param::get("offb_posctl/POS_Y", pos_y) &&
        ros::param::get("offb_posctl/POS_Z", pos_z) &&

        ros::param::get("offb_posctl/MC_X_P", x_p) &&
        ros::param::get("offb_posctl/MC_Y_P", y_p) &&
        ros::param::get("offb_posctl/MC_Z_P", z_p) &&

        ros::param::get("offb_posctl/MC_X_I", x_i) &&
        ros::param::get("offb_posctl/MC_Y_I", y_i) &&
        ros::param::get("offb_posctl/MC_Z_I", z_i) &&

        ros::param::get("offb_posctl/MC_X_D", x_d) &&
        ros::param::get("offb_posctl/MC_Y_D", y_d) &&
        ros::param::get("offb_posctl/MC_Z_D", z_d) &&

        ros::param::get("offb_posctl/MC_VX_P", vx_p) &&
        ros::param::get("offb_posctl/MC_VY_P", vy_p) &&
        ros::param::get("offb_posctl/MC_VZ_P", vz_p) &&

        ros::param::get("offb_posctl/MC_VX_I", vx_i) &&
        ros::param::get("offb_posctl/MC_VY_I", vy_i) &&
        ros::param::get("offb_posctl/MC_VZ_I", vz_i) &&

        ros::param::get("offb_posctl/MC_VX_D", vx_d) &&
        ros::param::get("offb_posctl/MC_VY_D", vy_d) &&
        ros::param::get("offb_posctl/MC_VZ_D", vz_d)) )
    {
        std::cout << "parameter file err" << std::endl;
        return false;
    }

    std::cout << "read config file successfully!"<<std::endl;

    return true;
}