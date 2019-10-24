//
// Created by ubuntu on 19-5-9.
//

#include "BUFFER.h"
#include <time.h>
#include <cmath>
#include <queue>
#include <vector>
#include <iostream>

geometry_msgs::Point BUFFER::pos_buffer(geometry_msgs::Point data_pos)
{
    geometry_msgs::Point data_out;

    if(POSbuffer_list.size() < delay_step){
        POSbuffer_list.push_back(data_pos);
        data_out = data_pos;
    }
    else{
        POSbuffer_list.erase(POSbuffer_list.begin());
        POSbuffer_list.push_back(data_pos);
        data_out = POSbuffer_list.begin().operator*();
        std::cout<<"data in:"<<data_pos<<"\tdata out:"<<data_out<<std::endl;
    }
    return data_out;
}

geometry_msgs::Vector3 BUFFER::vel_buffer(geometry_msgs::Vector3 data_vel)
{
    geometry_msgs::Vector3 data_out2;

    if(VELbuffer_list.size() < delay_step){
        VELbuffer_list.push_back(data_vel);
        data_out2 = data_vel;
    }
    else{
        VELbuffer_list.erase(VELbuffer_list.begin());
        VELbuffer_list.push_back(data_vel);
        data_out2 = VELbuffer_list.begin().operator*();
//        std::cout<<"data in:"<<data_vel<<"\tdata out:"<<data_out2<<std::endl;
    }
    return data_out2;
}