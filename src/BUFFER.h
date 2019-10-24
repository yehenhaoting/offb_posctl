//
// Created by ubuntu on 19-5-9.
//

#ifndef OFFB_POSCTL_BUFFER_H
#define OFFB_POSCTL_BUFFER_H
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

class BUFFER{

    std::vector <geometry_msgs::Point  > POSbuffer_list;
    std::vector <geometry_msgs::Vector3  > VELbuffer_list;


public:
    int delay_step;

    geometry_msgs::Point pos_buffer(geometry_msgs::Point data_pos);
    geometry_msgs::Vector3 vel_buffer(geometry_msgs::Vector3 data_vel);

};

#endif //OFFB_POSCTL_BUFFER_H
