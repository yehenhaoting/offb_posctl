//
// Created by zm on 19-4-25.
//

#ifndef OFFB_POSCTL_DOB_H
#define OFFB_POSCTL_DOB_H

#include <vector>


class DOB_DATA {
public:
    float cur_time;
    float cur_vel;
    float des_acc;
};


class DOB {
    float delta_time;
    DOB_DATA DOB_data_in;
    std::vector <DOB_DATA> DOB_list; // [1st time, 2nd vel, 3rd acc]


public:
    DOB();
    void add_data(float curtime, float curvel, float desacc);
    float dob_output();

};

#endif //OFFB_POSCTL_DOB_H
