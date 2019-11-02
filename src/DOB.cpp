//
// Created by zm on 19-4-25.
//

#include "DOB.h"
#include <vector>
#include <iostream>
#include <queue>
#include<iomanip>

DOB::DOB() {
    DOB_data_in = {0.0f, 0.0f, 0.0f};
    DOB_list.push_back(DOB_data_in);
}

void DOB::add_data(float curtime, float curvel, float desacc) {
    DOB_data_in.cur_time = curtime;
    DOB_data_in.cur_vel = curvel;
    DOB_data_in.des_acc = desacc;

    if(DOB_list.size() == 1){
        delta_time = curtime;
    } else{
        delta_time = curtime - DOB_list.rbegin()->cur_time;
    }

    if(DOB_list.size() < 10){
        DOB_list.push_back(DOB_data_in);
    } else{
        std::vector<DOB_DATA>::iterator k_beg = DOB_list.begin();
        DOB_list.erase(k_beg);
        DOB_list.push_back(DOB_data_in);
    }
}

float DOB::dob_output() {

    if(DOB_list.size() < 10){

        return 0.0f;
    }

    else{
        std::vector<DOB_DATA>::iterator dob_k;
        float acc_sum = 0.0f;
        for(dob_k = DOB_list.begin(); dob_k != DOB_list.end(); ++ dob_k){
            acc_sum = acc_sum + dob_k->des_acc;
        }
        float data2return = ( acc_sum * delta_time - (DOB_list.back().cur_vel - DOB_list.front().cur_vel) ) / (DOB_list.back().cur_time - DOB_list.front().cur_time);

        float acc_realdata = (DOB_list.back().cur_vel - DOB_list.front().cur_vel);
        float acc_nominaldata = acc_sum * delta_time;
        float acc_disdata = data2return;
//        std::cout <<std::fixed << std::setprecision(3) << "acc_realdata:\t" << acc_realdata << std::endl<<"acc_nominaldata:\t" << acc_nominaldata <<std::endl<< "acc_disdata:\t" << acc_disdata << std::endl;

        return data2return;
    }

}