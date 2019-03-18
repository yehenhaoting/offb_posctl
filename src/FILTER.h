//
// Created by mz on 19-3-18.
//

#ifndef OFFB_POSCTL_FILTER_H
#define OFFB_POSCTL_FILTER_H

#include <vector>


class FILTER{

public:
    FILTER();
    float delta_time;                  //时间间隔dt
    float filter_data;
    float Output_filter;

    bool start_intergrate_flag;        //是否积分标志[进入offboard(启控)后,才开始积分]

    std::vector <std::pair<float, float> > filter_list;

    float satfunc(float data, float Max, float Thres);
    bool filter_input(float data2fliter, float curtime);
    void filter_output();

};



#endif //OFFB_POSCTL_FILTER_H
