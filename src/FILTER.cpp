//
// Created by zm on 19-3-11.
//

#include "FILTER.h"
#include <time.h>
#include <cmath>
#include <queue>
#include <vector>


using namespace std;

FILTER::FILTER(){
    filter_list.push_back(make_pair(0.0f, 0.0f));
    filter_data = 0;
    Output_filter = 0;
    start_filter_flag = false;

}

float FILTER::satfunc(float data, float Max, float Thres)
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

bool FILTER::filter_input(float data2fliter, float curtime)
{
    if(filter_list.size() == 1)
    {
        delta_time = curtime;
    }
    else{
        delta_time = curtime - filter_list.rbegin()->first;
    }

    filter_data = data2fliter;
    if(filter_list.size() < 10){
        filter_list.push_back(make_pair(curtime, filter_data));
    }
    else{
        vector<pair<float, float > > ::iterator fil_iter = filter_list.begin();
        filter_list.erase(fil_iter);
        std::pair<float, float > temp_iter(curtime, filter_data);
        filter_list.push_back(temp_iter);
    }
    return true;
}

void FILTER::filter_output()
{
    if(filter_list.size() < 10 || ! start_filter_flag){
        Output_filter = 0;
    }
    else{
        vector<pair<float, float> >::iterator filter_k;
        float filter_sum = 0;
        for(filter_k = filter_list.begin(); filter_k != filter_list.end(); ++ filter_k){
            filter_sum = filter_sum + filter_k->second;
        }
        Output_filter = filter_sum * delta_time/(filter_list.back().first - filter_list.front().first);

    }
}