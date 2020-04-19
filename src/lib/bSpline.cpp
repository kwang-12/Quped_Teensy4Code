#include "bSpline.h"
bSpline::bSpline(float *ctrl_point, float *time)
{
    for (int tick=0; tick<27; tick++)
    {
        if (tick<21)
        {
            time_[tick] = time[tick];
        }
        ctrl_point_[tick] = ctrl_point[tick];
    }
    calc_knot();
}
void bSpline::calc_knot()
{
    for(int tick=0; tick<35; tick++)   
    {
        if (tick<k+1)
        {
            U[tick] = 0;
        }
        else if (tick>num_points+k-1)
        {
            U[tick] = time_[20];
        }
        else
        {
            U[tick] = U[tick-1] + time_[tick-k]-time_[tick-k-1];
        }
    }
}
float bSpline::calc_base(float *time_stamp, unsigned short count, unsigned short k)
{
    float pos = 0.0;
    if (k == 0)
    {
        if (*time_stamp != U[35])
        {
            if (U[count] <= *time_stamp && *time_stamp < U[count+1])
            {
                pos=1.0;
            }
            else
            {
                pos=0.0;
            }
        }
        else
        {
            if (U[count] <= *time_stamp && *time_stamp <= U[count+1])
            {
                pos=1.0;
            }
            else
            {
                pos=0.0;
            }
        }
    }
    else
    {
        float a = 0.0;
        float b = 0.0;
        if(U[count+k]-U[count]!=0)
        {
            a = (*time_stamp-U[count])/(U[count+k]-U[count]);
        }
        if(U[count+k+1]-U[count+1]!=0)
        {
            b = (U[count+k+1]-*time_stamp)/(U[count+k+1]-U[count+1]);
        }
        pos = a*calc_base(time_stamp,count,k-1)+b*calc_base(time_stamp,count+1,k-1);
    }
    return pos;
}
float bSpline::get_point(float *timestamp)
{
    float pos = 0.0;
    for (unsigned short tick=0; tick<n+1; tick++)
    {
        pos = pos+calc_base(timestamp, tick, k)*ctrl_point_[tick];
    }
    return pos;
}