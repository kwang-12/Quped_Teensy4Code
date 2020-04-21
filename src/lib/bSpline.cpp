#include "bSpline.h"
bSpline::bSpline(float *ctrl_point, float *time, unsigned short num_points, unsigned short dof)
{
    num_points_ = num_points;
    k_ = dof;
    for (int tick=0; tick<num_points_+k_-1; tick++)
    {
        // Serial.print("tick: ");
        // Serial.print(tick);
        if (tick<num_points_)
        {
            time_[tick] = time[tick];
        // Serial.print(" time: ");
        //     Serial.print(time_[tick],4);
        }
        ctrl_point_[tick] = ctrl_point[tick];
        // Serial.print(" cp: ");
        //     Serial.println(ctrl_point_[tick],4);
    }
    calc_knot();
}
void bSpline::calc_knot()
{
    // Serial.println("U: ");
    for(int tick=0; tick<num_points_+k_*2; tick++)   
    {
        if (tick<k_+1)
        {
            U[tick] = 0;
        }
        else if (tick>num_points_+k_-1)
        {
            U[tick] = time_[num_points_-1];
        }
        else
        {
            U[tick] = U[tick-1] + time_[tick-k_]-time_[tick-k_-1];
        }
        // Serial.print(tick);
        // Serial.print(' ');
        // Serial.println(U[tick],4);
    }
}
float bSpline::calc_base(float *time_stamp, unsigned short count, unsigned short k)
{
    float pos = 0.0;
    if (k == 0)
    {
        if (*time_stamp != U[num_points_+2*k_-1])
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
    int boundary_val = 0;
    for (unsigned short tick=0; tick<num_points_+k_-1; tick++)
    {
        pos = pos+calc_base(timestamp, tick, k_)*ctrl_point_[tick];
    }
    return pos;
}