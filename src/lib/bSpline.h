#pragma once
#include "globals.h"
class bSpline
{
private:
public:
        // degree of freedom
        unsigned short k_ = 7;
        // number of operational space points
        unsigned short num_points_;
        // knot vector
        float U[70];
        // time vector
        float time_[60];
        // control points
        float ctrl_point_[60];
public:
    /**
     * Constructor
     * @param ctrl_point pointer to control points
     * @param time pointer to operational trajectories' time array
     * @param cp_size number of control points
     * @param 
     * Note that x y z are segemented by time
     */
    bSpline(float *ctrl_point, float *time, unsigned short num_points);

    /**
     * Get bSpline point given time input
     * @param time pointer to the time stamp of the bSpline point. Needs to be within the intialized time range
     */
    float get_point(float *time);

private:
    /**
     * Calculate the knot vector
     */
    void calc_knot();
    /**
     * Calculate bSpline basis function
     */
    float calc_base(float *time_stamp, unsigned short count, unsigned short k);

};