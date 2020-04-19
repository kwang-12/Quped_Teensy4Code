#pragma once
#include "globals.h"
class bSpline
{
private:
public:
        // degree of freedom
        unsigned short k = 7;
        // number of points
        unsigned short num_points = 21;
        unsigned short n = 26;
        // knot vector
        float U[35];
        // time vector
        float time_[21];
        // control points
        float ctrl_point_[27];
public:
    /**
     * Constructor
     * @param ctrl_point pointer to control points
     * @param time pointer to operational trajectories' time array
     * Note that x y z are segemented by time
     */
    bSpline(float *ctrl_point, float *time);

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