#pragma once
#include "globals.h"
class bSpline
{
private:
    /**
     * bSpline properties
     */
    struct properties
    {
        // degree of freedom
        unsigned short k = 7;
        // number of points
        unsigned short num_points = ;
        unsigned short n = 
        // knot vector
        float U = ;
    }properties;
    /**
     * Control points
     */
    struct control_points
    {
        // float AB = [];
        // float HIP = [];
        // float KNEE = [];
        // float AB_velo = [];
        // float HIP_velo = [];
        // float KNEE_velo = [];
    }control_points;
public:
    /**
     * Constructor
     * @param x pointer to operational trajectory in x direction
     * @param y pointer to operational trajectory in y direction
     * @param z pointer to operational trajectory in z direction
     * @param time pointer to operational trajectories' time vector
     * @param size number of elements in the vector
     * Note that x y z are segemented by time
     */
    bSpline(float *x, float *y, float *z, float *time, unsigned short size);

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
     * Calculate bSpline control points
     */
    void calc_ctrlPoint();
    /**
     * Calculate bSpline basis function
     */
    void calc_base();
    /**
     * Calculate bSpline ratio function
     */
    void calc_ratio();

};