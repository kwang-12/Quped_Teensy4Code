#pragma once
#include "globals.h"
class Traj
{
private:
    /**
     * Leg kinematic properties
     */
    struct leg_properties
    {
        float d2 = 0.092; //ab to hip(knee) offset, [m]
        float a2 = 0.218; //hip length, [m]
        float a3 = 0.230; //knee length, [m]
    } leg_properties;

public:
    /**
     * Operational-space trajectories array
     */
    struct op_traj
    {
        float x[21];
        float y[21];
        float z[21];
        float time_vector[21];
        float ini_velo[3];
        float end_velo[3];
    } op_traj;
    /**
     * Joint-space trajectories array
     */
    struct jt_traj
    {
        float ab[21];
        float hip[21];
        float knee[21];
        float ini_velo[3];
        float end_velo[3];
    } jt_traj;

    /**
     * Constructor
     * @param x pointer to operational trajectory in x direction
     * @param y pointer to operational trajectory in y direction
     * @param z pointer to operational trajectory in z direction
     * @param time pointer to operational trajectories' time vector
     * Note that x y z are segemented by time
     */
    Traj(float *x, float *y, float *z, float *time);

private:
    /**
     * Calculates the initial operational-space velocities as constraints 
     * for solving the joint space trajectories bspline
     */
    void calc_op_velo();
    /**
     * Calculates the joint-space position trajectories by solving inverse 
     * kinematics for each operational-space positions
     */
    void calc_jt_traj();
    /**
     * Inverse kinematics calculation
     */
    float* iK(float *current_ab, float *current_hip, float *current_knee,
              float *desired_x, float *desired_y, float *desired_z);
    /**
     * Jacbian calculation
     */
    
};