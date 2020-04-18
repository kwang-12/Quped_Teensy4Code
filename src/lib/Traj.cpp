#include "Traj.h"
Traj::Traj(float *x, float *y, float *z, float *time)
{
    for (int tick = 0; tick < 21; tick++)
    {
        op_traj.x[tick] = x[tick];
        op_traj.y[tick] = y[tick];
        op_traj.z[tick] = z[tick];
        op_traj.time_vector[tick] = time[tick];
    }
    op_traj.ini_velo[0] = (op_traj.x[1] - op_traj.x[0]) / (op_traj.time_vector[1] - op_traj.time_vector[0]);
    op_traj.ini_velo[1] = (op_traj.y[1] - op_traj.y[0]) / (op_traj.time_vector[1] - op_traj.time_vector[0]);
    op_traj.ini_velo[2] = (op_traj.z[1] - op_traj.z[0]) / (op_traj.time_vector[1] - op_traj.time_vector[0]);
    op_traj.end_velo[0] = (op_traj.x[20] - op_traj.x[19]) / (op_traj.time_vector[20] - op_traj.time_vector[19]);
    op_traj.end_velo[1] = (op_traj.y[20] - op_traj.y[19]) / (op_traj.time_vector[20] - op_traj.time_vector[19]);
    op_traj.end_velo[2] = (op_traj.z[20] - op_traj.z[19]) / (op_traj.time_vector[20] - op_traj.time_vector[19]);
}

// Note that the function does not check for no solution case
float Traj::*iK(float *current_ab, float *current_hip, float *current_knee,
                float *desired_x, float *desired_y, float *desired_z)
{
    static float array[3] = {0.0, 0.0, 0.0};
    // determine knee target position
    float c3 = (pow(*desired_x, (float)2.0) + pow(*desired_y, (float)2.0) + pow(*desired_z, (float)2.0) - pow(leg_properties.d2, (float)2.0) - pow(leg_properties.a2, float(2.0)) - pow(leg_properties.a3, 2)) / (2 * leg_properties.a2 * leg_properties.a3);
    if (pow(c3, (float)2.0) <= (float)1.0)
    {
        float theta_3_1 = atan2(sqrt(1 - pow(c3, (float)2.0)), c3);
        float theta_3_2 = atan2(-sqrt(1 - pow(c3, (float)2.0)), c3);
        if (abs(theta_3_1 - *current_knee) >= abs(theta_3_2 - *current_knee))
        {
            array[2] = theta_3_2;
        }
        else
        {
            array[2] = theta_3_1;
        }
    }
    // determine hip target position
    float a = leg_properties.a3 * sin(array[2]);
    float b = -leg_properties.a3 * cos(array[2]) - leg_properties.a2;
    float c = *desired_z;
    if (pow(a, (float)2.0) + pow(b, (float)2.0) - pow(c, (float)2.0) >= (float)0.0)
    {
        float theta_2_1 = atan2(sqrt(pow(a, (float)2.0) + pow(b, (float)2.0) - pow(c, (float)2.0)), c) + atan2(b, a);
        float theta_2_2 = atan2(-sqrt(pow(a, (float)2.0) + pow(b, (float)2.0) - pow(c, (float)2.0)), c) + atan2(b, a);
        if (abs(theta_2_1-*current_hip) >= abs(theta_2_2 - *current_hip))
        {
            array[1] = theta_2_2;
        }
        else
        {
            array[1] = theta_2_1;
        }
    }
    // determine ab target position
    a = leg_properties.a2 * cos(array[1]) + leg_properties.a3*cos(array[1]-array[2]);
    b = -leg_properties.d2;
    c = *desired_x;
    if (pow(a, (float)2.0) + pow(b, (float)2.0) - pow(c, (float)2.0) >= (float)0.0)
    {
        float theta_1_1 = atan2(sqrt(pow(a, (float)2.0) + pow(b, (float)2.0) - pow(c, (float)2.0)), c) + atan2(b, a);
        float theta_1_2 = atan2(-sqrt(pow(a, (float)2.0) + pow(b, (float)2.0) - pow(c, (float)2.0)), c) + atan2(b, a);
        if (abs(theta_1_1-*current_ab) >= abs(theta_1_2-*current_ab))
        {
            array[0] = theta_1_2;
        }
        else
        {
            array[0] = theta_1_1;
        }
    }
    return array;
}