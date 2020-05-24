#include "kinematics.h"

void leg_forwardKinematics(float ab_deg_now, float hip_deg_now, float knee_deg_now,
                           float &x_calculated, float &y_calculated, float &z_calculated,
                           char leg_choice)
{
    float a2 = DIMENSION_A2;
    float a3 = DIMENSION_A3;
    float d2 = DIMENSION_D2;
    switch (leg_choice)
    {
    case FRONT_LEFT_LEG:
    {
        x_calculated = -d2 * sin(ab_deg_now) + cos(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        y_calculated = d2 * cos(ab_deg_now) + sin(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        z_calculated = -a3 * sin(hip_deg_now - knee_deg_now) - a2 * sin(hip_deg_now);
    }
    break;
    case FRONT_RIGHT_LEG:
    {
        x_calculated = d2 * sin(ab_deg_now) + cos(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        y_calculated = -d2 * cos(ab_deg_now) + sin(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        z_calculated = a3 * sin(hip_deg_now - knee_deg_now) + a2 * sin(hip_deg_now);
    }
    break;
    case BACK_LEFT_LEG:
    {
        x_calculated = d2 * sin(ab_deg_now) + cos(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        y_calculated = -d2 * cos(ab_deg_now) + sin(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        z_calculated = a3 * sin(hip_deg_now - knee_deg_now) + a2 * sin(hip_deg_now);
    }
    break;
    case BACK_RIGHT_LEG:
    {
        x_calculated = -d2 * sin(ab_deg_now) + cos(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        y_calculated = d2 * cos(ab_deg_now) + sin(ab_deg_now) * (a3 * cos(hip_deg_now - knee_deg_now) + a2 * cos(hip_deg_now));
        z_calculated = -a3 * sin(hip_deg_now - knee_deg_now) - a2 * sin(hip_deg_now);
    }
    break;
    }
}

void leg_inverseKinematics(float ab_deg_now, float hip_deg_now, float knee_deg_now,
                           float x_desired, float y_desired, float z_desired,
                           float &ab_deg_desired, float &hip_deg_desired, float &knee_deg_desired,
                           char leg_choice)
{
    float a2 = DIMENSION_A2;
    float a3 = DIMENSION_A3;
    float d2 = DIMENSION_D2;
    float cost[8];
    float theta_1[8];
    float theta_2[8];
    float theta_3[8];
    int loop_counter = 0;
    switch (leg_choice)
    {
    case FRONT_LEFT_LEG:
    {
        for (int i = 0; i <= 1; i++)
        {
            for (int j = 0; j <= 1; j++)
            {
                for (int k = 0; k <= 1; k++)
                {
                    // determine theta_3
                    float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(d2, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
                    float square_c3 = pow(c3, 2);
                    if (square_c3 <= 1)
                    {
                        if (i == 0)
                        {
                            theta_3[loop_counter] = atan2(sqrt(1 - square_c3), c3);
                        }
                        else
                        {
                            theta_3[loop_counter] = atan2(-sqrt(1 - square_c3), c3);
                        }
                    }
                    // determine theta_2
                    float a = a3 * sin(theta_3[loop_counter]);
                    float b = -a3 * cos(theta_3[loop_counter]) - a2;
                    float c = z_desired;
                    float arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (j == 0)
                        {
                            theta_2[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_2[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    // determine theta_1
                    a = a2 * cos(theta_2[loop_counter]) + a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                    b = -d2;
                    c = x_desired;
                    arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (k == 0)
                        {
                            theta_1[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_1[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    loop_counter++;
                }
            }
        }
    }
    break;
    case FRONT_RIGHT_LEG:
    {
        for (int i = 0; i <= 1; i++)
        {
            for (int j = 0; j <= 1; j++)
            {
                for (int k = 0; k <= 1; k++)
                {
                    // determine theta_3
                    float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(d2, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
                    float square_c3 = pow(c3, 2);
                    if (square_c3 <= 1)
                    {
                        if (i == 0)
                        {
                            theta_3[loop_counter] = atan2(sqrt(1 - square_c3), c3);
                        }
                        else
                        {
                            theta_3[loop_counter] = atan2(-sqrt(1 - square_c3), c3);
                        }
                    }
                    // determine theta_2
                    float a = -a3 * sin(theta_3[loop_counter]);
                    float b = a3 * cos(theta_3[loop_counter]) + a2;
                    float c = z_desired;
                    float arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (j == 0)
                        {
                            theta_2[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_2[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    // determine theta_1
                    a = a2 * cos(theta_2[loop_counter]) + a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                    b = d2;
                    c = x_desired;
                    arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (k == 0)
                        {
                            theta_1[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_1[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    loop_counter++;
                }
            }
        }
    }
    break;
    case BACK_LEFT_LEG:
    {
        for (int i = 0; i <= 1; i++)
        {
            for (int j = 0; j <= 1; j++)
            {
                for (int k = 0; k <= 1; k++)
                {
                    // determine theta_3
                    float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(d2, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
                    float square_c3 = pow(c3, 2);
                    if (square_c3 <= 1)
                    {
                        if (i == 0)
                        {
                            theta_3[loop_counter] = atan2(sqrt(1 - square_c3), c3);
                        }
                        else
                        {
                            theta_3[loop_counter] = atan2(-sqrt(1 - square_c3), c3);
                        }
                    }
                    // determine theta_2
                    float a = -a3 * sin(theta_3[loop_counter]);
                    float b = a3 * cos(theta_3[loop_counter]) + a2;
                    float c = z_desired;
                    float arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (j == 0)
                        {
                            theta_2[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_2[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    // determine theta_1
                    a = a2 * cos(theta_2[loop_counter]) + a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                    b = d2;
                    c = x_desired;
                    arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (k == 0)
                        {
                            theta_1[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_1[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    loop_counter++;
                }
            }
        }
    }
    break;
    case BACK_RIGHT_LEG:
    {
        for (int i = 0; i <= 1; i++)
        {
            for (int j = 0; j <= 1; j++)
            {
                for (int k = 0; k <= 1; k++)
                {
                    // determine theta_3
                    float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(d2, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
                    float square_c3 = pow(c3, 2);
                    if (square_c3 <= 1)
                    {
                        if (i == 0)
                        {
                            theta_3[loop_counter] = atan2(sqrt(1 - square_c3), c3);
                        }
                        else
                        {
                            theta_3[loop_counter] = atan2(-sqrt(1 - square_c3), c3);
                        }
                    }
                    // determine theta_2
                    float a = a3 * sin(theta_3[loop_counter]);
                    float b = -a3 * cos(theta_3[loop_counter]) - a2;
                    float c = z_desired;
                    float arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (j == 0)
                        {
                            theta_2[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_2[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    // determine theta_1
                    a = a2 * cos(theta_2[loop_counter]) + a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                    b = -d2;
                    c = x_desired;
                    arg = pow(a, 2) + pow(b, 2) - pow(c, 2);
                    if (arg >= 0)
                    {
                        if (k == 0)
                        {
                            theta_1[loop_counter] = atan2(sqrt(arg), c) + atan2(b, a);
                        }
                        else
                        {
                            theta_1[loop_counter] = atan2(-sqrt(arg), c) + atan2(b, a);
                        }
                    }
                    loop_counter++;
                }
            }
        }
    }
    break;
    }
    // calculate the cost of each calculated desired location, the set with the lowest cost is chosen as the output
    // cost function is defined as:
    // cost = EuclideanNorm(desired_cartesian_pos - calculated_cartesian_pos) * 0.8 + EuclideanNorm(current_joint_position - calculated_joint_position) * 0.2
    for (int i = 0; i <= 7; i++)
    {
        float x,y,z;
        leg_forwardKinematics(ab_deg_now, hip_deg_now, knee_deg_now, x, y, z, leg_choice);
        float opSpace_cost = sqrt(pow(x-x_desired,2)+pow(y-y_desired,2)+pow(z-z_desired,2));
        float jtSpace_cost = sqrt(pow(theta_1[i]-ab_deg_now,2)+pow(theta_2[i]-hip_deg_now,2)+pow(theta_3[i]-knee_deg_now,2));
        cost[i] = 0.8*opSpace_cost + 0.2*jtSpace_cost;
        // Serial.print(theta_1[i] / PI_math * 180);
        // Serial.print(" ");
        // Serial.print(theta_2[i] / PI_math * 180);
        // Serial.print(" ");
        // Serial.println(theta_3[i] / PI_math * 180);
    }
    loop_counter = 0;
    float smallest_cost = 100;
    for (int i = 0; i <= 7; i++)
    {
        if (cost[i] < smallest_cost)
        {
            smallest_cost = cost[i];
            loop_counter = i;
        }
    }
    ab_deg_desired = theta_1[loop_counter];
    hip_deg_desired = theta_2[loop_counter];
    knee_deg_desired = theta_3[loop_counter];
}