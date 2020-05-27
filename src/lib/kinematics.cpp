#include "kinematics.h"

void leg_forwardKinematics_pos(float ab_deg_now, float hip_deg_now, float knee_deg_now,
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

bool leg_inverseKinematics_pos(float ab_deg_now, float hip_deg_now, float knee_deg_now,
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
    int index[8];
    bool in_workspace_[8] = {true, true, true, true, true, true, true, true};
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
                    else
                    {
                        in_workspace_[loop_counter] = false;
                    }
                    // determine theta_2
                    if (in_workspace_[loop_counter] == true)
                    {
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
                        else
                        {
                            in_workspace_[loop_counter] = false;
                        }
                        // determine theta_1
                        if (in_workspace_[loop_counter] == true)
                        {
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
                            else
                            {
                                in_workspace_[loop_counter] = false;
                            }
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
    
    // exclude the useless calculated joint positions that are not in the workspace
    loop_counter = 0;
    for (int i = 0; i<= 7; i++)
    {
        if (in_workspace_[i] == true)
        {
            index[loop_counter] = i;
            loop_counter++;
        }
    }
    int num_valid = loop_counter;
    if (num_valid > 0)
    {
        // calculate the cost of the valid calculated desired location, the set with the lowest cost is chosen as the output
        // cost function is defined as:
        // cost = EuclideanNorm(desired_cartesian_pos - calculated_cartesian_pos) * 0.8 + EuclideanNorm(current_joint_position - calculated_joint_position) * 0.2
        for (int i = 0; i <= num_valid-1; i++)
        {
            float x, y, z;
            leg_forwardKinematics_pos(theta_1[index[i]], theta_2[index[i]] , theta_3[index[i]] , x, y, z, leg_choice);
            float opSpace_cost = 0.99 * sqrt(pow(x - x_desired, 2) + pow(y - y_desired, 2) + pow(z - z_desired, 2));
            float jtSpace_cost = 0.01 * sqrt(pow(theta_1[index[i]] - ab_deg_now, 2) + pow(theta_2[index[i]] - hip_deg_now, 2) + pow(theta_3[index[i]] - knee_deg_now, 2));
            cost[i] = opSpace_cost + jtSpace_cost;

            Serial.print(theta_1[index[i]]/PI_math*180);
            Serial.print(" ");
            Serial.print(theta_2[index[i]]/PI_math*180);
            Serial.print(" ");
            Serial.println(theta_3[index[i]]/PI_math*180);

            Serial.print(ab_deg_now/PI_math*180);
            Serial.print(" ");
            Serial.print(hip_deg_now/PI_math*180);
            Serial.print(" ");
            Serial.println(knee_deg_now/PI_math*180);

            Serial.print(opSpace_cost,4);
            Serial.print(" ");
            Serial.print(jtSpace_cost,4);
            Serial.print(" ");
            Serial.println(cost[i],4);
            Serial.println("-----------");
        }
        loop_counter = 0;
        float smallest_cost = 100;
        for (int i = 0; i <= num_valid-1; i++)
        {
            if (cost[i] < smallest_cost)
            {
                smallest_cost = cost[i];
                loop_counter = i;
            }
        }
        ab_deg_desired = theta_1[index[loop_counter]];
        hip_deg_desired = theta_2[index[loop_counter]];
        knee_deg_desired = theta_3[index[loop_counter]];
        return true;
    }
    else
    {
        return false;
    }
    
}

bool calc_posture_joint_pos(float yaw_desired, float pitch_desired, float roll_desired,
                            float forward_delta, float horizontal_delta, float vertical_delta,
                            float &FL_end_x, float &FL_end_y, float &FL_end_z, bool FL_support, 
                            float &FR_end_x, float &FR_end_y, float &FR_end_z, bool FR_support, 
                            float &BR_end_x, float &BR_end_y, float &BR_end_z, bool BL_support, 
                            float &BL_end_x, float &BL_end_y, float &BL_end_z, bool BR_support)
{
    float Sx = sin(roll_desired);
    float Cx = cos(roll_desired);
    float Sy = sin(pitch_desired);
    float Cy = cos(pitch_desired);
    float Sz = sin(yaw_desired);
    float Cz = cos(yaw_desired);
    bool FL_inworkspace = true;
    bool FR_inworkspace = true;
    bool BL_inworkspace = true;
    bool BR_inworkspace = true;


    BLA::Matrix<4,4> T_ground2body ={Cy*Cz, Cz*Sx*Sy-Cx*Sz, Sx*Sz+Cx*Cz*Sy, forward_desired,
                                    Cy*Sz, Cx*Cz+Sx*Sy*Sz, Cx*Sy*Sz-Cz*Sx, horizontal_desired,
                                    -Sy,   Cy*Sx,          Cx*Cy,          vertical_desired,
                                    0,     0,              0,              1};

    BLA::Matrix<4,4> T_ground2FLab = T_ground2body * T_body2FLab;
    BLA::Matrix<4> temp = T_ground2FLab.Inverse() * P_ground2FLend;
    FL_inworkspace =  leg_inverseKinematics_pos(AB_STANDBY_POS_DEG/180*PI_math, HIP_STANDBY_POS_DEG/180*PI_math, KNEE_STANDBY_POS_DEG/180*PI_math,
                                                temp(0), temp(1), temp(2),
                                                FL_ab, FL_hip, FL_knee,
                                                FRONT_LEFT_LEG);
                                                Serial.print(FL_ab/PI_math*180);
                                                Serial.print(" ");
                                                Serial.print(FL_hip/PI_math*180);
                                                Serial.print(" ");
                                                Serial.println(FL_knee/PI_math*180);

    BLA::Matrix<4,4> T_ground2FRab = T_ground2body * T_body2FRab;
    temp = T_ground2FRab.Inverse() * P_ground2FRend;
    FR_inworkspace = leg_inverseKinematics_pos(-AB_STANDBY_POS_DEG/180*PI_math, -HIP_STANDBY_POS_DEG/180*PI_math, -KNEE_STANDBY_POS_DEG/180*PI_math,
                                                temp(0), temp(1), temp(2),
                                                FR_ab, FR_hip, FR_knee,
                                                FRONT_RIGHT_LEG);
                                                Serial.print(FR_ab/PI_math*180);
                                                Serial.print(" ");
                                                Serial.print(FR_hip/PI_math*180);
                                                Serial.print(" ");
                                                Serial.println(FR_knee/PI_math*180);

    BLA::Matrix<4,4> T_ground2BLab = T_ground2body * T_body2BLab;
    temp = T_ground2BLab.Inverse() * P_ground2BLend;
    BL_inworkspace = leg_inverseKinematics_pos(-AB_STANDBY_POS_DEG/180*PI_math, HIP_STANDBY_POS_DEG/180*PI_math, KNEE_STANDBY_POS_DEG/180*PI_math,
                                                temp(0), temp(1), temp(2),
                                                BL_ab, BL_hip, BL_knee,
                                                BACK_LEFT_LEG);
                                                Serial.print(BL_ab/PI_math*180);
                                                Serial.print(" ");
                                                Serial.print(BL_hip/PI_math*180);
                                                Serial.print(" ");
                                                Serial.println(BL_knee/PI_math*180);

    BLA::Matrix<4,4> T_ground2BRab = T_ground2body * T_body2BRab;
    temp = T_ground2BRab.Inverse() * P_ground2BRend;
    BR_inworkspace = leg_inverseKinematics_pos(AB_STANDBY_POS_DEG/180*PI_math, -HIP_STANDBY_POS_DEG/180*PI_math, -KNEE_STANDBY_POS_DEG/180*PI_math,
                                                temp(0), temp(1), temp(2),
                                                BR_ab, BR_hip, BR_knee,
                                                BACK_RIGHT_LEG);
                                                Serial.print(BR_ab/PI_math*180);
                                                Serial.print(" ");
                                                Serial.print(BR_hip/PI_math*180);
                                                Serial.print(" ");
                                                Serial.println(BR_knee/PI_math*180);
}