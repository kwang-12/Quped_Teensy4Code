#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include "radio.h"
#include "geometry.h"
#include "globals.h"

class kinematics
{
public:
/**
 *  Homo-transformation from body frame to AB motors
 *  These are constant regardless the body posture or leg positions
 */
BLA::Matrix<4,4> tX_body2FLab ={0, 0, 1, dimension_length/2,
                                     0, 1, 0, dimension_width/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};

BLA::Matrix<4,4> tX_body2FRab ={0, 0, 1, dimension_length/2,
                                     0, 1, 0,-dimension_width/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};

BLA::Matrix<4,4> tX_body2BLab ={0, 0,-1,-dimension_length/2,
                                     0,-1, 0, dimension_width/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};

BLA::Matrix<4,4> tX_body2BRab ={0, 0,-1,-dimension_length/2,
                                     0,-1, 0,-dimension_width/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};


    struct leg_pos
    {
        float x = 0;
        float y = 0;
        float z = 0;
    };

    struct leg
    {
        float ab_rad = 0;
        float hip_rad = 0;
        float knee_rad = 0;
        bool swing_complete = false;
        leg_tag tag;
        BLA::Matrix<4, 4> tX_ab2end;

        void init(leg_tag tag_, float ini_height, float leg_pos_offset_forward, float leg_pos_offset_horizontal)
        {
            tag = tag_;
            switch (tag)
            {
            case tag_FL:
            {
                leg_inverseKinematics_pos(0, 0.6981, 1.3963, ini_height, dimension_d2+leg_pos_offset_horizontal, leg_pos_offset_forward, ab_rad, hip_rad, knee_rad);
            }
            break;
            case tag_FR:
            {
                leg_inverseKinematics_pos(0, -0.6981, -1.3963, ini_height, -dimension_d2-leg_pos_offset_horizontal, leg_pos_offset_forward, ab_rad, hip_rad, knee_rad);
            }
            break;
            case tag_BL:
            {
                leg_inverseKinematics_pos(0, -0.6981, -1.3963, ini_height, -dimension_d2-leg_pos_offset_horizontal, leg_pos_offset_forward, ab_rad, hip_rad, knee_rad);
            }
            break;
            case tag_BR:
            {
                leg_inverseKinematics_pos(0, 0.6981, 1.3963, ini_height, dimension_d2+leg_pos_offset_horizontal, leg_pos_offset_forward, ab_rad, hip_rad, knee_rad);
            }
            break;
            }
            update(ab_rad, hip_rad, knee_rad);
        }

        void update(float ab_, float hip_, float knee_)
        {
            ab_rad = ab_;
            hip_rad = hip_;
            knee_rad = knee_;
            float x=0;
            float y=0;
            float z=0;
            leg_forwardKinematics_pos(ab_rad, hip_rad, knee_rad,
                                      x, y, z);
            BLA::Matrix<4, 1> position = {x, y, z, 1};
            BLA::Matrix<4, 3> rotation;
            if ((tag == tag_FL) | (tag == tag_BR))
            {
                rotation = {cos(hip_rad + knee_rad) * cos(ab_rad), -sin(hip_rad + knee_rad) * cos(ab_rad), -sin(ab_rad),
                            cos(hip_rad + knee_rad) * sin(ab_rad), -sin(hip_rad + knee_rad) * sin(ab_rad), cos(ab_rad),
                            -sin(hip_rad + knee_rad), -cos(hip_rad + knee_rad), 0,
                            0, 0, 0};
            }
            else
            {
                rotation = {cos(hip_rad - knee_rad) * cos(ab_rad), sin(hip_rad - knee_rad) * cos(ab_rad), -sin(ab_rad),
                            cos(hip_rad - knee_rad) * sin(ab_rad), sin(hip_rad - knee_rad) * sin(ab_rad), cos(ab_rad),
                            sin(hip_rad - knee_rad), -cos(hip_rad - knee_rad), 0,
                            0, 0, 0};
            }
            tX_ab2end = rotation || position;
        }

        void leg_forwardKinematics_pos(float ab_deg_now, float hip_deg_now, float knee_deg_now,
                                       float &x_calculated, float &y_calculated, float &z_calculated)
        {
            switch (tag)
            {
            case tag_FL:
            {
                x_calculated = -dimension_d2 * sin(ab_deg_now) + cos(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                y_calculated = dimension_d2 * cos(ab_deg_now) + sin(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                z_calculated = -dimension_a3 * sin(hip_deg_now - knee_deg_now) - dimension_a2 * sin(hip_deg_now);
            }
            break;
            case tag_FR:
            {
                x_calculated = dimension_d2 * sin(ab_deg_now) + cos(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                y_calculated = -dimension_d2 * cos(ab_deg_now) + sin(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                z_calculated = dimension_a3 * sin(hip_deg_now - knee_deg_now) + dimension_a2 * sin(hip_deg_now);
            }
            break;
            case tag_BL:
            {
                x_calculated = dimension_d2 * sin(ab_deg_now) + cos(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                y_calculated = -dimension_d2 * cos(ab_deg_now) + sin(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                z_calculated = dimension_a3 * sin(hip_deg_now - knee_deg_now) + dimension_a2 * sin(hip_deg_now);
            }
            break;
            case tag_BR:
            {
                x_calculated = -dimension_d2 * sin(ab_deg_now) + cos(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                y_calculated = dimension_d2 * cos(ab_deg_now) + sin(ab_deg_now) * (dimension_a3 * cos(hip_deg_now - knee_deg_now) + dimension_a2 * cos(hip_deg_now));
                z_calculated = -dimension_a3 * sin(hip_deg_now - knee_deg_now) - dimension_a2 * sin(hip_deg_now);
            }
            break;
            }
        }

        bool leg_inverseKinematics_pos(float ab_rad_now, float hip_rad_now, float knee_rad_now,
                                       float x_desired, float y_desired, float z_desired,
                                       float &ab_rad_desired, float &hip_rad_desired, float &knee_rad_desired)
        {
            float cost[8];
            float theta_1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            float theta_2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            float theta_3[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            int index[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            bool in_workspace_[8] = {true, true, true, true, true, true, true, true};
            int loop_counter = 0;
            switch (tag)
            {
            case tag_FL:
            {
                for (int i = 0; i <= 1; i++)
                {
                    for (int j = 0; j <= 1; j++)
                    {
                        for (int k = 0; k <= 1; k++)
                        {
                            // determine theta_3
                            float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(dimension_d2, 2) - pow(dimension_a2, 2) - pow(dimension_a3, 2)) / (2 * dimension_a2 * dimension_a3);
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
                            if (theta_3[loop_counter] > knee_rad_max || theta_3[loop_counter] < -knee_rad_max)      // calculated solution is out of joint bound
                            {
                                in_workspace_[loop_counter] = false;
                            }
                            // determine theta_2
                            if (in_workspace_[loop_counter] == true)
                            {
                                float a = dimension_a3 * sin(theta_3[loop_counter]);
                                float b = -dimension_a3 * cos(theta_3[loop_counter]) - dimension_a2;
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
                                if (theta_2[loop_counter] > hip_rad_max || theta_2[loop_counter] < -hip_rad_max)      // calculated solution is out of joint bound
                                {
                                    in_workspace_[loop_counter] = false;
                                }
                                // determine theta_1
                                if (in_workspace_[loop_counter] == true)
                                {
                                    a = dimension_a2 * cos(theta_2[loop_counter]) + dimension_a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                                    b = -dimension_d2;
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
                                    if (theta_1[loop_counter] > ab_rad_max || theta_1[loop_counter] < -ab_rad_max)      // calculated solution is out of joint bound
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
            case tag_FR:
            {
                for (int i = 0; i <= 1; i++)
                {
                    for (int j = 0; j <= 1; j++)
                    {
                        for (int k = 0; k <= 1; k++)
                        {
                            // determine theta_3
                            float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(dimension_d2, 2) - pow(dimension_a2, 2) - pow(dimension_a3, 2)) / (2 * dimension_a2 * dimension_a3);
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
                            if (theta_3[loop_counter] > knee_rad_max || theta_3[loop_counter] < -knee_rad_max)      // calculated solution is out of joint bound
                            {
                                in_workspace_[loop_counter] = false;
                            }
                            // determine theta_2
                            if (in_workspace_[loop_counter] == true)
                            {
                                float a = -dimension_a3 * sin(theta_3[loop_counter]);
                                float b = dimension_a3 * cos(theta_3[loop_counter]) + dimension_a2;
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
                                if (theta_2[loop_counter] > hip_rad_max || theta_2[loop_counter] < -hip_rad_max)      // calculated solution is out of joint bound
                                {
                                    in_workspace_[loop_counter] = false;
                                }
                                // determine theta_1
                                if (in_workspace_[loop_counter] == true)
                                {
                                    a = dimension_a2 * cos(theta_2[loop_counter]) + dimension_a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                                    b = dimension_d2;
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
                                    if (theta_1[loop_counter] > ab_rad_max || theta_1[loop_counter] < -ab_rad_max)      // calculated solution is out of joint bound
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
            case tag_BL:
            {
                for (int i = 0; i <= 1; i++)
                {
                    for (int j = 0; j <= 1; j++)
                    {
                        for (int k = 0; k <= 1; k++)
                        {
                            // determine theta_3
                            float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(dimension_d2, 2) - pow(dimension_a2, 2) - pow(dimension_a3, 2)) / (2 * dimension_a2 * dimension_a3);
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
                            if (theta_3[loop_counter] > knee_rad_max || theta_3[loop_counter] < -knee_rad_max)      // calculated solution is out of joint bound
                            {
                                in_workspace_[loop_counter] = false;
                            }
                            // determine theta_2
                            if (in_workspace_[loop_counter] == true)
                            {
                                float a = -dimension_a3 * sin(theta_3[loop_counter]);
                                float b = dimension_a3 * cos(theta_3[loop_counter]) + dimension_a2;
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
                                if (theta_2[loop_counter] > hip_rad_max || theta_2[loop_counter] < -hip_rad_max)      // calculated solution is out of joint bound
                                {
                                    in_workspace_[loop_counter] = false;
                                }
                                // determine theta_1
                                if (in_workspace_[loop_counter] == true)
                                {
                                    a = dimension_a2 * cos(theta_2[loop_counter]) + dimension_a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                                    b = dimension_d2;
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
                                    if (theta_1[loop_counter] > ab_rad_max || theta_1[loop_counter] < -ab_rad_max)      // calculated solution is out of joint bound
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
            case tag_BR:
            {
                for (int i = 0; i <= 1; i++)
                {
                    for (int j = 0; j <= 1; j++)
                    {
                        for (int k = 0; k <= 1; k++)
                        {
                            // determine theta_3
                            float c3 = (pow(x_desired, 2) + pow(y_desired, 2) + pow(z_desired, 2) - pow(dimension_d2, 2) - pow(dimension_a2, 2) - pow(dimension_a3, 2)) / (2 * dimension_a2 * dimension_a3);
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
                            if (theta_3[loop_counter] > knee_rad_max || theta_3[loop_counter] < -knee_rad_max)      // calculated solution is out of joint bound
                            {
                                in_workspace_[loop_counter] = false;
                            }
                            // determine theta_2
                            if (in_workspace_[loop_counter] == true)
                            {
                                float a = dimension_a3 * sin(theta_3[loop_counter]);
                                float b = -dimension_a3 * cos(theta_3[loop_counter]) - dimension_a2;
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
                                if (theta_2[loop_counter] > hip_rad_max || theta_2[loop_counter] < -hip_rad_max)      // calculated solution is out of joint bound
                                {
                                    in_workspace_[loop_counter] = false;
                                }
                                // determine theta_1
                                if (in_workspace_[loop_counter] == true)
                                {
                                    a = dimension_a2 * cos(theta_2[loop_counter]) + dimension_a3 * cos(theta_2[loop_counter] - theta_3[loop_counter]);
                                    b = -dimension_d2;
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
                                    if (theta_1[loop_counter] > ab_rad_max || theta_1[loop_counter] < -ab_rad_max)      // calculated solution is out of joint bound
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
            }

            // exclude the useless calculated joint positions that are not in the workspace
            loop_counter = 0;
            for (int i = 0; i <= 7; i++)
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
                for (int i = 0; i <= num_valid - 1; i++)
                {
                    float x=0;
                    float y=0;
                    float z=0;
                    leg_forwardKinematics_pos(theta_1[index[i]], theta_2[index[i]], theta_3[index[i]], x, y, z);
                    float opSpace_cost = 0.99 * sqrt(pow(x - x_desired, 2) + pow(y - y_desired, 2) + pow(z - z_desired, 2));
                    float jtSpace_cost = 0.01 * sqrt(pow(theta_1[index[i]] - ab_rad_now, 2) + pow(theta_2[index[i]] - hip_rad_now, 2) + pow(theta_3[index[i]] - knee_rad_now, 2));
                    cost[i] = opSpace_cost + jtSpace_cost;
                }
                loop_counter = 0;
                float smallest_cost = 100;
                for (int i = 0; i <= num_valid - 1; i++)
                {
                    if (cost[i] < smallest_cost)
                    {
                        smallest_cost = cost[i];
                        loop_counter = i;
                    }
                }
                ab_rad_desired = theta_1[index[loop_counter]];
                hip_rad_desired = theta_2[index[loop_counter]];
                knee_rad_desired = theta_3[index[loop_counter]];
                return true;
            }
            else
            {
                // maintain current joint angles when no viabale solution
                ab_rad_desired = ab_rad_now;
                hip_rad_desired = hip_rad_now;
                knee_rad_desired = knee_rad_now;
                // ab_rad_desired = 0;
                // hip_rad_desired = 0;
                // knee_rad_desired = 0;
                return false;
            }
        }
    };

    struct body_simp
    {
        float forward = 0;
        float horizontal = 0;
        float height = 0;
        float yaw = 0;
        float pitch = 0;
        float roll = 0;
    };

    struct body
    {
        BLA::Matrix<4,4> tX_body2FLab ={0, 0, 1, dimension_length/2,
                                     0, 1, 0, dimension_width/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};

        BLA::Matrix<4,4> tX_body2FRab ={0, 0, 1, dimension_length/2,
                                            0, 1, 0,-dimension_width/2,
                                            -1, 0, 0, 0,
                                            0, 0, 0, 1};

        BLA::Matrix<4,4> tX_body2BLab ={0, 0,-1,-dimension_length/2,
                                            0,-1, 0, dimension_width/2,
                                            -1, 0, 0, 0,
                                            0, 0, 0, 1};

        BLA::Matrix<4,4> tX_body2BRab ={0, 0,-1,-dimension_length/2,
                                            0,-1, 0,-dimension_width/2,
                                            -1, 0, 0, 0,
                                            0, 0, 0, 1};
        float forward = 0;
        float horizontal = 0;
        float height = 0;
        float yaw = 0;
        float pitch = 0;
        float roll = 0;
        /**
         * Transformations - updated every time tick
         */
        BLA::Matrix<4, 4> tX_Ground2Body;
        BLA::Matrix<4, 4> tX_Ground2FL_ab;
        BLA::Matrix<4, 4> tX_Ground2FR_ab;
        BLA::Matrix<4, 4> tX_Ground2BL_ab;
        BLA::Matrix<4, 4> tX_Ground2BR_ab;
        BLA::Matrix<4, 4> tX_Ground2FL_end;
        BLA::Matrix<4, 4> tX_Ground2FR_end;
        BLA::Matrix<4, 4> tX_Ground2BL_end;
        BLA::Matrix<4, 4> tX_Ground2BR_end;

        void update(float fwd, float hzt, float hgt,
                    float yaw_, float pitch_, float roll_,
                    leg &FL, leg &FR, leg &BL, leg &BR)
        {
            float Sx = sin(roll_);
            float Cx = cos(roll_);
            float Sy = sin(pitch_);
            float Cy = cos(pitch_);
            float Sz = sin(yaw_);
            float Cz = cos(yaw_);
            forward = fwd;
            horizontal = hzt;
            height = hgt;
            yaw = yaw_;
            pitch = pitch_;
            roll = roll_;
            tX_Ground2Body = {Cy * Cz, Cz * Sx * Sy - Cx * Sz, Sx * Sz + Cx * Cz * Sy, fwd,
                              Cy * Sz, Cx * Cz + Sx * Sy * Sz, Cx * Sy * Sz - Cz * Sx, hzt,
                              -Sy, Cy * Sx, Cx * Cy, hgt,
                              0, 0, 0, 1};
            tX_Ground2FL_ab = tX_Ground2Body * tX_body2FLab;
            tX_Ground2FR_ab = tX_Ground2Body * tX_body2FRab;
            tX_Ground2BL_ab = tX_Ground2Body * tX_body2BLab;
            tX_Ground2BR_ab = tX_Ground2Body * tX_body2BRab;
            tX_Ground2FL_end = tX_Ground2FL_ab * FL.tX_ab2end;
            tX_Ground2FR_end = tX_Ground2FR_ab * FR.tX_ab2end;
            tX_Ground2BL_end = tX_Ground2BL_ab * BL.tX_ab2end;
            tX_Ground2BR_end = tX_Ground2BR_ab * BR.tX_ab2end;
    // Serial.print("yaw ");
    // Serial.print(yaw,4);
    // Serial.print(" ");
    // Serial.print(Sz,4);
    // Serial.print(" ");
    // Serial.print(Cz,4);
    // Serial.print(" ");
    // Serial.print(sin(yaw),4);
    // Serial.print(" ");
    // Serial.println(cos(yaw),4);

    // Serial.print("[");
    // Serial.print(tX_Ground2Body(0,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(0,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(0,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2Body(0,3),4);
    // Serial.print(tX_Ground2Body(1,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(1,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(1,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2Body(1,3),4);
    // Serial.print(tX_Ground2Body(2,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(2,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(2,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2Body(2,3),4);
    // Serial.print(tX_Ground2Body(3,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(3,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(3,2),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2Body(3,3),4);
    // Serial.println("]");

    // Serial.print("[");
    // Serial.print(tX_Ground2FL_ab(0,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(0,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(0,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2FL_ab(0,3),4);
    // Serial.print(tX_Ground2FL_ab(1,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(1,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(1,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2FL_ab(1,3),4);
    // Serial.print(tX_Ground2FL_ab(2,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(2,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(2,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2FL_ab(2,3),4);
    // Serial.print(tX_Ground2FL_ab(3,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(3,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(3,2),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_ab(3,3),4);
    // Serial.println("]");

    // Serial.print("[");
    // Serial.print(tX_Ground2FL_end(0,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(0,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(0,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2FL_end(0,3),4);
    // Serial.print(tX_Ground2FL_end(1,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(1,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(1,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2FL_end(1,3),4);
    // Serial.print(tX_Ground2FL_end(2,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(2,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(2,2),4);
    // Serial.print(" ");
    // Serial.println(tX_Ground2FL_end(2,3),4);
    // Serial.print(tX_Ground2FL_end(3,0),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(3,1),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(3,2),4);
    // Serial.print(" ");
    // Serial.print(tX_Ground2FL_end(3,3),4);
    // Serial.println("]");


        }


    };

    /**
     * Posture shifting - motion plan related variables
     */
    body pos_now;
    body_simp pos_ini; // posture shift - initial posture
    body_simp pos_end; // posture shift - target posture
    BLA::Matrix<4, 4> Target;
    float dist;                           // distance between inital COB and target COB
    float margin;                         // posture shift - stability margin
    float posture_time_span;              // time allotted to shifting posture
    bool posture_val_set = false;         // true - pos_ini and pos_end are updated
    bool posture_neutral_request = false; // true - request for shifting to neutral pos
    int gait = 1423;
    float posture_idle_height;
    float posture_low_height;
    float ini_yaw;
    float pitch_target;
    float roll_target;
    float yaw_target;
    float posture_k_p = 0.012 * msg_timer_interval/5;
    float posture_pitch_max;
    float posture_roll_max;
    float posture_yaw_max;

    /**
     * Leg swing - motion plan related variables
     */
    leg FL;
    leg FR;
    leg BL;
    leg BR;
    leg_pos leg_pos_ini;              // leg swing - initial pos
    leg_pos leg_pos_end;              // leg swing - target pos
    float leg_swing_time_span;         // time allotted to leg swing
    float leg_swing_forward_max;       // maximum length of the stride in fwd
    float leg_swing_horizontal_max;    // maximum length of the stride in hzt
    float leg_swing_height;            // maximum height of the stride
    float leg_swing_yaw_max;           // maximum value of yaw delta between postures
    leg_tag leg_swing_choice = tag_FL; // flag for leg swing choice
    float leg_pos_offset_forward;
    float leg_pos_offset_horizontal;

    /**
     * Motion plan state machine
     */
    qPed_state current_state = STATE_IDLE;
    walk_task walk_state_task = TASK_IDLE;
    input_mode walk_mode = MODE_END;
    bool yaw_ini_flag = false;

    /**
     * Radio input interpretation
     */
    struct input
    {
        float x_target = 0;
        float y_target = 0;
        float yaw_target = 0;
    } input;

    /**
     * Constructor
     */
    kinematics(float margin_ = 0.10, float posture_time_span_ = 3, float swing_time_span_ = 2,
               float body_idle_height = 0.35, float body_low_height = 0.18,
               float leg_pos_offset_forward_ = 0, float leg_pos_offset_horizontal_ = 0,
               float max_FWD = 0.08, float max_HZT = 0.04, float max_HGT = 0.10,
               float max_YAW = static_cast<float>(8) / 180 * PI_math,
               float max_p_PITCH = static_cast<float>(10) / 180 * PI_math, 
               float max_p_ROLL = static_cast<float>(10) / 180 * PI_math,
               float max_p_YAW = static_cast<float>(10) / 180 * PI_math);
    
    void reset();

    /**
     * Determine states based on radio inputs
     */
    void calc_input(radio radio_readings);

    /**
     * 
     */
    void update_posture_tx(Point p, Point q, Point r, Point s);

    /**
     * 
     */
    void update_posture_FL(Point p, Point q, Point r, Point s);

    /**
     * 
     */
    void update_posture_FR(Point p, Point q, Point r, Point s);

    /**
     * 
     */
    void update_posture_BL(Point p, Point q, Point r, Point s);

    /**
     * 
     */
    void update_posture_BR(Point p, Point q, Point r, Point s);

    /**
     * 
     */
    void update_posture_neutral(Point p, Point q, Point r, Point s);

    /**
     * Update motion targets
     */
    void update_posture();
    
    void update_swing();

    /**
     * Swing leg inverse kinematics
     * Calculate the required joint angles to achieve the desired leg position
     * @note leg object is updated after calling the function
     */
    void swing_iK(leg_pos desired_leg_pos);

    /**
     * Body posture inverse kinematics
     * Calculate the required joint angles to achieve the desired body posture
     * @note leg objects are updated after calling the function
     * @note transformations in the body object is also updated after calling the function
     */
    void body_iK(body_simp desired_posture);

    /**
     * State machine
     */
    void swing_mgr();

    void walk_mgr(float& time_elapsed_);
    
    void state_mgr(float& time_elapsed_);
    /**
     * 
     */
    void update(float& time_elapsed_, radio radio_readings);

    /**
     * 
     */
    void debug_print();

    /**
     * 
     */
    void update_gains(float kp_);
};
