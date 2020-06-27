#include "kinematics.h"

kinematics::kinematics(float margin_, float posture_time_span_, float swing_time_span_,
                       float body_height, float leg_pos_offset_forward_, float leg_pos_offset_horizontal_,
                       float max_FWD, float max_HZT, float max_HGT,
                       float max_YAW)
{
    FL.init(tag_FL, body_height, leg_pos_offset_forward_, leg_pos_offset_horizontal_);
    FR.init(tag_FR, body_height, leg_pos_offset_forward_, leg_pos_offset_horizontal_);
    BL.init(tag_BL, body_height, leg_pos_offset_forward_, leg_pos_offset_horizontal_);
    BR.init(tag_BR, body_height, leg_pos_offset_forward_, leg_pos_offset_horizontal_);
    pos_now.update(0, 0, body_height, 0, 0, 0, FL, FR, BL, BR);

    margin = margin_;
    posture_time_span = posture_time_span_;
    leg_swing_time_span = swing_time_span_;
    leg_swing_forward_max = max_FWD;
    leg_swing_horizontal_max = max_HZT;
    leg_swing_height = max_HGT;
    leg_swing_yaw_max = max_YAW;
    leg_pos_offset_forward = leg_pos_offset_forward_;
    leg_pos_offset_horizontal = leg_pos_offset_horizontal_;
}

void kinematics::reset()
{
    FL.init(tag_FL, 0.35, leg_pos_offset_forward, leg_pos_offset_horizontal);
    FR.init(tag_FR, 0.35, leg_pos_offset_forward, leg_pos_offset_horizontal);
    BL.init(tag_BL, 0.35, leg_pos_offset_forward, leg_pos_offset_horizontal);
    BR.init(tag_BR, 0.35, leg_pos_offset_forward, leg_pos_offset_horizontal);
    pos_now.update(0, 0, 0.35, 0, 0, 0, FL, FR, BL, BR);
    kinematic_task = TASK_IDLE;
    kinematic_mode = MODE_END;
    yaw_ini_flag = false;
    leg_swing_choice = tag_FL;
    posture_val_set = false;
    posture_neutral_request = false;
}

void kinematics::calc_input(radio radio_readings)
{
    input.x_target = leg_swing_forward_max * radio_readings.ch_1.val;
    input.y_target = leg_swing_horizontal_max * radio_readings.ch_2.val;
    input.yaw_target = leg_swing_yaw_max * radio_readings.ch_4.val;
    if ((kinematic_task == TASK_IDLE) & ((radio_readings.ch_1.val != 0) || (radio_readings.ch_2.val != 0) || (radio_readings.ch_4.val != 0)))
    {
        kinematic_task = TASK_SHIFT_POSTURE;
    }
    if ((radio_readings.ch_1.val == 0) & (radio_readings.ch_2.val == 0) & (radio_readings.ch_4.val == 0))
    {
        kinematic_mode = MODE_END;
    }
    else
    {
        kinematic_mode = MODE_CONT;
    }
}

void kinematics::update_posture()
{
    Point p, q, r, s;
    p = {pos_now.tX_Ground2FL_end(0, 3), pos_now.tX_Ground2FL_end(1, 3)};
    q = {pos_now.tX_Ground2FR_end(0, 3), pos_now.tX_Ground2FR_end(1, 3)};
    r = {pos_now.tX_Ground2BR_end(0, 3), pos_now.tX_Ground2BR_end(1, 3)};
    s = {pos_now.tX_Ground2BL_end(0, 3), pos_now.tX_Ground2BL_end(1, 3)};

    pos_ini.forward = pos_now.forward;
    pos_ini.horizontal = pos_now.horizontal;
    pos_ini.height = pos_now.height;
    pos_ini.yaw = pos_now.yaw;
    pos_ini.pitch = pos_now.pitch;
    pos_ini.roll = pos_now.roll;

    pos_end.height = pos_ini.height;
    pos_end.pitch = pos_ini.pitch;
    pos_end.roll = pos_ini.roll;

    if (gait == 1423)
    {
        if (posture_neutral_request == false) // shift posture based on gait choice
        {
            if (FL.swing_complete == false)
            {
                if (yaw_ini_flag == false)
                {
                    // At the beginning of the gait cycle:
                    // Calculate desired body posture by the end of the gait cycle
                    // current body posture transformation (relative to ground)
                    Point poly_2[] = {p, q, r, s};
                    int poly_2_n = 4;
                    Point centroid = calc_newCOB(p, q, r, s, poly_2, poly_2_n, -1);
                    BLA::Matrix<4, 1> position = {centroid.x, centroid.y, pos_now.height, 1};
                    BLA::Matrix<4, 4> t1 = pos_now.tX_Ground2Body.Submatrix(BLA::Slice<0,4>(),BLA::Slice<0,3>()) || position;
                    // initialize yaw target for the gait cycle
                    yaw_target = input.yaw_target;
                    // desired body posture transformation (relative to body)
                    float Sx = sin(0);
                    float Cx = cos(0);
                    float Sy = sin(0);
                    float Cy = cos(0);
                    float Sz = sin(yaw_target);
                    float Cz = cos(yaw_target);
                    BLA::Matrix<4, 3> tx_R = {Cy * Cz, Cz * Sx * Sy - Cx * Sz, Sx * Sz + Cx * Cz * Sy,
                                            Cy * Sz, Cx * Cz + Sx * Sy * Sz, Cx * Sy * Sz - Cz * Sx,
                                            -Sy, Cy * Sx, Cx * Cy,
                                            0, 0, 0};
                    BLA::Matrix<4, 1> tx_P = {input.x_target, input.y_target, 0, 1};
                    BLA::Matrix<4, 4> tx = tx_R || tx_P;

                    BLA::Multiply(t1, tx, Target);

                    yaw_ini_flag = true;
                }
                Point poly[] = {q, r, s};
                int poly_n = 3;
                Point new_COB = calc_newCOB(p, r, q, s, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(q, s, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(q, s, poly, poly_n, -1);
                pos_end.forward = new_COB.x;
                pos_end.horizontal = new_COB.y;
            }
            else if (BR.swing_complete == false)
            {
                Point poly[] = {p, q, s};
                int poly_n = 3;
                Point new_COB = calc_newCOB(r, p, q, s, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(q, s, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(q, s, poly, poly_n, -1);
                pos_end.forward = new_COB.x;
                pos_end.horizontal = new_COB.y;
            }
            else if (FR.swing_complete == false)
            {
                Point poly[] = {p, r, s};
                int poly_n = 3;
                Point new_COB = calc_newCOB(q, s, p, r, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(p, r, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(p, r, poly, poly_n, -1);
                pos_end.forward = new_COB.x;
                pos_end.horizontal = new_COB.y;
            }
            else if (BL.swing_complete == false)
            {
                Point poly[] = {p, q, r};
                int poly_n = 3;
                Point new_COB = calc_newCOB(s, q, p, r, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(p, r, poly, poly_n, margin);
                // Point new_COB = calc_newCOB(p, r, poly, poly_n, -1);
                pos_end.forward = new_COB.x;
                pos_end.horizontal = new_COB.y;
            }
            if (fabs(yaw_target) < 0.0000000001)
            {
                pos_end.yaw = pos_ini.yaw;
            }
            else
            {
                pos_end.yaw = yaw_target * 0.25 + pos_ini.yaw;
            }
        }
        else // move to neutral position
        {
            Point poly[] = {p, q, r, s};
            int poly_n = 4;
            Point centroid = calc_newCOB(r, s, p, q, poly, poly_n, -1);
            pos_end.forward = centroid.x;
            pos_end.horizontal = centroid.y;
            pos_end.yaw = pos_ini.yaw;
        }
    }
}

void kinematics::update_swing()
{
    if (leg_swing_choice == tag_FL)
    {
        leg_pos_ini = {pos_now.tX_Ground2FL_end(0,3),pos_now.tX_Ground2FL_end(1,3),pos_now.tX_Ground2FL_end(2,3)};
        BLA::Matrix<4,1> pos = {pos_now.height,dimension_d2+leg_pos_offset_horizontal,leg_pos_offset_forward,1};
        BLA::Matrix<4,1> P_G_leg_end= Target * tX_body2FLab * pos;
        leg_pos_end = {P_G_leg_end(0), P_G_leg_end(1), P_G_leg_end(2)};
    }
    else if (leg_swing_choice == tag_FR)
    {
        leg_pos_ini = {pos_now.tX_Ground2FR_end(0,3),pos_now.tX_Ground2FR_end(1,3),pos_now.tX_Ground2FR_end(2,3)};
        BLA::Matrix<4,1> pos = {pos_now.height,-dimension_d2-leg_pos_offset_horizontal,leg_pos_offset_forward,1};
        BLA::Matrix<4,1> P_G_leg_end= Target * tX_body2FRab * pos;
        leg_pos_end = {P_G_leg_end(0), P_G_leg_end(1), P_G_leg_end(2)};
    }
    else if (leg_swing_choice == tag_BL)
    {
        leg_pos_ini = {pos_now.tX_Ground2BL_end(0,3),pos_now.tX_Ground2BL_end(1,3),pos_now.tX_Ground2BL_end(2,3)};
        BLA::Matrix<4,1> pos = {pos_now.height,-dimension_d2-leg_pos_offset_horizontal,leg_pos_offset_forward,1};
        BLA::Matrix<4,1> P_G_leg_end= Target * tX_body2BLab * pos;
        leg_pos_end = {P_G_leg_end(0), P_G_leg_end(1), P_G_leg_end(2)};
    }
    else if (leg_swing_choice == tag_BR)
    {
        leg_pos_ini = {pos_now.tX_Ground2BR_end(0,3),pos_now.tX_Ground2BR_end(1,3),pos_now.tX_Ground2BR_end(2,3)};
        BLA::Matrix<4,1> pos = {pos_now.height,dimension_d2+leg_pos_offset_horizontal,leg_pos_offset_forward,1};
        BLA::Matrix<4,1> P_G_leg_end= Target * tX_body2BRab * pos;
        leg_pos_end = {P_G_leg_end(0), P_G_leg_end(1), P_G_leg_end(2)};
    }
}

void kinematics::swing_iK(leg_pos desired_leg_pos)
{
    BLA::Matrix<4,1> P_G_leg_end = {desired_leg_pos.x,desired_leg_pos.y,desired_leg_pos.z,1};
    if (leg_swing_choice == tag_FL)
    {
        BLA::Matrix<4,1> temp = pos_now.tX_Ground2FL_ab.Inverse() * P_G_leg_end;
        FL.leg_inverseKinematics_pos(FL.ab_rad, FL.hip_rad, FL.knee_rad,
                                     temp(0),temp(1),temp(2),
                                     FL.ab_rad, FL.hip_rad, FL.knee_rad);
        FL.update(FL.ab_rad,FL.hip_rad,FL.knee_rad);
    }
    else if (leg_swing_choice == tag_FR)
    {
        BLA::Matrix<4,1> temp = pos_now.tX_Ground2FR_ab.Inverse() * P_G_leg_end;
        FR.leg_inverseKinematics_pos(FR.ab_rad, FR.hip_rad, FR.knee_rad,
                                     temp(0),temp(1),temp(2),
                                     FR.ab_rad, FR.hip_rad, FR.knee_rad);
        FR.update(FR.ab_rad,FR.hip_rad,FR.knee_rad);
    }
    else if (leg_swing_choice == tag_BL)
    {
        BLA::Matrix<4,1> temp = pos_now.tX_Ground2BL_ab.Inverse() * P_G_leg_end;
        BL.leg_inverseKinematics_pos(BL.ab_rad, BL.hip_rad, BL.knee_rad,
                                     temp(0),temp(1),temp(2),
                                     BL.ab_rad, BL.hip_rad, BL.knee_rad);
        BL.update(BL.ab_rad,BL.hip_rad,BL.knee_rad);
    }
    else if (leg_swing_choice == tag_BR)
    {
        BLA::Matrix<4,1> temp = pos_now.tX_Ground2BR_ab.Inverse() * P_G_leg_end;
        BR.leg_inverseKinematics_pos(BR.ab_rad, BR.hip_rad, BR.knee_rad,
                                     temp(0),temp(1),temp(2),
                                     BR.ab_rad, BR.hip_rad, BR.knee_rad);
        BR.update(BR.ab_rad,BR.hip_rad,BR.knee_rad);
    }
    pos_now.update(pos_now.forward, pos_now.horizontal, pos_now.height,
                   pos_now.yaw, pos_now.pitch, pos_now.roll,
                   FL, FR, BL, BR);
}

void kinematics::body_iK(body_simp desired_posture)
{
    float Sx = sin(desired_posture.roll);
    float Cx = cos(desired_posture.roll);
    float Sy = sin(desired_posture.pitch);
    float Cy = cos(desired_posture.pitch);
    float Sz = sin(desired_posture.yaw);
    float Cz = cos(desired_posture.yaw);
    BLA::Matrix<4, 3> rotation = {Cy * Cz, Cz * Sx * Sy - Cx * Sz, Sx * Sz + Cx * Cz * Sy,
                                Cy * Sz, Cx * Cz + Sx * Sy * Sz, Cx * Sy * Sz - Cz * Sx,
                                -Sy, Cy * Sx, Cx * Cy,
                                0, 0, 0};
    BLA::Matrix<4, 1> position = {desired_posture.forward, desired_posture.horizontal, desired_posture.height, 1};
    BLA::Matrix<4, 4> t_G_B = rotation || position;
    BLA::Matrix<4, 4> t_FL = t_G_B * tX_body2FLab;
    BLA::Matrix<4, 4> t_FR = t_G_B * tX_body2FRab;
    BLA::Matrix<4, 4> t_BL = t_G_B * tX_body2BLab;
    BLA::Matrix<4, 4> t_BR = t_G_B * tX_body2BRab;

    BLA::Matrix<4, 1> temp = t_FL.Inverse()*pos_now.tX_Ground2FL_end.Submatrix(BLA::Slice<0,4>(),BLA::Slice<3,4>());
    FL.leg_inverseKinematics_pos(FL.ab_rad, FL.hip_rad, FL.knee_rad, temp(0), temp(1), temp(2), FL.ab_rad, FL.hip_rad, FL.knee_rad);
    FL.update(FL.ab_rad,FL.hip_rad,FL.knee_rad);

    temp = t_FR.Inverse()*pos_now.tX_Ground2FR_end.Submatrix(BLA::Slice<0,4>(),BLA::Slice<3,4>());
    FR.leg_inverseKinematics_pos(FR.ab_rad, FR.hip_rad, FR.knee_rad, temp(0), temp(1), temp(2), FR.ab_rad, FR.hip_rad, FR.knee_rad);
    FR.update(FR.ab_rad,FR.hip_rad,FR.knee_rad);

    temp = t_BL.Inverse()*pos_now.tX_Ground2BL_end.Submatrix(BLA::Slice<0,4>(),BLA::Slice<3,4>());
    BL.leg_inverseKinematics_pos(BL.ab_rad, BL.hip_rad, BL.knee_rad, temp(0), temp(1), temp(2), BL.ab_rad, BL.hip_rad, BL.knee_rad);
    BL.update(BL.ab_rad,BL.hip_rad,BL.knee_rad);

    temp = t_BR.Inverse()*pos_now.tX_Ground2BR_end.Submatrix(BLA::Slice<0,4>(),BLA::Slice<3,4>());
    BR.leg_inverseKinematics_pos(BR.ab_rad, BR.hip_rad, BR.knee_rad, temp(0), temp(1), temp(2), BR.ab_rad, BR.hip_rad, BR.knee_rad);
    BR.update(BR.ab_rad,BR.hip_rad,BR.knee_rad);

    // Serial.print("[");
    // Serial.print(t_G_B(0,0),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(0,1),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(0,2),4);
    // Serial.print(" ");
    // Serial.println(t_G_B(0,3),4);
    // Serial.print(t_G_B(1,0),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(1,1),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(1,2),4);
    // Serial.print(" ");
    // Serial.println(t_G_B(1,3),4);
    // Serial.print(t_G_B(2,0),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(2,1),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(2,2),4);
    // Serial.print(" ");
    // Serial.println(t_G_B(2,3),4);
    // Serial.print(t_G_B(3,0),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(3,1),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(3,2),4);
    // Serial.print(" ");
    // Serial.print(t_G_B(3,3),4);
    // Serial.println("]");

    // Serial.print(desired_posture.forward,4);
    // Serial.print(" ");
    // Serial.print(desired_posture.horizontal,4);
    // Serial.print(" ");
    // Serial.print(desired_posture.height,4);
    // Serial.print(" ");
    // Serial.print(desired_posture.yaw,4);
    // Serial.print(" ");
    // Serial.print(desired_posture.pitch,4);
    // Serial.print(" ");
    // Serial.println(desired_posture.roll,4);

    pos_now.update(desired_posture.forward, desired_posture.horizontal, desired_posture.height,
                   desired_posture.yaw, desired_posture.pitch, desired_posture.roll,
                   FL, FR, BL, BR);
}

void kinematics::swing_mgr()
{
    if (gait == 1423)
    {
        if (FL.swing_complete == false)
        {
            FL.swing_complete = true;
            leg_swing_choice = tag_BR;
        }
        else if (BR.swing_complete == false)
        {
            BR.swing_complete = true;
            leg_swing_choice = tag_FR;
        }
        else if (FR.swing_complete == false)
        {
            FR.swing_complete = true;
            leg_swing_choice = tag_BL;
        }
        else if (BL.swing_complete == false)
        {
            BL.swing_complete = true;
            leg_swing_choice = tag_FL;
        }
    }
}

void kinematics::state_mgr(float &time_elapsed_)
{
    // gait completion check
    bool gait_complete_check = true;
    if (kinematic_task != TASK_IDLE)
    {
        gait_complete_check = gait_complete_check & FL.swing_complete;
        gait_complete_check = gait_complete_check & FR.swing_complete;
        gait_complete_check = gait_complete_check & BL.swing_complete;
        gait_complete_check = gait_complete_check & BR.swing_complete;
    }
    // task completion check
    bool task_complete_check = false;
    if (kinematic_task == TASK_SHIFT_POSTURE)
    {
        task_complete_check = time_elapsed_ / posture_time_span > 1 + 0.5 * msg_timer_interval/1000 / posture_time_span;
    }
    else if (kinematic_task == TASK_SWING_LEG)
    {
        task_complete_check = time_elapsed_ / leg_swing_time_span > 1 + 0.5 * msg_timer_interval/1000 / leg_swing_time_span;
    }
    if (!gait_complete_check)
    {
        if (!task_complete_check)
        {
            if (kinematic_task == TASK_SHIFT_POSTURE)
            {
                float progress = time_elapsed_ / posture_time_span;
                if (posture_val_set == false)
                {
                    update_posture();
                    posture_val_set = true;     ///////// not checked yet
                }
                body_simp temp_posture;
                temp_posture.forward = progress * (pos_end.forward-pos_ini.forward) + pos_ini.forward;
                temp_posture.horizontal = progress * (pos_end.horizontal-pos_ini.horizontal) + pos_ini.horizontal;
                temp_posture.height = progress * (pos_end.height-pos_ini.height) + pos_ini.height;
                temp_posture.yaw = progress * (pos_end.yaw-pos_ini.yaw) + pos_ini.yaw;
                temp_posture.pitch = progress * (pos_end.pitch-pos_ini.pitch) + pos_ini.pitch;
                temp_posture.roll = progress * (pos_end.roll-pos_ini.roll) + pos_ini.roll;
                body_iK(temp_posture);    
            }
            else if (kinematic_task == TASK_SWING_LEG)
            {
                float progress = time_elapsed_ / leg_swing_time_span;
                // leg_pos temp_leg_pos = {progress * (leg_pos_end.x - leg_pos_ini.x) + leg_pos_ini.x,
                //                         progress * (leg_pos_end.y - leg_pos_ini.y) + leg_pos_ini.y,
                //                         leg_swing_height * cos(PI_math*progress - PI_math/2)};
                leg_pos temp_leg_pos;
                temp_leg_pos.x = progress * (leg_pos_end.x - leg_pos_ini.x) + leg_pos_ini.x;
                temp_leg_pos.y = progress * (leg_pos_end.y - leg_pos_ini.y) + leg_pos_ini.y;
                temp_leg_pos.z = leg_swing_height * cos(PI_math*progress - PI_math/2);
                swing_iK(temp_leg_pos);
            }
        }
        else
        {
            time_elapsed_ = msg_timer_interval/1000;
            if (kinematic_task == TASK_SHIFT_POSTURE)
            {
                // the last task was shifting to neutral position which has been completed
                // set all swing flags to true so the kinematic task can be switch to idle
                if (posture_neutral_request == true)
                {
                    posture_neutral_request = false;
                    FL.swing_complete = true;
                    FR.swing_complete = true;
                    BL.swing_complete = true;
                    BR.swing_complete = true;
                    posture_val_set = false;
                    return;
                }
                posture_val_set = false;
                kinematic_task = TASK_SWING_LEG;
                float progress = time_elapsed_ / leg_swing_time_span;
                update_swing();
                // leg_pos temp_leg_pos = {progress * (leg_pos_end.x - leg_pos_ini.x) + leg_pos_ini.x,
                //                         progress * (leg_pos_end.y - leg_pos_ini.y) + leg_pos_ini.y,
                //                         leg_swing_height * cos(PI_math*progress - PI_math/2)};
                leg_pos temp_leg_pos;
                temp_leg_pos.x = progress * (leg_pos_end.x - leg_pos_ini.x) + leg_pos_ini.x;
                temp_leg_pos.y = progress * (leg_pos_end.y - leg_pos_ini.y) + leg_pos_ini.y;
                temp_leg_pos.z = leg_swing_height * cos(PI_math*progress - PI_math/2);
                swing_iK(temp_leg_pos);
            }
            else if (kinematic_task == TASK_SWING_LEG)
            {
                swing_mgr();
                kinematic_task = TASK_SHIFT_POSTURE;
                float progress = time_elapsed_ / posture_time_span;
                bool eval = FL.swing_complete & FR.swing_complete & BL.swing_complete & BR.swing_complete;
                if (eval)
                {
                    FL.swing_complete = false;
                    FR.swing_complete = false;
                    BL.swing_complete = false;
                    BR.swing_complete = false;
                    yaw_ini_flag = false;
                    if (kinematic_mode == MODE_END)
                    {
                        posture_neutral_request = true;
                    }
                }
                update_posture();
                posture_val_set = true;
                body_simp temp_posture;
                temp_posture.forward = progress * (pos_end.forward-pos_ini.forward) + pos_ini.forward;
                temp_posture.horizontal = progress * (pos_end.horizontal-pos_ini.horizontal) + pos_ini.horizontal;
                temp_posture.height = progress * (pos_end.height-pos_ini.height) + pos_ini.height;
                temp_posture.yaw = progress * (pos_end.yaw-pos_ini.yaw) + pos_ini.yaw;
                temp_posture.pitch = progress * (pos_end.pitch-pos_ini.pitch) + pos_ini.pitch;
                temp_posture.roll = progress * (pos_end.roll-pos_ini.roll) + pos_ini.roll;
                body_iK(temp_posture);
            }
        }
    }
    else
    {
        kinematic_task = TASK_IDLE;
        FL.swing_complete = false;
        FR.swing_complete = false;
        BL.swing_complete = false;
        BR.swing_complete = false;
    }
}

void kinematics::update(float &time_elapsed_, radio radio_readings)
{
    calc_input(radio_readings);
    state_mgr(time_elapsed_);
}

void kinematics::debug_print()
{
    Serial.print("G-frame ");
    Serial.print("FL - x: ");
    Serial.print(pos_now.tX_Ground2FL_end(0,3),4);
    Serial.print(", y: ");
    Serial.print(pos_now.tX_Ground2FL_end(1,3),4);
    Serial.print(", z: ");
    Serial.println(pos_now.tX_Ground2FL_end(2,3),4);

    Serial.print("G-frame ");
    Serial.print("FR - x: ");
    Serial.print(pos_now.tX_Ground2FR_end(0,3),4);
    Serial.print(", y: ");
    Serial.print(pos_now.tX_Ground2FR_end(1,3),4);
    Serial.print(", z: ");
    Serial.println(pos_now.tX_Ground2FR_end(2,3),4);
    
    Serial.print("G-frame ");
    Serial.print("BL - x: ");
    Serial.print(pos_now.tX_Ground2BL_end(0,3),4);
    Serial.print(", y: ");
    Serial.print(pos_now.tX_Ground2BL_end(1,3),4);
    Serial.print(", z: ");
    Serial.println(pos_now.tX_Ground2BL_end(2,3),4);
    
    Serial.print("G-frame ");
    Serial.print("BR - x: ");
    Serial.print(pos_now.tX_Ground2BR_end(0,3),4);
    Serial.print(", y: ");
    Serial.print(pos_now.tX_Ground2BR_end(1,3),4);
    Serial.print(", z: ");
    Serial.println(pos_now.tX_Ground2BR_end(2,3),4);

    // Serial.print("FL: ");
    // Serial.print(FL.ab_rad,4);
    // Serial.print(" ");
    // Serial.print(FL.hip_rad,4);
    // Serial.print(" ");
    // Serial.println(FL.knee_rad,4);

    // Serial.print("FR: ");
    // Serial.print(FR.ab_rad,4);
    // Serial.print(" ");
    // Serial.print(FR.hip_rad,4);
    // Serial.print(" ");
    // Serial.println(FR.knee_rad,4);
    
    // Serial.print("BL: ");
    // Serial.print(BL.ab_rad,4);
    // Serial.print(" ");
    // Serial.print(BL.hip_rad,4);
    // Serial.print(" ");
    // Serial.println(BL.knee_rad,4);

    // Serial.print("BR: ");
    // Serial.print(BR.ab_rad,4);
    // Serial.print(" ");
    // Serial.print(BR.hip_rad,4);
    // Serial.print(" ");
    // Serial.println(BR.knee_rad,4);

    // Serial.print("FL: ");
    // Serial.print(FL.tX_ab2end(0,3),4);
    // Serial.print(" ");
    // Serial.print(FL.tX_ab2end(1,3),4);
    // Serial.print(" ");
    // Serial.println(FL.tX_ab2end(2,3),4);

    // Serial.print("FR: ");
    // Serial.print(FR.tX_ab2end(0,3),4);
    // Serial.print(" ");
    // Serial.print(FR.tX_ab2end(1,3),4);
    // Serial.print(" ");
    // Serial.println(FR.tX_ab2end(2,3),4);
    
    // Serial.print("BL: ");
    // Serial.print(BL.tX_ab2end(0,3),4);
    // Serial.print(" ");
    // Serial.print(BL.tX_ab2end(1,3),4);
    // Serial.print(" ");
    // Serial.println(BL.tX_ab2end(2,3),4);

    // Serial.print("BR: ");
    // Serial.print(BR.tX_ab2end(0,3),4);
    // Serial.print(" ");
    // Serial.print(BR.tX_ab2end(1,3),4);
    // Serial.print(" ");
    // Serial.println(BR.tX_ab2end(2,3),4);

    Serial.print("FWD: ");
    Serial.print(pos_now.forward,4);
    Serial.print(" HZT: ");
    Serial.print(pos_now.horizontal,4);
    Serial.print(" HGT: ");
    Serial.print(pos_now.height,4);
    Serial.print(" YAW: ");
    Serial.print(pos_now.yaw,4);
    Serial.print(" PCH: ");
    Serial.print(pos_now.pitch,4);
    Serial.print(" ROL: ");
    Serial.println(pos_now.roll,4);

    Serial.print("X-target: ");
    Serial.print(input.x_target,4);
    Serial.print(" Y-target: ");
    Serial.print(input.y_target,4);
    Serial.print(" YAW-target: ");
    Serial.println(input.yaw_target,4);

    // Serial.print(pos_end.forward,4);
    // Serial.print(" ");
    // Serial.print(pos_end.horizontal,4);
    // Serial.print(" ");
    // Serial.print(pos_end.height,4);
    // Serial.print(" ");
    // Serial.print(pos_end.yaw,4);
    // Serial.print(" ");
    // Serial.print(pos_end.pitch,4);
    // Serial.print(" ");
    // Serial.println(pos_end.roll,4);

    // Serial << Target << "\n";

                // Serial.print("FL: ");
                // Serial.print(pos_now.tX_Ground2FL_end(0, 3),4);
                // Serial.print(" ");
                // Serial.print(pos_now.tX_Ground2FL_end(0, 3),4);
                // Serial.print(" ");
                // Serial.print("FR: ");
                // Serial.print(pos_now.tX_Ground2FR_end(0, 3),4);
                // Serial.print(" ");
                // Serial.print(pos_now.tX_Ground2FR_end(0, 3),4);
                // Serial.print(" ");
                // Serial.print("BR: ");
                // Serial.print(pos_now.tX_Ground2BR_end(0, 3),4);
                // Serial.print(" ");
                // Serial.print(pos_now.tX_Ground2BR_end(0, 3),4);
                // Serial.print(" ");
                // Serial.print("BL: ");
                // Serial.print(pos_now.tX_Ground2BL_end(0, 3),4);
                // Serial.print(" ");
                // Serial.println(pos_now.tX_Ground2BL_end(0, 3),4);
    Serial.print("MODE: ");
    if (kinematic_mode == MODE_CONT)
    {
        Serial.print("CONT");
    }
    else if (kinematic_mode == MODE_END)
    {
        Serial.print("END");
    }
    Serial.print(" TASK: ");
    if (kinematic_task == TASK_IDLE)
    {
        Serial.print("IDLE");
    }
    else if (kinematic_task == TASK_SHIFT_POSTURE)
    {
        Serial.println("SPo");
    }
    else if (kinematic_task == TASK_SWING_LEG)
    {
        Serial.print("SWg ");
        if (leg_swing_choice == tag_FL)
        {
            Serial.println("FL");
        }
        else if (leg_swing_choice == tag_FR)
        {
            Serial.println("FR");
        }
        else if (leg_swing_choice == tag_BL)
        {
            Serial.println("BL");
        }
        else if (leg_swing_choice == tag_BR)
        {
            Serial.println("BR");
        }
    }
    Serial.println("-----");
}