#include "jointPositions.h"

jointPositions::jointPositions()
{
}

void jointPositions::initialize_moveTo_constantPos(char mode, ODriveArduino *front_AB_ptr, ODriveArduino *front_HIP_ptr, ODriveArduino *front_KNEE_ptr,
                                                   ODriveArduino *back_AB_ptr, ODriveArduino *back_HIP_ptr, ODriveArduino *back_KNEE_ptr)
{
    if (mode == STANDBY_POS_FLAG)
    {
#ifdef ENABLE_FRONT_LEFT
        joint_initial_pos_deg.fL_ab_pos = front_AB_ptr->transPosition_num2deg(LEFT, front_AB_ptr->getAxisPos(LEFT, true));
        joint_initial_pos_deg.fL_hip_pos = front_HIP_ptr->transPosition_num2deg(LEFT, front_HIP_ptr->getAxisPos(LEFT, true));
        joint_initial_pos_deg.fL_knee_pos = front_KNEE_ptr->transPosition_num2deg(LEFT, front_KNEE_ptr->getAxisPos(LEFT, true));
        joint_vel_target_deg.fL_ab_velo = (LEG_LEFT_POS_MULTIPLIER * AB_STANDBY_POS_DEG - joint_initial_pos_deg.fL_ab_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.fL_hip_velo = (LEG_LEFT_POS_MULTIPLIER * HIP_STANDBY_POS_DEG - joint_initial_pos_deg.fL_hip_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.fL_knee_velo = (LEG_LEFT_POS_MULTIPLIER * KNEE_STANDBY_POS_DEG - joint_initial_pos_deg.fL_knee_pos) / POS_CONVERSION_TIME;
#endif
#ifdef ENABLE_FRONT_RIGHT
        joint_initial_pos_deg.fR_ab_pos = front_AB_ptr->transPosition_num2deg(RIGHT, front_AB_ptr->getAxisPos(RIGHT, true));
        joint_initial_pos_deg.fR_hip_pos = front_HIP_ptr->transPosition_num2deg(RIGHT, front_HIP_ptr->getAxisPos(RIGHT, true));
        joint_initial_pos_deg.fR_knee_pos = front_KNEE_ptr->transPosition_num2deg(RIGHT, front_KNEE_ptr->getAxisPos(RIGHT, true));
        joint_vel_target_deg.fR_ab_velo = (LEG_RIGHT_POS_MULTIPLIER * AB_STANDBY_POS_DEG - joint_initial_pos_deg.fR_ab_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.fR_hip_velo = (LEG_RIGHT_POS_MULTIPLIER * HIP_STANDBY_POS_DEG - joint_initial_pos_deg.fR_hip_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.fR_knee_velo = (LEG_RIGHT_POS_MULTIPLIER * KNEE_STANDBY_POS_DEG - joint_initial_pos_deg.fR_knee_pos) / POS_CONVERSION_TIME;
#endif
#ifdef ENABLE_BACK_LEFT
        joint_initial_pos_deg.bL_ab_pos = back_AB_ptr->transPosition_num2deg(LEFT, back_AB_ptr->getAxisPos(LEFT, true));
        joint_initial_pos_deg.bL_hip_pos = back_HIP_ptr->transPosition_num2deg(LEFT, back_HIP_ptr->getAxisPos(LEFT, true));
        joint_initial_pos_deg.bL_knee_pos = back_KNEE_ptr->transPosition_num2deg(LEFT, back_KNEE_ptr->getAxisPos(LEFT, true));
        joint_vel_target_deg.bL_ab_velo = (LEG_LEFT_POS_MULTIPLIER * AB_STANDBY_POS_DEG - joint_initial_pos_deg.bL_ab_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.bL_hip_velo = (LEG_LEFT_POS_MULTIPLIER * HIP_STANDBY_POS_DEG - joint_initial_pos_deg.bL_hip_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.bL_knee_velo = (LEG_LEFT_POS_MULTIPLIER * KNEE_STANDBY_POS_DEG - joint_initial_pos_deg.bL_knee_pos) / POS_CONVERSION_TIME;
#endif
#ifdef ENABLE_BACK_RIGHT
        joint_initial_pos_deg.bR_ab_pos = back_AB_ptr->transPosition_num2deg(RIGHT, back_AB_ptr->getAxisPos(RIGHT, true));
        joint_initial_pos_deg.bR_hip_pos = back_HIP_ptr->transPosition_num2deg(RIGHT, back_HIP_ptr->getAxisPos(RIGHT, true));
        joint_initial_pos_deg.bR_knee_pos = back_KNEE_ptr->transPosition_num2deg(RIGHT, back_KNEE_ptr->getAxisPos(RIGHT, true));
        joint_vel_target_deg.bR_ab_velo = (LEG_RIGHT_POS_MULTIPLIER * AB_STANDBY_POS_DEG - joint_initial_pos_deg.bR_ab_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.bR_hip_velo = (LEG_RIGHT_POS_MULTIPLIER * HIP_STANDBY_POS_DEG - joint_initial_pos_deg.bR_hip_pos) / POS_CONVERSION_TIME;
        joint_vel_target_deg.bR_knee_velo = (LEG_RIGHT_POS_MULTIPLIER * KNEE_STANDBY_POS_DEG - joint_initial_pos_deg.bR_knee_pos) / POS_CONVERSION_TIME;
#endif
    }
}

void jointPositions::update_constantPos(float elapsedSec)
{
#ifdef ENABLE_FRONT_LEFT
    joint_pos_target_deg.fL_ab_pos = elapsedSec * joint_vel_target_deg.fL_ab_velo + joint_initial_pos_deg.fL_ab_pos;
    joint_pos_target_deg.fL_hip_pos = elapsedSec * joint_vel_target_deg.fL_hip_velo + joint_initial_pos_deg.fL_hip_pos;
    joint_pos_target_deg.fL_knee_pos = elapsedSec * joint_vel_target_deg.fL_knee_velo + joint_initial_pos_deg.fL_knee_pos;
#endif
#ifdef ENABLE_FRONT_RIGHT
    joint_pos_target_deg.fR_ab_pos = elapsedSec * joint_vel_target_deg.fR_ab_velo + joint_initial_pos_deg.fR_ab_pos;
    joint_pos_target_deg.fR_hip_pos = elapsedSec * joint_vel_target_deg.fR_hip_velo + joint_initial_pos_deg.fR_hip_pos;
    joint_pos_target_deg.fR_knee_pos = elapsedSec * joint_vel_target_deg.fR_knee_velo + joint_initial_pos_deg.fR_knee_pos;
#endif
#ifdef ENABLE_BACK_LEFT
    joint_pos_target_deg.bR_ab_pos = elapsedSec * joint_vel_target_deg.bR_ab_velo + joint_initial_pos_deg.bR_ab_pos;
    joint_pos_target_deg.bR_hip_pos = elapsedSec * joint_vel_target_deg.bR_hip_velo + joint_initial_pos_deg.bR_hip_pos;
    joint_pos_target_deg.bR_knee_pos = elapsedSec * joint_vel_target_deg.bR_knee_velo + joint_initial_pos_deg.bR_knee_pos;
#endif
#ifdef ENABLE_BACK_RIGHT
    joint_pos_target_deg.bL_ab_pos = elapsedSec * joint_vel_target_deg.bL_ab_velo + joint_initial_pos_deg.bL_ab_pos;
    joint_pos_target_deg.bL_hip_pos = elapsedSec * joint_vel_target_deg.bL_hip_velo + joint_initial_pos_deg.bL_hip_pos;
    joint_pos_target_deg.bL_knee_pos = elapsedSec * joint_vel_target_deg.bL_knee_velo + joint_initial_pos_deg.bL_knee_pos;
#endif
}

bool jointPositions::checkPos(char mode, ODriveArduino *front_AB_ptr, ODriveArduino *front_HIP_ptr, ODriveArduino *front_KNEE_ptr,
                              ODriveArduino *back_AB_ptr, ODriveArduino *back_HIP_ptr, ODriveArduino *back_KNEE_ptr)
{
    if (mode == STANDBY_POS_FLAG)
    {
        initialize_moveTo_constantPos(mode, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);

        if (
#ifdef ENABLE_FRONT_LEFT
            (joint_vel_target_deg.fL_ab_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fL_ab_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fL_hip_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fL_hip_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fL_knee_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fL_knee_velo > -POS_CONVERSION_MIN_SPEED)
#endif
#ifdef ENABLE_FRONT_RIGHT
            && (joint_vel_target_deg.fR_ab_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fR_ab_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fR_hip_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fR_hip_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fR_knee_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.fR_knee_velo > -POS_CONVERSION_MIN_SPEED)
#endif
#ifdef ENABLE_BACK_LEFT
            && (joint_vel_target_deg.bL_ab_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bL_ab_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bL_hip_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bL_hip_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bL_knee_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bL_knee_velo > -POS_CONVERSION_MIN_SPEED)
#endif
#ifdef ENABLE_BACK_RIGHT
            && (joint_vel_target_deg.bR_ab_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bR_ab_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bR_hip_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bR_hip_velo > -POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bR_knee_velo < POS_CONVERSION_MIN_SPEED && joint_vel_target_deg.bR_knee_velo > -POS_CONVERSION_MIN_SPEED)
#endif
        )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

void jointPositions::update_traceTraj(int tick_fL, int tick_fR, int tick_bL, int tick_bR, int set_selection)
{
    if (set_selection == 1)
    {
#ifdef ENABLE_FRONT_LEFT
    joint_pos_target_deg.fL_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.fL_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.fL_hip_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_pos_1[tick_fL];
    joint_vel_target_deg.fL_hip_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_velo_1[tick_fL];
    joint_pos_target_deg.fL_knee_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_pos_1[tick_fL];
    joint_vel_target_deg.fL_knee_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_velo_1[tick_fL];
#endif
#ifdef ENABLE_FRONT_RIGHT
    joint_pos_target_deg.fR_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.fR_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.fR_hip_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_pos_1[tick_fR];
    joint_vel_target_deg.fR_hip_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_velo_1[tick_fR];
    joint_pos_target_deg.fR_knee_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_pos_1[tick_fR];
    joint_vel_target_deg.fR_knee_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_velo_1[tick_fR];
#endif
#ifdef ENABLE_BACK_LEFT
    joint_pos_target_deg.bL_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.bL_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.bL_hip_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_pos_1[tick_bL];
    joint_vel_target_deg.bL_hip_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_velo_1[tick_bL];
    joint_pos_target_deg.bL_knee_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_pos_1[tick_bL];
    joint_vel_target_deg.bL_knee_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_velo_1[tick_bL];
#endif
#ifdef ENABLE_BACK_RIGHT
    joint_pos_target_deg.bR_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.bR_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.bR_hip_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_pos_1[tick_bR];
    joint_vel_target_deg.bR_hip_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_velo_1[tick_bR];
    joint_pos_target_deg.bR_knee_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_pos_1[tick_bR];
    joint_vel_target_deg.bR_knee_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_velo_1[tick_bR];
#endif
    }
    else if (set_selection == 2)
    {
#ifdef ENABLE_FRONT_LEFT
    joint_pos_target_deg.fL_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.fL_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.fL_hip_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_pos_2[tick_fL];
    joint_vel_target_deg.fL_hip_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_velo_2[tick_fL];
    joint_pos_target_deg.fL_knee_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_pos_2[tick_fL];
    joint_vel_target_deg.fL_knee_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_velo_2[tick_fL];
#endif
#ifdef ENABLE_FRONT_RIGHT
    joint_pos_target_deg.fR_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.fR_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.fR_hip_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_pos_2[tick_fR];
    joint_vel_target_deg.fR_hip_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_velo_2[tick_fR];
    joint_pos_target_deg.fR_knee_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_pos_2[tick_fR];
    joint_vel_target_deg.fR_knee_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_velo_2[tick_fR];
#endif
#ifdef ENABLE_BACK_LEFT
    joint_pos_target_deg.bL_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.bL_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.bL_hip_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_pos_2[tick_bL];
    joint_vel_target_deg.bL_hip_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_velo_2[tick_bL];
    joint_pos_target_deg.bL_knee_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_pos_2[tick_bL];
    joint_vel_target_deg.bL_knee_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_velo_2[tick_bL];
#endif
#ifdef ENABLE_BACK_RIGHT
    joint_pos_target_deg.bR_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.bR_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.bR_hip_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_pos_2[tick_bR];
    joint_vel_target_deg.bR_hip_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_velo_2[tick_bR];
    joint_pos_target_deg.bR_knee_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_pos_2[tick_bR];
    joint_vel_target_deg.bR_knee_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_velo_2[tick_bR];
#endif
    }
    else if (set_selection == 3)
    {
#ifdef ENABLE_FRONT_LEFT
    joint_pos_target_deg.fL_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.fL_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.fL_hip_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_pos_3[tick_fL];
    joint_vel_target_deg.fL_hip_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_velo_3[tick_fL];
    joint_pos_target_deg.fL_knee_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_pos_3[tick_fL];
    joint_vel_target_deg.fL_knee_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_velo_3[tick_fL];
#endif
#ifdef ENABLE_FRONT_RIGHT
    joint_pos_target_deg.fR_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.fR_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.fR_hip_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_pos_3[tick_fR];
    joint_vel_target_deg.fR_hip_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_velo_3[tick_fR];
    joint_pos_target_deg.fR_knee_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_pos_3[tick_fR];
    joint_vel_target_deg.fR_knee_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_velo_3[tick_fR];
#endif
#ifdef ENABLE_BACK_LEFT
    joint_pos_target_deg.bL_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.bL_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.bL_hip_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_pos_3[tick_bL];
    joint_vel_target_deg.bL_hip_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_hip_velo_3[tick_bL];
    joint_pos_target_deg.bL_knee_pos = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_pos_3[tick_bL];
    joint_vel_target_deg.bL_knee_velo = LEG_LEFT_POS_MULTIPLIER * jtraj_knee_velo_3[tick_bL];
#endif
#ifdef ENABLE_BACK_RIGHT
    joint_pos_target_deg.bR_ab_pos = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_vel_target_deg.bR_ab_velo = LEG_LEFT_POS_MULTIPLIER * (float)0.0;
    joint_pos_target_deg.bR_hip_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_pos_3[tick_bR];
    joint_vel_target_deg.bR_hip_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_hip_velo_3[tick_bR];
    joint_pos_target_deg.bR_knee_pos = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_pos_3[tick_bR];
    joint_vel_target_deg.bR_knee_velo = LEG_RIGHT_POS_MULTIPLIER * jtraj_knee_velo_3[tick_bR];
#endif
    }
}