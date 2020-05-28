#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include "globals.h"

/**
 * Homo-transformation from ground frame to body frame
 * @note At DEFAULT POSITION!
 */
const BLA::Matrix<4,4> T_ground2body_default = {1, 0, 0, 0,
                                                0, 1, 0, 0,
                                                0, 0, 1, 0.38,
                                                0, 0, 0, 1};
/**
 *  Homo-transformation from body frame to AB motors
 *  These are constant regardless the body posture or leg positions
 */
const BLA::Matrix<4,4> T_body2FLab ={0, 0, 1, DIMENSION_LENGTH/2,
                                     0, 1, 0, DIMENSION_WIDTH/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};

const BLA::Matrix<4,4> T_body2FRab ={0, 0, 1, DIMENSION_LENGTH/2,
                                     0, 1, 0,-DIMENSION_WIDTH/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};

const BLA::Matrix<4,4> T_body2BLab ={0, 0,-1,-DIMENSION_LENGTH/2,
                                     0,-1, 0, DIMENSION_WIDTH/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};

const BLA::Matrix<4,4> T_body2BRab ={0, 0,-1,-DIMENSION_LENGTH/2,
                                     0,-1, 0,-DIMENSION_WIDTH/2,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1};
/**
 * Homo-transformation from  AB motors to LEG ends
 * @note At DEFAULT POSITION!
 */
const BLA::Matrix<4,4> T_FLab2end ={-0.120887, -0.992666, 0, 0.38,
                                     0, 0, 1, 0.092,
                                    -0.992666, 0.120887, 0, 0,
                                     0, 0, 0, 1};

const BLA::Matrix<4,4> T_FRab2end ={-0.856842, 0.515579, 0, 0.38,
                                     0, 0, 1, -0.092,
                                    0.515579, -0.856842, 0, 0,
                                     0, 0, 0, 1};

const BLA::Matrix<4,4> T_BLab2end ={0.856842, -0.515579, 0, 0.38,
                                     0, 0, 1, -0.092,
                                    -0.515579, -0.856842, 0, 0,
                                     0, 0, 0, 1};

const BLA::Matrix<4,4> T_BRab2end ={-0.120887, 0.992666, 0, 0.38,
                                     0, 0, 1, 0.092,
                                    0.992666, 0.120887, 0, 0,
                                     0, 0, 0, 1};
/**
 * Homo-transformation from ground to LEG ends
 * @note At DEFAULT POSITION!
 */
const BLA::Matrix<4,4> T_ground2FLend = T_ground2body_default * T_body2FLab * T_FLab2end;
const BLA::Matrix<4,4> T_ground2FRend = T_ground2body_default * T_body2FRab * T_FRab2end;
const BLA::Matrix<4,4> T_ground2BLend = T_ground2body_default * T_body2BLab * T_BLab2end;
const BLA::Matrix<4,4> T_ground2BRend = T_ground2body_default * T_body2BRab * T_BRab2end;

/**
 * Position vector from ground to LEG ends
 * @note At DEFAULT POSITION!
 */
const BLA::Matrix<4> P_ground2FLend = {T_ground2FLend(0,3), T_ground2FLend(1,3), T_ground2FLend(2,3), 1};
const BLA::Matrix<4> P_ground2FRend = {T_ground2FRend(0,3), T_ground2FRend(1,3), T_ground2FRend(2,3), 1};
const BLA::Matrix<4> P_ground2BLend = {T_ground2BLend(0,3), T_ground2BLend(1,3), T_ground2BLend(2,3), 1};
const BLA::Matrix<4> P_ground2BRend = {T_ground2BRend(0,3), T_ground2BRend(1,3), T_ground2BRend(2,3), 1};

void leg_forwardKinematics_pos(float ab_deg_now, float hip_deg_now, float knee_deg_now,
                           float &x_calculated, float &y_calculated, float &z_calculated, 
                           char leg_choice);

bool leg_inverseKinematics_pos(float ab_deg_now, float hip_deg_now, float knee_deg_now,
                               float x_desired, float y_desired, float z_desired,
                               float &ab_deg_desired, float &hip_deg_desired, float &knee_deg_desired,
                               char leg_choice);

/**
 * Find the desired joint angles for a desired body posture
 * The function assumes that 1. the robot is operating on flat surfaces
 * 2. body is leveled when all leg's extension can be described by the same vector in ground frame
 */
bool calc_posture_joint_pos(float yaw_desired, float pitch_desired, float roll_desired,
                            float forward_delta, float horizontal_delta, float vertical_delta,
                            float &FL_ab, float &FL_hip, float &FL_knee, bool FL_support, 
                            float &FR_ab, float &FR_hip, float &FR_knee, bool FR_support, 
                            float &BR_ab, float &BR_hip, float &BR_knee, bool BL_support, 
                            float &BL_ab, float &BL_hip, float &BL_knee, bool BR_support);