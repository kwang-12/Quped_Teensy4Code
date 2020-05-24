#pragma once
#include <Arduino.h>
#include <math.h>
#include "globals.h"

void leg_forwardKinematics(float ab_deg_now, float hip_deg_now, float knee_deg_now,
                           float &x_calculated, float &y_calculated, float &z_calculated, 
                           char leg_choice);

void leg_inverseKinematics(float ab_deg_now, float hip_deg_now, float knee_deg_now,
                           float x_desired, float y_desired, float z_desired,
                           float &ab_deg_desired, float &hip_deg_desired, float &knee_deg_desired,
                           char leg_choice);
