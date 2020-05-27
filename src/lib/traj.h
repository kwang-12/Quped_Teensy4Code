#pragma once
#include <Arduino.h>
#include "globals.h"

/**
 * An object that defines the leg's cartesian trajectory
 * Two types are allowed:
 * 1. Interpolated trajectory between two points
 * 2. Analytical trajectory define by cosine
 */
class traj
{
private:
    enum traj_type
    {
        INTERPOLATED = 0,
        ANALYTICAL = 1
    };
    traj_type type_ = INTERPOLATED;

    /**
     * interpolated trajectory defined as:
     * a straight cartesian trajectory interpolated between two points
     */

    /**
     *  analytical cosine function defined as:
     *  func = para_A_ * cos(para_alpha_ * time + para_beta_) + para_b_
     *  time is dependent of user input
     */
    float para_A_ = 0;
    float para_alpha_ = 0;
    float para_beta_ = 0;
    float para_b_;


public:
};