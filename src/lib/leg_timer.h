#pragma once
#include <Arduino.h>
#include "globals.h"
#include <Metro.h>

/**
 * An object that counts time in loop fashion
 * The counted time loops from 0 to the maximum value and repeats
 */
class leg_timer
{
private:
    float time = 0.0;           // the time value that's being tracked  time = tick * update_interval
    int tick = 0;               // the tick value that's being tracked

    float time_max_val_ = 0.0;   // the maximum value of time
    unsigned int update_interval_ = 5;
    int max_tick_ = 0;

    elapsedMicros timer;

    // Metro timer = Metro(update_interval_);
public:
    /**
     * Constructor
     * @param max_time the maximum value of the counted time
     * @param update_interval update time interval in millis
     * @param max_tick maximum number of ticks
     */
    leg_timer(float max_time, int update_interval, int max_tick);

    /**
     * update internal time counter
     * must be ran every cycle
     */ 
    void update();

    /**
     * Reset the internal time value and time checker
     */
    void reset();

    /**
     * Get current time value
     */
    float get_time();

};