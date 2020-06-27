#pragma once

#include <Arduino.h>
#include "globals.h"

class radio
{
public:
    struct channel{
        float max;
        float min;
        float range;
        float default_val;
        float blind_zone;
        float val;
        bool reverse_output = false;
        bool is_ch3 = false;
        void ini(float max_, float min_, float blind_, bool is_ch3_, bool reverse_output_)
        {
            max = max_;
            min = min_;
            blind_zone = blind_;
            range = max - min - 2*blind_zone;
            if (!is_ch3_)
            {
                default_val = (max + min)/2;
            }
            else
            {
                is_ch3 = true;
                default_val = min;
            }
            val = default_val;
            if (reverse_output_)
            {
                reverse_output = true;
            }
        }
        void update(float reading)
        {
            bool is_center = false;
            if (!is_ch3)
            {
                is_center = (reading >= default_val - blind_zone) & (reading <= default_val + blind_zone);
            }
            bool is_max = false;
            bool is_min = false;
            if (!is_center)
            {
                is_max = reading >= max - blind_zone;
            }
            if (!is_max)
            {
                is_min = reading <= min + blind_zone;
            }
            if (is_max)
            {
                val = 1;
            }
            else if (is_min)
            {
                val = -1;
            }
            else if (is_center)
            {
                val = 0;
            }
            else
            {
                if (reading > default_val + blind_zone)
                {
                    val = reading - (default_val + blind_zone);
                    val = val / range;
                }
                else
                {
                    val = reading - (default_val - blind_zone);
                    val = val / range;
                }
            }
            if (reverse_output)
            {
                val = -val;
            }
        }
    };
    channel ch_1;
    channel ch_2;
    channel ch_3;
    channel ch_4;


    /**
     * Constructor
     */
    radio();

    /**
     * Initialize routine
     * @return true if initialization was successful
     */
    void ini();

    /**
     * Update routine
     * @note called every loop cycle
     */
    void update();

    /**
     * Manually initialize routine
     * @note for debug only
     */
    void debug_ini(float max_1 = 2000, float min_1 = 1000, float blind_1 = 10,
                   float max_2 = 2000, float min_2 = 1000, float blind_2 = 10,
                   float max_3 = 2000, float min_3 = 1000, float blind_3 = 10,
                   float max_4 = 2000, float min_4 = 1000, float blind_4 = 10);

    /**
     * Manually set value for the channels
     * @note for debug only
     */
    void debug_update(float ch_1_, float ch_2_, float ch_3_, float ch_4_);
};