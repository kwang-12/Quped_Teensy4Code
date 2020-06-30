#pragma once

#include <Arduino.h>
#include "globals.h"

class radio
{
public:
    struct channel{
        int ch_pin;
        volatile bool state;
        volatile long timer;
        volatile long signal; 
        long max;
        long min;
        long range;
        long range_upper;
        long range_lower;
        long default_val;
        long blind_zone;
        float val;
        bool reverse_output = false;
        bool is_ch3 = false;
        void ini(int ch_pin_, long blind_, bool is_ch3_, bool reverse_output_)
        {
            ch_pin = ch_pin_;
            blind_zone = blind_;
            is_ch3 = is_ch3_;
            reverse_output = reverse_output_;
        }

        void calibrate()
        {
            long val_1,val_2;
            while (true)
            {
                Serial.println("Push the stick to boundary value 1...");
                Serial.println("Enter yes to capture the boundary value...");
                while (Serial.available() == 0);
                String serial_input = rdStr();
                if (serial_input == "yes")
                {
                    val_1 = signal;
                    while (true)
                    {
                        Serial.println("Push the stick to boundary value 2...");
                        Serial.println("Enter yes to capture the boundary value...");
                        while (Serial.available() == 0);
                        serial_input = rdStr();
                        if (serial_input == "yes")
                        {
                            val_2 = signal;
                            break;
                        }
                    }
                    break;
                }
            }
            Serial.print("Captured boundary value 1 is: ");
            Serial.println(val_1);
            Serial.print("Captured boundary value 2 is: ");
            Serial.println(val_2);
            if (val_1 > val_2)
            {
                max = val_1;
                min = val_2;
            }
            else
            {
                max = val_2;
                min = val_1;
            }
            if (!is_ch3)
            {
                default_val = (max+min)/2;
                range_upper = max-default_val-2*blind_zone;
                range_lower = default_val-min-2*blind_zone;
            }
            else
            {
                default_val = min;
                range_upper = max-default_val-2*blind_zone;
                range_lower = range_upper;
            }
            Serial.print("max: ");
            Serial.print(max);
            Serial.print(" default: ");
            Serial.print(default_val);
            Serial.print(" min: ");
            Serial.println(min);
            
        }

        void update(long reading)
        {
            cli();
            bool is_center = false;
            if (!is_ch3)
            {
                is_center = (reading>=default_val-blind_zone) & (reading<=default_val+blind_zone);
            }
            bool is_max = reading>=max-blind_zone;
            bool is_min = reading<=min+blind_zone;
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
                if (is_ch3)
                {
                        val = reading - (max+min)/2;
                        val = val / range_upper * 2;
                }
                else
                {
                    if (reading > default_val+blind_zone)
                    {
                        val = reading - (default_val+blind_zone);
                        val = val / range_upper;
                    }
                    else
                    {
                        val = reading - (default_val-blind_zone);
                        val = val / range_lower;
                    }
                }
            }
            if (reverse_output)
            {
                val = -val;
            }
            sei();
        }

        void reset()
        {
            cli();
            state = false;
            signal = 0;
            timer = 0;
            sei();
        }

        void ISR()
        {
            cli();
            if (state && digitalRead(ch_pin)==0)
            {
                state = false;
                signal = micros() - timer;
            }
            else if (!state && digitalRead(ch_pin)==1)
            {
                state = true;
                timer = micros();
            }
            sei();
        }

        void debug_ini( long max_,  long min_,  long blind_, 
                       bool is_ch3_, bool reverse_output_)
        {
            max = max_;
            min = min_;
            blind_zone = blind_;
            range = max - min - 2*blind_zone;
            if (!is_ch3_)
            {
                default_val = (max + min)/2;
                range_upper = max-default_val-2*blind_zone;
                range_lower = max-default_val-2*blind_zone;
            }
            else
            {
                is_ch3 = true;
                default_val = min;
                range_upper = (max-default_val-2*blind_zone)/2;
                range_lower = (max-default_val-2*blind_zone)/2;
            }
            val = default_val;
            if (reverse_output_)
            {
                reverse_output = true;
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
    void ini(int ch_1_pin, int ch_2_pin, int ch_3_pin, int ch_4_pin,
             long blind_1,  long blind_2, 
             long blind_3,  long blind_4);

    /**
     * Update routine
     * @note called every loop cycle
     */
    void update();

    /**
     * Channel calibration
     */
    void calibrate();

    /**
     * Reset signal readings
     */
    void reset();

    /**
     * Manually initialize routine
     * @note for debug only
     */
    void debug_ini( long max_1 = 2000,  long min_1 = 1000,  long blind_1 = 10,
                    long max_2 = 2000,  long min_2 = 1000,  long blind_2 = 10,
                    long max_3 = 2000,  long min_3 = 1000,  long blind_3 = 10,
                    long max_4 = 2000,  long min_4 = 1000,  long blind_4 = 10);

    /**
     * Manually set value for the channels
     * @note for debug only
     */
    void debug_update( long ch_1_,  long ch_2_,  long ch_3_,  long ch_4_);
};