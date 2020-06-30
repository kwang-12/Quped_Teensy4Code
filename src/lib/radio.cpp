#include "radio.h"

radio::radio()
{
}

void radio::ini(int ch_1_pin, int ch_2_pin, int ch_3_pin, int ch_4_pin,
                long blind_1,  long blind_2, 
                long blind_3,  long blind_4)
{
    ch_1.ini(ch_1_pin, blind_1, false, false);
    ch_2.ini(ch_2_pin, blind_2, false, true);
    ch_3.ini(ch_3_pin, blind_3, true, false);
    ch_4.ini(ch_4_pin, blind_4, false, true);
}

void radio::update()
{
    if (ch_1.signal > (ch_1.max+50) || ch_1.signal < (ch_1.min-50))
    {
        ch_1.reset();
    }
    else
    {
        ch_1.update(ch_1.signal);
    }
    if (ch_2.signal > (ch_2.max+50) || ch_2.signal < (ch_2.min-50))
    {
        ch_2.reset();
    }
    else
    {
        ch_2.update(ch_2.signal);
    }
    if (ch_3.signal > (ch_3.max+50) || ch_3.signal < (ch_3.min-50))
    {
        ch_3.reset();
    }
    else
    {
        ch_3.update(ch_3.signal);
    }
    if (ch_4.signal > (ch_4.max+50) || ch_4.signal < (ch_4.min-50))
    {
        ch_4.reset();
    }
    else
    {
        ch_4.update(ch_4.signal);
    }
}

void radio::calibrate()
{
    Serial.println("Calibrating radio channels...");
    Serial.println("Channel 1...");
    ch_1.calibrate();
    Serial.println("Channel 2...");
    ch_2.calibrate();
    Serial.println("Channel 3...");
    ch_3.calibrate();
    Serial.println("Channel 4...");
    ch_4.calibrate();
}

void radio::reset()
{
    ch_1.reset();
    ch_2.reset();
    ch_3.reset();
    ch_4.reset();
}

void radio::debug_ini( long max_1,  long min_1,  long blind_1, 
                       long max_2,  long min_2,  long blind_2,
                       long max_3,  long min_3,  long blind_3,
                       long max_4,  long min_4,  long blind_4)
{
    ch_1.debug_ini(max_1, min_1, blind_1, false, false);
    ch_2.debug_ini(max_2, min_2, blind_2, false, true);
    ch_3.debug_ini(max_3, min_3, blind_3, true, false);
    ch_4.debug_ini(max_4, min_4, blind_4, false, true);
}

void radio::debug_update( long ch_1_,  long ch_2_,  long ch_3_,  long ch_4_)
{
    ch_1.update(ch_1_);
    ch_2.update(ch_2_);
    ch_3.update(ch_3_);
    ch_4.update(ch_4_);
}