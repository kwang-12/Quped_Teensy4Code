#include "leg_timer.h"

leg_timer::leg_timer(float max_time, int update_interval, int max_tick)
{
    time_max_val_ = max_time;
    update_interval_ = update_interval;
    max_tick_ = max_tick;
    timer = 0;
    // timer = Metro(update_interval_);
    // timer.reset();
}

void leg_timer::update()
{
    if(timer >= update_interval_*1000)
    {
    // Serial.println(time,4);
    timer = timer - update_interval_*1000;
        tick++;
        if (tick < max_tick_)
        {
            time = static_cast<float>(tick * update_interval_) / 1000;
        }
        else if (tick == max_tick_)
        {
            time = time_max_val_;
        }
        else
        {
            tick = 1;
            time = static_cast<float>(tick * update_interval_) / 1000;
        }
    }
}

void leg_timer::reset()
{
    time = 0;
    tick = 0;
    timer = 0;
    // timer.reset();
}

float leg_timer::get_time()
{
    return time;
}