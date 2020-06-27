#include "radio.h"

radio::radio()
{
}

void radio::ini()
{
}

void radio::update()
{
}

void radio::debug_ini(float max_1, float min_1, float blind_1, 
                      float max_2, float min_2, float blind_2,
                      float max_3, float min_3, float blind_3,
                      float max_4, float min_4, float blind_4)
{
    ch_1.ini(max_1, min_1, blind_1, false, false);
    ch_2.ini(max_2, min_2, blind_2, false, true);
    ch_3.ini(max_3, min_3, blind_3, true, false);
    ch_4.ini(max_4, min_4, blind_4, false, true);
}

void radio::debug_update(float ch_1_, float ch_2_, float ch_3_, float ch_4_)
{
    ch_1.update(ch_1_);
    ch_2.update(ch_2_);
    ch_3.update(ch_3_);
    ch_4.update(ch_4_);
}