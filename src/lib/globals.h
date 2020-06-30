#pragma once
// #include <cmath>
#include <BasicLinearAlgebra.h>
#include <Arduino.h>
// Development modes:
#define DEBUG_SERIAL
// #define NORMAL_OPERATION

#define ENABLE_FRONT_LEFT
#define ENABLE_FRONT_RIGHT
#define ENABLE_BACK_LEFT
#define ENABLE_BACK_RIGHT

#if defined(ENABLE_FRONT_LEFT) || defined(ENABLE_FRONT_RIGHT)
  #define ENABLE_FRONT
#endif

#if defined(ENABLE_BACK_LEFT) || defined(ENABLE_BACK_RIGHT)
  #define ENABLE_BACK
#endif

#define SERIAL_BAUD_RATE 921600
#define SERIAL_MSG_TIMER 5
#define SERIAL_MSG_TIME_INTERVAL (float)5/1000

#define LEFT 'L'
#define RIGHT 'R'
#define AB 'a'
#define HIP 'h'
#define KNEE 'k'

#define KNEE_GearRatio 1.25f
#define DIMENSION_A2 0.218f
#define DIMENSION_A3 0.230f
#define DIMENSION_D2 0.092f
#define DIMENSION_LENGTH 0.517f
#define DIMENSION_WIDTH 0.2f

#define FRONT_LEFT_LEG 'q'
#define FRONT_RIGHT_LEG 'w'
#define BACK_LEFT_LEG 'e'
#define BACK_RIGHT_LEG 'r'

#define STANDBY_POS_FLAG 's'
#define POS_CONVERSION_TIME (float)5.0
#define POS_CONVERSION_MIN_SPEED (float)0.1
// #define AB_STANDBY_POS_DEG (float)7.99
// #define HIP_STANDBY_POS_DEG (float)27.67
// #define KNEE_STANDBY_POS_DEG (float)68.79
#define AB_STANDBY_POS_DEG (float)4.85
#define HIP_STANDBY_POS_DEG (float)32.84
#define KNEE_STANDBY_POS_DEG (float)72.95

#define AB_POS_1 (float)-14.6465
#define HIP_POS_1 (float)27.5516
#define KNEE_POS_1 (float)53.5543

#define AB_POS_2 (float)15.3339
#define HIP_POS_2 (float)48.998
#define KNEE_POS_2 (float)94.6667

#define LEG_LEFT_POS_MULTIPLIER (short int)-1
#define LEG_RIGHT_POS_MULTIPLIER (short int)1

#define PI_math (float)3.141592653

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

const float knee_gearRatio = 1.25;
const float dimension_a2 = 0.218;
const float dimension_a3 = 0.230;
const float dimension_d2 = 0.092;
const float dimension_length = 0.517;
const float dimension_width = 0.2;


enum leg_tag
{
    tag_FL = 1,
    tag_FR = 2,
    tag_BL = 4,
    tag_BR = 8
};         

enum program_state
{
  LOOP_IDLE = 0,
  LOOP_K_UPDATE = 1,
  LOOP_UPDATE = 3
};

enum qPed_state
{
  STATE_LOW = 1,
  STATE_LOW_TO_IDLE = 2,
  STATE_IDLE = 3,
  STATE_IDLE_TO_LOW = 4,
  STATE_POST = 5,
  STATE_POST_TO_IDLE = 6,
  STATE_WALK = 7,
};

enum input_mode
{
  MODE_END = 0,
  MODE_CONT = 1
};

enum walk_task
{
  TASK_IDLE = 0,
  TASK_SHIFT_POSTURE = 1,
  TASK_SWING_LEG = 2
};





const float msg_timer_interval = 5; // interval between sending msg (millisecond)

#define KNEE_GearRatio 1.25f
#define DIMENSION_A2 0.218f
#define DIMENSION_A3 0.230f
#define DIMENSION_D2 0.092f
#define DIMENSION_LENGTH 0.517f
#define DIMENSION_WIDTH 0.2f

long manual_input_num();
String rdStr();