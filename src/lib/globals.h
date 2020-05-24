#pragma once
// #include <cmath>
#include <arduino.h>
// Development modes:
#define DEBUG_SERIAL
// #define NORMAL_OPERATION

#define ENABLE_FRONT_LEFT
// #define ENABLE_FRONT_RIGHT
// #define ENABLE_BACK_LEFT
// #define ENABLE_BACK_RIGHT

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

#define FRONT_LEFT_LEG 'q'
#define FRONT_RIGHT_LEG 'w'
#define BACK_LEFT_LEG 'e'
#define BACK_RIGHT_LEG 'r'


#define STANDBY_POS_FLAG 's'
#define POS_CONVERSION_TIME (float)5.0
#define POS_CONVERSION_MIN_SPEED (float)0.1
#define AB_STANDBY_POS_DEG (float)0.0
#define HIP_STANDBY_POS_DEG (float)40.0
#define KNEE_STANDBY_POS_DEG (float)80.0

#define AB_POS_1 (float)-14.6465
#define HIP_POS_1 (float)27.5516
#define KNEE_POS_1 (float)53.5543

#define AB_POS_2 (float)15.3339
#define HIP_POS_2 (float)48.998
#define KNEE_POS_2 (float)94.6667

#define LEG_LEFT_POS_MULTIPLIER (short int)-1
#define LEG_RIGHT_POS_MULTIPLIER (short int)1

/**
 * Different choice of gaits
 */
#define TROT_GAIT 't'
#define PACE_GAIT 'p'
#define BOUND_GAIT 'b'
#define CRAWL_GAIT 'c'    // Standard crawl gait where each leg is off-phased by 0.25
#define WAVE_GAIT 'w'

/**
 * State of the gaits
 * Default state: GAIT_IDLE - Indicates that the leg is stationary and holding at default position
 */
#define GAIT_IDLE 'i'
#define GAIT_STARTING 's'
#define GAIT_ENDING 'e'
#define GAIT_NORMAL 'n'
#define GAIT_ONESTEP '1'

/**
 * State-machine states
 */
#define STATE_IDLE 'z'
#define STATE_1_STEP '1'
#define STATE_CONTINUOUS 'c'
#define STATE_POS_1 'q'
#define STATE_POS_2 'w'
#define STATE_test 'e'

#define PI_math (float)3.141592653

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }
// template <class T>
// inline Print &operator<<(Print &obj, T arg)
// {
//   obj.print(arg);
//   return obj;
// }
// template <>
// inline Print &operator<<(Print &obj, float arg)
// {
//   obj.print(arg, 4);
//   return obj;
// }
