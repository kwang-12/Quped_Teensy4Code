#pragma once
// Development modes:
#define DEBUG_SERIAL
// #define NORMAL_OPERATION

// Firmware dependent variables:
#define SERIAL_BAUD_RATE 921600
#define LEFT 'L'
#define RIGHT 'R'
#define AB 'a'
#define HIP 'h'
#define KNEE 'k'
#define KNEE_GearRatio 1.25

template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}