// Libraries:
#include "src/lib/globals.h"
#include "src/lib/ODriveArduino.h"

// Development modes:
#define DEBUG_SERIAL
//#define NORMAL_OPERATION

// Firmware dependent variables:
#define SERIAL_BAUD_RATE 921600
#define LEFT 'L'
#define RIGHT 'R'

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

// ODrive object
ODriveArduino back_KNEE(Serial1, 1, RIGHT, LEFT, SERIAL_BAUD_RATE);  //knee back, axis0-RB, axis1-LB
ODriveArduino back_HIP(Serial2, 2, RIGHT, LEFT, SERIAL_BAUD_RATE);   //hip back, axis0-RB, axis1-LB
ODriveArduino back_AB(Serial3, 3, LEFT, RIGHT, SERIAL_BAUD_RATE);    //ab back, axis0-LB, axis1-RB
ODriveArduino front_KNEE(Serial4, 4, RIGHT, LEFT, SERIAL_BAUD_RATE); //knee front, axis0-RB, axis1-LB
ODriveArduino front_HIP(Serial5, 5, LEFT, RIGHT, SERIAL_BAUD_RATE);  //hip front, axis0-LB, axis1-RB
ODriveArduino front_AB(Serial7, 7, LEFT, RIGHT, SERIAL_BAUD_RATE);   //ab front, axis0-LB, axis1-RB
// Constants

// Variables
String serial_input; // store the input from computer monitor port
char state = 'z';    // store the state of loop program. By default, 'z' means do nothing.

// Debug Variables
#ifdef DEBUG_SERIAL
#endif

void setup()
{
  // Serial to PC @ 921600 baud rate
  Serial.begin(1000000);
  while (!Serial)
    ; // wait for Arduino Serial Monitor to open
  Serial.println("Ready!");

  // Serial to odrives @ 921600 baud rate
  while (!back_KNEE.ini())
  {
#ifdef DEBUG_SERIAL
    back_KNEE.readAxisError(RIGHT);
    back_KNEE.readAxisError(LEFT);
    if (back_KNEE.axis0_error_ > 0)
    {
      Serial.print("back_KNEE: Axis-0 motor error code:");
      Serial.println(back_KNEE.axis0_error_);
    }
    else if (back_KNEE.axis1_error_ > 0)
    {
      Serial.print("back_KNEE: Axis-1 motor error code:");
      Serial.println(back_KNEE.axis1_error_);
    }
#endif
  }
  while (!back_HIP.ini())
  {
#ifdef DEBUG_SERIAL
    back_HIP.readAxisError(RIGHT);
    back_HIP.readAxisError(LEFT);
    if (back_HIP.axis0_error_ > 0)
    {
      Serial.print("back_HIP: Axis-0 motor error code:");
      Serial.println(back_HIP.axis0_error_);
    }
    else if (back_HIP.axis1_error_ > 0)
    {
      Serial.print("back_HIP: Axis-1 motor error code:");
      Serial.println(back_HIP.axis1_error_);
    }
#endif
  }
  while (!back_AB.ini())
  {
#ifdef DEBUG_SERIAL
    back_AB.readAxisError(RIGHT);
    back_AB.readAxisError(LEFT);
    if (back_AB.axis0_error_ > 0)
    {
      Serial.print("back_AB: Axis-0 motor error code:");
      Serial.println(back_AB.axis0_error_);
    }
    else if (back_AB.axis1_error_ > 0)
    {
      Serial.print("back_AB: Axis-1 motor error code:");
      Serial.println(back_AB.axis1_error_);
    }
#endif
  }
  while (!front_KNEE.ini())
  {
#ifdef DEBUG_SERIAL
    front_KNEE.readAxisError(RIGHT);
    front_KNEE.readAxisError(LEFT);
    if (front_KNEE.axis0_error_ > 0)
    {
      Serial.print("front_KNEE: Axis-0 motor error code:");
      Serial.println(front_KNEE.axis0_error_);
    }
    else if (front_KNEE.axis1_error_ > 0)
    {
      Serial.print("front_KNEE: Axis-1 motor error code:");
      Serial.println(front_KNEE.axis1_error_);
    }
#endif
  }
  while (!front_HIP.ini())
  {
#ifdef DEBUG_SERIAL
    front_HIP.readAxisError(RIGHT);
    front_HIP.readAxisError(LEFT);
    if (front_HIP.axis0_error_ > 0)
    {
      Serial.print("front_HIP: Axis-0 motor error code:");
      Serial.println(front_HIP.axis0_error_);
    }
    else if (front_HIP.axis1_error_ > 0)
    {
      Serial.print("front_HIP: Axis-1 motor error code:");
      Serial.println(front_HIP.axis1_error_);
    }
#endif
  }
  while (!front_AB.ini())
  {
#ifdef DEBUG_SERIAL
    front_AB.readAxisError(RIGHT);
    front_AB.readAxisError(LEFT);
    if (front_AB.axis0_error_ > 0)
    {
      Serial.print("front_AB: Axis-0 motor error code:");
      Serial.println(front_AB.axis0_error_);
    }
    else if (front_AB.axis1_error_ > 0)
    {
      Serial.print("front_AB: Axis-1 motor error code:");
      Serial.println(front_AB.axis1_error_);
    }
#endif
  }
}

void loop()
{
#ifdef DEBUG_SERIAL
  if (Serial.available() > 0)
  {
    serial_input = Serial.readString();
    if (serial_input == "ba")
    {}
    else if (serial_input == "bh")
    {}
    else if (serial_input == "bk")
    {
      Serial.println("Back Knee:");
      while (Serial.available() == 0);
      {
        serial_input = Serial.readString();
        if (serial_input != 'i')
        {
          Serial.println(serial_input);
          back_KNEE.EnterCommand(serial_input);
          Serial.println(back_KNEE.readString());
        }
        else
        {
          Serial.println("Back Knee ODrv info:");
          back_KNEE.EnterCommand(serial_input);
          Serial.println(back_KNEE.readString());
          Serial.println(back_KNEE.readString());
          Serial.println(back_KNEE.readString());
        }
      }
    }
    else if (serial_input == "fa")
    {}
    else if (serial_input == "fh")
    {} 
    else if (serial_input == "fk")
    {} 
    else
    {
      Serial.println("Specify the odrive to talk to by entering one of the following:");
      Serial.println("ba, bh, bk, fa, fh, fk");
    }
  }
#endif
    // //  Serial.println(state);
    //   if (Serial.available() > 0)
    //   {
    //     serial_input = Serial.readString();
    //     // one time operations
    //     if (serial_input == "i1")
    //     { // read back_KNEE's hw/sw version and serial num
    //       Serial1 << "i" << '\n';
    //       Serial << "ODRV-1 INFO:" << '\n';
    //       Serial << back_KNEE.readString() << '\n';
    //       Serial << back_KNEE.readString() << '\n';
    //       Serial << back_KNEE.readString() << '\n';
    //       state = 'z'; // do nothing
    //     }
    //     else if (serial_input == "i2")
    //     { // read back_HIP's hw/sw version and serial num
    //       Serial2 << "i" << '\n';
    //       Serial << "ODRV-2 INFO:" << '\n';
    //       Serial << back_HIP.readString() << '\n';
    //       Serial << back_HIP.readString() << '\n';
    //       Serial << back_HIP.readString() << '\n';
    //       state = 'z'; // do nothing
    //     }
    //     else if (serial_input == "i3")
    //     { // read back_AB's hw/sw version and serial num
    //       Serial3 << "i" << '\n';
    //       Serial << "ODRV-3 INFO:" << '\n';
    //       Serial << back_AB.readString() << '\n';
    //       Serial << back_AB.readString() << '\n';
    //       Serial << back_AB.readString() << '\n';
    //       state = 'z'; // do nothing
    //     }
    //     else if (serial_input == "i4")
    //     { // read back_AB's hw/sw version and serial num
    //       Serial4 << "i" << '\n';
    //       Serial << "ODRV-4 INFO:" << '\n';
    //       Serial << front_KNEE.readString() << '\n';
    //       Serial << front_KNEE.readString() << '\n';
    //       Serial << front_KNEE.readString() << '\n';
    //       state = 'z'; // do nothing
    //     }
    //     else if (serial_input == "i5")
    //     { // read back_AB's hw/sw version and serial num
    //       Serial5 << "i" << '\n';
    //       Serial << "ODRV-5 INFO:" << '\n';
    //       Serial << front_HIP.readString() << '\n';
    //       Serial << front_HIP.readString() << '\n';
    //       Serial << front_HIP.readString() << '\n';
    //       state = 'z'; // do nothing
    //     }
    //     else if (serial_input == "i6")
    //     { // read back_AB's hw/sw version and serial num
    //       Serial7 << "i" << '\n';
    //       Serial << "ODRV-6 INFO:" << '\n';
    //       Serial << front_AB.readString() << '\n';
    //       Serial << front_AB.readString() << '\n';
    //       Serial << front_AB.readString() << '\n';
    //       state = 'z'; // do nothing
    //     }
    //     else if (serial_input == "e1")
    //     {
    //       state = 'a'; // switch state to 'a' to start pulling back_KNEE's encoder pos and velo
    //     }
    //     else if (serial_input == "e2")
    //     {
    //       state = 'b'; // switch state to 'b' to start pulling back_HIP's encoder pos and velo
    //     }
    //     else if (serial_input == "e3")
    //     {
    //       state = 'c'; // switch state to 'c' to start pulling back_AB's encoder pos and velo
    //     }
    //     else if (serial_input == "e4")
    //     {
    //       state = 'd'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    //     }
    //     else if (serial_input == "e5")
    //     {
    //       state = 'e'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    //     }
    //     else if (serial_input == "e6")
    //     {
    //       state = 'f'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    //     }
    //     else if (serial_input == "e7")
    //     {
    //       state = 'g'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    //     }
    //     else
    //     {
    //       Serial << "Entered input: '" << serial_input << "'\n";
    //       Serial1.println(serial_input);
    //       Serial << back_KNEE.readString() << '\n';
    //     }
    //   }

    //   // loop operations (repeat with the base program loop based on states determined by user input)
    //   if (state == 'a')
    //   { // pull back_KNEE encoder pos and velo
    // #ifdef DEBUG_SERIAL
    //     Serial.println("state a");
    // #endif
    // #ifdef NORMAL_OPERATION
    //     back_KNEE.readEncoderData(2);
    //     Serial.print("back_KNEE - axis0: ");
    //     Serial.println(back_KNEE.encoder_readings.a0_pos_reading);
    //     Serial.print("back_KNEE - axis1: ");
    //     Serial.println(back_KNEE.encoder_readings.a1_pos_reading);
    // #endif
    //   }
    //   else if (state == 'b')
    //   { // pull back_KNEE encoder pos and velo
    // #ifdef DEBUG_SERIAL
    //     Serial.println("state b");
    // #endif
    // #ifdef NORMAL_OPERATION
    //     back_HIP.readEncoderData(2);
    //     Serial.print("back_HIP - axis0: ");
    //     Serial.println(back_HIP.encoder_readings.a0_pos_reading);
    //     Serial.print("back_HIP - axis1: ");
    //     Serial.println(back_HIP.encoder_readings.a1_pos_reading);
    // #endif
    //   }
    //   else if (state == 'c')
    //   { // pull back_KNEE encoder pos and velo
    // #ifdef DEBUG_SERIAL
    //     Serial.println("state c");
    // #endif
    // #ifdef NORMAL_OPERATION
    //     back_AB.readEncoderData(2);
    //     Serial.print("back_AB - axis0: ");
    //     Serial.println(back_AB.encoder_readings.a0_pos_reading);
    //     Serial.print("back_AB - axis1: ");
    //     Serial.println(back_AB.encoder_readings.a1_pos_reading);
    // #endif
    //   }
    //   else if (state == 'd')
    //   { // pull back_KNEE encoder pos and velo
    // #ifdef DEBUG_SERIAL
    //     Serial.println("state d");
    // #endif
    // #ifdef NORMAL_OPERATION
    //     front_KNEE.readEncoderData(2);
    //     Serial.print("front_KNEE - axis0: ");
    //     Serial.println(front_KNEE.encoder_readings.a0_pos_reading);
    //     Serial.print("front_KNEE - axis1: ");
    //     Serial.println(front_KNEE.encoder_readings.a1_pos_reading);
    // #endif
    //   }
    //   else if (state == 'e')
    //   { // pull back_KNEE encoder pos and velo
    // #ifdef DEBUG_SERIAL
    //     Serial.println("state e");
    // #endif
    // #ifdef NORMAL_OPERATION
    //     front_HIP.readEncoderData(2);
    //     Serial.print("front_HIP - axis0: ");
    //     Serial.println(front_HIP.encoder_readings.a0_pos_reading);
    //     Serial.print("front_HIP - axis1: ");
    //     Serial.println(front_HIP.encoder_readings.a1_pos_reading);
    // #endif
    //   }
    //   else if (state == 'f')
    //   { // pull back_KNEE encoder pos and velo
    // #ifdef DEBUG_SERIAL
    //     Serial.println("state f");
    // #endif
    // #ifdef NORMAL_OPERATION
    //     front_AB.readEncoderData(2);
    //     Serial.print("front_AB - axis0: ");
    //     Serial.println(front_AB.encoder_readings.a0_pos_reading);
    //     Serial.print("front_AB - axis1: ");
    //     Serial.println(front_AB.encoder_readings.a1_pos_reading);
    // #endif
    //   }
    //   else if (state == 'g')
    //   { // pull back_KNEE, back_HIP and back_AB's encoder pos and velo
    // #ifdef DEBUG_SERIAL
    //     Serial.println("state g");
    // #endif
    // #ifdef NORMAL_OPERATION
    //     Serial.println("-----------------------------------");
    //     back_KNEE.readEncoderData(2);
    //     Serial.print("back_KNEE - axis0: ");
    //     Serial.println(back_KNEE.encoder_readings.a0_pos_reading);
    //     Serial.print("back_KNEE - axis1: ");
    //     Serial.println(back_KNEE.encoder_readings.a1_pos_reading);
    //     back_HIP.readEncoderData(2);
    //     Serial.print("back_HIP - axis0: ");
    //     Serial.println(back_HIP.encoder_readings.a0_pos_reading);
    //     Serial.print("back_HIP - axis1: ");
    //     Serial.println(back_HIP.encoder_readings.a1_pos_reading);
    //     back_AB.readEncoderData(2);
    //     Serial.print("back_AB - axis0: ");
    //     Serial.println(back_AB.encoder_readings.a0_pos_reading);
    //     Serial.print("back_AB - axis1: ");
    //     Serial.println(back_AB.encoder_readings.a1_pos_reading);
    //     front_KNEE.readEncoderData(2);
    //     Serial.print("front_KNEE - axis0: ");
    //     Serial.println(front_KNEE.encoder_readings.a0_pos_reading);
    //     Serial.print("front_KNEE - axis1: ");
    //     Serial.println(front_KNEE.encoder_readings.a1_pos_reading);
    //     front_HIP.readEncoderData(2);
    //     Serial.print("front_HIP - axis0: ");
    //     Serial.println(front_HIP.encoder_readings.a0_pos_reading);
    //     Serial.print("front_HIP - axis1: ");
    //     Serial.println(front_HIP.encoder_readings.a1_pos_reading);
    //     front_AB.readEncoderData(2);
    //     Serial.print("front_AB - axis0: ");
    //     Serial.println(front_AB.encoder_readings.a0_pos_reading);
    //     Serial.print("front_AB - axis1: ");
    //     Serial.println(front_AB.encoder_readings.a1_pos_reading);
    // #endif
    //   }
    //   else if (state == 'z')
    //   {
    //     // do nothing
    //   }
  }
