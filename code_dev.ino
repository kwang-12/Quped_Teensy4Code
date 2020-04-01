#include <ODriveArduino.h>
//#define DEBUG_SERIAL
#define NORMAL_OPERATION

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
ODriveArduino back_KNEE(Serial1);     //knee back, axis0-RB, axis1-LB
ODriveArduino back_HIP(Serial2);      //hip back, axis0-RB, axis1-LB
ODriveArduino back_AB(Serial3);       //ab back, axis0-LB, axis1-RB
ODriveArduino front_KNEE(Serial4);    //knee front, axis0-RB, axis1-LB
ODriveArduino front_HIP(Serial5);     //hip front, axis0-LB, axis1-RB
ODriveArduino front_AB(Serial7);      //ab front, axis0-LB, axis1-RB
// Constants

// Variables
String serial_input; // store the input from computer monitor port
char state = 'z';    // store the state of loop program. By default, 'z' means do nothing.

void setup()
{
  // Serial to ODrive @ 921600 baud rate
  Serial1.begin(921600);
  Serial2.begin(921600);
  Serial3.begin(921600);
  Serial4.begin(921600);
  Serial5.begin(921600);
  Serial7.begin(921600);

  // Serial to PC @ 921600 baud rate
  Serial.begin(1000000);
  while (!Serial)
    ; // wait for Arduino Serial Monitor to open
  Serial.println("Ready!");
}

void loop()
{
//  Serial.println(state);
  if (Serial.available() > 0)
  {
    serial_input = Serial.readString();
    // one time operations
    if (serial_input == "i1")
    { // read back_KNEE's hw/sw version and serial num
      Serial1 << "i" << '\n';
      Serial << "ODRV-1 INFO:" << '\n';
      Serial << back_KNEE.readString() << '\n';
      Serial << back_KNEE.readString() << '\n';
      Serial << back_KNEE.readString() << '\n';
      state = 'z'; // do nothing
    }
    else if (serial_input == "i2")
    { // read back_HIP's hw/sw version and serial num
      Serial2 << "i" << '\n';
      Serial << "ODRV-2 INFO:" << '\n';
      Serial << back_HIP.readString() << '\n';
      Serial << back_HIP.readString() << '\n';
      Serial << back_HIP.readString() << '\n';
      state = 'z'; // do nothing
    }
    else if (serial_input == "i3")
    { // read back_AB's hw/sw version and serial num
      Serial3 << "i" << '\n';
      Serial << "ODRV-3 INFO:" << '\n';
      Serial << back_AB.readString() << '\n';
      Serial << back_AB.readString() << '\n';
      Serial << back_AB.readString() << '\n';
      state = 'z'; // do nothing
    }
    else if (serial_input == "i4")
    { // read back_AB's hw/sw version and serial num
      Serial4 << "i" << '\n';
      Serial << "ODRV-4 INFO:" << '\n';
      Serial << front_KNEE.readString() << '\n';
      Serial << front_KNEE.readString() << '\n';
      Serial << front_KNEE.readString() << '\n';
      state = 'z'; // do nothing
    }
    else if (serial_input == "i5")
    { // read back_AB's hw/sw version and serial num
      Serial5 << "i" << '\n';
      Serial << "ODRV-5 INFO:" << '\n';
      Serial << front_HIP.readString() << '\n';
      Serial << front_HIP.readString() << '\n';
      Serial << front_HIP.readString() << '\n';
      state = 'z'; // do nothing
    }
    else if (serial_input == "i6")
    { // read back_AB's hw/sw version and serial num
      Serial7 << "i" << '\n';
      Serial << "ODRV-6 INFO:" << '\n';
      Serial << front_AB.readString() << '\n';
      Serial << front_AB.readString() << '\n';
      Serial << front_AB.readString() << '\n';
      state = 'z'; // do nothing
    }
    else if (serial_input == "e1")
    {
      state = 'a'; // switch state to 'a' to start pulling back_KNEE's encoder pos and velo
    }
    else if (serial_input == "e2")
    {
      state = 'b'; // switch state to 'b' to start pulling back_HIP's encoder pos and velo
    }
    else if (serial_input == "e3")
    {
      state = 'c'; // switch state to 'c' to start pulling back_AB's encoder pos and velo
    }
    else if (serial_input == "e4")
    {
      state = 'd'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    }
    else if (serial_input == "e5")
    {
      state = 'e'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    }
    else if (serial_input == "e6")
    {
      state = 'f'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    }
    else if (serial_input == "e7")
    {
      state = 'g'; // switch state to 'd' to start pulling back_KNEE, back_HIP and back_AB's encoder pos and velo
    }
    else
    {
      Serial << "Entered input: '" << serial_input << "'\n";
      Serial1.println(serial_input);
      Serial << back_KNEE.readString() << '\n';
    }
  }

  // loop operations (repeat with the base program loop based on states determined by user input)
  if (state == 'a')
  { // pull back_KNEE encoder pos and velo
#ifdef DEBUG_SERIAL
    Serial.println("state a");
#endif
#ifdef NORMAL_OPERATION
    back_KNEE.readEncoderData(2);
    Serial.print("back_KNEE - axis0: ");
    Serial.println(back_KNEE.encoder_readings.a0_pos_reading);
    Serial.print("back_KNEE - axis1: ");
    Serial.println(back_KNEE.encoder_readings.a1_pos_reading);
#endif
  }
  else if (state == 'b')
  { // pull back_KNEE encoder pos and velo
#ifdef DEBUG_SERIAL
    Serial.println("state b");
#endif
#ifdef NORMAL_OPERATION
    back_HIP.readEncoderData(2);
    Serial.print("back_HIP - axis0: ");
    Serial.println(back_HIP.encoder_readings.a0_pos_reading);
    Serial.print("back_HIP - axis1: ");
    Serial.println(back_HIP.encoder_readings.a1_pos_reading);
#endif
  }
  else if (state == 'c')
  { // pull back_KNEE encoder pos and velo
#ifdef DEBUG_SERIAL
    Serial.println("state c");
#endif
#ifdef NORMAL_OPERATION
    back_AB.readEncoderData(2);
    Serial.print("back_AB - axis0: ");
    Serial.println(back_AB.encoder_readings.a0_pos_reading);
    Serial.print("back_AB - axis1: ");
    Serial.println(back_AB.encoder_readings.a1_pos_reading);
#endif
  }
  else if (state == 'd')
  { // pull back_KNEE encoder pos and velo
#ifdef DEBUG_SERIAL
    Serial.println("state d");
#endif
#ifdef NORMAL_OPERATION
    front_KNEE.readEncoderData(2);
    Serial.print("front_KNEE - axis0: ");
    Serial.println(front_KNEE.encoder_readings.a0_pos_reading);
    Serial.print("front_KNEE - axis1: ");
    Serial.println(front_KNEE.encoder_readings.a1_pos_reading);
#endif
  }
  else if (state == 'e')
  { // pull back_KNEE encoder pos and velo
#ifdef DEBUG_SERIAL
    Serial.println("state e");
#endif
#ifdef NORMAL_OPERATION
    front_HIP.readEncoderData(2);
    Serial.print("front_HIP - axis0: ");
    Serial.println(front_HIP.encoder_readings.a0_pos_reading);
    Serial.print("front_HIP - axis1: ");
    Serial.println(front_HIP.encoder_readings.a1_pos_reading);
#endif
  }
  else if (state == 'f')
  { // pull back_KNEE encoder pos and velo
#ifdef DEBUG_SERIAL
    Serial.println("state f");
#endif
#ifdef NORMAL_OPERATION
    front_AB.readEncoderData(2);
    Serial.print("front_AB - axis0: ");
    Serial.println(front_AB.encoder_readings.a0_pos_reading);
    Serial.print("front_AB - axis1: ");
    Serial.println(front_AB.encoder_readings.a1_pos_reading);
#endif
  }
  else if (state == 'g')
  { // pull back_KNEE, back_HIP and back_AB's encoder pos and velo
#ifdef DEBUG_SERIAL
    Serial.println("state g");
#endif
#ifdef NORMAL_OPERATION
    Serial.println("-----------------------------------");
    back_KNEE.readEncoderData(2);
    Serial.print("back_KNEE - axis0: ");
    Serial.println(back_KNEE.encoder_readings.a0_pos_reading);
    Serial.print("back_KNEE - axis1: ");
    Serial.println(back_KNEE.encoder_readings.a1_pos_reading);
    back_HIP.readEncoderData(2);
    Serial.print("back_HIP - axis0: ");
    Serial.println(back_HIP.encoder_readings.a0_pos_reading);
    Serial.print("back_HIP - axis1: ");
    Serial.println(back_HIP.encoder_readings.a1_pos_reading);
    back_AB.readEncoderData(2);
    Serial.print("back_AB - axis0: ");
    Serial.println(back_AB.encoder_readings.a0_pos_reading);
    Serial.print("back_AB - axis1: ");
    Serial.println(back_AB.encoder_readings.a1_pos_reading);
    front_KNEE.readEncoderData(2);
    Serial.print("front_KNEE - axis0: ");
    Serial.println(front_KNEE.encoder_readings.a0_pos_reading);
    Serial.print("front_KNEE - axis1: ");
    Serial.println(front_KNEE.encoder_readings.a1_pos_reading);
    front_HIP.readEncoderData(2);
    Serial.print("front_HIP - axis0: ");
    Serial.println(front_HIP.encoder_readings.a0_pos_reading);
    Serial.print("front_HIP - axis1: ");
    Serial.println(front_HIP.encoder_readings.a1_pos_reading);
    front_AB.readEncoderData(2);
    Serial.print("front_AB - axis0: ");
    Serial.println(front_AB.encoder_readings.a0_pos_reading);
    Serial.print("front_AB - axis1: ");
    Serial.println(front_AB.encoder_readings.a1_pos_reading);
#endif
  }
  else if (state == 'z')
  {
    // do nothing
  }
}