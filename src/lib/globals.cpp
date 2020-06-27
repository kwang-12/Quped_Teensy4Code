#include "globals.h"

String rdStr()
{
  String str = "";
  static const unsigned long timeout = 1000;
  unsigned long timeout_start = millis();
  for (;;)
  {
    while (!Serial.available())
    {
      if (millis() - timeout_start >= timeout)
      {
        return str;
      }
    }
    char c = Serial.read();
    if (c == '\n')
      break;
    str += c;
  }
  return str;
}

long manual_input_num()
{
    while(true)
    {
        Serial.println("Enter number manually...");
        while (Serial.available()==0);
        String serial_input = rdStr();
        Serial.print("Entered:");
        long num = serial_input.toInt();
        Serial.println(num);
        Serial.println("Enter yes to confirm, anything else to enter again.");
        while (Serial.available()==0);
        serial_input = rdStr();
        if (serial_input == "yes")
        {
            return num;
        }
    }
}