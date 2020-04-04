#include "ODriveArduino.h"
#include <Metro.h>

// Constructor
ODriveArduino::ODriveArduino(Stream &serial, String odrv_name, int serial_num, char axis0_tag, char axis1_tag, int serial_baud_rate) :serial_(serial)
{
    serial_num_ = serial_num;
    odrv_name_ = odrv_name;
    axis0_tag_ = axis0_tag;
    axis1_tag_ = axis1_tag;
    serial_baud_rate_ = serial_baud_rate;
}

bool ODriveArduino::ini()
{
    begin();
    readAxisError(axis0_tag_);
    readAxisError(axis1_tag_);
    if (axis0_error_ == 0 && axis1_error_ == 0)
    {
        return true;    //ready to go
    }
    else 
    {
        #ifdef DEBUG_SERIAL
        if (axis0_error_ > 0 && axis1_error_ > 0)
        {
            Serial.print(odrv_name_);
            Serial.print(": Axis-0 motor error code: ");
            Serial.println(axis0_error_);
            Serial.print(odrv_name_);
            Serial.print(": Axis-1 motor error code: ");
            Serial.println(axis1_error_);
        }
        else if (axis0_error_ > 0)
        {
            Serial.print(odrv_name_);
            Serial.print(": Axis-0 motor error code: ");
            Serial.println(axis0_error_);
        }
        else if (axis1_error_ > 0)
        {
            Serial.print(odrv_name_);
            Serial.print(": Axis-1 motor error code: ");
            Serial.println(axis1_error_);
        }
        #endif
        return false;
    }
}

void ODriveArduino::begin()
{
    switch(serial_num_)
    {
        case 1:
        Serial1.begin(serial_baud_rate_);
            break;
        case 2:
        Serial2.begin(serial_baud_rate_);
            break;
        case 3:
        Serial3.begin(serial_baud_rate_);
            break;
        case 4:
        Serial4.begin(serial_baud_rate_);
            break;
        case 5:
        Serial5.begin(serial_baud_rate_);
            break;
        case 6:
        Serial6.begin(serial_baud_rate_);
            break;
        case 7:
        Serial7.begin(serial_baud_rate_);
            break;
    }
}

void ODriveArduino::EnterCommand(String command)
{
    serial_ << command << '\n';
}

void ODriveArduino::iniTimer(char axis_tag, unsigned long time_interval)
{
    if (axis_tag == axis0_tag_)
    {
        axis0_timer_ = Metro(time_interval);
        axis0_timer_.reset();
    }
    else if (axis_tag == axis1_tag_)
    {
        axis1_timer_ = Metro(time_interval);
        axis1_timer_.reset();
    }
}

void ODriveArduino::modifyTimer(char axis_tag, unsigned long time_interval)
{
    if (axis_tag == axis0_tag_)
    {
        axis0_timer_.interval(time_interval);
        axis0_timer_.reset();
    }
    else if (axis_tag == axis1_tag_)
    {
        axis1_timer_.interval(time_interval);
        axis1_timer_.reset();
    }
}

void ODriveArduino::SetPosition(char axis_tag, float position)
{
    SetPosition(axis_tag, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(char axis_tag, float position, float velocity_feedforward)
{
    SetPosition(axis_tag, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(char axis_tag, float position, float velocity_feedforward, float current_feedforward)
{
    if (axis_tag == axis0_tag_)
    {
        serial_ << "p " << 0 << ' ' << position << ' ' << velocity_feedforward << ' ' << current_feedforward << '\n';
    }
    else if (axis_tag == axis1_tag_)
    {
        serial_ << "p " << 1 << ' ' << position << ' ' << velocity_feedforward << ' ' << current_feedforward << '\n';
    }
}

void ODriveArduino::readEncoderData(char axis_tag='a')
{
    if (axis_tag == axis0_tag_)
    {       //read encoder position of motor on axis 0
        serial_ << "e " << 0 << '\n'; //request pos&velo from the specified axis
        String str = readString();
        int delim_pos = str.indexOf(' ');
        encoder_readings.a0_pos_reading = str.substring(0, delim_pos).toInt();
        encoder_readings.a0_velo_reading = str.substring(delim_pos + 1).toInt();
    }
    else if (axis_tag == axis1_tag_)
    {       //read encoder position of motor on axis 1
        serial_ << "e " << 1 << '\n'; //request pos&velo from the specified axis
        String str = readString();
        int delim_pos = str.indexOf(' ');
        encoder_readings.a1_pos_reading = str.substring(0, delim_pos).toInt();
        encoder_readings.a1_velo_reading = str.substring(delim_pos + 1).toInt();
    }
    else    //read encoder position of both motor
    {
        serial_ << "e" << '\n';
        String str = readString();
        int delim_pos_a = str.indexOf(' ');
        int delim_pos_b = str.indexOf(' ', delim_pos_a + 1);
        encoder_readings.a0_pos_reading = str.substring(0, delim_pos_a).toInt();
        encoder_readings.a0_velo_reading = str.substring(delim_pos_a + 1, delim_pos_b).toInt();
        delim_pos_a = str.indexOf(' ', delim_pos_b + 1);
        encoder_readings.a1_pos_reading = str.substring(delim_pos_b + 1, delim_pos_a).toInt();
        encoder_readings.a1_velo_reading = str.substring(delim_pos_a + 1).toInt();
    }
}

void ODriveArduino::readAxisError(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        serial_ << "axis0.error" << '\n';
        axis0_error_ = readInt();
    }
    else if (axis_tag == axis1_tag_)
    {
        serial_ << "axis1.error" << '\n';
        axis1_error_ = readInt();
    }
}

String ODriveArduino::readString()
{
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;)
    {
        while (!serial_.available())
        {
            if (millis() - timeout_start >= timeout)
            {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}

float ODriveArduino::readFloat()
{
    return readString().toFloat();
}

int32_t ODriveArduino::readInt()
{
    return readString().toInt();
}

void ODriveArduino::find_joint_neutral_position(char mode, char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        joint_numbers.a0_zero_pos = calibrate_joint(mode, axis_tag);
    }
    else if (axis_tag == axis1_tag_)
    {
        joint_numbers.a1_zero_pos = calibrate_joint(mode, axis_tag);
    }
}

// find the neutral position of the joint (two modes)
long int ODriveArduino::calibrate_joint(char mode, char axis_tag)
{
  if (mode == 'r')        //find the neutral positoin by finding the max and min
  {
    long int pos_1;
    long int pos_2;
    long int pos_neutral;
    bool pos_confirm = false;
    bool pos_1_confirm = false;
    bool pos_2_confirm = false;
    while (pos_confirm == false)
    {
      while (pos_1_confirm == false)
      {
        Serial.println("Manually move to position 1...");
        Serial.println("Enter y when boundary position 1 is reached.");
        if (pos_1_confirm == false)
        {
          while(Serial.available()==0);//do nothing. wait for input
          String input_level_1 = Serial.readString();
          if (input_level_1 == 'y')
          {
            Serial.print("Current motor position is: ");
            if (axis_tag == axis0_tag_)
            {
                readEncoderData(axis0_tag_);
                pos_1 = encoder_readings.a0_pos_reading;
                Serial.println(pos_1);
                Serial.println("Is this the boundary position 1?");
                Serial.println("Enter y to confirm");
                Serial.println("Enter anything else to read the position again");
                bool pos_input_confirm = false;
                while (pos_input_confirm == false)
                {
                    while(Serial.available()==0);   //do nothing. wait for input
                    String input_level_2 = Serial.readString();
                    if (input_level_2 == 'y')
                    {
                        pos_input_confirm = true;
                    }
                    else
                    {
                        Serial.println("Current motor position is: ");
                        readEncoderData(axis0_tag_);
                        pos_1 = encoder_readings.a0_pos_reading;
                        Serial.println(pos_1);
                        Serial.println("Is this the boundary position 1?");
                        Serial.println("Enter y to confirm");
                        Serial.println("Enter anything else to read the position again");
                    }
                }
                pos_1_confirm = true;
            }
            else if (axis_tag == axis1_tag_)
            {
                readEncoderData(axis1_tag_);
                pos_1 = encoder_readings.a1_pos_reading;
                Serial.println(pos_1);
                Serial.println("Is this the boundary position 1?");
                Serial.println("Enter y to confirm");
                Serial.println("Enter anything else to read the position again");
                bool pos_input_confirm = false;
                while (pos_input_confirm == false)
                {
                    while(Serial.available()==0);   //do nothing. wait for input
                    String input_level_2 = Serial.readString();
                    if (input_level_2 == 'y')
                    {
                        pos_input_confirm = true;
                    }
                    else
                    {
                        Serial.println("Current motor position is: ");
                        readEncoderData(axis1_tag_);
                        pos_1 = encoder_readings.a1_pos_reading;
                        Serial.println(pos_1);
                        Serial.println("Is this the boundary position 1?");
                        Serial.println("Enter y to confirm");
                        Serial.println("Enter anything else to read the position again");
                    }
                }
                pos_1_confirm = true;
            }
          }
        }
      }
      while (pos_2_confirm == false)
      {
        Serial.println("Manually move to position 2...");
        Serial.println("Enter y when boundary position 2 is reached.");
        if (pos_2_confirm == false)
        {
          while (Serial.available()==0);//do nothing. wait for input
          String input_level_1 = Serial.readString();
          if (input_level_1 == 'y')
          {
            Serial.print("Current motor position is: ");
            if (axis_tag == axis0_tag_)
            {
                readEncoderData(axis0_tag_);
                pos_2 = encoder_readings.a0_pos_reading;
                Serial.println(pos_2);
                Serial.println("Is this the boundary position 2?");
                Serial.println("Enter y to confirm");
                Serial.println("Enter anything else to read the position again");
                bool pos_input_confirm = false;
                while (pos_input_confirm == false)
                {
                    while(Serial.available()==0);   //do nothing. wait for input
                    String input_level_2 = Serial.readString();
                    if (input_level_2 == 'y')
                    {
                        pos_input_confirm = true;
                    }
                    else
                    {
                        Serial.println("Current motor position is: ");
                        readEncoderData(axis0_tag_);
                        pos_2 = encoder_readings.a0_pos_reading;
                        Serial.println(pos_2);
                        Serial.println("Is this the boundary position 2?");
                        Serial.println("Enter y to confirm");
                        Serial.println("Enter anything else to read the position again");
                    }
                }
                pos_2_confirm = true;
            }
            else if (axis_tag == axis1_tag_)
            {
                readEncoderData(axis1_tag_);
                pos_2 = encoder_readings.a1_pos_reading;
                Serial.println(pos_2);
                Serial.println("Is this the boundary position 2?");
                Serial.println("Enter y to confirm");
                Serial.println("Enter anything else to read the position again");
                bool pos_input_confirm = false;
                while (pos_input_confirm == false)
                {
                    while(Serial.available()==0);   //do nothing. wait for input
                    String input_level_2 = Serial.readString();
                    if (input_level_2 == 'y')
                    {
                        pos_input_confirm = true;
                    }
                    else
                    {
                        Serial.println("Current motor position is: ");
                        readEncoderData(axis1_tag_);
                        pos_2 = encoder_readings.a1_pos_reading;
                        Serial.println(pos_2);
                        Serial.println("Is this the boundary position 2?");
                        Serial.println("Enter y to confirm");
                        Serial.println("Enter anything else to read the position again");
                    }
                }
                pos_2_confirm = true;
            }
          }
        }
      }
      if (pos_1_confirm == true && pos_2_confirm == true)
      {
        Serial.println("Both positions confirmed!");
        Serial.println("Neutral position(zero deg) will be the average of both values");
        Serial.print("Boundary position 1: ");
        Serial.println(pos_1);
        Serial.print("Boundary position 2: ");
        Serial.println(pos_2);
        Serial.println("Enter y to confirm both positions");
        Serial.println("Enter 1 or 2 to modify position 1 or position 2");
        Serial.println("Enter n to redo the process");
        while (Serial.available()==0);//do nothing. wait for input
        String input = Serial.readString();
        while (input != 'y' && input != '1' && input != '2' && input != 'n')
        {
            Serial.println("Both positions confirmed!");
            Serial.println("Neutral position(zero deg) will be the average of both values");
            Serial.print("Boundary position 1: ");
            Serial.println(pos_1);
            Serial.print("Boundary position 2: ");
            Serial.println(pos_2);
            Serial.println("Enter y to confirm both positions");
            Serial.println("Enter 1 or 2 to modify position 1 or position 2");
            Serial.println("Enter n to redo the process");
            while (Serial.available()==0);//do nothing. wait for input
            input = Serial.readString();
        }
        if (input == 'y')
        {
            pos_neutral = (pos_1+pos_2)/2;
            Serial.println(odrv_name_);
            Serial.println("pos-1    pos-neutral    pos-2");
            Serial.print(pos_1);
            Serial.print("\t\t");
            Serial.print(pos_neutral);
            Serial.print("\t\t");
            Serial.println(pos_2);
            pos_confirm = true;
        }
        else if (input == '1')
        {
            Serial.println("Modifying boundary position 1...");
            pos_1_confirm = false;
        }
        else if (input == '2')
        {
            Serial.println("Modifying boundary positoin 2...");
            pos_2_confirm = false;
        }
        else if (input == 'n')
        {
            Serial.println("Redo the process...");
            pos_1_confirm = false;
            pos_2_confirm = false;
        }
      }
    }
    return pos_neutral;
  }
  else if (mode == 'c')    //find the neutral position by direct confirmation
  {
      long int a = 1;
      return a;
  }
  else if (mode == 'u')    //find the neutral position by user input
  {
      long int a = 1;
      return a;
  }
  return 1;
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait)
{
    int timeout_ctr = 100;
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait)
    {
        do
        {
            delay(100);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}
