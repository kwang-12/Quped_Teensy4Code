
#include "Arduino.h"
#include "ODriveArduino.h"

// Print with stream operator
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

// Constructor
ODriveArduino::ODriveArduino(Stream &serial, int serial_num, char axis0_tag, char axis1_tag, int serial_baud_rate) :serial_(serial)
{
    serial_num_ = serial_num;
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
    else return false;
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
