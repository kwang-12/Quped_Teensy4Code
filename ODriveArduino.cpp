
#include "Arduino.h"
#include "ODriveArduino.h"

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

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
ODriveArduino::ODriveArduino(Stream &serial, char axis0_tag, char axis1_tag)
{
    serial_(serial);
    axis0_tag_ = axis0_tag;
    axis1_tag_ = axis1_tag;
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
        serial_ << "p " << 0 << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
    }
    else if (axis_tag == axis1_tag_)
    {
        serial_ << "p " << 1 << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
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