#include "ODriveArduino.h"
#include <Metro.h>

// Constructor
ODriveArduino::ODriveArduino(Stream &serial, String odrv_name, char odrv_prop, int serial_num, char axis0_tag, char axis1_tag, int serial_baud_rate) : serial_(serial)
{
    serial_num_ = serial_num;
    odrv_name_ = odrv_name;
    odrv_prop_ = odrv_prop;
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
        return true; //ready to go
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
    switch (serial_num_)
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
    serial_.println(command);
    // serial_ << command << '\n';
}

long int ODriveArduino::transPosition_deg2num(char axis_tag, float deg)
{
    if (axis_tag == axis0_tag_)
    {
        if (odrv_prop_ == KNEE)
        {
            return (long int)(KNEE_GearRatio * deg / 360 * 180224 + joint_pos.a0_zero_pos);
        }
        else
        {
            return (long int)(deg / 360 * 180224 + joint_pos.a0_zero_pos);
        }
    }
    else
    {
        if (odrv_prop_ == KNEE)
        {
            return (long int)(KNEE_GearRatio * deg / 360 * 180224 + joint_pos.a1_zero_pos);
        }
        else
        {
            return (long int)(deg / 360 * 180224 + joint_pos.a1_zero_pos);
        }
    }
}

long int ODriveArduino::transVelocity_deg2num(char axis_tag, float degPerSec)
{
    if (odrv_prop_ == KNEE)
    {
        return (long int)(degPerSec / 360 * 180224 * KNEE_GearRatio);
    }
    else
    {
        return (long int)(degPerSec / 360 * 180224);
    }
}

float ODriveArduino::transPosition_num2deg(char axis_tag, long int num)
{
    if (axis_tag == axis0_tag_)
    {
        if (odrv_prop_ == KNEE)
        {
            return ((float)(num - joint_pos.a0_zero_pos)) * 360 / 180224 / KNEE_GearRatio;
        }
        else
        {
            return ((float)(num - joint_pos.a0_zero_pos)) * 360 / 180224;
        }
    }
    else
    {
        if (odrv_prop_ == KNEE)
        {
            return ((float)(num - joint_pos.a1_zero_pos)) * 360 / 180224 / KNEE_GearRatio;
        }
        else
        {
            return ((float)(num - joint_pos.a1_zero_pos)) * 360 / 180224;
        }
    }
}

float ODriveArduino::transVelocity_num2deg(char axis_tag, long int numPerSec)
{
    if (odrv_prop_ == KNEE)
    {
        return ((float)numPerSec) * 360 / 180224 / KNEE_GearRatio;
    }
    else
    {
        return ((float)numPerSec) * 360 / 180224;
    }
}

bool ODriveArduino::moveTo_constVelo(char axis_tag, float target_deg, float inSec)
{
    if (axis_tag == axis0_tag_)
    {
        if (joint_target.a0_movementActivation == false) //initalize movement
        {
            joint_target.a0_start_deg = transPosition_num2deg(axis_tag, getAxisPos(axis_tag, true));
            joint_target.a0_target_deg = target_deg;
            float deg_delta = joint_target.a0_start_deg - joint_target.a0_target_deg;
            //position difference is too small to make meaningful input
            if (deg_delta < 0.1 && deg_delta > -0.1)
            {
                return true;
            }
            joint_target.a0_velo_deg = (target_deg - joint_target.a0_start_deg) / inSec;
            joint_target.a0_travelTime_total = inSec;
            joint_target.a0_traveltime_start = static_cast<float>(millis()) / 1000;
            axis0_timer_.reset();
            joint_target.a0_movementActivation = true;
        }
        float time_elapsed_sec = static_cast<float>(millis()) / 1000 - joint_target.a0_traveltime_start;
        // check if time has elapsed
        if (time_elapsed_sec >= joint_target.a0_travelTime_total)
        //&&transPosition_num2deg(axis_tag, getAxisPos(axis_tag, true)) >= joint_target.a0_target_deg)
        {
            joint_target.a0_movementActivation = false;
            return true;
        }
        else //not enogh time has elapsed -> keep moving
        {
            if (axis0_timer_.check() == true)
            {
                SetPosition(axis_tag,
                            time_elapsed_sec * joint_target.a0_velo_deg + joint_target.a0_start_deg,
                            joint_target.a0_velo_deg);
                Serial.print(odrv_name_);
                Serial.print("--- ");
                Serial.print(axis0_tag_);
                Serial.print(" target pos: ");
                Serial.println(time_elapsed_sec * joint_target.a0_velo_deg + joint_target.a0_start_deg);
                // Serial.print("velocity: ");
                // Serial.println(joint_target.a0_velo_deg);
            }
            return false;
        }
    }
    else if (axis_tag == axis1_tag_)
    {
        if (joint_target.a1_movementActivation == false) //initalize movement
        {
            joint_target.a1_start_deg = transPosition_num2deg(axis_tag, getAxisPos(axis_tag, true));
            joint_target.a1_target_deg = target_deg;
            float deg_delta = joint_target.a1_start_deg - joint_target.a1_target_deg;
            //position difference is too small to make meaningful input
            if (deg_delta < 0.1 && deg_delta > -0.1)
            {
                return true;
            }
            joint_target.a1_velo_deg = (target_deg - joint_target.a1_start_deg) / inSec;
            joint_target.a1_travelTime_total = inSec;
            joint_target.a1_traveltime_start = static_cast<float>(millis()) / 1000;
            axis1_timer_.reset();
            joint_target.a1_movementActivation = true;
        }
        float time_elapsed_sec = static_cast<float>(millis()) / 1000 - joint_target.a1_traveltime_start;
        // check if time has elapsed
        if (time_elapsed_sec >= joint_target.a1_travelTime_total)
        //&&transPosition_num2deg(axis_tag, getAxisPos(axis_tag, true)) >= joint_target.a0_target_deg)
        {
            joint_target.a1_movementActivation = false;
            return true;
        }
        else //not enogh time has elapsed -> keep moving
        {
            if (axis1_timer_.check() == true)
            {
                SetPosition(axis_tag,
                            time_elapsed_sec * joint_target.a1_velo_deg + joint_target.a1_start_deg,
                            joint_target.a1_velo_deg);
                Serial.print(odrv_name_);
                Serial.print("--- ");
                Serial.print(axis1_tag_);
                Serial.print(" target pos: ");
                Serial.println(time_elapsed_sec * joint_target.a1_velo_deg + joint_target.a1_start_deg);
                // Serial.print("velocity: ");
                // Serial.println(joint_target.a1_velo_deg);
            }
            return false;
        }
    }
    return false;
}

void ODriveArduino::update_target(char axis_tag, bool calc_required, unsigned long inMicroSec, float target_deg, float target_velo)
{
    // target velocity is specified -> target degree must also be specified
    // joint targets are directly updated without any calculations
    if (calc_required != true)
    {
        if (axis_tag == axis0_tag_)
        {
            joint_target.a0_target_deg = target_deg;
            joint_target.a0_velo_deg = target_velo;
            // joint_target.a0_travelTime = inMicroSec;
            joint_target.a0_calc_required_flag = false;
        }
        else
        {
            joint_target.a1_target_deg = target_deg;
            joint_target.a1_velo_deg = target_velo;
            // joint_target.a1_travelTime = inMicroSec;
            joint_target.a1_calc_required_flag = false;
        }
    }
    // target velocity is not specified -> joint targets need t
    else
    {
        // get current joint position and velocities
        readEncoderData('a');
        if (axis_tag == axis0_tag_)
        {
            joint_target.a0_start_deg = transPosition_num2deg(axis_tag, encoder_readings.a0_pos_reading);
            joint_target.a0_target_deg = target_deg;
            joint_target.a0_travelTime = inMicroSec;
            joint_target.a0_calc_required_flag = true;
            joint_target.a0_travel_start_time = micros();
            float deg_delta = joint_target.a0_target_deg - joint_target.a0_start_deg;
            // difference between target and current position is bigger than 0.1 deg
            if (deg_delta > 0.1 || deg_delta < -0.1)
            {
                joint_target.a0_velo_deg = deg_delta / (joint_target.a0_travelTime / 1000000);
            }
            else
            {
                joint_target.a0_velo_deg = 0.0;
            }
            Serial.print(odrv_name_);
            Serial.print(' ');
            Serial.print(axis_tag);
            Serial.print(' ');
            Serial.print("start deg: ");
            Serial.print(joint_target.a0_start_deg);
            Serial.print(' ');
            Serial.print("target deg: ");
            Serial.print(joint_target.a0_target_deg);
            Serial.print(' ');
            Serial.print("velocity: ");
            Serial.println(joint_target.a0_velo_deg);
        }
        else
        {
            joint_target.a1_start_deg = transPosition_num2deg(axis_tag, encoder_readings.a1_pos_reading);
            joint_target.a1_target_deg = target_deg;
            joint_target.a1_travelTime = inMicroSec;
            joint_target.a1_calc_required_flag = true;
            joint_target.a1_travel_start_time = micros();
            float deg_delta = joint_target.a1_target_deg - joint_target.a1_start_deg;
            // difference between target and current position is bigger than 0.1 deg
            if (deg_delta > 0.1 || deg_delta < -0.1)
            {
                joint_target.a1_velo_deg = deg_delta / (joint_target.a1_travelTime / 1000000);
            }
            else
            {
                joint_target.a1_velo_deg = 0.0;
            }
            Serial.print(odrv_name_);
            Serial.print(' ');
            Serial.print(axis_tag);
            Serial.print(' ');
            Serial.print("start deg: ");
            Serial.print(joint_target.a1_start_deg);
            Serial.print(' ');
            Serial.print("target deg: ");
            Serial.print(joint_target.a1_target_deg);
            Serial.print(' ');
            Serial.print("velocity: ");
            Serial.println(joint_target.a1_velo_deg);
            // Serial.println(joint_target.a1_start_deg);
            // Serial.println(joint_target.a1_target_deg);
            // Serial.println(joint_target.a1_travelTime);
            // Serial.println(joint_target.a1_calc_required_flag);
            // Serial.println(joint_target.a1_travel_start_time);
            // Serial.println(deg_delta);
            // Serial.println(joint_target.a1_velo_deg);
            // Serial.println("=====");
        }
    }
}

void ODriveArduino::update(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        if (joint_target.a0_calc_required_flag == true)
        {
            // time has elapsed beyond the original set travel time duration
            // send the desired position directly without any calculation
            if (micros() >= joint_target.a0_travelTime + joint_target.a0_travel_start_time)
            {
                SetPosition(axis_tag, joint_target.a0_target_deg);
            }
            else
            {
                SetPosition(axis_tag, joint_target.a0_start_deg + joint_target.a0_velo_deg * static_cast<float>(micros() - joint_target.a0_travel_start_time) / 1000000, joint_target.a0_velo_deg);
            }
        }
        else
        {
            SetPosition(axis_tag, joint_target.a0_target_deg, joint_target.a0_velo_deg);
        }
    }
    else
    {
        if (joint_target.a1_calc_required_flag == true)
        {
            // time has elapsed beyond the original set travel time duration
            // send the desired position directly without any calculation
            if (micros() >= joint_target.a1_travelTime + joint_target.a1_travel_start_time)
            {
                SetPosition(axis_tag, joint_target.a1_target_deg);
            }
            else
            {
                SetPosition(axis_tag, joint_target.a1_start_deg + joint_target.a1_velo_deg * static_cast<float>(micros() - joint_target.a1_travel_start_time) / 1000000, joint_target.a1_velo_deg);
            }
        }
        else
        {
            SetPosition(axis_tag, joint_target.a1_target_deg, joint_target.a1_velo_deg);
        }
    }
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

bool ODriveArduino::checkTimer(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        return axis0_timer_.check();
    }
    else
    {
        return axis1_timer_.check();
    }
}

bool ODriveArduino::armAxis(char axis_tag)
{
    if (axis_tag == axis0_tag_) //check if already armed
    {
        serial_.println("r axis0.current_state");
        // serial_ << "r axis0.current_state" << '\n';
        if (readInt() == 8)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 0 arming success!");
            return true;
        }
        else
        {
            serial_.println("w axis0.requested_state 8");
            // serial_ << "w axis0.requested_state 8" << '\n';
            delay(100);
            serial_.println("r axis0.current_state");
            // serial_ << "r axis0.current_state" << '\n';
            if (readInt() != 8)
            {
                Serial.print(odrv_name_);
                Serial.print("---");
                Serial.println(axis_tag);
                Serial.println("axis 0 arming failed!");
                readAxisError(axis_tag);
                Serial.print("error code: ");
                Serial.println(axis0_error_);
                return false;
            }
            else
            {
                Serial.print(odrv_name_);
                Serial.print("---");
                Serial.println(axis_tag);
                Serial.println("axis 0 arming success!");
                return true;
            }
        }
    }
    else if (axis_tag == axis1_tag_)
    {
        serial_.println("r axis1.current_state");
        // serial_ << "r axis1.current_state" << '\n';
        if (readInt() == 8)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 1 arming success!");
            return true;
        }
        else
        {
            serial_.println("w axis1.requested_state 8");
            // serial_ << "w axis1.requested_state 8" << '\n';
            delay(100);
            serial_.println("r axis1.current_state");
            // serial_ << "r axis1.current_state" << '\n';
            if (readInt() != 8)
            {
                Serial.print(odrv_name_);
                Serial.print("---");
                Serial.println(axis_tag);
                Serial.println("axis 1 arming failed!");
                readAxisError(axis_tag);
                Serial.print("error code: ");
                Serial.println(axis1_error_);
                return false;
            }
            else
            {
                Serial.print(odrv_name_);
                Serial.print("---");
                Serial.println(axis_tag);
                Serial.println("axis 1 arming success!");
                return true;
            }
        }
    }
    serial_.println("r axis0.current_state");
    // serial_ << "r axis0.current_state" << '\n';
    int state_a0 = readInt();
    serial_.println("r axis1.current_state");
    // serial_ << "r axis1.current_state" << '\n';
    int state_a1 = readInt();
    if (state_a0 == 8 && state_a1 == 8)
    {
        Serial.print(odrv_name_);
        Serial.print("---");
        Serial.println(axis_tag);
        Serial.println("axis 0 arming success!");
        Serial.println("axis 1 arming success!");
        return true;
    }
    else
    {
        serial_.println("w axis0.requested_state 8");
        serial_.println("w axis1.requested_state 8");
        // serial_ << "w axis0.requested_state 8" << '\n';
        // serial_ << "w axis1.requested_state 8" << '\n';
        delay(100);
        serial_.println("r axis0.current_state");
        // serial_ << "r axis0.current_state" << '\n';
        state_a0 = readInt();
        if (state_a0 != 8)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 0 arming failed!");
            readAxisError(axis0_tag_);
            Serial.print("error code: ");
            Serial.println(axis0_error_);
        }
        else
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 0 arming success!");
        }
        serial_.println("r axis1.current_state");
        // serial_ << "r axis1.current_state" << '\n';
        state_a1 = readInt();
        if (state_a1 != 8)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 1 arming failed!");
            readAxisError(axis1_tag_);
            Serial.print("error code: ");
            Serial.println(axis1_error_);
        }
        else
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 1 arming success!");
        }
        if (state_a0 == 8 && state_a1 == 8)
        {
            return true;
        }
    }
    return false;
}

bool ODriveArduino::disarmAxis(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        serial_.println("w axis0.requested_state 1");
        // serial_ << "w axis0.requested_state 1" << '\n';
        delay(100);
        serial_.println("r axis0.current_state");
        // serial_ << "r axis0.current_state" << '\n';
        if (readInt() != 1)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 0 disarming failed");
            readAxisError(axis_tag);
            Serial.print("error code: ");
            Serial.println(axis0_error_);
            return false;
        }
        else
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 0 disarming success");
            return true;
        }
    }
    else if (axis_tag == axis1_tag_)
    {
        serial_.println("w axis1.requested_state 1");
        // serial_ << "w axis1.requested_state 1" << '\n';
        delay(100);
        serial_.println("r axis1.current_state");
        // serial_ << "r axis1.current_state" << '\n';
        if (readInt() != 1)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 1 disarming failed");
            readAxisError(axis_tag);
            Serial.print("error code: ");
            Serial.println(axis1_error_);
            return false;
        }
        else
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 1 disarming success");
            return true;
        }
    }
    serial_ << "r axis0.current_state" << '\n';
    int state_a0 = readInt();
    serial_ << "r axis1.current_state" << '\n';
    int state_a1 = readInt();
    if (state_a0 == 1 && state_a1 == 1)
    {
        Serial.print(odrv_name_);
        Serial.print("---");
        Serial.println(axis_tag);
        Serial.println("axis 0 disarming success!");
        Serial.println("axis 1 disarming success!");
        return true;
    }
    if (state_a0 != 1)
    {
        serial_.println("w axis0.requested_state 1");
        delay(100);
        serial_.println("r axis0.current_state");
        state_a0 = readInt();
        if (state_a0 != 1)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 0 disarming failed!");
            readAxisError(axis0_tag_);
            Serial.print("error code: ");
            Serial.println(axis0_error_);
        }
        else
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 0 disarming success!");
        }
    }
    if (state_a1 != 1)
    {
        serial_.println("w axis1.requested_state 1");
        delay(100);
        serial_.println("r axis1.current_state");
        state_a1 = readInt();
        if (state_a1 != 1)
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 1 disarming failed!");
            readAxisError(axis1_tag_);
            Serial.print("error code: ");
            Serial.println(axis1_error_);
        }
        else
        {
            Serial.print(odrv_name_);
            Serial.print("---");
            Serial.println(axis_tag);
            Serial.println("axis 1 disarming success!");
        }
    }
    return false;
}

void ODriveArduino::SetPosition(char axis_tag, float deg)
{
    SetPosition(axis_tag, deg, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(char axis_tag, float deg, float degPerSec)
{
    SetPosition(axis_tag, deg, degPerSec, 0.0f);
}

void ODriveArduino::SetPosition(char axis_tag, float deg, float degPerSec, float current_feedforward)
{
    if (axis_tag == axis0_tag_)
    {
        serial_ << "p " << 0 << ' ' << transPosition_deg2num(axis_tag, deg) << ' ' << transVelocity_deg2num(axis_tag, degPerSec) << ' ' << current_feedforward << '\n';
    // Serial.print(odrv_name_);
    // Serial.print(' ');
    // Serial.print(axis_tag);
    // Serial.print(' ');
    //     Serial.print(deg);
    //     Serial.print(' ');
    //     Serial.println(degPerSec);
    }
    else if (axis_tag == axis1_tag_)
    {
        serial_ << "p " << 1 << ' ' << transPosition_deg2num(axis_tag, deg) << ' ' << transVelocity_deg2num(axis_tag, degPerSec) << ' ' << current_feedforward << '\n';
    // Serial.print(odrv_name_);
    // Serial.print(' ');
    // Serial.print(axis_tag);
    // Serial.print(' ');
    //     Serial.print(deg);
    //     Serial.print(' ');
    //     Serial.println(degPerSec);
    }
}

/**
 *  TODO: The firmware is currently bugged. 
 * At line 108: "if(numscan < 2)"" should be changed to "if(numscan < 1)"
 * ATM, odrv returns all pos and velo at the same time instead of for the
 * specialized axis.
*/
void ODriveArduino::readEncoderData(char axis_tag = 'a')
{
    // if (axis_tag == axis0_tag_)
    // {       //read encoder position of motor on axis 0
    //     serial_ << "e " << 0 << '\n'; //request pos&velo from the specified axis
    //     String str = readString();
    //     Serial.println(" ");
    //     Serial.println(str);
    //     int delim_pos = str.indexOf(' ');
    //     encoder_readings.a0_pos_reading = str.substring(0, delim_pos).toInt();
    //     encoder_readings.a0_velo_reading = str.substring(delim_pos + 1).toInt();
    // }
    // else if (axis_tag == axis1_tag_)
    // {       //read encoder position of motor on axis 1
    //     serial_ << "e " << 1 << '\n'; //request pos&velo from the specified axis
    //     String str = readString();
    //     Serial.println(" ");
    //     Serial.println(str);
    //     int delim_pos = str.indexOf(' ');
    //     encoder_readings.a1_pos_reading = str.substring(0, delim_pos).toInt();
    //     encoder_readings.a1_velo_reading = str.substring(delim_pos + 1).toInt();
    // }
    // else    //read encoder position of both motor
    // {
    // serial_.println('e');
    serial_ << "e" << '\n';
    String str = readString();
    int delim_pos_a = str.indexOf(' ');
    int delim_pos_b = str.indexOf(' ', delim_pos_a + 1);
    encoder_readings.a0_pos_reading = str.substring(0, delim_pos_a).toInt();
    encoder_readings.a0_velo_reading = str.substring(delim_pos_a + 1, delim_pos_b).toInt();
    delim_pos_a = str.indexOf(' ', delim_pos_b + 1);
    encoder_readings.a1_pos_reading = str.substring(delim_pos_b + 1, delim_pos_a).toInt();
    encoder_readings.a1_velo_reading = str.substring(delim_pos_a + 1).toInt();
    // }
}

void ODriveArduino::readAxisError(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        // serial_.println("r axis0.error");
        serial_ << "r axis0.error" << '\n';
        String feedback = readString();
        if (feedback == "")
        {
            axis0_error_ = 404;
        }
        else
        {
            axis0_error_ = feedback.toInt();
        }
    }
    else if (axis_tag == axis1_tag_)
    {
        // serial_.println("r axis1.error");
        serial_ << "r axis1.error" << '\n';
        String feedback = readString();
        if (feedback == "")
        {
            axis1_error_ = 404;
        }
        else
        {
            axis1_error_ = feedback.toInt();
        }
    }
}

long int ODriveArduino::getAxisNeutralPos(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        return joint_pos.a0_zero_pos;
    }
    else
    {
        return joint_pos.a1_zero_pos;
    }
}

long int ODriveArduino::getAxisPos(char axis_tag, bool refresh_flag)
{
    if (axis_tag == axis0_tag_)
    {
        if (refresh_flag == true)
        {
            ODriveArduino::readEncoderData(axis_tag);
        }
        return encoder_readings.a0_pos_reading;
    }
    else
    {
        if (refresh_flag == true)
        {
            ODriveArduino::readEncoderData(axis_tag);
        }
        return encoder_readings.a1_pos_reading;
    }
}

long int ODriveArduino::getAxisVelo(char axis_tag, bool refresh_flag)
{
    if (axis_tag == axis0_tag_)
    {
        if (refresh_flag == true)
        {
            ODriveArduino::readEncoderData(axis_tag);
        }
        return encoder_readings.a0_velo_reading;
    }
    else
    {
        if (refresh_flag == true)
        {
            ODriveArduino::readEncoderData(axis_tag);
        }
        return encoder_readings.a1_velo_reading;
    }
}

void ODriveArduino::readAxisCurrent(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        serial_<< "r axis0.motor.current_control.Iq_setpoint" << '\n';
        current_readings.a0_Iq_board_setpoint = readString().toFloat();
        serial_<< "r axis0.motor.current_control.Iq_measured" << '\n';
        current_readings.a0_Iq_measured = readString().toFloat();
    }
    else if (axis_tag == axis1_tag_)
    {
        serial_<< "r axis1.motor.current_control.Iq_setpoint" << '\n';
        current_readings.a1_Iq_board_setpoint = readString().toFloat();
        serial_<< "r axis1.motor.current_control.Iq_measured" << '\n';
        current_readings.a1_Iq_measured = readString().toFloat();
    }
    else
    {
        serial_<< "r axis0.motor.current_control.Iq_setpoint" << '\n';
        current_readings.a0_Iq_board_setpoint = readString().toFloat();
        serial_<< "r axis0.motor.current_control.Iq_measured" << '\n';
        current_readings.a0_Iq_measured = readString().toFloat();
        serial_<< "r axis1.motor.current_control.Iq_setpoint" << '\n';
        current_readings.a1_Iq_board_setpoint = readString().toFloat();
        serial_<< "r axis1.motor.current_control.Iq_measured" << '\n';
        current_readings.a1_Iq_measured = readString().toFloat();
    }
}

float ODriveArduino::getAxisIqBoardSetpoint(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        return current_readings.a0_Iq_board_setpoint;
    }
    else
    {
        return current_readings.a1_Iq_board_setpoint;
    }
}

float ODriveArduino::getAxisIqMeasured(char axis_tag)
{
    if (axis_tag == axis0_tag_)
    {
        return current_readings.a0_Iq_measured;
    }
    else
    {
        return current_readings.a1_Iq_measured;
    }
}

int ODriveArduino::getAxisError(char axis_tag, bool refresh_flag)
{
    if (axis_tag == axis0_tag_)
    {
        if (refresh_flag == true)
        {
            readAxisError(axis_tag);
        }
        return axis0_error_;
    }
    else
    {
        if (refresh_flag == true)
        {
            readAxisError(axis_tag);
        }
        return axis1_error_;
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
        Serial.println("---------------------------------");
        Serial.print(odrv_name_);
        Serial.print("---");
        Serial.println(axis_tag);
        calibrate_joint(mode, axis_tag);
        // reorder pos1 and pos2 so that pos1 > pos2
        long int max_pos;
        long int min_pos;
        if (joint_pos.a0_pos_1 > joint_pos.a0_pos_2)
        {
            // ppos 1 is bigger than pos 2. do nothing
            joint_pos.a0_range = joint_pos.a0_pos_1 - joint_pos.a0_pos_2;
        }
        else if (joint_pos.a0_pos_1 < joint_pos.a0_pos_2)
        {
            max_pos = joint_pos.a0_pos_2;
            min_pos = joint_pos.a0_pos_1;
            joint_pos.a0_pos_1 = max_pos;
            joint_pos.a0_pos_2 = min_pos;
            joint_pos.a0_range = joint_pos.a0_pos_1 - joint_pos.a0_pos_2;
        }
        else
        {
            joint_pos.a0_range = 0;
        }
    }
    else if (axis_tag == axis1_tag_)
    {
        Serial.println("---------------------------------");
        Serial.print(odrv_name_);
        Serial.print("---");
        Serial.println(axis_tag);
        calibrate_joint(mode, axis_tag);
        // reorder pos1 and pos2 so that pos1 > pos2
        long int max_pos;
        long int min_pos;
        if (joint_pos.a1_pos_1 > joint_pos.a1_pos_2)
        {
            // ppos 1 is bigger than pos 2. do nothing
            joint_pos.a1_range = joint_pos.a1_pos_1 - joint_pos.a1_pos_2;
        }
        else if (joint_pos.a1_pos_1 < joint_pos.a1_pos_2)
        {
            max_pos = joint_pos.a1_pos_2;
            min_pos = joint_pos.a1_pos_1;
            joint_pos.a1_pos_1 = max_pos;
            joint_pos.a1_pos_2 = min_pos;
            joint_pos.a1_range = joint_pos.a1_pos_1 - joint_pos.a1_pos_2;
        }
        else
        {
            joint_pos.a1_range = 0;
        }
    }
}

// find the neutral position of the joint (two modes)
void ODriveArduino::calibrate_joint(char mode, char axis_tag)
{
    if (mode == 'r') //find the neutral positoin by finding the max and min
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
                Serial.println("---------------------------------");
                Serial.println("Manually move to position 1...");
                Serial.println("Enter y when boundary position 1 is reached.");
                if (pos_1_confirm == false)
                {
                    while (Serial.available() == 0)
                        ; //do nothing. wait for input
                    String input_level_1 = Serial.readString();
                    if (input_level_1 == "y")
                    {
                        Serial.println("---------------------------------");
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
                                while (Serial.available() == 0)
                                    ; //do nothing. wait for input
                                String input_level_2 = Serial.readString();
                                if (input_level_2 == "y")
                                {
                                    pos_input_confirm = true;
                                }
                                else
                                {
                                    Serial.println("---------------------------------");
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
                                while (Serial.available() == 0)
                                    ; //do nothing. wait for input
                                String input_level_2 = Serial.readString();
                                if (input_level_2 == "y")
                                {
                                    pos_input_confirm = true;
                                }
                                else
                                {
                                    Serial.println("---------------------------------");
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
                Serial.println("---------------------------------");
                Serial.println("Manually move to position 2...");
                Serial.println("Enter y when boundary position 2 is reached.");
                if (pos_2_confirm == false)
                {
                    while (Serial.available() == 0)
                        ; //do nothing. wait for input
                    String input_level_1 = Serial.readString();
                    if (input_level_1 == "y")
                    {
                        Serial.println("---------------------------------");
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
                                while (Serial.available() == 0)
                                    ; //do nothing. wait for input
                                String input_level_2 = Serial.readString();
                                if (input_level_2 == "y")
                                {
                                    pos_input_confirm = true;
                                }
                                else
                                {
                                    Serial.println("---------------------------------");
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
                                while (Serial.available() == 0)
                                    ; //do nothing. wait for input
                                String input_level_2 = Serial.readString();
                                if (input_level_2 == "y")
                                {
                                    pos_input_confirm = true;
                                }
                                else
                                {
                                    Serial.println("---------------------------------");
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
                Serial.println("---------------------------------");
                Serial.println("Both positions confirmed!");
                Serial.println("Neutral position(zero deg) will be the average of both values");
                Serial.print("Boundary position 1: ");
                Serial.println(pos_1);
                Serial.print("Boundary position 2: ");
                Serial.println(pos_2);
                Serial.println("Enter y to confirm both positions");
                Serial.println("Enter 1 or 2 to modify position 1 or position 2");
                Serial.println("Enter n to redo the process");
                while (Serial.available() == 0)
                    ; //do nothing. wait for input
                String input = Serial.readString();
                while (input != "y" && input != "1" && input != "2" && input != "n")
                {
                    Serial.println("---------------------------------");
                    Serial.println("Both positions confirmed!");
                    Serial.println("Neutral position(zero deg) will be the average of both values");
                    Serial.print("Boundary position 1: ");
                    Serial.println(pos_1);
                    Serial.print("Boundary position 2: ");
                    Serial.println(pos_2);
                    Serial.println("Enter y to confirm both positions");
                    Serial.println("Enter 1 or 2 to modify position 1 or position 2");
                    Serial.println("Enter n to redo the process");
                    while (Serial.available() == 0)
                        ; //do nothing. wait for input
                    input = Serial.readString();
                }
                if (input == "y")
                {
                    pos_neutral = (pos_1 + pos_2) / 2;
                    Serial.println(odrv_name_);
                    Serial.println("pos-1    pos-neutral    pos-2");
                    Serial.print(pos_1);
                    Serial.print("\t");
                    Serial.print(pos_neutral);
                    Serial.print("\t");
                    Serial.println(pos_2);
                    pos_confirm = true;
                    if (axis_tag == axis0_tag_)
                    {
                        joint_pos.a0_zero_pos = pos_neutral;
                        joint_pos.a0_pos_1 = pos_1;
                        joint_pos.a0_pos_2 = pos_2;
                    }
                    else if (axis_tag == axis1_tag_)
                    {
                        joint_pos.a1_zero_pos = pos_neutral;
                        joint_pos.a1_pos_1 = pos_1;
                        joint_pos.a1_pos_2 = pos_2;
                    }
                }
                else if (input == "1")
                {
                    Serial.println("Modifying boundary position 1...");
                    pos_1_confirm = false;
                }
                else if (input == "2")
                {
                    Serial.println("Modifying boundary positoin 2...");
                    pos_2_confirm = false;
                }
                else if (input == "n")
                {
                    Serial.println("Redo the process...");
                    pos_1_confirm = false;
                    pos_2_confirm = false;
                }
            }
        }
    }
    else if (mode == 'c') //find the neutral position by direct confirmation
    {
        long int pos_neutral;
        bool pos_confirm = false;
        while (pos_confirm == false)
        {
            Serial.println("---------------------------------");
            Serial.println("Manually move to neutral position");
            Serial.println("Enter y when neutral position is reached.");
            while (Serial.available() == 0)
                ;
            String input_level_1 = Serial.readString();
            if (input_level_1 == "y")
            {
                Serial.println("---------------------------------");
                Serial.print("Current motor position is: ");
                if (axis_tag == axis0_tag_)
                {
                    readEncoderData(axis0_tag_);
                    pos_neutral = encoder_readings.a0_pos_reading;
                    Serial.println(pos_neutral);
                    Serial.println("Is this the neutral position?");
                    Serial.println("Enter y to confirm");
                    Serial.println("Enter anything else to read the position again");
                    bool pos_input_confirm = false;
                    while (pos_input_confirm == false)
                    {
                        while (Serial.available() == 0)
                            ;
                        String input_level_2 = Serial.readString();
                        if (input_level_2 == "y")
                        {
                            joint_pos.a0_zero_pos = pos_neutral;
                            pos_input_confirm = true;
                        }
                        else
                        {
                            Serial.println("---------------------------------");
                            Serial.println("Current motor position is: ");
                            readEncoderData(axis0_tag_);
                            pos_neutral = encoder_readings.a0_pos_reading;
                            Serial.println(pos_neutral);
                            Serial.println("Is this the neutral position?");
                            Serial.println("Enter y to confirm");
                            Serial.println("Enter anything else to read the position again");
                        }
                    }
                    pos_confirm = true;
                }
                else if (axis_tag == axis1_tag_)
                {
                    readEncoderData(axis1_tag_);
                    pos_neutral = encoder_readings.a1_pos_reading;
                    Serial.println(pos_neutral);
                    Serial.println("Is this the neutral position?");
                    Serial.println("Enter y to confirm");
                    Serial.println("Enter anything else to read the position again");
                    bool pos_input_confirm = false;
                    while (pos_input_confirm == false)
                    {
                        while (Serial.available() == 0)
                            ;
                        String input_level_2 = Serial.readString();
                        if (input_level_2 == "y")
                        {
                            joint_pos.a1_zero_pos = pos_neutral;
                            pos_input_confirm = true;
                        }
                        else
                        {
                            Serial.println("---------------------------------");
                            Serial.println("Current motor position is: ");
                            readEncoderData(axis1_tag_);
                            pos_neutral = encoder_readings.a1_pos_reading;
                            Serial.println(pos_neutral);
                            Serial.println("Is this the neutral position?");
                            Serial.println("Enter y to confirm");
                            Serial.println("Enter anything else to read the position again");
                        }
                    }
                    pos_confirm = true;
                }
            }
        }
    }
    else if (mode == 'u') //find the neutral position by user input
    {
        if (axis_tag == axis0_tag_)
        {
            Serial.println("Enter a0");
            joint_pos.a0_zero_pos = manual_input_num();
        }
        else
        {
            Serial.println("Enter a1");
            joint_pos.a1_zero_pos = manual_input_num();
        }
    }
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait)
{
    int timeout_ctr = 100;
    serial_.print("w axis");
    serial_.print(axis);
    serial_.print(".requested_state ");
    serial_.println(requested_state);
    // serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait)
    {
        do
        {
            delay(100);
            serial_.print("r axis");
            serial_.print(axis);
            serial_.println(".current_state");
            // serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}
