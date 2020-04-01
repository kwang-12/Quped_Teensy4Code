
#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"

class ODriveArduino {
public:
    Stream& serial_;
    char axis0_tag_;
    char axis1_tag_;

    enum AxisState_t {
        AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
    };
    struct encoder_readings{
        long int a0_pos_reading = 0.0;
        long int a0_velo_reading = 0.0;
        long int a0_current_reading = 0.0;
        long int a1_pos_reading = 0.0;
        long int a1_velo_reading = 0.0;
        long int a1_current_reading = 0.0;
    } encoder_readings;

    // ODriveArduino(Stream& serial);
    ODriveArduino(Stream& serial, char axis0_tag, char axis1_tag);  //Constructor

    // Write commands
    void SetPosition(char axis_tag, float position);
    void SetPosition(char axis_tag, float position, float velocity_feedforward);
    void SetPosition(char axis_tag, float position, float velocity_feedforward, float current_feedforward);

    // Read commands
    void readEncoderData(char axis_tag);

    // General params
    String readString();
    float readFloat();
    int32_t readInt();


    // State helper
    bool run_state(int axis, int requested_state, bool wait);

};

#endif //ODriveArduino_h
