#pragma once

#include <Arduino.h>
#include "globals.h"
#include <Metro.h>

class ODriveArduino {
public:
    Stream& serial_;

    String odrv_name_;

    char odrv_prop_;
    char axis0_tag_;
    char axis1_tag_;
    int serial_num_;

    actuator_type type;
    axis_tag axis0;
    axis_tag axis1;
    serial_port port_num;
    
    int serial_baud_rate_;
    int axis0_error_ = 0;
    int axis1_error_ = 0;


    Metro axis0_timer_ = Metro(SERIAL_MSG_TIMER);
    Metro axis1_timer_ = Metro(SERIAL_MSG_TIMER);

    struct encoder_readings{
        long int a0_pos_reading = 0;
        long int a0_velo_reading = 0;
        long int a0_current_reading = 0;
        long int a1_pos_reading = 0;
        long int a1_velo_reading = 0;
        long int a1_current_reading = 0;
    }encoder_readings;
    struct current_readings{
        float a0_Iq_board_setpoint = 0.0;
        float a0_Iq_measured = 0.0;
        float a1_Iq_board_setpoint = 0.0;
        float a1_Iq_measured = 0.0;
    }current_readings;
    struct joint_pos{
        long int a0_zero_pos = 0;
        long int a0_pos_1 = 0;
        long int a0_pos_2 = 0;
        long int a0_range = 0;
        long int a1_zero_pos = 0;
        long int a1_pos_1 = 0;
        long int a1_pos_2 = 0;
        long int a1_range = 0;
    }joint_pos;
    struct joint_target{
        float a0_start_deg = 0.0;
        float a0_target_deg = 0.0;
        float a0_velo_deg = 0.0;
        unsigned long a0_travel_start_time = 0;
        unsigned long a0_travelTime = 0;
        bool a0_calc_required_flag = false;

        float a1_start_deg = 0.0;
        float a1_target_deg = 0.0;
        float a1_velo_deg = 0.0;
        unsigned long a1_travel_start_time = 0;
        unsigned long a1_travelTime = 0;
        bool a1_calc_required_flag = false;

        float a0_travelTime_total = 0.0;
        float a0_traveltime_start = 0.0;
        // float a0_travelTime_current = 0.0;
        bool a0_arrival = false;
        bool a0_movementActivation = false;

        float a1_travelTime_total = 0.0;
        float a1_traveltime_start = 0.0;
        // float a1_traveltime_current = 0.0;
        bool a1_arrival = false;
        bool a1_movementActivation = false;
    }joint_target;

    // ODriveArduino(Stream& serial);
    ODriveArduino(Stream& serial, String odrv_name, char odrv_prop, int serial_num, char axis0_tag, char axis1_tag, int serial_baud_rate);  //Constructor

    /**
     * Initiate odrive communication
     * 
     * @param None
     * @return whether the initiation is successful
    */
    bool ini();

    /**
     * Initiate serial communication
    */
    void begin();
    void EnterCommand(String command);
    /**
     *  Transfrom user degree input to odrive numbered input
    */
    long int transPosition_deg2num(char axis_tag, float deg);
    long int transVelocity_deg2num(char axis_tag, float degPerSec);
    /**
     * Transform odrive output numbered readings to degree readings
    */ 
    float transPosition_num2deg(char axis_tag, long int num);
    float transVelocity_num2deg(char axis_tag, long int numPerSec);

    /**
     * Exclusively for shifting posture 
     * @param axis_tag specify which axis to move
     * @param target_deg target position in degree
     * @param inSec time to travel to the position
     * @param group_complete checks if other related motions are complete
     * @return whether the input time to travel has elapsed
     * 
    */
    bool moveTo_constVelo(char axis_tag, float target_deg, float inSec);

    /**
     * Update joint target position
     */
    void update_target(char axis_tag, bool calc_required, unsigned long inMicroSec, float target_deg, float target_velo=0.0);

    /**
     * 
     */
    void update(char axis_tag);

    // Timer
    void iniTimer(char axis_tag, unsigned long time_interval);
    void modifyTimer(char axis_tag, unsigned long time_interval);
    bool checkTimer(char axis_tag);
    
    // axis states
    bool armAxis(char axis_tag='a');
    bool disarmAxis(char axis_tag='a');

    // Write commands
    void SetPosition(char axis_tag, float deg);
    void SetPosition(char axis_tag, float deg, float degPerSec);
    void SetPosition(char axis_tag, float deg, float degPerSec, float current_feedforward);

    // Read commands
    void readEncoderData(char axis_tag);
    void readAxisError(char axis_tag);
    void readAxisCurrent(char axis_tag);

    // getters
    long int getAxisNeutralPos(char axis_tag);
    long int getAxisPos(char axis_tag, bool refresh_flag);
    long int getAxisVelo(char axis_tag, bool refresh_flag);
    float getAxisIqBoardSetpoint(char axis_tag);
    float getAxisIqMeasured(char axis_tag);

    int getAxisError(char axis_tag, bool refresh_flag);

    // General params
    String readString();
    float readFloat();
    int32_t readInt();

    // Calibratation
    void find_joint_neutral_position(char mode, char axis_tag);
    void calibrate_joint(char mode, char axis_tag);

    // State helper
    bool run_state(int axis, int requested_state, bool wait);

};