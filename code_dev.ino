// Libraries:
#include "src/lib/globals.h"
#include "src/lib/ODriveArduino.h"
#include "src/lib/kinematics.h"
#include <Metro.h>

// knee gear ratio: 16:20
String odrv1_name = "back_KNEE";
String odrv2_name = "back_HIP";
String odrv3_name = "back_AB";
String odrv4_name = "front_KNEE";
String odrv5_name = "front_HIP";
String odrv6_name = "front_AB";

// ODrive object
// ODriveArduino back_KNEE(Serial1, odrv1_name, KNEE, 1, RIGHT, LEFT, SERIAL_BAUD_RATE);  //knee back, axis0-RB, axis1-LB
// ODriveArduino back_HIP(Serial2, odrv2_name, HIP, 2, RIGHT, LEFT, SERIAL_BAUD_RATE);    //hip back, axis0-RB, axis1-LB
// ODriveArduino back_AB(Serial3, odrv3_name, AB, 3, LEFT, RIGHT, SERIAL_BAUD_RATE);      //ab back, axis0-LB, axis1-RB
// ODriveArduino front_KNEE(Serial4, odrv4_name, KNEE, 4, LEFT, RIGHT, SERIAL_BAUD_RATE); //knee front, axis0-RB, axis1-LB
// ODriveArduino front_HIP(Serial5, odrv5_name, HIP, 5, LEFT, RIGHT, SERIAL_BAUD_RATE);   //hip front, axis0-LB, axis1-RB
// ODriveArduino front_AB(Serial7, odrv6_name, AB, 7, RIGHT, LEFT, SERIAL_BAUD_RATE);     //ab front, axis0-LB, axis1-RB
ODriveArduino back_KNEE(Serial1, odrv1_name, KNEE, 1, RIGHT, LEFT, SERIAL_BAUD_RATE);  //knee back, axis0-RB, axis1-LB
ODriveArduino back_HIP(Serial2, odrv2_name, HIP, 2, RIGHT, LEFT, SERIAL_BAUD_RATE);    //hip back, axis0-RB, axis1-LB
ODriveArduino back_AB(Serial3, odrv3_name, AB, 3, LEFT, RIGHT, SERIAL_BAUD_RATE);      //ab back, axis0-LB, axis1-RB
ODriveArduino front_KNEE(Serial4, odrv4_name, KNEE, 4, LEFT, RIGHT, SERIAL_BAUD_RATE); //knee front, axis0-RB, axis1-LB
ODriveArduino front_HIP(Serial5, odrv5_name, HIP, 5, LEFT, RIGHT, SERIAL_BAUD_RATE);   //hip front, axis0-LB, axis1-RB
ODriveArduino front_AB(Serial7, odrv6_name, AB, 7, RIGHT, LEFT, SERIAL_BAUD_RATE);     //ab front, axis0-LB, axis1-RB
ODriveArduino *back_KNEE_ptr = &back_KNEE;
ODriveArduino *back_HIP_ptr = &back_HIP;
ODriveArduino *back_AB_ptr = &back_AB;
ODriveArduino *front_KNEE_ptr = &front_KNEE;
ODriveArduino *front_HIP_ptr = &front_HIP;
ODriveArduino *front_AB_ptr = &front_AB;

// Initialize timer
Metro timer = Metro(msg_timer_interval);

float leg_pos_offset_forward = 0.03;
float leg_pos_offset_horizontal = 0.03;

radio radio_readings;
kinematics qPed(0.075, 3, 2, 0.35, leg_pos_offset_forward, leg_pos_offset_horizontal);
float kine_time = 0;
int test_timer = 0;
bool manual_stop = false;

loop_state state = STATE_IDLE;

unsigned long timerrr;

/**
 * Connect ODrive
 * @param odrv odrive object to be connected
 */
void odrv_connect(ODriveArduino odrv)
{
  if (odrv.ini())
  {
#ifdef DEBUG_SERIAL
    Serial.print(odrv.odrv_name_);
    Serial.println("COM activated");
#endif
  }
  else
  {
#ifdef DEBUG_SERIAL
    Serial.print(odrv.odrv_name_);
    Serial.println("COM failed");
#endif
  }
}

/**
 * Find the upper and lower limit of each joint
 * Serial communication with a PC is required
 */
void find_joint_neutral_position()
{
  String find_input = "yeah";
  while (find_input != "exit")
  {
    Serial.println("-----------------Find joint limits------------------");
#ifdef ENABLE_FRONT_LEFT
    Serial.println("Enter FL to find joint limits for front left leg");
#endif
#ifdef ENABLE_FRONT_RIGHT
    Serial.println("Enter FR to find joint limits for front right leg");
#endif
#ifdef ENABLE_BACK_LEFT
    Serial.println("Enter BL to find joint limits for back left leg");
#endif
#ifdef ENABLE_BACK_RIGHT
    Serial.println("Enter BR to find joint limits for back right leg");
#endif
    Serial.println("Enter exit to exit the find joint limits sequence");
    while (Serial.available() == 0)
      ;
    find_input = rdStr();
    if (find_input == "FL")
    {
#ifdef ENABLE_FRONT_LEFT
      front_AB.find_joint_neutral_position('r', LEFT);
      front_HIP.find_joint_neutral_position('c', LEFT);
      front_KNEE.find_joint_neutral_position('r', LEFT);
#endif
    }
    else if (find_input == "FR")
    {
#ifdef ENABLE_FRONT_RIGHT
      front_AB.find_joint_neutral_position('r', RIGHT);
      front_HIP.find_joint_neutral_position('c', RIGHT);
      front_KNEE.find_joint_neutral_position('r', RIGHT);
#endif
    }
    else if (find_input == "BL")
    {
#ifdef ENABLE_BACK_LEFT
      back_AB.find_joint_neutral_position('r', LEFT);
      back_HIP.find_joint_neutral_position('c', LEFT);
      back_KNEE.find_joint_neutral_position('r', LEFT);
#endif
    }
    else if (find_input == "BR")
    {
#ifdef ENABLE_BACK_RIGHT
      back_AB.find_joint_neutral_position('r', RIGHT);
      back_HIP.find_joint_neutral_position('c', RIGHT);
      back_KNEE.find_joint_neutral_position('r', RIGHT);
#endif
    }
  }
}

void manual_find()
{
  String find_input = "yeah";
  while (find_input != "exit")
  {
    Serial.println("--------------Manually find joint limits---------------");
#ifdef ENABLE_FRONT_LEFT
    Serial.println("Enter FL to find joint limits for front left leg");
#endif
#ifdef ENABLE_FRONT_RIGHT
    Serial.println("Enter FR to find joint limits for front right leg");
#endif
#ifdef ENABLE_BACK_LEFT
    Serial.println("Enter BL to find joint limits for back left leg");
#endif
#ifdef ENABLE_BACK_RIGHT
    Serial.println("Enter BR to find joint limits for back right leg");
#endif
    Serial.println("Enter exit to exit the find joint limits sequence");
    while (Serial.available() == 0)
      ;
    find_input = rdStr();
    if (find_input == "FL")
    {
#ifdef ENABLE_FRONT_LEFT
      front_AB.find_joint_neutral_position('u', LEFT);
      front_HIP.find_joint_neutral_position('u', LEFT);
      front_KNEE.find_joint_neutral_position('u', LEFT);
#endif
    }
    else if (find_input == "FR")
    {
#ifdef ENABLE_FRONT_RIGHT
      front_AB.find_joint_neutral_position('u', RIGHT);
      front_HIP.find_joint_neutral_position('u', RIGHT);
      front_KNEE.find_joint_neutral_position('u', RIGHT);
#endif
    }
    else if (find_input == "BL")
    {
#ifdef ENABLE_BACK_LEFT
      back_AB.find_joint_neutral_position('u', LEFT);
      back_HIP.find_joint_neutral_position('u', LEFT);
      back_KNEE.find_joint_neutral_position('u', LEFT);
#endif
    }
    else if (find_input == "BR")
    {
#ifdef ENABLE_BACK_RIGHT
      back_AB.find_joint_neutral_position('u', RIGHT);
      back_HIP.find_joint_neutral_position('u', RIGHT);
      back_KNEE.find_joint_neutral_position('u', RIGHT);
#endif
    }
  }
}

/**
 * Read ODrives' error codes
 * Read and display the error codes of ODrives
 * Serial communication with a PC is required
 */
void readError()
{
  Serial.println("Error codes: ");
#ifdef ENABLE_FRONT_LEFT
  Serial.print(" F-KNEE-L: ");
  Serial.print(front_KNEE.getAxisError(LEFT, true));
  Serial.print(" F-HIP-L: ");
  Serial.print(front_HIP.getAxisError(LEFT, true));
  Serial.print(" F-AB-L: ");
  Serial.println(front_AB.getAxisError(LEFT, true));
#endif
#ifdef ENABLE_FRONT_RIGHT
  Serial.print(" F-KNEE-R: ");
  Serial.print(front_KNEE.getAxisError(RIGHT, true));
  Serial.print(" F-HIP-R: ");
  Serial.print(front_HIP.getAxisError(RIGHT, true));
  Serial.print(" F-AB-R: ");
  Serial.println(front_AB.getAxisError(RIGHT, true));
#endif
#ifdef ENABLE_BACK_LEFT
  Serial.print(" B-KNEE-L: ");
  Serial.print(back_KNEE.getAxisError(LEFT, true));
  Serial.print(" B-HIP-L: ");
  Serial.print(back_HIP.getAxisError(LEFT, true));
  Serial.print(" B-AB-L: ");
  Serial.println(back_AB.getAxisError(LEFT, true));
#endif
#ifdef ENABLE_BACK_RIGHT
  Serial.print(" B-KNEE-R: ");
  Serial.print(back_KNEE.getAxisError(RIGHT, true));
  Serial.print(" B-HIP-R: ");
  Serial.print(back_HIP.getAxisError(RIGHT, true));
  Serial.print(" B-AB-R: ");
  Serial.println(back_AB.getAxisError(RIGHT, true));
#endif
}

/**
 * Read and print out joint positions in degrees
 * Serial communication with a PC is required
 */
void readJointPosition()
{
#ifdef ENABLE_FRONT_LEFT
  Serial.print(" F-AB-L: ");
  Serial.print(front_AB.transPosition_num2deg(LEFT, front_AB.getAxisPos(LEFT, true)));
  Serial.print(" F-HIP-L: ");
  Serial.print(front_HIP.transPosition_num2deg(LEFT, front_HIP.getAxisPos(LEFT, true)));
  Serial.print(" F-KNEE-L: ");
  Serial.println(front_KNEE.transPosition_num2deg(LEFT, front_KNEE.getAxisPos(LEFT, true)));
#endif
#ifdef ENABLE_FRONT_RIGHT
  Serial.print(" F-AB-R: ");
  Serial.print(front_AB.transPosition_num2deg(RIGHT, front_AB.getAxisPos(RIGHT, true)));
  Serial.print(" F-HIP-R: ");
  Serial.print(front_HIP.transPosition_num2deg(RIGHT, front_HIP.getAxisPos(RIGHT, true)));
  Serial.print(" F-KNEE-R: ");
  Serial.println(front_KNEE.transPosition_num2deg(RIGHT, front_KNEE.getAxisPos(RIGHT, true)));
#endif
#ifdef ENABLE_BACK_LEFT
  Serial.print(" B-AB-L: ");
  Serial.print(back_AB.transPosition_num2deg(LEFT, back_AB.getAxisPos(LEFT, true)));
  Serial.print(" B-HIP-L: ");
  Serial.print(back_HIP.transPosition_num2deg(LEFT, back_HIP.getAxisPos(LEFT, true)));
  Serial.print(" B-KNEE-L: ");
  Serial.println(back_KNEE.transPosition_num2deg(LEFT, back_KNEE.getAxisPos(LEFT, true)));
#endif
#ifdef ENABLE_BACK_RIGHT
  Serial.print(" B-AB-R: ");
  Serial.print(back_AB.transPosition_num2deg(RIGHT, back_AB.getAxisPos(RIGHT, true)));
  Serial.print(" B-HIP-R: ");
  Serial.print(back_HIP.transPosition_num2deg(RIGHT, back_HIP.getAxisPos(RIGHT, true)));
  Serial.print(" B-KNEE-R: ");
  Serial.println(back_KNEE.transPosition_num2deg(RIGHT, back_KNEE.getAxisPos(RIGHT, true)));
#endif
}

/**
 * Arm joints
 * Set the control state of ODrives to "state 8" - closed loop control state
 */
void armJoints()
{
#ifdef ENABLE_FRONT
  front_AB.armAxis();
  front_HIP.armAxis();
  front_KNEE.armAxis();
#endif
#ifdef ENABLE_BACK
  back_AB.armAxis();
  back_HIP.armAxis();
  back_KNEE.armAxis();
#endif
}

/**
 * Disarm joints
 * Set the control state of ODrives to "state 1" - idle state
 */
void disarmJoints()
{
#ifdef ENABLE_FRONT
  front_AB.disarmAxis();
  front_HIP.disarmAxis();
  front_KNEE.disarmAxis();
#endif
#ifdef ENABLE_BACK
  back_AB.disarmAxis();
  back_HIP.disarmAxis();
  back_KNEE.disarmAxis();
#endif
}

/**
 * Calibrate joints
 * Set the control state of ODrives to "state 3" - calibration state
 */
void calibJoints()
{
  String calib_input = "yeah";
  while (calib_input != "exit")
  {
    Serial.println("-----------------------------------------");
#ifdef ENABLE_FRONT
    Serial.println("Enter fab to calibrate front AB joints");
    Serial.println("Enter fhip to calibrate front HIP joints");
    Serial.println("Enter fknee to calibrate front KNEE joints");
#endif
#ifdef ENABLE_BACK
    Serial.println("Enter bab to calibrate back AB joints");
    Serial.println("Enter bhip to calibrate back HIP joints");
    Serial.println("Enter bknee to calibrate back KNEE joints");
#endif

    Serial.println("Enter exit to exit the calibration sequence");
    while (Serial.available() == 0)
      ;
    calib_input = rdStr();
    if (calib_input == "fab")
    {
#ifdef ENABLE_FRONT
      Serial.println("Calibrating front AB joints");
      front_AB.run_state(0, 3, false);
      front_AB.run_state(1, 3, false);
#endif
    }
    else if (calib_input == "fhip")
    {
#ifdef ENABLE_FRONT
      Serial.println("Calibrating front HIP joints");
      front_HIP.run_state(0, 3, false);
      front_HIP.run_state(1, 3, false);
#endif
    }
    else if (calib_input == "fknee")
    {
#ifdef ENABLE_FRONT
      Serial.println("Calibrating front KNEE joints");
      front_KNEE.run_state(0, 3, false);
      front_KNEE.run_state(1, 3, false);
#endif
    }
    else if (calib_input == "bab")
    {
#ifdef ENABLE_BACK
      Serial.println("Calibrating back AB joints");
      back_AB.run_state(0, 3, false);
      back_AB.run_state(1, 3, false);
#endif
    }
    else if (calib_input == "bhip")
    {
#ifdef ENABLE_BACK
      Serial.println("Calibrating back HIP joints");
      back_HIP.run_state(0, 3, false);
      back_HIP.run_state(1, 3, false);
#endif
    }
    else if (calib_input == "bknee")
    {
#ifdef ENABLE_BACK
      Serial.println("Calibrating back KNEE joints");
      back_KNEE.run_state(0, 3, false);
      back_KNEE.run_state(1, 3, false);
#endif
    }
  }
}

void moveToPos_STDBY_blocking()
{
  armJoints();
// #ifdef ENABLE_FRONT_LEFT
//   front_AB.update_target(LEFT, true, 5000000, -AB_STANDBY_POS_DEG);
//   front_HIP.update_target(LEFT, true, 5000000, -HIP_STANDBY_POS_DEG);
//   front_KNEE.update_target(LEFT, true, 5000000, -KNEE_STANDBY_POS_DEG);
// #endif
// #ifdef ENABLE_FRONT_RIGHT
//   front_AB.update_target(RIGHT, true, 5000000, AB_STANDBY_POS_DEG);
//   front_HIP.update_target(RIGHT, true, 5000000, HIP_STANDBY_POS_DEG);
//   front_KNEE.update_target(RIGHT, true, 5000000, KNEE_STANDBY_POS_DEG);
// #endif
// #ifdef ENABLE_BACK_LEFT
//   back_AB.update_target(LEFT, true, 5000000, AB_STANDBY_POS_DEG);
//   back_HIP.update_target(LEFT, true, 5000000, -HIP_STANDBY_POS_DEG);
//   back_KNEE.update_target(LEFT, true, 5000000, -KNEE_STANDBY_POS_DEG);
// #endif
// #ifdef ENABLE_BACK_RIGHT
//   back_AB.update_target(RIGHT, true, 5000000, -AB_STANDBY_POS_DEG);
//   back_HIP.update_target(RIGHT, true, 5000000, HIP_STANDBY_POS_DEG);
//   back_KNEE.update_target(RIGHT, true, 5000000, KNEE_STANDBY_POS_DEG);
// #enfdif
#ifdef ENABLE_FRONT_LEFT
  front_AB.update_target(LEFT, true, 5000000, -AB_STANDBY_POS_DEG);
  front_HIP.update_target(LEFT, true, 5000000, -HIP_STANDBY_POS_DEG);
  front_KNEE.update_target(LEFT, true, 5000000, -KNEE_STANDBY_POS_DEG);
#endif
#ifdef ENABLE_FRONT_RIGHT
  front_AB.update_target(RIGHT, true, 5000000, AB_STANDBY_POS_DEG);
  front_HIP.update_target(RIGHT, true, 5000000, HIP_STANDBY_POS_DEG);
  front_KNEE.update_target(RIGHT, true, 5000000, KNEE_STANDBY_POS_DEG);
#endif
#ifdef ENABLE_BACK_LEFT
  back_AB.update_target(LEFT, true, 5000000, AB_STANDBY_POS_DEG);
  back_HIP.update_target(LEFT, true, 5000000, HIP_STANDBY_POS_DEG);
  back_KNEE.update_target(LEFT, true, 5000000, KNEE_STANDBY_POS_DEG);
#endif
#ifdef ENABLE_BACK_RIGHT
  back_AB.update_target(RIGHT, true, 5000000, -AB_STANDBY_POS_DEG);
  back_HIP.update_target(RIGHT, true, 5000000, -HIP_STANDBY_POS_DEG);
  back_KNEE.update_target(RIGHT, true, 5000000, -KNEE_STANDBY_POS_DEG);
#endif
  timer.reset();
  int stdbytimer = millis();
  while (millis() - stdbytimer <= 5000)
  {
    if (timer.check())
    {
#ifdef ENABLE_FRONT_LEFT
      front_AB.update(LEFT);
      front_HIP.update(LEFT);
      front_KNEE.update(LEFT);
#endif
#ifdef ENABLE_FRONT_RIGHT
      front_AB.update(RIGHT);
      front_HIP.update(RIGHT);
      front_KNEE.update(RIGHT);
#endif
#ifdef ENABLE_BACK_LEFT
      back_AB.update(LEFT);
      back_HIP.update(LEFT);
      back_KNEE.update(LEFT);
#endif
#ifdef ENABLE_BACK_RIGHT
      back_AB.update(RIGHT);
      back_HIP.update(RIGHT);
      back_KNEE.update(RIGHT);
#endif
    }
  }
}

void config_sequence()
{
  while (true)
  {
    Serial.println("=========================================");
    Serial.println("Enter readError to read axes error code");
    Serial.println("Enter calib to initiate joint calibration sequence");
    Serial.println("Enter find to calibrate neutral positions");
    Serial.println("Enter mfind to calibrate neutral positions");
    Serial.println("-----------------------------------------");
    Serial.println("Enter stdby to proceed to STANDBY pos");
    Serial.println("Enter dis to disarm all motors");
    Serial.println("Enter deg to view joint positions");
    Serial.println("Enter manual to enter manual command mode");
    Serial.println("Enter 1 to disarm front left leg");
    Serial.println("Enter 2 to disarm front right leg");
    Serial.println("Enter 3 to disarm back left leg");
    Serial.println("Enter 4 to disarm back right leg");
    Serial.println("-----------------------------------------");
    Serial.println("Enter exit to exit the config routine");
    Serial.println("=========================================");
    while (Serial.available() == 0)
      ;
    String serial_input = rdStr();
    Serial.println(serial_input);
    if (serial_input == "stdby")
    {
      moveToPos_STDBY_blocking();
    }
    else if (serial_input == "dis")
    {
      disarmJoints();
    }
    else if (serial_input == "deg")
    {
      readJointPosition();
    }
    else if (serial_input == "manual")
    {
      bool exit_flag = false;
      while (exit_flag == false)
      {
        //serial com shit
      }
    }
    else if (serial_input == "readError")
    {
      readError();
    }
    else if (serial_input == "calib")
    {
      calibJoints();
    }
    else if (serial_input == "find")
    {
      find_joint_neutral_position();
    }
    else if (serial_input == "mfind")
    {
      manual_find();
    }
    else if (serial_input == "1")
    {
      front_AB.disarmAxis(LEFT);
      front_HIP.disarmAxis(LEFT);
      front_KNEE.disarmAxis(LEFT);
    }
    else if (serial_input == "2")
    {
      front_AB.disarmAxis(RIGHT);
      front_HIP.disarmAxis(RIGHT);
      front_KNEE.disarmAxis(RIGHT);
    }
    else if (serial_input == "3")
    {
      back_AB.disarmAxis(LEFT);
      back_HIP.disarmAxis(LEFT);
      back_KNEE.disarmAxis(LEFT);
    }
    else if (serial_input == "4")
    {
      back_AB.disarmAxis(RIGHT);
      back_HIP.disarmAxis(RIGHT);
      back_KNEE.disarmAxis(RIGHT);
    }
    else if (serial_input == "qwe")
    {
      float a = manual_input_num();
    }
    else if (serial_input == "exit")
    {
      break;
    }
  }
}

void setup()
{
  // Serial to PC @ 921600 baud rate
  Serial.begin(1000000);
  while (!Serial)
    ;
  Serial.println("Computer Serial connected");
#ifdef ENABLE_FRONT
  odrv_connect(front_AB);
  odrv_connect(front_HIP);
  odrv_connect(front_KNEE);
#endif
#ifdef ENABLE_BACK
  odrv_connect(back_AB);
  odrv_connect(back_HIP);
  odrv_connect(back_KNEE);
#endif
  // radio.calibration();
  config_sequence();

  radio_readings.debug_ini();

  Serial.println("ending setup");
}

void loop()
{

  // process state requests
  String serial_input;
  if (Serial.available() != 0)
  {
    serial_input = rdStr();
    Serial.println(serial_input);
    Serial.println("-------");
  }

  if (serial_input == "dis")
  {
    disarmJoints();
    Serial.println("stop");
    state = STATE_IDLE;
  }
  else if (serial_input == "config")
  {
    state = STATE_IDLE;
    config_sequence();
  }
  else if (serial_input == "test")
  {
    Serial.println("testing...");
    timer.reset();
    test_timer = millis();
    state = STATE_K_UPDATE;
  }
  else if (serial_input == "stop")
  {
    manual_stop = true;
  }
  else if (serial_input == "reset")
  {
    manual_stop = false;
    qPed.reset();
  }

  // state machine
  switch (state)
  {
  case STATE_UPDATE:
  {
    if (timer.check())
    {
      front_AB.update(LEFT);
      front_AB.update(RIGHT);

      front_HIP.update(LEFT);
      front_HIP.update(RIGHT);

      front_KNEE.update(LEFT);
      front_KNEE.update(RIGHT);

      back_AB.update(LEFT);
      back_AB.update(RIGHT);

      back_HIP.update(LEFT);
      back_HIP.update(RIGHT);

      back_KNEE.update(LEFT);
      back_KNEE.update(RIGHT);

      // Serial.println("----------------");
    }
  }
  break;
  case STATE_K_UPDATE:
  {
    if (timer.check())
    {
      if (!manual_stop)
      {
        radio_readings.debug_update(1500, 1500, 1500, 2000);
      }
      // else if (millis()-test_timer >= 30000 && millis()-test_timer <= 40000 && !manual_stop)
      // {
      //   radio_readings.debug_update(1500,1500,1500,1000);
      // }
      // else if (millis()-test_timer >= 50000 && millis()-test_timer <= 55000 && !manual_stop)
      // {
      //   radio_readings.debug_update(2000,1500,1500,1500);
      // }
      else
      {
        radio_readings.debug_update(1500, 1500, 1500, 1500);
      }

      qPed.update(kine_time, radio_readings);
      if (qPed.kinematic_task != TASK_IDLE)
      {
        kine_time = kine_time + msg_timer_interval / 1000;
      }
      else
      {
        kine_time = 0;
      }
      Serial.print("Time: ");
      Serial.println(kine_time, 4);
      qPed.debug_print();

      front_AB.SetPosition(LEFT, -qPed.FL.ab_rad / PI_math * 180);
      front_HIP.SetPosition(LEFT, -qPed.FL.hip_rad / PI_math * 180);
      front_KNEE.SetPosition(LEFT, -qPed.FL.knee_rad / PI_math * 180);

      front_AB.SetPosition(RIGHT, -qPed.FR.ab_rad / PI_math * 180);
      front_HIP.SetPosition(RIGHT, -qPed.FR.hip_rad / PI_math * 180);
      front_KNEE.SetPosition(RIGHT, -qPed.FR.knee_rad / PI_math * 180);

      back_AB.SetPosition(LEFT, -qPed.BL.ab_rad / PI_math * 180);
      back_HIP.SetPosition(LEFT, -qPed.BL.hip_rad / PI_math * 180);
      back_KNEE.SetPosition(LEFT, -qPed.BL.knee_rad / PI_math * 180);

      back_AB.SetPosition(RIGHT, -qPed.BR.ab_rad / PI_math * 180);
      back_HIP.SetPosition(RIGHT, -qPed.BR.hip_rad / PI_math * 180);
      back_KNEE.SetPosition(RIGHT, -qPed.BR.knee_rad / PI_math * 180);
    }
  }
  break;
  default:
    break;
  }
}
