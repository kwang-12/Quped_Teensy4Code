// Libraries:
#include "src/lib/globals.h"
#include "src/lib/ODriveArduino.h"
#include "src/lib/jointPositions.h"
#include "src/lib/leg_timer.h"
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

// jointPositions object
jointPositions joint;
jointPositions *joint_ptr = &joint;

// Initialize timer
Metro timer = Metro(SERIAL_MSG_TIMER);

leg_timer legtimertest(2.0, 5, 400);

// Initialize trajectory ticker
int tick_fL = 0;
int tick_fR = 0;
int tick_bL = 0;
int tick_bR = 0;

// Initialize trajectory time stamp
float time_fL = 0.0;
float time_fR = 0.0;
float time_bL = 0.0;
float time_bR = 0.0;
char gait_state = GAIT_IDLE;
bool flag_fL_end = false;
bool flag_fR_end = false;
bool flag_bL_end = false;
bool flag_bR_end = false;
bool flag_normal_cycle = false;
bool flag_end_cycle = false;
int step_counter = 0;
char requested_gait_state = GAIT_IDLE;
bool cycle_end_flag = false;

/**
 * State variable
 * 'z' = idle
 * '1' = bspline
 */
char loop_state = STATE_IDLE;

unsigned long timerrr;

float pos3_ab = 0.0;
float pos3_hip = 0.0;
float pos3_knee = 0.0;

float pos4_ab = 0.0;
float pos4_hip = 0.0;
float pos4_knee = 0.0;

float pos5_ab_FL = 0.0;
float pos5_hip_FL = 0.0;
float pos5_knee_FL = 0.0;
float pos5_ab_FR = 0.0;
float pos5_hip_FR = 0.0;
float pos5_knee_FR = 0.0;
float pos5_ab_bL = 0.0;
float pos5_hip_bL = 0.0;
float pos5_knee_bL = 0.0;
float pos5_ab_bR = 0.0;
float pos5_hip_bR = 0.0;
float pos5_knee_bR = 0.0;

float pos6_ab_FL = 0.0;
float pos6_hip_FL = 0.0;
float pos6_knee_FL = 0.0;
float pos6_ab_FR = 0.0;
float pos6_hip_FR = 0.0;
float pos6_knee_FR = 0.0;
float pos6_ab_bL = 0.0;
float pos6_hip_bL = 0.0;
float pos6_knee_bL = 0.0;
float pos6_ab_bR = 0.0;
float pos6_hip_bR = 0.0;
float pos6_knee_bR = 0.0;

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
#ifdef ENABLE_FRONT_LEFT
  front_AB.find_joint_neutral_position('r', LEFT);
  front_HIP.find_joint_neutral_position('c', LEFT);
  front_KNEE.find_joint_neutral_position('r', LEFT);
#endif
#ifdef ENABLE_FRONT_RIGHT
  front_AB.find_joint_neutral_position('r', RIGHT);
  front_HIP.find_joint_neutral_position('c', RIGHT);
  front_KNEE.find_joint_neutral_position('r', RIGHT);
#endif
#ifdef ENABLE_BACK_LEFT
  back_AB.find_joint_neutral_position('r', LEFT);
  back_HIP.find_joint_neutral_position('c', LEFT);
  back_KNEE.find_joint_neutral_position('r', LEFT);
#endif
#ifdef ENABLE_BACK_RIGHT
  back_AB.find_joint_neutral_position('r', RIGHT);
  back_HIP.find_joint_neutral_position('c', RIGHT);
  back_KNEE.find_joint_neutral_position('r', RIGHT);
#endif
}

/**
 * reset the traj array ticks to 0
 */
void reset_tick()
{
  tick_fL = 0;
  tick_fR = 0;
  tick_bL = 0;
  tick_bR = 0;
}

/**
 * update the traj array tick
 * @param mode - choice of gait
 * @param state - state of gait
 */
void update_tick(char mode, char state)
{
  if (mode == TROT_GAIT)
  {
    if (state == GAIT_STARTING)
    {
      tick_fL++;
      tick_fR = 0;
      tick_bL = 0;
      tick_bR++;
    }
    else if (state == GAIT_ENDING)
    {
      tick_fL = 0;
      tick_fR++;
      tick_bL++;
      tick_bR = 0;
    }
    else if (state == GAIT_NORMAL)
    {
      tick_fL++;
      if (tick_fL >= 0 && tick_fL <= 99)
      {
        tick_bR = tick_fL;
        tick_fR = tick_fL + 100;
        tick_bL = tick_fL + 100;
      }
      else
      {
        tick_bR = tick_fL;
        tick_fR = tick_fL - 100;
        tick_bL = tick_fL - 100;
      }
    }
  }
  else if (mode == PACE_GAIT)
  {
  }
  else if (mode == BOUND_GAIT)
  {
  }
  else if (mode == WAVE_GAIT)
  {
  }
}

/**
 * reset the time stamps to 0
 */
void reset_time()
{
  time_bL = 0;
  time_bR = 0;
  time_fL = 0;
  time_fR = 0;
  flag_fL_end = false;
  flag_fR_end = false;
  flag_bL_end = false;
  flag_bR_end = false;
  flag_end_cycle = false;
  step_counter = 0;
  gait_state = GAIT_STARTING;
  flag_normal_cycle = false;
  cycle_end_flag = false;
}

/**
 * update time stamp based on walk cycle time, duty factor and gait choice
 * @param cycleTime - time of a one step cycle [sec]
 * @param dutyFactor - percent support period
 * @param mode - choice of gait
 * @param choice - state of gait
 */
void update_time(float cycleTime, float dutyFactor, char mode, char choice)
{
  // unsigned short delta_ticker = 0.25 * cycleTime / SERIAL_MSG_TIME_INTERVAL;
  if (mode == CRAWL_GAIT)
  {
    // Serial.println("238");
    if (choice == GAIT_STARTING)
    {
      if (gait_state == GAIT_STARTING)
      {
        time_fL = time_fL + SERIAL_MSG_TIME_INTERVAL;
      }
      // The starting phase has completed. Switch to normal phase.
      if (time_fL > cycleTime)
      {
        gait_state = GAIT_NORMAL;
        // 1423 crawl gait
        time_fL = 0.0 + SERIAL_MSG_TIME_INTERVAL;
        time_bR = 0.75 * cycleTime + SERIAL_MSG_TIME_INTERVAL;
        time_fR = 0.5 * cycleTime + SERIAL_MSG_TIME_INTERVAL;
        time_bL = 0.25 * cycleTime + SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_fL > 0.25 * cycleTime)
      {
        time_bR = time_bR + SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_fL > 0.5 * cycleTime)
      {
        time_fR = time_fR + SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_fL > 0.75 * cycleTime)
      {
        time_bL = time_bL + SERIAL_MSG_TIME_INTERVAL;
      }
    }
    else if (choice == GAIT_ENDING)
    {
      /**
       * Increment the time stamp of each leg indenpendently until the time stamp has reached
       * the end point (flag_XX_end is set to true)
       * @param flag_end_cycle is set to true when all the leg has finished the cycle
       */
      if (flag_fL_end == false)
      {
        time_fL = time_fL + SERIAL_MSG_TIME_INTERVAL;
      }
      if (flag_bR_end == false)
      {
        time_bR = time_bR + SERIAL_MSG_TIME_INTERVAL;
      }
      if (flag_fR_end == false)
      {
        time_fR = time_fR + SERIAL_MSG_TIME_INTERVAL;
      }
      if (flag_bL_end == false)
      {
        time_bL = time_bL + SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_fL > cycleTime)
      {
        time_fL = 0.0;
        flag_fL_end = true;
      }
      if (time_bR > cycleTime)
      {
        time_bR = 0.0;
        flag_bR_end = true;
      }
      if (time_fR > cycleTime)
      {
        time_fR = 0.0;
        flag_fR_end = true;
      }
      if (time_bL > cycleTime)
      {
        time_bL = 0.0;
        flag_bL_end = true;
      }
      if (flag_fL_end == true && flag_bL_end == true && flag_fR_end == true && flag_bR_end == true)
      {
        flag_end_cycle = true;
      }
      else
      {
        gait_state = GAIT_ENDING;
      }
    }
    /**
     *  The normal phase. Time stamp of each leg increments independently of each other.
     *  @param flag_normal_cycle is set to true everytime a cycle is completed
     */
    else if (choice == GAIT_NORMAL)
    {
      time_fL = time_fL + SERIAL_MSG_TIME_INTERVAL;
      time_bR = time_bR + SERIAL_MSG_TIME_INTERVAL;
      time_fR = time_fR + SERIAL_MSG_TIME_INTERVAL;
      time_bL = time_bL + SERIAL_MSG_TIME_INTERVAL;
      if (time_fL > 2.0)
      {
        time_fL = SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_bR > 2.0)
      {
        time_bR = SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_fR > 2.0)
      {
        time_fR = SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_bL > 2.0)
      {
        time_bL = SERIAL_MSG_TIME_INTERVAL;
        flag_normal_cycle = true;
      }
    }
    else if (choice == GAIT_ONESTEP)
    {
      if (flag_fL_end == false)
      {
        time_fL = time_fL + SERIAL_MSG_TIME_INTERVAL;
      }
      if ((time_fL > 0.25 * cycleTime || flag_fL_end == true) && flag_bR_end == false)
      {
        time_bR = time_bR + SERIAL_MSG_TIME_INTERVAL;
      }
      if ((time_fL > 0.5 * cycleTime || flag_fL_end == true) && flag_fR_end == false)
      {
        time_fR = time_fR + SERIAL_MSG_TIME_INTERVAL;
      }
      if ((time_fL > 0.75 * cycleTime || flag_fL_end == true) && flag_bL_end == false)
      {
        time_bL = time_bL + SERIAL_MSG_TIME_INTERVAL;
      }
      if (time_fL > cycleTime)
      {
        time_fL = 0.0;
        flag_fL_end = true;
      }
      if (time_bR > cycleTime)
      {
        time_bR = 0.0;
        flag_bR_end = true;
      }
      if (time_fR > cycleTime)
      {
        time_fR = 0.0;
        flag_fR_end = true;
      }
      if (time_bL > cycleTime)
      {
        time_bL = 0.0;
        flag_bL_end = true;
      }
      if (flag_fL_end == true && flag_bR_end == true && flag_fR_end == true && flag_bL_end == true)
      {
        flag_end_cycle = true;
      }
    }
  }
  else if (mode == WAVE_GAIT)
  {
  }
}

/**
 * Send desired joint position and velocities to ODrives
 * send the updated joint positions to odrives
 * Note that the positions usually need to be updated prior to sending
 */
void send(jointPositions *joint_ptr, ODriveArduino *front_AB_ptr, ODriveArduino *front_HIP_ptr, ODriveArduino *front_KNEE_ptr,
          ODriveArduino *back_AB_ptr, ODriveArduino *back_HIP_ptr, ODriveArduino *back_KNEE_ptr)
{
  // Serial.print(millis());
  // Serial.print(" - fL: ");
  // Serial.print(tick_fL);
  // Serial.print(" - fR: ");
  // Serial.print(tick_fR);
  // Serial.print(" - bL: ");
  // Serial.print(tick_bL);
  // Serial.print(" - bR: ");
  // Serial.println(tick_bR);
#ifdef ENABLE_FRONT_RIGHT
  front_AB_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.fR_ab_pos, joint_ptr->joint_vel_target_deg.fR_ab_velo);
  front_HIP_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.fR_hip_pos, joint_ptr->joint_vel_target_deg.fR_hip_velo);
  front_KNEE_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.fR_knee_pos, joint_ptr->joint_vel_target_deg.fR_knee_velo);
#endif
#ifdef ENABLE_FRONT_LEFT
  front_AB_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.fL_ab_pos, joint_ptr->joint_vel_target_deg.fL_ab_velo);
  front_HIP_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.fL_hip_pos, joint_ptr->joint_vel_target_deg.fL_hip_velo);
  front_KNEE_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.fL_knee_pos, joint_ptr->joint_vel_target_deg.fL_knee_velo);
#endif
#ifdef ENABLE_BACK_RIGHT
  back_AB_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.bR_ab_pos, joint_ptr->joint_vel_target_deg.bR_ab_velo);
  back_HIP_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.bR_hip_pos, joint_ptr->joint_vel_target_deg.bR_hip_velo);
  back_KNEE_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.bR_knee_pos, joint_ptr->joint_vel_target_deg.bR_knee_velo);
#endif
#ifdef ENABLE_BACK_LEFT
  back_AB_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.bL_ab_pos, joint_ptr->joint_vel_target_deg.bL_ab_velo);
  back_HIP_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.bL_hip_pos, joint_ptr->joint_vel_target_deg.bL_hip_velo);
  back_KNEE_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.bL_knee_pos, joint_ptr->joint_vel_target_deg.bL_knee_velo);
#endif
}

/**
 * Read String from PC until newline is detected
 */
String readString()
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
    calib_input = readString();
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
  back_HIP.update_target(LEFT, true, 5000000, -HIP_STANDBY_POS_DEG);
  back_KNEE.update_target(LEFT, true, 5000000, -KNEE_STANDBY_POS_DEG);
#endif
#ifdef ENABLE_BACK_RIGHT
  back_AB.update_target(RIGHT, true, 5000000, -AB_STANDBY_POS_DEG);
  back_HIP.update_target(RIGHT, true, 5000000, HIP_STANDBY_POS_DEG);
  back_KNEE.update_target(RIGHT, true, 5000000, KNEE_STANDBY_POS_DEG);
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

void manualInput_pos(int choice)
{
  String input;
  if (choice == 3)
  {
    Serial.println("P3");
    Serial.println("Enter ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos3_ab = input.toInt();

    Serial.println("Enter hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos3_hip = input.toInt();

    Serial.println("Enter knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos3_knee = input.toInt();
  }
  if (choice == 4)
  {
    Serial.println("P4");
    Serial.println("Enter ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos4_ab = input.toInt();

    Serial.println("Enter hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos4_hip = input.toInt();

    Serial.println("Enter knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos4_knee = input.toInt();
  }
  if (choice == 5)
  {
    Serial.println("P5 FL");
    Serial.println("Enter FL ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_FL = input.toFloat();

    Serial.println("Enter FL hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_FL = input.toFloat();

    Serial.println("Enter FL knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_FL = input.toFloat();

    Serial.println("P5 FR");
    Serial.println("Enter FR ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_FR = input.toFloat();

    Serial.println("Enter FR hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_FR = input.toFloat();

    Serial.println("Enter FR knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_FR = input.toFloat();

    Serial.println("P5 BL");
    Serial.println("Enter BL ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_bL = input.toFloat();

    Serial.println("Enter BL hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_bL = input.toFloat();

    Serial.println("Enter BL knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_bL = input.toFloat();

    Serial.println("P5 BR");
    Serial.println("Enter BR ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_bR = input.toFloat();

    Serial.println("Enter BR hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_bR = input.toFloat();

    Serial.println("Enter BR knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_bR = input.toFloat();
  }
  if (choice == 6)
  {
    Serial.println("P6 FL");
    Serial.println("Enter FL ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_FL = input.toFloat();

    Serial.println("Enter FL hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_FL = input.toFloat();

    Serial.println("Enter FL knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos6_knee_FL = input.toFloat();

    Serial.println("P5 FR");
    Serial.println("Enter FR ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_FR = input.toFloat();

    Serial.println("Enter FR hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_FR = input.toFloat();

    Serial.println("Enter FR knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos6_knee_FR = input.toFloat();

    Serial.println("P5 BL");
    Serial.println("Enter BL ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_bL = input.toFloat();

    Serial.println("Enter BL hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_bL = input.toFloat();

    Serial.println("Enter BL knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos6_knee_bL = input.toFloat();

    Serial.println("P5 BR");
    Serial.println("Enter BR ab pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_bR = input.toFloat();

    Serial.println("Enter BR hip pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_bR = input.toFloat();

    Serial.println("Enter BR knee pos");
    while (Serial.available() == 0)
      ;
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos6_knee_bR = input.toFloat();
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
    Serial.println("-----------------------------------------");
    Serial.println("Enter stdby to proceed to STANDBY pos");
    Serial.println("Enter dis to disarm all motors");
    Serial.println("Enter deg to view joint positions");
    Serial.println("Enter test to test time for bspline calculation");
    Serial.println("Enter manual to enter manual command mode");
    Serial.println("Enter 1 to disarm front left leg");
    Serial.println("Enter 2 to disarm front right leg");
    Serial.println("Enter 3 to disarm back left leg");
    Serial.println("Enter 4 to disarm back right leg");
    Serial.println("Enter c3 to manually enter pos3");
    Serial.println("Enter c4 to manually enter pos4");
    Serial.println("Enter c5 to manually enter pos5");
    Serial.println("Enter c6 to manually enter pos6");
    Serial.println("Enter check to display all pos deg");
    Serial.println("-----------------------------------------");
    Serial.println("Enter exit to exit the config routine");
    Serial.println("=========================================");
    while (Serial.available() == 0)
      ;
    String serial_input = readString();
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
    else if (serial_input == "c3")
    {
      manualInput_pos(3);
    }
    else if (serial_input == "c4")
    {
      manualInput_pos(4);
    }
    else if (serial_input == "c5")
    {
      manualInput_pos(5);
    }
    else if (serial_input == "c6")
    {
      manualInput_pos(6);
    }
    else if (serial_input == "check")
    {
      Serial.println("pos1");
      Serial.print(AB_POS_1);
      Serial.print(' ');
      Serial.print(HIP_POS_1);
      Serial.print(' ');
      Serial.println(KNEE_POS_1);

      Serial.println("pos2");
      Serial.print(AB_POS_2);
      Serial.print(' ');
      Serial.print(HIP_POS_2);
      Serial.print(' ');
      Serial.println(KNEE_POS_2);

      Serial.println("pos3");
      Serial.print(pos3_ab);
      Serial.print(' ');
      Serial.print(pos3_hip);
      Serial.print(' ');
      Serial.println(pos3_knee);

      Serial.println("pos4");
      Serial.print(pos4_ab);
      Serial.print(' ');
      Serial.print(pos4_hip);
      Serial.print(' ');
      Serial.println(pos4_knee);

      Serial.println("pos5");
      Serial.print("FL ");
      Serial.print(pos5_ab_FL);
      Serial.print(' ');
      Serial.print(pos5_hip_FL);
      Serial.print(' ');
      Serial.println(pos5_knee_FL);
      Serial.print("FR ");
      Serial.print(pos5_ab_FR);
      Serial.print(' ');
      Serial.print(pos5_hip_FR);
      Serial.print(' ');
      Serial.println(pos5_knee_FR);
      Serial.print("BL ");
      Serial.print(pos5_ab_bL);
      Serial.print(' ');
      Serial.print(pos5_hip_bL);
      Serial.print(' ');
      Serial.println(pos5_knee_bL);
      Serial.print("BR ");
      Serial.print(pos5_ab_bR);
      Serial.print(' ');
      Serial.print(pos5_hip_bR);
      Serial.print(' ');
      Serial.println(pos5_knee_bR);

      Serial.println("pos6");
      Serial.print("FL ");
      Serial.print(pos6_ab_FL);
      Serial.print(' ');
      Serial.print(pos6_hip_FL);
      Serial.print(' ');
      Serial.println(pos6_knee_FL);
      Serial.print("FR ");
      Serial.print(pos6_ab_FR);
      Serial.print(' ');
      Serial.print(pos6_hip_FR);
      Serial.print(' ');
      Serial.println(pos6_knee_FR);
      Serial.print("BL ");
      Serial.print(pos6_ab_bL);
      Serial.print(' ');
      Serial.print(pos6_hip_bL);
      Serial.print(' ');
      Serial.println(pos6_knee_bL);
      Serial.print("BR ");
      Serial.print(pos6_ab_bR);
      Serial.print(' ');
      Serial.print(pos6_hip_bR);
      Serial.print(' ');
      Serial.println(pos6_knee_bR);
    }
    else if (serial_input == "trig")
    {
      float t1;
      float t2;
      float t3;
      bool test;
      Serial.println(micros());
      test = leg_inverseKinematics_pos(0, 0, 0, 0.4, 0.142, 0.05, t1, t2, t3, FRONT_LEFT_LEG);
      Serial.print(t1 / PI_math * 180);
      Serial.print(' ');
      Serial.print(t2 / PI_math * 180);
      Serial.print(' ');
      Serial.println(t3 / PI_math * 180);

      Serial.println(micros());
      Serial.println(micros());
      test = leg_inverseKinematics_pos(0, 0, 0, 0.4, 0.142, 0.05, t1, t2, t3, FRONT_RIGHT_LEG);
      Serial.print(t1 / PI_math * 180);
      Serial.print(' ');
      Serial.print(t2 / PI_math * 180);
      Serial.print(' ');
      Serial.println(t3 / PI_math * 180);

      Serial.println(micros());
      Serial.println(micros());
      test = leg_inverseKinematics_pos(0, 0, 0, 0.4, 0.142, -0.05, t1, t2, t3, BACK_LEFT_LEG);
      Serial.print(t1 / PI_math * 180);
      Serial.print(' ');
      Serial.print(t2 / PI_math * 180);
      Serial.print(' ');
      Serial.println(t3 / PI_math * 180);

      Serial.println(micros());
      Serial.println(micros());
      test = leg_inverseKinematics_pos(0, 0, 0, 0.4, 0.142, -0.05, t1, t2, t3, BACK_RIGHT_LEG);
      Serial.print(t1 / PI_math * 180);
      Serial.print(' ');
      Serial.print(t2 / PI_math * 180);
      Serial.print(' ');
      Serial.println(t3 / PI_math * 180);

      Serial.println(micros());
    }
    else if (serial_input == "trans")
    {
      float yaw_desired = static_cast<float>(30)/180*PI_math;
      float pitch_desired = static_cast<float>(10)/180*PI_math;
      float roll_desired = static_cast<float>(10)/180*PI_math;
      float forward_delta = 0;
      float horizontal_delta = 0;
      float vertical_delta = -0.05;
      bool FL_support = true;
      float FL_ab, FL_hip, FL_knee;
      bool FR_support = true;
      float FR_ab, FR_hip, FR_knee;
      bool BL_support = true;
      float BL_ab, BL_hip, BL_knee;
      bool BR_support = true;
      float BR_ab, BR_hip, BR_knee;

      bool test;

      BLA::Matrix<3> FL_vec;
      BLA::Matrix<3> FR_vec;
      BLA::Matrix<3> BL_vec;
      BLA::Matrix<3> BR_vec;
      Serial.println(micros());
      test = calc_posture_joint_pos(yaw_desired, pitch_desired, roll_desired, 
                                    forward_delta, horizontal_delta, vertical_delta,
                                    FL_ab, FL_hip, FL_knee, FL_support, 
                                    FR_ab, FR_hip, FR_knee, FR_support, 
                                    BR_ab, BR_hip, BR_knee, BR_support,
                                    BL_ab, BL_hip, BL_knee, BL_support);
      Serial.println(test);
      Serial.println(micros());
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

  Serial.println("ending setup");
}

void loop()
{
  legtimertest.update();

  // process state requests
  String serial_input;
  if (Serial.available() != 0)
  {
    serial_input = readString();
    Serial.println(serial_input);
    Serial.println("-------");
  }

  if (serial_input == "dis")
  {
    disarmJoints();
    loop_state = STATE_IDLE;
  }
  else if (serial_input == "config")
  {
    loop_state = STATE_IDLE;
    config_sequence();
  }
  else if (serial_input == "pos1")
  {
    loop_state = STATE_test;
#ifdef ENABLE_FRONT_LEFT
    front_AB.update_target(LEFT, true, 5000000, -AB_POS_1);
    front_HIP.update_target(LEFT, true, 5000000, -HIP_POS_1);
    front_KNEE.update_target(LEFT, true, 5000000, -KNEE_POS_1);
#endif
#ifdef ENABLE_FRONT_RIGHT
    front_AB.update_target(RIGHT, true, 5000000, AB_POS_2);
    front_HIP.update_target(RIGHT, true, 5000000, HIP_POS_2);
    front_KNEE.update_target(RIGHT, true, 5000000, KNEE_POS_2);
#endif
#ifdef ENABLE_BACK_LEFT
    back_AB.update_target(LEFT, true, 5000000, AB_POS_1);
    back_HIP.update_target(LEFT, true, 5000000, -HIP_POS_1);
    back_KNEE.update_target(LEFT, true, 5000000, -KNEE_POS_1);
#endif
#ifdef ENABLE_BACK_RIGHT
    back_AB.update_target(RIGHT, true, 5000000, -AB_POS_2);
    back_HIP.update_target(RIGHT, true, 5000000, HIP_POS_2);
    back_KNEE.update_target(RIGHT, true, 5000000, KNEE_POS_2);
#endif
    Serial.println("POS1");
    timer.reset();
  }
  else if (serial_input == "pos2")
  {
    loop_state = STATE_test;
#ifdef ENABLE_FRONT_LEFT
    front_AB.update_target(LEFT, true, 5000000, -AB_POS_2);
    front_HIP.update_target(LEFT, true, 5000000, -HIP_POS_2);
    front_KNEE.update_target(LEFT, true, 5000000, -KNEE_POS_2);
#endif
#ifdef ENABLE_FRONT_RIGHT
    front_AB.update_target(RIGHT, true, 5000000, AB_POS_1);
    front_HIP.update_target(RIGHT, true, 5000000, HIP_POS_1);
    front_KNEE.update_target(RIGHT, true, 5000000, KNEE_POS_1);
#endif
#ifdef ENABLE_BACK_LEFT
    back_AB.update_target(LEFT, true, 5000000, AB_POS_2);
    back_HIP.update_target(LEFT, true, 5000000, -HIP_POS_2);
    back_KNEE.update_target(LEFT, true, 5000000, -KNEE_POS_2);
#endif
#ifdef ENABLE_BACK_RIGHT
    back_AB.update_target(RIGHT, true, 5000000, -AB_POS_1);
    back_HIP.update_target(RIGHT, true, 5000000, HIP_POS_1);
    back_KNEE.update_target(RIGHT, true, 5000000, KNEE_POS_1);
#endif
    Serial.println("POS2");
    timer.reset();
  }
  else if (serial_input == "pos3")
  {
    loop_state = STATE_test;
#ifdef ENABLE_FRONT_LEFT
    front_AB.update_target(LEFT, true, 5000000, -pos3_ab);
    front_HIP.update_target(LEFT, true, 5000000, -pos3_hip);
    front_KNEE.update_target(LEFT, true, 5000000, -pos3_knee);
#endif
#ifdef ENABLE_FRONT_RIGHT
    front_AB.update_target(RIGHT, true, 5000000, pos4_ab);
    front_HIP.update_target(RIGHT, true, 5000000, pos4_hip);
    front_KNEE.update_target(RIGHT, true, 5000000, pos4_knee);
#endif
#ifdef ENABLE_BACK_LEFT
    back_AB.update_target(LEFT, true, 5000000, pos3_ab);
    back_HIP.update_target(LEFT, true, 5000000, -pos3_hip);
    back_KNEE.update_target(LEFT, true, 5000000, -pos3_knee);
#endif
#ifdef ENABLE_BACK_RIGHT
    back_AB.update_target(RIGHT, true, 5000000, -pos4_ab);
    back_HIP.update_target(RIGHT, true, 5000000, pos4_hip);
    back_KNEE.update_target(RIGHT, true, 5000000, pos4_knee);
#endif
    timer.reset();
  }
  else if (serial_input == "pos4")
  {
    loop_state = STATE_test;
#ifdef ENABLE_FRONT_LEFT
    front_AB.update_target(LEFT, true, 5000000, -pos4_ab);
    front_HIP.update_target(LEFT, true, 5000000, -pos4_hip);
    front_KNEE.update_target(LEFT, true, 5000000, -pos4_knee);
#endif
#ifdef ENABLE_FRONT_RIGHT
    front_AB.update_target(RIGHT, true, 5000000, pos3_ab);
    front_HIP.update_target(RIGHT, true, 5000000, pos3_hip);
    front_KNEE.update_target(RIGHT, true, 5000000, pos3_knee);
#endif
#ifdef ENABLE_BACK_LEFT
    back_AB.update_target(LEFT, true, 5000000, pos4_ab);
    back_HIP.update_target(LEFT, true, 5000000, -pos4_hip);
    back_KNEE.update_target(LEFT, true, 5000000, -pos4_knee);
#endif
#ifdef ENABLE_BACK_RIGHT
    back_AB.update_target(RIGHT, true, 5000000, -pos3_ab);
    back_HIP.update_target(RIGHT, true, 5000000, pos3_hip);
    back_KNEE.update_target(RIGHT, true, 5000000, pos3_knee);
#endif
    timer.reset();
  }
  else if (serial_input == "pos5")
  {
    loop_state = STATE_test;
#ifdef ENABLE_FRONT_LEFT
    front_AB.update_target(LEFT, true, 5000000, -pos5_ab_FL);
    front_HIP.update_target(LEFT, true, 5000000, -pos5_hip_FL);
    front_KNEE.update_target(LEFT, true, 5000000, -pos5_knee_FL);
#endif
#ifdef ENABLE_FRONT_RIGHT
    front_AB.update_target(RIGHT, true, 5000000, pos5_ab_FR);
    front_HIP.update_target(RIGHT, true, 5000000, pos5_hip_FR);
    front_KNEE.update_target(RIGHT, true, 5000000, pos5_knee_FR);
#endif
#ifdef ENABLE_BACK_LEFT
    back_AB.update_target(LEFT, true, 5000000, pos5_ab_bL);
    back_HIP.update_target(LEFT, true, 5000000, -pos5_hip_bL);
    back_KNEE.update_target(LEFT, true, 5000000, -pos5_knee_bL);
#endif
#ifdef ENABLE_BACK_RIGHT
    back_AB.update_target(RIGHT, true, 5000000, -pos5_ab_bR);
    back_HIP.update_target(RIGHT, true, 5000000, pos5_hip_bR);
    back_KNEE.update_target(RIGHT, true, 5000000, pos5_knee_bR);
#endif
    timer.reset();
  }
  else if (serial_input == "pos6")
  {
    loop_state = STATE_test;
#ifdef ENABLE_FRONT_LEFT
    front_AB.update_target(LEFT, true, 5000000, -pos6_ab_FL);
    front_HIP.update_target(LEFT, true, 5000000, -pos6_hip_FL);
    front_KNEE.update_target(LEFT, true, 5000000, -pos6_knee_FL);
#endif
#ifdef ENABLE_FRONT_RIGHT
    front_AB.update_target(RIGHT, true, 5000000, pos6_ab_FR);
    front_HIP.update_target(RIGHT, true, 5000000, pos6_hip_FR);
    front_KNEE.update_target(RIGHT, true, 5000000, pos6_knee_FR);
#endif
#ifdef ENABLE_BACK_LEFT
    back_AB.update_target(LEFT, true, 5000000, pos6_ab_bL);
    back_HIP.update_target(LEFT, true, 5000000, -pos6_hip_bL);
    back_KNEE.update_target(LEFT, true, 5000000, -pos6_knee_bL);
#endif
#ifdef ENABLE_BACK_RIGHT
    back_AB.update_target(RIGHT, true, 5000000, -pos6_ab_bR);
    back_HIP.update_target(RIGHT, true, 5000000, pos6_hip_bR);
    back_KNEE.update_target(RIGHT, true, 5000000, pos6_knee_bR);
#endif
    timer.reset();
  }
  else if (serial_input == "time")
  {
    Serial.println("checking....");
    legtimertest.reset();
    timer.reset();
    loop_state = STATE_TIME_TEST;
    timerrr = millis();
  }
  else if (serial_input == "cp1")
  {

  }
  // state machine
  switch (loop_state)
  {
    case STATE_test:
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
    case STATE_TIME_TEST:
    {
      if (timer.check())
      {
      Serial.println(static_cast<float>(millis() - timerrr)/1000, 4);
      Serial.print("timer: ");
      Serial.println(legtimertest.get_time(), 4);
      Serial.println("---------");
      }
    }
    break;
    default:
      break;
  }
}
