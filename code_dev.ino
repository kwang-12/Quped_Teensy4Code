// Libraries:
#include "src/lib/globals.h"
#include "src/lib/ODriveArduino.h"
#include "src/lib/jointPositions.h"
#include "src/lib/bSpline.h"
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
Metro timer = Metro(5);

// Initialize trajectory ticker
int tick_fL = 0;
int tick_fR = 0;
int tick_bL = 0;
int tick_bR = 0;

// Initialize control positions
struct cp_1{
  float ab_pos[23] = {
    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0};
  float ab_vel[22] = {
    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
    0.0,0.0};
  float hip_pos[23] = {
    0.695828411290931,0.710114125576645,0.738685554148074,0.776962478967295,0.827152316651153,
    0.869562720354763,0.908402735322119,0.923215614040563,1.05555765214887,1.25554843780019,
    1.07940031135145,0.913295921962569,0.684610016441594,0.447931665200037,0.340028544922051,
    0.336167105336494,0.409113820820484,0.475149071905042,0.538740476445783,0.606583290638416,
    0.652971268433788,0.681542697005217,0.695828411290931};
  float hip_vel[22] = {
    0.446428571428570,0.446428571428572,0.398717966866883,0.392108106905145,0.315553598985191,
    0.356985431685255,0.178039407673613,2.29760482826924,6.24971205160361,-5.50462895152300,
    -5.19076216840257,-7.14643454753045,-7.39619847629865,-3.37197250868707,-0.0670388816936952,
    0.876763407259488,0.606941645997781,0.473150331404317,0.530021985879946,0.483208102035126,
    0.446428571428575,0.446428571428573};
  float knee_pos[23] = {
    1.34883464940023,1.34883464940023,1.34883464940023,1.34092511661378,1.32475585330151,
    1.30279731457381,1.27890634121836,1.22139644000072,1.35307177158772,1.64642431633239,
    1.70265459835609,1.76191049428161,1.70265459835609,1.64642431633239,1.35307177158772,
    1.22139644000072,1.27890634121836,1.30279731457381,1.32475585330151,1.34092511661378,
    1.34883464940023,1.34883464940023,1.34883464940023};
  float knee_vel[22] = {
    0.0,0.0,-0.0823909665255210,-0.126322369627074,-0.163381984581145,-0.219586152164022,
    -0.691224774250535,2.28603006227436,9.16726702327093,1.75719631324067,1.85174674767244,
    -1.85174674767244,-1.75719631324066,-9.16726702327097,-2.28603006227431,0.691224774250477,
    0.219586152164075,0.163381984581116,0.126322369627086,0.0823909665255187,0.0,0.0};
  float time[19] = {
    0.0,0.16,0.32,0.48,0.64,0.672,0.704,0.736,0.768,0.80,0.832,
    0.864,0.896,0.928,0.96,1.12,1.28,1.44,1.6};
}cp_1;

// Initialize bSpline object
bSpline traj_1_ab_pos(cp_1.ab_pos, cp_1.time, 19, 5);
bSpline traj_1_ab_vel(cp_1.ab_vel, cp_1.time, 19, 4);
bSpline traj_1_hip_pos(cp_1.hip_pos, cp_1.time, 19, 5);
bSpline traj_1_hip_vel(cp_1.hip_vel, cp_1.time, 19, 4);
bSpline traj_1_knee_pos(cp_1.knee_pos, cp_1.time, 19, 5);
bSpline traj_1_knee_vel(cp_1.knee_vel, cp_1.time, 19, 4);

// DEBUG variable
bool break_out_flag = false;
String read_axis0_error = "r axis0.error";
String read_axis1_error = "r axis1.error";

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
 * @param mode - trot selection
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
 * 
 */

/**
 * Update time stamp
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
    calib_input = Serial.readString();
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

void setup()
{
  // Serial to PC @ 921600 baud rate
  Serial.begin(1000000);
  while(!Serial);
  Serial.println("Computer Serial connected");
#ifdef ENABLE_FRONT
  // odrv_connect(front_AB);
  // odrv_connect(front_HIP);
  // odrv_connect(front_KNEE);
#endif
#ifdef ENABLE_BACK
  odrv_connect(back_AB);
  odrv_connect(back_HIP);
  odrv_connect(back_KNEE);
#endif

float tim = 0.0;
while (true)
{
    // Serial.print(tim,4);
    // Serial.print('|');
    Serial.print(traj_1_ab_pos.get_point(&tim),4);
    Serial.print(' ');
//    Serial.print(traj_1_ab_vel.get_point(&tim),4);
//    Serial.print(' ');
    Serial.print(traj_1_hip_pos.get_point(&tim),4);
    Serial.print(' ');
//    Serial.print(traj_1_hip_vel.get_point(&tim),4);
//    Serial.print(' ');
    Serial.println(traj_1_knee_pos.get_point(&tim),4);
//    Serial.print(' ');
//    Serial.println(traj_1_knee_vel.get_point(&tim),4);
    tim = tim + 0.005;
  if (tim > 1.6)
  {
    break;
  }
}

//   // radio.calibration();
//   while (true)
//   {
//     Serial.println("=========================================");
//     Serial.println("Enter readError to read axes error code");
//     Serial.println("Enter calib to initiate joint calibration sequence");
//     Serial.println("Enter find to calibrate neutral positions");
//     Serial.println("-----------------------------------------");
//     Serial.println("Enter stdby to proceed to STANDBY pos");
//     Serial.println("Enter traj1 to track trajectory-1");
//     Serial.println("Enter traj2 to track trajectory-2");
//     Serial.println("Enter traj3 to track trajectory-3");
//     Serial.println("Enter dis to disarm all motors");
//     Serial.println("Enter deg to view joint positions");
//     Serial.println("Enter test to test time for bspline calculation");
//     Serial.println("Enter manual to enter manual command mode");
//     while (Serial.available() == 0)
//       ;
//     String serial_input = Serial.readString();
//     if (serial_input == "stdby")
//     {
// // Arm all axes
// #ifdef ENABLE_FRONT
//       front_AB.armAxis();
//       front_HIP.armAxis();
//       front_KNEE.armAxis();
// #endif
// #ifdef ENABLE_BACK
//       back_AB.armAxis();
//       back_HIP.armAxis();
//       back_KNEE.armAxis();
// #endif
//       timer.reset();
//       /**
//      * Start posture conversion until the posture is reached or emergency break from the controller input
//     */
//       unsigned long int stdby_timer = millis();
//       while (!joint.checkPos(STANDBY_POS_FLAG, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr) && millis() - stdby_timer < 4000)
//       {
//         unsigned long int standby_pos_start_time = millis();
//         float elapsedSec = (millis() - standby_pos_start_time) / 1000.0;
//         while (elapsedSec < POS_CONVERSION_TIME)
//         {
//           if (Serial.available() > 0)
//           {
//             Serial.println("stopping pos conversion!");
//             front_AB.disarmAxis();
//             front_HIP.disarmAxis();
//             front_KNEE.disarmAxis();
//             break_out_flag = true;
//             break;
//           }
//           joint.update_constantPos(elapsedSec);
//           // Serial.println(elapsedSec);
//           if (timer.check())
//           {
//             send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           }
//           elapsedSec = (millis() - standby_pos_start_time) / 1000.0;
//         }
//       }
//     }
//     else if (serial_input == "traj1")
//     {
//       Serial.println("delaying...");
//       delay(3000);
//       Serial.println("starting...");
//       int loop_counter = 0;
//       reset_tick();
//       //GAIT_STARTING
//       timer.reset();
//       while (tick_fL <= 99)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break_out_flag = true;
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 1);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_STARTING);
//         }
//       }
//       //GAIT_NORMAL
//       while (loop_counter < 5)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break_out_flag = true;
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 1);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_NORMAL);
//         }
//         if (tick_fL == 200)
//         {
//           loop_counter++;
//           tick_fL = 0;
//           tick_fR = 100;
//           tick_bL = 100;
//           tick_bR = 0;
//         }
//       }
//       //GAIT_ENDING
//       while (tick_fR < 200)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 1);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_ENDING);
//         }
//       }
//       while (tick_fR == 200)
//       {
//         if (timer.check())
//         {
//           reset_tick();
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 1);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//         }
//       }
//       timer.reset();
//       break_out_flag = false;
//     }

//     else if (serial_input == "traj2")
//     {
//       Serial.println("delaying...");
//       delay(3000);
//       Serial.println("starting...");
//       int loop_counter = 0;
//       reset_tick();
//       //GAIT_STARTING
//       timer.reset();
//       while (tick_fL <= 99)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break_out_flag = true;
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 2);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_STARTING);
//         }
//       }
//       //GAIT_NORMAL
//       while (loop_counter < 5)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break_out_flag = true;
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 2);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_NORMAL);
//         }
//         if (tick_fL == 200)
//         {
//           loop_counter++;
//           tick_fL = 0;
//           tick_fR = 100;
//           tick_bL = 100;
//           tick_bR = 0;
//         }
//       }
//       //GAIT_ENDING
//       while (tick_fR < 200)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 2);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_ENDING);
//         }
//       }
//       while (tick_fR == 200)
//       {
//         if (timer.check())
//         {
//           reset_tick();
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 2);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//         }
//       }
//       timer.reset();
//       break_out_flag = false;
//     }

//     else if (serial_input == "traj3")
//     {
//       Serial.println("delaying...");
//       delay(3000);
//       Serial.println("starting...");
//       int loop_counter = 0;
//       reset_tick();
//       //GAIT_STARTING
//       timer.reset();
//       while (tick_fL <= 99)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break_out_flag = true;
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 3);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_STARTING);
//         }
//       }
//       //GAIT_NORMAL
//       while (loop_counter < 5)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break_out_flag = true;
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 3);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_NORMAL);
//         }
//         if (tick_fL == 200)
//         {
//           loop_counter++;
//           tick_fL = 0;
//           tick_fR = 100;
//           tick_bL = 100;
//           tick_bR = 0;
//         }
//       }
//       //GAIT_ENDING
//       while (tick_fR < 200)
//       {
//         if (Serial.available() > 0)
//         {
//           Serial.println("stopping traj trace!");
//           front_AB.disarmAxis();
//           front_HIP.disarmAxis();
//           front_KNEE.disarmAxis();
//           break;
//         }
//         if (break_out_flag == true)
//         {
//           break;
//         }
//         if (timer.check())
//         {
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 3);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//           update_tick(TROT_GAIT, GAIT_ENDING);
//         }
//       }
//       while (tick_fR == 200)
//       {
//         if (timer.check())
//         {
//           reset_tick();
//           joint.update_traceTraj(tick_fL, tick_fR, tick_bL, tick_bR, 3);
//           send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//         }
//       }
//       timer.reset();
//       break_out_flag = false;
//     }
//     else if (serial_input == "deg")
//     {
// #ifdef ENABLE_FRONT_LEFT
//       Serial.print(" F-AB-L: ");
//       Serial.print(front_AB.transPosition_num2deg(LEFT, front_AB.getAxisPos(LEFT, true)));
//       Serial.print(" F-HIP-L: ");
//       Serial.print(front_HIP.transPosition_num2deg(LEFT, front_HIP.getAxisPos(LEFT, true)));
//       Serial.print(" F-KNEE-L: ");
//       Serial.println(front_KNEE.transPosition_num2deg(LEFT, front_KNEE.getAxisPos(LEFT, true)));
// #endif
// #ifdef ENABLE_FRONT_RIGHT
//       Serial.print(" F-AB-R: ");
//       Serial.print(front_AB.transPosition_num2deg(RIGHT, front_AB.getAxisPos(RIGHT, true)));
//       Serial.print(" F-HIP-R: ");
//       Serial.print(front_HIP.transPosition_num2deg(RIGHT, front_HIP.getAxisPos(RIGHT, true)));
//       Serial.print(" F-KNEE-R: ");
//       Serial.println(front_KNEE.transPosition_num2deg(RIGHT, front_KNEE.getAxisPos(RIGHT, true)));
// #endif
// #ifdef ENABLE_BACK_LEFT
//       Serial.print(" B-AB-L: ");
//       Serial.print(back_AB.transPosition_num2deg(LEFT, back_AB.getAxisPos(LEFT, true)));
//       Serial.print(" B-HIP-L: ");
//       Serial.print(back_HIP.transPosition_num2deg(LEFT, back_HIP.getAxisPos(LEFT, true)));
//       Serial.print(" B-KNEE-L: ");
//       Serial.println(back_KNEE.transPosition_num2deg(LEFT, back_KNEE.getAxisPos(LEFT, true)));
// #endif
// #ifdef ENABLE_BACK_RIGHT
//       Serial.print(" B-AB-R: ");
//       Serial.print(back_AB.transPosition_num2deg(RIGHT, back_AB.getAxisPos(RIGHT, true)));
//       Serial.print(" B-HIP-R: ");
//       Serial.print(back_HIP.transPosition_num2deg(RIGHT, back_HIP.getAxisPos(RIGHT, true)));
//       Serial.print(" B-KNEE-R: ");
//       Serial.println(back_KNEE.transPosition_num2deg(RIGHT, back_KNEE.getAxisPos(RIGHT, true)));
// #endif
//     }
//     else if (serial_input == "dis")
//     {
// #ifdef ENABLE_FRONT
//       front_AB.disarmAxis();
//       front_HIP.disarmAxis();
//       front_KNEE.disarmAxis();
// #endif
// #ifdef ENABLE_BACK
//       back_AB.disarmAxis();
//       back_HIP.disarmAxis();
//       back_KNEE.disarmAxis();
// #endif
//     }
//     else if (serial_input == "find")
//     {
//       find_joint_neutral_position();
//     }
//     else if (serial_input == "readError")
//     {
//       readError();
//     }
//     else if (serial_input == "calib")
//     {
//       calibJoints();
//     }
//     else if (serial_input == "test")
//     {
//       float time_point = 0.1;
//       Serial.println(traj_1_ab_pos.get_point(&time_point));
//     }
//     else if (serial_input == "manual")
//     {
//       bool exit_flag = false;
//       while (exit_flag == false)
//       {
//         //serial com shit
//       }
//     }
//   }
}

void loop()
{
  // //changes in radio signal is big enough
  // if (radio.meaningful())
  // {
  //   controlPoint.update(radio);
  //   bSpline.update(controlPoint);
  // }
  // else if (radio.getState() == SHUTDOWN)
  // {
  //   front_AB.disarmAxis();
  //   front_HIP.disarmAxis();
  //   front_KNEE.disarmAxis();
  //   back_AB.disarmAxis();
  //   back_HIP.disarmAxis();
  //   back_KNEE.disarmAxis();
  // }
  // // if signal send time step has reached and the radio's input state does not dictate STANDBY posture
  // if (Timer.check() && radio.getState() != STANDBY)
  // {
  //   jointPositions.update(mode=bSpline, bSpline, time);
  //   messenger.send(jointPositions);
  // }
  // // if signal send time step has reached and the radio's input state dictates STANDBY posture
  // else if (Timer.check() && radio.getState() == STANDBY)
  // {
  //   jointPositions.update(mode=constantPos, STANDBY_POS, time);
  //   messenger.send(jointPositions);
  // }
}
