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
struct predefined_cp
{
  float ab_pos[57];
  float ab_vel[56];
  float hip_pos[57];
  float hip_vel[56];
  float knee_pos[57];
  float knee_vel[56];
  float time[51];
};
struct predefined_cp cp_1 = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.695828411290931, 0.697869227617462, 0.701950860270523, 0.708073309250116, 0.715852112565038, 0.726056563863813,
     0.737443195846243, 0.750929482377992, 0.763867112990799, 0.776643418429691, 0.789049593492080, 0.801173577558574,
     0.812985911697761, 0.824442517904772, 0.835651792510401, 0.846370227730575, 0.857047562975957, 0.866819142639193,
     0.877286181162126, 0.885436325369085, 0.896889232777751, 0.901052951067348, 0.918304720281233, 0.905631550494400,
     0.979779023045553, 1.03888924160227, 1.23426567236769, 1.08365927668478, 0.900053662188299, 0.658549545348478,
     0.429579003625227, 0.302704959519052, 0.369968440253966, 0.366883800273621, 0.378201565540077, 0.402982964692283,
     0.417460137120555, 0.437500395248435, 0.454220388996230, 0.472419734138677, 0.489519051983478, 0.506896297395742,
     0.523806854900757, 0.540647345475319, 0.557192788691992, 0.573569627923553, 0.589680073410896, 0.605612747590257,
     0.621231469062051, 0.636753077253562, 0.651769697586045, 0.664844052419730, 0.675039228615584, 0.683583513331747,
     0.689705962311339, 0.693787594964400, 0.695828411290931},
    {0.446428571428577, 0.446428571428613, 0.446428571428658, 0.425403306284784, 0.446444744321416, 0.415137624359429, 0.421446454117153,
     0.404300956650203, 0.399259544965384, 0.387692970699658, 0.378874502077943, 0.369135441849577, 0.358018943969113, 0.350289831425887,
     0.334951100630452, 0.333666726418186, 0.305361864476109, 0.327094953841659, 0.254692006467487, 0.357903356520809, 0.130116196549897,
     0.539117787933908, -0.396036555838544, 2.31710851722355, 1.84719432989728, 6.10551346141963, -4.70644986509111, -5.73767545301501,
     -7.54700365124439, -7.15532942885161, -3.96481387831796, 2.10198377296606, -0.0963949993857804, 0.353680164576741, 0.774418723506464,
     0.452411638383492, 0.626258066496230, 0.522499804618600, 0.568729535701477, 0.534353682650025, 0.543038919133270, 0.528454922031695,
     0.526265330455085, 0.517045100521016, 0.511776225986294, 0.503451421479447, 0.497896068105030, 0.488085045993580, 0.485050255984720,
     0.469269385390078, 0.476669186644774, 0.446038958568601, 0.467265570415174, 0.446428571428593, 0.446428571428589, 0.446428571428576},
    {1.34883464940023, 1.34883464940023, 1.34883464940023, 1.34883464940023, 1.34810134845467, 1.34811022701445, 1.34649341121750,
     1.34502726041864, 1.34255153693923, 1.33982795923347, 1.33644852023482, 1.33262796359585, 1.32825258248514, 1.32337435640052,
     1.31797437039684, 1.31203720330831, 1.30560861555345, 1.29858234642422, 1.29115505202017, 1.28294315344867, 1.27467297736397,
     1.26481685605132, 1.25749409191898, 1.23461098437371, 1.30850629956077, 1.30118302653888, 1.60934352508590, 1.68188104858737,
     1.73724010654212, 1.68188104858737, 1.60934352508590, 1.30118302653888, 1.30850629956077, 1.23461098437371, 1.25749409191898,
     1.26481685605132, 1.27467297736397, 1.28294315344867, 1.29115505202017, 1.29858234642422, 1.30560861555345, 1.31203720330831,
     1.31797437039684, 1.32337435640053, 1.32825258248514, 1.33262796359585, 1.33644852023482, 1.33982795923347, 1.34255153693924,
     1.34502726041863, 1.34649341121751, 1.34811022701443, 1.34810134845469, 1.34883464940023, 1.34883464940023, 1.34883464940023,
     1.34883464940023},
    {0.0, 0.0, 0.0, -0.0401023954602706, 0.000388436990066132, -0.0589464092635855, -0.0458172124645911, -0.0773663587313109,
     -0.0851118033050671, -0.105607468707747, -0.119392394967999, -0.136730659709691, -0.152444565144190, -0.168749562615103,
     -0.185536471516617, -0.200893367339285, -0.219570910288472, -0.232102950126455, -0.256621830359356, -0.258443002647157,
     -0.308003791020181, -0.228836379135698, -0.715097110789438, 2.30922859959540, -0.228852281933915, 9.63001557959433,
     2.26679760942091, 1.72997056108586, -1.72997056108586, -2.26679760942091, -9.63001557959434, 0.228852281933950,
     -2.30922859959545, 0.715097110789473, 0.228836379135663, 0.308003791020230, 0.258443002647109, 0.256621830359370,
     0.232102950126538, 0.219570910288312, 0.200893367339430, 0.185536471516506, 0.168749562615235, 0.152444565144037,
     0.136730659709802, 0.119392394967971, 0.105607468707677, 0.0851118033052267, 0.0773663587310056, 0.0458172124651046,
     0.0589464092627597, -0.000388436988997542, 0.0401023954593720, 0.0, 0.0, 0.0},
    {0.0, 0.0320, 0.0640, 0.0960, 0.128, 0.160, 0.192, 0.224, 0.256, 0.288, 0.320, 0.352, 0.384, 0.416, 0.448, 0.480, 0.512, 0.544, 0.576,
     0.608, 0.640, 0.672, 0.704, 0.736, 0.768, 0.800, 0.832, 0.864, 0.896, 0.928, 0.960, 0.992, 1.024, 1.056, 1.088, 1.12, 1.152, 1.184, 1.216,
     1.248, 1.28, 1.312, 1.344, 1.376, 1.408, 1.44, 1.472, 1.504, 1.536, 1.568, 1.60}};

// Initialize bSpline object
bSpline traj_1_ab_pos(cp_1.ab_pos, cp_1.time, 51);
bSpline traj_1_ab_vel(cp_1.ab_vel, cp_1.time, 51);
bSpline traj_1_hip_pos(cp_1.hip_pos, cp_1.time, 51);
bSpline traj_1_hip_vel(cp_1.hip_vel, cp_1.time, 51);
bSpline traj_1_knee_pos(cp_1.knee_pos, cp_1.time, 51);
bSpline traj_1_knee_vel(cp_1.knee_vel, cp_1.time, 51);

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
  Serial.println("yes");
  unsigned long sdf = 0;
//  while (Serial.available()==0)
//  {
//    Serial.println(sdf);
//    sdf++;
//  }
    ; // wait for Arduino Serial Monitor to open
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
