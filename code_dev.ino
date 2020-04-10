// Libraries:
#include "src/lib/globals.h"
#include "src/lib/ODriveArduino.h"
#include "src/lib/jointPositions.h"
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

void send(jointPositions *joint_ptr, ODriveArduino *front_AB_ptr, ODriveArduino *front_HIP_ptr, ODriveArduino *front_KNEE_ptr,
          ODriveArduino *back_AB_ptr, ODriveArduino *back_HIP_ptr, ODriveArduino *back_KNEE_ptr)
{
#ifdef ENABLE_FRONT_RIGHT
  front_AB_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.fR_ab_pos, joint_ptr->joint_vel_target_deg.fR_ab_velo);
  front_HIP_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.fR_hip_pos, joint_ptr->joint_vel_target_deg.fR_hip_velo);
  front_KNEE_ptr->SetPosition(RIGHT, joint_ptr->joint_pos_target_deg.fR_knee_pos, joint_ptr->joint_vel_target_deg.fR_knee_velo);
#endif
#ifdef ENABLE_FRONT_LEFT
  front_AB_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.fL_ab_pos, joint_ptr->joint_vel_target_deg.fL_ab_velo);
  front_HIP_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.fL_hip_pos, joint_ptr->joint_vel_target_deg.fL_hip_velo);
  front_KNEE_ptr->SetPosition(LEFT, joint_ptr->joint_pos_target_deg.fL_knee_pos, joint_ptr->joint_vel_target_deg.fL_knee_velo);
  Serial.print("ab-pos: ");
  Serial.print(joint_ptr->joint_pos_target_deg.fL_ab_pos);
  Serial.print(" ab-velo: ");
  Serial.print(joint_ptr->joint_vel_target_deg.fL_ab_velo);
  Serial.print(" hip-pos: ");
  Serial.print(joint_ptr->joint_pos_target_deg.fL_hip_pos);
  Serial.print(" hip-velo: ");
  Serial.print(joint_ptr->joint_vel_target_deg.fL_hip_velo);
  Serial.print(" knee-pos: ");
  Serial.print(joint_ptr->joint_pos_target_deg.fL_knee_pos);
  Serial.print(" knee-velo: ");
  Serial.println(joint_ptr->joint_vel_target_deg.fL_knee_velo);
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

void setup()
{
  // Serial to PC @ 921600 baud rate
  Serial.begin(1000000);
  while (!Serial)
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

  find_joint_neutral_position();

  // radio.calibration();
  while (true)
  {
    Serial.println("Enter stdby to proceed to STANDBY pos");
    Serial.println("Enter traj to track trajectory");
    Serial.println("Enter deg to view joint positions");
    Serial.println("Enter find to calibrate neutral positions");
    Serial.println("Enter manual to enter manual command mode");
    while (Serial.available() == 0)
      ;
    String serial_input = Serial.readString();
    if (serial_input == "stdby")
    {
// Arm all axes
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
      timer.reset();
      /**
     * Start posture conversion until the posture is reached or emergency break from the controller input
    */
      unsigned long int stdby_timer = millis();
      while (!joint.checkPos(STANDBY_POS_FLAG, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr) && millis() - stdby_timer < 4000)
      {
        unsigned long int standby_pos_start_time = millis();
        float elapsedSec = (millis() - standby_pos_start_time) / 1000.0;
        while (elapsedSec < POS_CONVERSION_TIME)
        {
        if (Serial.available() > 0)
        {
          Serial.println("stopping pos conversion!");
      front_AB.disarmAxis();
      front_HIP.disarmAxis();
      front_KNEE.disarmAxis();
          break;
        }
          joint.update_constantPos(elapsedSec);
          // Serial.println(elapsedSec);
          if (timer.check())
          {
            send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
          }
          elapsedSec = (millis() - standby_pos_start_time) / 1000.0;
        }
      }
    }
    else if (serial_input == "traj")
    {
      int loop_counter = 0;
      int tick_fL = 0;
      int tick_fR = 100;
      timer.reset();
      while (loop_counter < 5)
      {
        if (Serial.available() > 0)
        {
          Serial.println("stopping traj trace!");
      front_AB.disarmAxis();
      front_HIP.disarmAxis();
      front_KNEE.disarmAxis();
          break;
        }
        if (timer.check())
        {
          joint.update_traceTraj(tick_fL,tick_fR,1,1);
          send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
          tick_fL++;
          if (tick_fL>=0&&tick_fL<=99){
            tick_fR = tick_fL+100;
          }
          else
          {
            tick_fR = tick_fL-100;
          }
          
        }
        if (tick_fL == 200)
        {
          loop_counter++;
          tick_fL = 0;
          tick_fR = 100;
        }
      }
    }
    else if (serial_input == "deg")
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
    else if (serial_input == "dis")
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
    else if (serial_input == "find")
    {
      find_joint_neutral_position();
    }
    else if (serial_input == "manual")
    {
      bool exit_flag = false;
      while (exit_flag == false)
      {
        //serial com shit
      }
    }
  }
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
