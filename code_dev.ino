// Libraries:
#include "src/lib/globals.h"
#include "src/lib/ODriveArduino.h"
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
ODriveArduino back_HIP(Serial2, odrv2_name, HIP, 2, RIGHT, LEFT, SERIAL_BAUD_RATE);   //hip back, axis0-RB, axis1-LB
ODriveArduino back_AB(Serial3, odrv3_name, AB, 3, LEFT, RIGHT, SERIAL_BAUD_RATE);    //ab back, axis0-LB, axis1-RB
ODriveArduino front_KNEE(Serial4, odrv4_name, KNEE, 4, RIGHT, LEFT, SERIAL_BAUD_RATE); //knee front, axis0-RB, axis1-LB
ODriveArduino front_HIP(Serial5, odrv5_name, HIP, 5, LEFT, RIGHT, SERIAL_BAUD_RATE);  //hip front, axis0-LB, axis1-RB
ODriveArduino front_AB(Serial7, odrv6_name, AB ,7, LEFT, RIGHT, SERIAL_BAUD_RATE);   //ab front, axis0-LB, axis1-RB
// Constants

// Variables
String serial_input; // store the input from computer monitor port
char state = 'z';    // store the state of loop program. By default, 'z' means do nothing.
float input_pos_state_c;
float input_pos_state_d;
float input_pos_state_e;
float target_k;
float target_h;
float target_a;


// Setup variables
bool setup_state = false;   //indicate the state of finding all joint ranges
bool front_right_knee_rdy = false;   //indicate if the joint range of front right knee is found
bool front_right_hip_rdy = false;   //indicate if the joint range of front right hip is found
bool front_right_ab_rdy = false;   //indicate if the joint range of front right ab is found
bool front_left_knee_rdy = false;   //indicate if the joint range of front left knee is found
bool front_left_hip_rdy = false;   //indicate if the joint range of front left hip is found
bool front_left_ab_rdy = false;   //indicate if the joint range of front left ab is found
bool back_right_knee_rdy = false;   //indicate if the joint range of back right knee is found
bool back_right_hip_rdy = false;   //indicate if the joint range of back right hip is found
bool back_right_ab_rdy = false;   //indicate if the joint range of back right ab is found
bool back_left_knee_rdy = false;   //indicate if the joint range of back left knee is found
bool back_left_hip_rdy = false;   //indicate if the joint range of back left hip is found
bool back_left_ab_rdy = false;   //indicate if the joint range of back left ab is found



String disable_axis0 = "w axis0.requested_state 1";
String disable_axis1 = "w axis1.requested_state 1";

// Debug Variables

void setup()
{
  // Serial to PC @ 921600 baud rate
  Serial.begin(1000000);
  while (!Serial)
    ; // wait for Arduino Serial Monitor to open
  Serial.println("Computer Serial connected");

  // Serial to odrives @ 921600 baud rate
  if(back_KNEE.ini())
  {
    #ifdef DEBUG_SERIAL
      Serial.println("back KNEE serial communication activated");
    #endif
  }
  else
  {
    #ifdef DEBUG_SERIAL
      Serial.println("back KNEE serial communication failed");
    #endif
  }
  
  if(back_HIP.ini())
  {
    #ifdef DEBUG_SERIAL
      Serial.println("back HIP serial communication activated");
    #endif
  }
  else
  {
    #ifdef DEBUG_SERIAL
      Serial.println("back HIP serial communication failed");
    #endif
  }

  if(back_AB.ini())
  {
    #ifdef DEBUG_SERIAL
      Serial.println("back AB serial communication activated");
    #endif
  }
  else
  {
    #ifdef DEBUG_SERIAL
      Serial.println("back AB serial communication failed");
    #endif
  }
    if(front_KNEE.ini())
  {
    #ifdef DEBUG_SERIAL
      Serial.println("front KNEE serial communication activated");
    #endif
  }
  else
  {
    #ifdef DEBUG_SERIAL
      Serial.println("front KNEE serial communication failed");
    #endif
  }
  
  if(front_HIP.ini())
  {
    #ifdef DEBUG_SERIAL
      Serial.println("front HIP serial communication activated");
    #endif
  }
  else
  {
    #ifdef DEBUG_SERIAL
      Serial.println("front HIP serial communication failed");
    #endif
  }

  if(front_AB.ini())
  {
    #ifdef DEBUG_SERIAL
      Serial.println("front AB serial communication activated");
    #endif
  }
  else
  {
    #ifdef DEBUG_SERIAL
      Serial.println("front AB serial communication failed");
    #endif
  }
  // while (setup_state == false)
  // {

  // }
  // front_KNEE.find_joint_neutral_position('r', RIGHT);
  // front_HIP.find_joint_neutral_position('c', RIGHT);
  // front_AB.find_joint_neutral_position('r', RIGHT);
  // front_KNEE.find_joint_neutral_position('r', LEFT);
  // front_HIP.find_joint_neutral_position('c', LEFT);
  // front_AB.find_joint_neutral_position('r', LEFT);
  // back_KNEE.find_joint_neutral_position('r', RIGHT);
  // back_HIP.find_joint_neutral_position('c', RIGHT);
  back_AB.find_joint_neutral_position('r', RIGHT);
  // back_KNEE.find_joint_neutral_position('r', LEFT);
  // back_HIP.find_joint_neutral_position('c', LEFT);
  back_AB.find_joint_neutral_position('r', LEFT);
}

void loop()
{
#ifdef DEBUG_SERIAL
  if (Serial.available() > 0)
  {
    serial_input = Serial.readString();
    Serial.println(serial_input);
    if (serial_input == "test")
    {
      long int neutral_pos = back_KNEE.getAxisNeutralPos(RIGHT);
      Serial.print("current position: ");
      Serial.println(back_KNEE.getAxisPos(RIGHT, true));
      Serial.print("neutral position: ");
      Serial.println(neutral_pos);
      Serial.println("start sweeping motion...");
      state = 'a';
    }
    else if (serial_input == "degb")
    {
      state = 'b';
    }
    else if (serial_input == "degf")
    {
      state = 'B';
    }
    else if (serial_input == "gk")
    {
      state = 'c';
    }
    else if (serial_input == "gh")
    {
      state = 'd';
    }
    else if (serial_input == "ga")
    {
      state = 'e';
    }
    else if (serial_input == "knee")
    {
      state = '!';
    }
    else if (serial_input == "hip")
    {
      state = '@';
    }
    else if (serial_input == "ab")
    {
      state = '#';
    }
    else if (serial_input == "disarm")
    {
      back_KNEE.disarmAxis();
      back_HIP.disarmAxis();
      back_AB.disarmAxis();
      // front_KNEE.disarmAxis();
      // front_HIP.disarmAxis();
      // front_AB.disarmAxis();
    }
    else if (serial_input == "bab")
    {
      Serial.println("send to back-ab...");
      while(Serial.available()==0);
      serial_input = Serial.readString();
      back_AB.EnterCommand(serial_input);
      Serial.println(back_AB.readString());
    }
    else if (serial_input == "bhip")
    {
      Serial.println("send to back-hip...");
      while(Serial.available()==0);
      serial_input = Serial.readString();
      back_HIP.EnterCommand(serial_input);
      Serial.println(back_HIP.readString());
    }
    else if (serial_input == "bknee")
    {
      Serial.println("send to back-knee...");
      while(Serial.available()==0);
      serial_input = Serial.readString();
      back_KNEE.EnterCommand(serial_input);
      Serial.println(back_KNEE.readString());
    }
    else if (serial_input == "fab")
    {
      Serial.println("send to front-ab...");
      while(Serial.available()==0);
      serial_input = Serial.readString();
      front_AB.EnterCommand(serial_input);
      Serial.println(front_AB.readString());
    }
    else if (serial_input == "fhip")
    {
      Serial.println("send to front-hip...");
      while(Serial.available()==0);
      serial_input = Serial.readString();
      front_HIP.EnterCommand(serial_input);
      Serial.println(front_HIP.readString());
    }
    else if (serial_input == "fknee")
    {
      Serial.println("send to front-knee...");
      while(Serial.available()==0);
      serial_input = Serial.readString();
      front_KNEE.EnterCommand(serial_input);
      Serial.println(front_KNEE.readString());
    }
    else
    {
      back_AB.EnterCommand(serial_input);
      Serial.println(back_AB.readString());
    }
  }
    if (state == 'a' )//&& back_KNEE.armAxis(RIGHT))
    {
      bool while_loop_state = true;

      const int max_step_per_cycle = 200;
      int step_counter = 0;
      unsigned long int step_time_interval = 5;
      int run_sec = 10;

      float ini_pos = back_KNEE.transPosition_num2deg(RIGHT, back_KNEE.getAxisPos(RIGHT,true));
      float vel_per_sec = 1.0;
      float vel_per_step = vel_per_sec/(float)max_step_per_cycle;

      int start_time = millis();
      back_KNEE.armAxis(RIGHT);
      back_KNEE.iniTimer(RIGHT, step_time_interval);
      while(while_loop_state == true)
      {
        if (step_counter == max_step_per_cycle*run_sec)
        {
          while_loop_state = false;
        }
        if (back_KNEE.checkTimer(RIGHT))
        {
          back_KNEE.SetPosition(RIGHT, (float)step_counter*vel_per_step+ini_pos, vel_per_step);
          Serial.println((float)step_counter*vel_per_step+ini_pos);
          Serial.println(millis()-start_time);
          step_counter++;
        }
      }
      back_KNEE.disarmAxis(RIGHT);
      state = 'z';
    }
    else if (state == 'b')
    {
      Serial.println("------");
      Serial.print("bkR");
      Serial.print(back_KNEE.transPosition_num2deg(RIGHT,back_KNEE.getAxisPos(RIGHT,true)));
      Serial.print('\t');
      Serial.print("bhR");
      Serial.print(back_HIP.transPosition_num2deg(RIGHT,back_HIP.getAxisPos(RIGHT,true)));
      Serial.print('\t');
      Serial.print("baR");
      Serial.print(back_AB.transPosition_num2deg(RIGHT,back_AB.getAxisPos(RIGHT,true)));
      Serial.print('\t');
      Serial.print("bkL");
      Serial.print(back_KNEE.transPosition_num2deg(LEFT,back_KNEE.getAxisPos(LEFT,true)));
      Serial.print('\t');
      Serial.print("bhL");
      Serial.print(back_HIP.transPosition_num2deg(LEFT,back_HIP.getAxisPos(LEFT,true)));
      Serial.print('\t');
      Serial.print("baL");
      Serial.println(back_AB.transPosition_num2deg(LEFT,back_AB.getAxisPos(LEFT,true)));
    }
    else if (state == 'B')
    {
      Serial.print("fkR");
      Serial.print(front_KNEE.transPosition_num2deg(RIGHT,front_KNEE.getAxisPos(RIGHT,true)));
      Serial.print('\t');
      Serial.print("fhR");
      Serial.print(front_HIP.transPosition_num2deg(RIGHT,front_HIP.getAxisPos(RIGHT,true)));
      Serial.print('\t');
      Serial.print("faR");
      Serial.print(front_AB.transPosition_num2deg(RIGHT,front_AB.getAxisPos(RIGHT,true)));
      Serial.print('\t');
      Serial.print("fkL");
      Serial.print(front_KNEE.transPosition_num2deg(LEFT,front_KNEE.getAxisPos(LEFT,true)));
      Serial.print('\t');
      Serial.print("fhL");
      Serial.print(front_HIP.transPosition_num2deg(LEFT,front_HIP.getAxisPos(LEFT,true)));
      Serial.print('\t');
      Serial.print("faL");
      Serial.println(front_AB.transPosition_num2deg(LEFT,front_AB.getAxisPos(LEFT,true)));
      // state = 'z';
    }
    else if (state == 'c')
    {
      // Serial.print("Current position:");
      // Serial.println(back_KNEE.transPosition_num2deg(RIGHT, back_KNEE.getAxisPos(RIGHT,true)));
      // Serial.println("Enter target position.");
      // while(Serial.available()==0);
      // input_pos_state_c = Serial.readString().toInt();
      // Serial.print("Target position: ");
      // Serial.println(input_pos_state_c);
      // Serial.println("Enter y to confirm. Enter anything else to redo.");
      // while(Serial.available()==0);
      // String input_state = Serial.readString();
      // while(input_state != 'y')
      // {
      //   Serial.print("Current position:");
      //   Serial.println(back_KNEE.transPosition_num2deg(RIGHT, back_KNEE.getAxisPos(RIGHT,true)));
      //   Serial.println("Enter target position.");
      //   while(Serial.available()==0);
      //   input_pos_state_c = Serial.readString().toInt();
      //   Serial.print("Target position: ");
      //   Serial.println(input_pos_state_c);
      //   Serial.println("Enter y to confirm. Enter anything else to redo.");
      //   while(Serial.available()==0);
      //   input_state = Serial.readString();
      // }
      // state = '1';
      // back_KNEE.armAxis(RIGHT);

      
      Serial.print("Current position:");
      Serial.println(back_KNEE.transPosition_num2deg(LEFT, back_KNEE.getAxisPos(LEFT,true)));
      Serial.println("Enter target position.");
      while(Serial.available()==0);
      input_pos_state_c = Serial.readString().toInt();
      Serial.print("Target position: ");
      Serial.println(input_pos_state_c);
      Serial.println("Enter y to confirm. Enter anything else to redo.");
      while(Serial.available()==0);
      String input_state = Serial.readString();
      while(input_state != 'y')
      {
        Serial.print("Current position:");
        Serial.println(back_KNEE.transPosition_num2deg(LEFT, back_KNEE.getAxisPos(LEFT,true)));
        Serial.println("Enter target position.");
        while(Serial.available()==0);
        input_pos_state_c = Serial.readString().toInt();
        Serial.print("Target position: ");
        Serial.println(input_pos_state_c);
        Serial.println("Enter y to confirm. Enter anything else to redo.");
        while(Serial.available()==0);
        input_state = Serial.readString();
      }
      state = '1';
      back_KNEE.armAxis(LEFT);
    }
    else if (state == '1')
    {
      if(!back_KNEE.moveTo_constVelo(LEFT, input_pos_state_c, 5.0))
      {

      }
      else
      {
        state = 'z';
      }
    }
    else if (state == 'd')
    {
      // Serial.print("Current position:");
      // Serial.println(back_HIP.transPosition_num2deg(RIGHT, back_HIP.getAxisPos(RIGHT,true)));
      // Serial.println("Enter target position.");
      // while(Serial.available()==0);
      // input_pos_state_d = Serial.readString().toInt();
      // Serial.print("Target position: ");
      // Serial.println(input_pos_state_d);
      // Serial.println("Enter y to confirm. Enter anything else to redo.");
      // while(Serial.available()==0);
      // String input_state = Serial.readString();
      // while(input_state != 'y')
      // {
      //   Serial.print("Current position:");
      //   Serial.println(back_HIP.transPosition_num2deg(RIGHT, back_HIP.getAxisPos(RIGHT,true)));
      //   Serial.println("Enter target position.");
      //   while(Serial.available()==0);
      //   input_pos_state_d = Serial.readString().toInt();
      //   Serial.print("Target position: ");
      //   Serial.println(input_pos_state_d);
      //   Serial.println("Enter y to confirm. Enter anything else to redo.");
      //   while(Serial.available()==0);
      //   input_state = Serial.readString();
      // }
      // state = '2';
      // back_HIP.armAxis(RIGHT);

      
      Serial.print("Current position:");
      Serial.println(back_HIP.transPosition_num2deg(LEFT, back_HIP.getAxisPos(LEFT,true)));
      Serial.println("Enter target position.");
      while(Serial.available()==0);
      input_pos_state_d = Serial.readString().toInt();
      Serial.print("Target position: ");
      Serial.println(input_pos_state_d);
      Serial.println("Enter y to confirm. Enter anything else to redo.");
      while(Serial.available()==0);
      String input_state = Serial.readString();
      while(input_state != 'y')
      {
        Serial.print("Current position:");
        Serial.println(back_HIP.transPosition_num2deg(LEFT, back_HIP.getAxisPos(LEFT,true)));
        Serial.println("Enter target position.");
        while(Serial.available()==0);
        input_pos_state_d = Serial.readString().toInt();
        Serial.print("Target position: ");
        Serial.println(input_pos_state_d);
        Serial.println("Enter y to confirm. Enter anything else to redo.");
        while(Serial.available()==0);
        input_state = Serial.readString();
      }
      state = '2';
      back_HIP.armAxis(LEFT);
    }
    else if (state == '2')
    {
      if(!back_HIP.moveTo_constVelo(LEFT, input_pos_state_d, 5.0))
      {

      }
      else
      {
        state = 'z';
      }
    }
    else if (state == 'e')
    {
      // Serial.print("Current position:");
      // Serial.println(back_AB.transPosition_num2deg(RIGHT, back_AB.getAxisPos(RIGHT,true)));
      // Serial.println("Enter target position.");
      // while(Serial.available()==0);
      // input_pos_state_e = Serial.readString().toInt();
      // Serial.print("Target position: ");
      // Serial.println(input_pos_state_d);
      // Serial.println("Enter y to confirm. Enter anything else to redo.");
      // while(Serial.available()==0);
      // String input_state = Serial.readString();
      // while(input_state != 'y')
      // {
      //   Serial.print("Current position:");
      //   Serial.println(back_AB.transPosition_num2deg(RIGHT, back_AB.getAxisPos(RIGHT,true)));
      //   Serial.println("Enter target position.");
      //   while(Serial.available()==0);
      //   input_pos_state_e = Serial.readString().toInt();
      //   Serial.print("Target position: ");
      //   Serial.println(input_pos_state_e);
      //   Serial.println("Enter y to confirm. Enter anything else to redo.");
      //   while(Serial.available()==0);
      //   input_state = Serial.readString();
      // }
      // state = '3';
      // back_AB.armAxis(RIGHT);

      
      Serial.print("Current position:");
      Serial.println(back_AB.transPosition_num2deg(LEFT, back_AB.getAxisPos(LEFT,true)));
      Serial.println("Enter target position.");
      while(Serial.available()==0);
      input_pos_state_e = Serial.readString().toInt();
      Serial.print("Target position: ");
      Serial.println(input_pos_state_d);
      Serial.println("Enter y to confirm. Enter anything else to redo.");
      while(Serial.available()==0);
      String input_state = Serial.readString();
      while(input_state != 'y')
      {
        Serial.print("Current position:");
        Serial.println(back_AB.transPosition_num2deg(LEFT, back_AB.getAxisPos(LEFT,true)));
        Serial.println("Enter target position.");
        while(Serial.available()==0);
        input_pos_state_e = Serial.readString().toInt();
        Serial.print("Target position: ");
        Serial.println(input_pos_state_e);
        Serial.println("Enter y to confirm. Enter anything else to redo.");
        while(Serial.available()==0);
        input_state = Serial.readString();
      }
      state = '3';
      back_AB.armAxis(LEFT);
    }
    else if (state == '3')
    {
      if(!back_AB.moveTo_constVelo(LEFT, input_pos_state_e, 5.0))
      {

      }
      else
      {
        state = 'z';
      }
    }
    else if (state == '!')  //all knee
    {
      Serial.println("Enter target position.");
      while(Serial.available()==0);
      target_k = Serial.readString().toInt();
      Serial.print("Target pos: ");
      Serial.println(target_k);
      Serial.println("Enter y to confirm. Enter anything else to redo.");
      while(Serial.available()==0);
      String input_state = Serial.readString();
      while(input_state != 'y')
      {
        Serial.println("Enter target position.");
        while(Serial.available()==0);
        target_k = Serial.readString().toInt();
        Serial.print("Target pos: ");
        Serial.println(target_k);
        Serial.println("Enter y to confirm. Enter anything else to redo.");
        while(Serial.available()==0);
        input_state = Serial.readString();
      }
      state = '^';
    }
    else if (state == '^')
    {
      if(!adjust_all_joint_by_type(KNEE, target_k, 5.0))
      {

      }
      else
      {
        state ='z';
      }
    }
    else if (state == '@')  //all hip
    {
      Serial.println("Enter target position.");
      while(Serial.available()==0);
      target_h = Serial.readString().toInt();
      Serial.print("Target pos: ");
      Serial.println(target_h);
      Serial.println("Enter y to confirm. Enter anything else to redo.");
      while(Serial.available()==0);
      String input_state = Serial.readString();
      while(input_state != 'y')
      {
        Serial.println("Enter target position.");
        while(Serial.available()==0);
        target_h = Serial.readString().toInt();
        Serial.print("Target pos: ");
        Serial.println(target_h);
        Serial.println("Enter y to confirm. Enter anything else to redo.");
        while(Serial.available()==0);
        input_state = Serial.readString();
      }
      state = '&';
    }
    else if (state == '&')
    {
      if(!adjust_all_joint_by_type(HIP, target_h, 5.0))
      {

      }
      else
      {
        state ='z';
      }
    }
    else if (state == '#')  //all ab
    {
      Serial.println("Enter target position.");
      while(Serial.available()==0);
      target_a = Serial.readString().toInt();
      Serial.print("Target pos: ");
      Serial.println(target_a);
      Serial.println("Enter y to confirm. Enter anything else to redo.");
      while(Serial.available()==0);
      String input_state = Serial.readString();
      while(input_state != 'y')
      {
        Serial.println("Enter target position.");
        while(Serial.available()==0);
        target_a = Serial.readString().toInt();
        Serial.print("Target pos: ");
        Serial.println(target_a);
        Serial.println("Enter y to confirm. Enter anything else to redo.");
        while(Serial.available()==0);
        input_state = Serial.readString();
      }
      state = '*';
      back_AB.armAxis();
    }
    else if (state == '*')
    {
      if(!adjust_all_joint_by_type(AB, target_a, 5.0))
      {

      }
      else
      {
        state ='z';
      }
    }
#endif
  }



bool adjust_all_joint_by_type(char jointType, float target_deg, float time_sec)
{
    if (jointType == AB)
    {
        Serial.println("---------------------------------");
        Serial.println("Adjusting AB motors location");
        Serial.print("Moving to: ");
        Serial.println(target_deg);
        // front_AB.armAxis();
        // back_AB.armAxis();
        // return(front_AB.moveTo_constVelo(RIGHT, -target_deg, time_sec)
        // && front_AB.moveTo_constVelo(LEFT, target_deg, time_sec)
        // && back_AB.moveTo_constVelo(RIGHT, target_deg, time_sec)
        // && back_AB.moveTo_constVelo(LEFT, -target_deg, time_sec));
        bool r1 = back_AB.moveTo_constVelo(RIGHT, target_deg, time_sec);
        // bool r1 = true;
        bool r2 = back_AB.moveTo_constVelo(LEFT, -target_deg, time_sec);
        return (r1 && r2);
    }
    else if (jointType == HIP)
    {
        Serial.println("---------------------------------");
        Serial.println("Adjusting HIP motors location");
        Serial.print("Moving to: ");
        Serial.println(target_deg);
        // front_HIP.armAxis();
        back_HIP.armAxis();
        // return(front_HIP.moveTo_constVelo(RIGHT, -target_deg, time_sec)
        // && front_HIP.moveTo_constVelo(LEFT, target_deg, time_sec)
        // && back_HIP.moveTo_constVelo(RIGHT, target_deg, time_sec)
        // && back_HIP.moveTo_constVelo(LEFT, -target_deg, time_sec));
        return(back_HIP.moveTo_constVelo(RIGHT, target_deg, time_sec)
        && back_HIP.moveTo_constVelo(LEFT, -target_deg, time_sec));
    }
    else if (jointType == KNEE)
    {
        Serial.println("---------------------------------");
        Serial.println("Adjusting KNEE motors location");
        Serial.print("Moving to: ");
        Serial.println(target_deg);
        // front_KNEE.armAxis();
        back_KNEE.armAxis();
        // return(front_KNEE.moveTo_constVelo(RIGHT, -target_deg, time_sec)
        // && front_KNEE.moveTo_constVelo(LEFT, target_deg, time_sec)
        // && back_KNEE.moveTo_constVelo(RIGHT, target_deg, time_sec)
        // && back_KNEE.moveTo_constVelo(LEFT, -target_deg, time_sec));
        return(back_KNEE.moveTo_constVelo(RIGHT, target_deg, time_sec)
        && back_KNEE.moveTo_constVelo(LEFT, -target_deg, time_sec));
    }
    return false;
}


void loop()
{
  if (radio.meaningful())
  {
    controlPoint.update(radio);
    bSpline.update(controlPoint);
  }
  if (checkTimer())
  {
    jointPositions.update(bSpline, time);
    messenger.send(jointPositions);
  }
}