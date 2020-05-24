// Libraries:
#include "src/lib/globals.h"
#include "src/lib/ODriveArduino.h"
#include "src/lib/jointPositions.h"
#include "src/lib/bSpline.h"
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

// Initialize control positions
struct cp_1
{
  float ab_pos[23] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0};
  float ab_vel[22] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0};
  float hip_pos[23] = {
      0.695828411290931, 0.710114125576645, 0.738685554148074, 0.776962478967295, 0.827152316651150,
      0.869562720354769, 0.908402735322112, 0.923215614040566, 1.05555765214887, 1.25554843780019,
      1.07940031135145, 0.913295921962567, 0.684610016441595, 0.447931665200037, 0.340028544922052,
      0.336167105336494, 0.409113820820486, 0.475149071905041, 0.538740476445784, 0.606583290638415,
      0.652971268433788, 0.681542697005217, 0.695828411290931};
  float hip_vel[22] = {
      0.357142857142859, 0.357142857142859, 0.318974373493508, 0.313686485524093, 0.252442879188211,
      0.285588345348106, 0.142431526138980, 1.83808386261535, 4.99976964128294, -4.40370316121842,
      -4.15260973472210, -5.71714763802429, -5.91695878103895, -2.69757800694964, -0.0536311053549747,
      0.701410725807621, 0.485553316798194, 0.378520265123474, 0.424017588703944, 0.386566481628109,
      0.357142857142856, 0.357142857142856};
  float knee_pos[23] = {
      1.34883464940023, 1.34883464940023, 1.34883464940023, 1.34092511661378, 1.32475585330151,
      1.30279731457381, 1.27890634121836, 1.22139644000072, 1.35307177158772, 1.64642431633239,
      1.70265459835609, 1.76191049428161, 1.70265459835609, 1.64642431633239, 1.35307177158772,
      1.22139644000072, 1.27890634121837, 1.30279731457380, 1.32475585330152, 1.34092511661378,
      1.34883464940023, 1.34883464940023, 1.34883464940023};
  float knee_vel[22] = {
      0.0, 0.0, -0.0659127732204186, -0.101057895701677, -0.130705587664862, -0.175668921731310,
      -0.552979819400347, 1.82882404981945, 7.33381361861677, 1.40575705059255, 1.48139739813791,
      -1.48139739813790, -1.40575705059256, -7.33381361861673, -1.82882404981952, 0.552979819400512,
      0.175668921731126, 0.130705587664976, 0.101057895701627, 0.0659127732204242, 0.0, 0.0};
  float time[19] = {
      0.0, 0.20, 0.40, 0.60, 0.80, 0.84, 0.88, 0.92, 0.96, 1.0,
      1.04, 1.08, 1.12, 1.16, 1.20, 1.40, 1.60, 1.80, 2.0};
} STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo;

struct cp_2
{
  float ab_pos[23] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0};
  float ab_vel[22] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0};
  float hip_pos[23] = {
      0.695828411290931, 0.695828411290931, 0.695828411290931, 0.830217112877060, 0.793869046768671,
      0.878370085436553, 0.907739619067237, 0.923292076837951, 1.05554233762791, 1.25555484749189,
      1.07939798308279, 0.913295921962567, 0.684612344710261, 0.447925255508338, 0.340043859443014,
      0.336090642539108, 0.409776937075361, 0.466341706823257, 0.572023746328263, 0.553328656728651,
      0.695828411290931, 0.695828411290931, 0.695828411290931};
  float hip_vel[22] = {
      0.0, 0.0, 1.11990584655107, -0.227175413177431, 0.502982373023104,
      0.215952453166798, 0.149542863179946, 1.83680917763829, 5.00031274659948, -4.40392161022754,
      -4.15255152800547, -5.71708943130766, -5.91717723004808, -2.69703490163310, -0.0549057903320299,
      0.708522062848585, 0.415917424616889, 0.629059758958367, -0.116844309997578, 1.18749795468567,
      0.0, 0.0};
  float knee_pos[23] = {
      1.34883464940023, 1.34883464940023, 1.34883464940023, 1.34092511661378, 1.32475585330151,
      1.30279731457381, 1.27890634121836, 1.22139644000072, 1.35307177158772, 1.64642431633239,
      1.70265459835609, 1.76191049428161, 1.70265459835609, 1.64642431633239, 1.35307177158772,
      1.22139644000072, 1.27890634121837, 1.30279731457380, 1.32475585330152, 1.34092511661378,
      1.34883464940023, 1.34883464940023, 1.34883464940023};
  float knee_vel[22] = {
      0.0, 0.0, -0.0659127732204186, -0.101057895701677, -0.130705587664862,
      -0.175668921731310, -0.552979819400347, 1.82882404981945, 7.33381361861677, 1.40575705059255,
      1.48139739813791, -1.48139739813790, -1.40575705059256, -7.33381361861673, -1.82882404981952,
      0.552979819400512, 0.175668921731126, 0.130705587664976, 0.101057895701627, 0.0659127732204242,
      0.0, 0.0};
  float time[19] = {
      0.0, 0.20, 0.40, 0.60, 0.80, 0.84, 0.88, 0.92, 0.96, 1.0,
      1.04, 1.08, 1.12, 1.16, 1.20, 1.40, 1.60, 1.80, 2.0};
} STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo;

struct cp_3
{
  float ab_pos[23] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0};
  float ab_vel[22] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0};
  float hip_pos[23] = {
      0.695828411290931, 0.710114125576645, 0.738685554148074, 0.776962442711200, 0.827152473978189,
      0.869562452032170, 0.908403037041549, 0.923215485008620, 1.05555774519973, 1.25554821193897,
      1.07940084010833, 0.913294692130314, 0.684612873467137, 0.447925029647121, 0.340043952493870,
      0.336090513507162, 0.409777238794797, 0.466341438500659, 0.572023903655302, 0.553328620472556,
      0.695828411290931, 0.695828411290931, 0.695828411290931};
  float hip_vel[22] = {
      0.357142857142859, 0.357142857142859, 0.318974071359386, 0.313687695418680, 0.252440345559413,
      0.285592536833666, 0.142427384298759, 1.83808694709871, 4.99976166848110, -4.40368429576609,
      -4.15265369945035, -5.71704546657942, -5.91719609550041, -2.69702692883125, -0.0549088748153923,
      0.708526204688798, 0.415913233131340, 0.629062292587157, -0.116845519892161, 1.18749825681979,
      0.0, 0.0};
  float knee_pos[23] = {
      1.34883464940023, 1.34883464940023, 1.34883464940023, 1.34092511661378, 1.32475585330151,
      1.30279731457381, 1.27890634121836, 1.22139644000072, 1.35307177158772, 1.64642431633239,
      1.70265459835609, 1.76191049428161, 1.70265459835609, 1.64642431633239, 1.35307177158772,
      1.22139644000072, 1.27890634121837, 1.30279731457380, 1.32475585330152, 1.34092511661378,
      1.34883464940023, 1.34883464940023, 1.34883464940023};
  float knee_vel[22] = {
      0.0, 0.0, -0.0659127732204186, -0.101057895701677, -0.130705587664862,
      -0.175668921731310, -0.552979819400347, 1.82882404981945, 7.33381361861677, 1.40575705059255,
      1.48139739813791, -1.48139739813790, -1.40575705059256, -7.33381361861673, -1.82882404981952,
      0.552979819400512, 0.175668921731126, 0.130705587664976, 0.101057895701627, 0.0659127732204242,
      0.0, 0.0};
  float time[19] = {
      0.0, 0.20, 0.40, 0.60, 0.80, 0.84, 0.88, 0.92, 0.96, 1.0,
      1.04, 1.08, 1.12, 1.16, 1.20, 1.40, 1.60, 1.80, 2.0};
} STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo;

struct cp_4
{
  float ab_pos[23] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0};
  float ab_vel[22] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0};
  float hip_pos[23] = {
      0.695828411290931, 0.695828411290931, 0.695828411290931, 0.830217149133155, 0.793868889441632,
      0.878370353759150, 0.907739317347801, 0.923292205869897, 1.05554224457705, 1.25555507335310,
      1.07939745432591, 0.913297151794821, 0.684609487684719, 0.447931891061254, 0.340028451871195,
      0.336167234368439, 0.409113519101050, 0.475149340227638, 0.538740319118746, 0.606583326894510,
      0.652971268433788, 0.681542697005217, 0.695828411290931};
  float hip_vel[22] = {
      0.0, 0.0, 1.11990614868520, -0.227176623072016, 0.502984906651893,
      0.215948261681255, 0.149547005020152, 1.83680609315493, 5.00032071940132, -4.40394047567987,
      -4.15250756327723, -5.71719160275253, -5.91693991558662, -2.69758597975148, -0.0536280208716162,
      0.701406583967413, 0.485557508283740, 0.378517731494685, 0.424018798598527, 0.386566179493987,
      0.357142857142856, 0.357142857142856};
  float knee_pos[23] = {
      1.34883464940023, 1.34883464940023, 1.34883464940023, 1.34092511661378, 1.32475585330151,
      1.30279731457381, 1.27890634121836, 1.22139644000072, 1.35307177158772, 1.64642431633239,
      1.70265459835609, 1.76191049428161, 1.70265459835609, 1.64642431633239, 1.35307177158772,
      1.22139644000072, 1.27890634121837, 1.30279731457380, 1.32475585330152, 1.34092511661378,
      1.34883464940023, 1.34883464940023, 1.34883464940023};
  float knee_vel[22] = {
      0.0, 0.0, -0.0659127732204186, -0.101057895701677, -0.130705587664862,
      -0.175668921731310, -0.552979819400347, 1.82882404981945, 7.33381361861677, 1.40575705059255,
      1.48139739813791, -1.48139739813790, -1.40575705059256, -7.33381361861673, -1.82882404981952,
      0.552979819400512, 0.175668921731126, 0.130705587664976, 0.101057895701627, 0.0659127732204242,
      0.0, 0.0};
  float time[19] = {
      0.0, 0.20, 0.40, 0.60, 0.80, 0.84, 0.88, 0.92, 0.96, 1.0,
      1.04, 1.08, 1.12, 1.16, 1.20, 1.40, 1.60, 1.80, 2.0};
} STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo;

// Initialize bSpline object
bSpline traj_continuous_forward_ab_pos(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.ab_pos, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.time, 19, 5);
bSpline traj_continuous_forward_ab_vel(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.ab_vel, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.time, 19, 4);
bSpline traj_continuous_forward_hip_pos(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.hip_pos, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.time, 19, 5);
bSpline traj_continuous_forward_hip_vel(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.hip_vel, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.time, 19, 4);
bSpline traj_continuous_forward_knee_pos(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.knee_pos, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.time, 19, 5);
bSpline traj_continuous_forward_knee_vel(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.knee_vel, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_nonZeroEndVelo.time, 19, 4);

bSpline traj_1step_forward_ab_pos(STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.ab_pos, STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.time, 19, 5);
bSpline traj_1step_forward_ab_vel(STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.ab_vel, STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.time, 19, 4);
bSpline traj_1step_forward_hip_pos(STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.hip_pos, STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.time, 19, 5);
bSpline traj_1step_forward_hip_vel(STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.hip_vel, STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.time, 19, 4);
bSpline traj_1step_forward_knee_pos(STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.knee_pos, STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.time, 19, 5);
bSpline traj_1step_forward_knee_vel(STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.knee_vel, STATIC_forward_eightyDutyCycle_ZeroIniVelo_ZeroEndVelo.time, 19, 4);

bSpline traj_start_forward_ab_pos(STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.ab_pos, STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.time, 19, 5);
bSpline traj_start_forward_ab_vel(STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.ab_vel, STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.time, 19, 4);
bSpline traj_start_forward_hip_pos(STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.hip_pos, STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.time, 19, 5);
bSpline traj_start_forward_hip_vel(STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.hip_vel, STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.time, 19, 4);
bSpline traj_start_forward_knee_pos(STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.knee_pos, STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.time, 19, 5);
bSpline traj_start_forward_knee_vel(STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.knee_vel, STATIC_forward_eightyDutyCycle_ZeroIniVelo_nonZeroEndVelo.time, 19, 4);

bSpline traj_end_forward_ab_pos(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.ab_pos, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.time, 19, 5);
bSpline traj_end_forward_ab_vel(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.ab_vel, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.time, 19, 4);
bSpline traj_end_forward_hip_pos(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.hip_pos, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.time, 19, 5);
bSpline traj_end_forward_hip_vel(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.hip_vel, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.time, 19, 4);
bSpline traj_end_forward_knee_pos(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.knee_pos, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.time, 19, 5);
bSpline traj_end_forward_knee_vel(STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.knee_vel, STATIC_forward_eightyDutyCycle_nonZeroIniVelo_ZeroEndVelo.time, 19, 4);

// bspline pointers
bSpline *trajPtr_ab_pos;
bSpline *trajPtr_ab_vel;
bSpline *trajPtr_hip_pos;
bSpline *trajPtr_hip_vel;
bSpline *trajPtr_knee_pos;
bSpline *trajPtr_knee_vel;

/**
 * State variable
 * 'z' = idle
 * '1' = bspline
 */
char loop_state = 'z';

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

/**
   * Move to posture
   * until at least one of the following is satisfied:
   * 1. posture is reached
   * 2. 2xPOS_CONVERSION_TIME has elapsed
   * The process will be interrupted if input is detected from computer
   */
// void moveToPos_STDBY_blocking()
// {
//   armJoints();
//   timer.reset();
//   // unsigned long int stdby_timer = millis();
//   // while (!joint.checkPos(STANDBY_POS_FLAG, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr) && millis() - stdby_timer < 2 * POS_CONVERSION_TIME * 1000)
//   // {
//   //   unsigned long int standby_pos_start_time = millis();
//   //   float elapsedSec = (millis() - standby_pos_start_time) / 1000.0;
//   //   while (elapsedSec < POS_CONVERSION_TIME)
//   //   {
//   //     if (Serial.available() > 0)
//   //     {
//   //       Serial.println("stopping pos conversion!");
//   //       disarmJoints();
//   //       break;
//   //     }
//   //     joint.update_constantPos(elapsedSec);
//   //     // Serial.println(elapsedSec);
//   //     if (timer.check())
//   //     {
//   //       send(joint_ptr, front_AB_ptr, front_HIP_ptr, front_KNEE_ptr, back_AB_ptr, back_HIP_ptr, back_KNEE_ptr);
//   //     }
//   //     elapsedSec = (millis() - standby_pos_start_time) / 1000.0;
//   //   }
//   // }
//   unsigned long timerrr = millis();
//   while(millis()-timerrr < 5*1000)
//   {
//     #ifdef ENABLE_FRONT_LEFT
//     front_AB.moveTo_constVelo(LEFT, -AB_STANDBY_POS_DEG, 5.0);
//     front_HIP.moveTo_constVelo(LEFT, -HIP_STANDBY_POS_DEG, 5.0);
//     front_KNEE.moveTo_constVelo(LEFT, -KNEE_STANDBY_POS_DEG, 5.0);
//     #endif
//     #ifdef ENABLE_FRONT_RIGHT
//     front_AB.moveTo_constVelo(RIGHT, AB_STANDBY_POS_DEG, 5.0);
//     front_HIP.moveTo_constVelo(RIGHT, HIP_STANDBY_POS_DEG, 5.0);
//     front_KNEE.moveTo_constVelo(RIGHT, KNEE_STANDBY_POS_DEG, 5.0);
//     #endif
//     #ifdef ENABLE_BACK_LEFT
//     back_AB.moveTo_constVelo(LEFT, AB_STANDBY_POS_DEG, 5.0);
//     back_HIP.moveTo_constVelo(LEFT, -HIP_STANDBY_POS_DEG, 5.0);
//     back_KNEE.moveTo_constVelo(LEFT, -KNEE_STANDBY_POS_DEG, 5.0);
//     #endif
//     #ifdef ENABLE_BACK_RIGHT
//     back_AB.moveTo_constVelo(RIGHT, -AB_STANDBY_POS_DEG, 5.0);
//     back_HIP.moveTo_constVelo(RIGHT, HIP_STANDBY_POS_DEG, 5.0);
//     back_KNEE.moveTo_constVelo(RIGHT, KNEE_STANDBY_POS_DEG, 5.0);
//     #endif
//   }
// }

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
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos3_ab = input.toInt();

    Serial.println("Enter hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos3_hip = input.toInt();
    
    Serial.println("Enter knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos3_knee = input.toInt();
  }
  if (choice == 4)
  {
    Serial.println("P4");
    Serial.println("Enter ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos4_ab = input.toInt();

    Serial.println("Enter hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos4_hip = input.toInt();
    
    Serial.println("Enter knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos4_knee = input.toInt();
  }
  if (choice == 5)
  {
    Serial.println("P5 FL");
    Serial.println("Enter FL ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_FL = input.toFloat();

    Serial.println("Enter FL hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_FL = input.toFloat();
    
    Serial.println("Enter FL knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_FL = input.toFloat();
    


    Serial.println("P5 FR");
    Serial.println("Enter FR ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_FR = input.toFloat();

    Serial.println("Enter FR hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_FR = input.toFloat();
    
    Serial.println("Enter FR knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_FR = input.toFloat();
    


    Serial.println("P5 BL");
    Serial.println("Enter BL ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_bL = input.toFloat();

    Serial.println("Enter BL hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_bL = input.toFloat();
    
    Serial.println("Enter BL knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_bL = input.toFloat();
    


    Serial.println("P5 BR");
    Serial.println("Enter BR ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos5_ab_bR = input.toFloat();

    Serial.println("Enter BR hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos5_hip_bR = input.toFloat();
    
    Serial.println("Enter BR knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos5_knee_bR = input.toFloat();
  }
  if (choice == 6)
  {
    Serial.println("P6 FL");
    Serial.println("Enter FL ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_FL = input.toFloat();

    Serial.println("Enter FL hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_FL = input.toFloat();
    
    Serial.println("Enter FL knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos6_knee_FL = input.toFloat();
    


    Serial.println("P5 FR");
    Serial.println("Enter FR ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_FR = input.toFloat();

    Serial.println("Enter FR hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_FR = input.toFloat();
    
    Serial.println("Enter FR knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos6_knee_FR = input.toFloat();
    


    Serial.println("P5 BL");
    Serial.println("Enter BL ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_bL = input.toFloat();

    Serial.println("Enter BL hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_bL = input.toFloat();
    
    Serial.println("Enter BL knee pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("knee = ");
    Serial.println(input);
    pos6_knee_bL = input.toFloat();
    


    Serial.println("P5 BR");
    Serial.println("Enter BR ab pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("ab = ");
    Serial.println(input);
    pos6_ab_bR = input.toFloat();

    Serial.println("Enter BR hip pos");
    while (Serial.available()==0);
    input = readString();
    Serial.print("hip = ");
    Serial.println(input);
    pos6_hip_bR = input.toFloat();
    
    Serial.println("Enter BR knee pos");
    while (Serial.available()==0);
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
      Serial.println(micros());
      leg_inverseKinematics(0,0,0,0.4,0.142,0.05,t1,t2,t3,FRONT_LEFT_LEG);
      Serial.print(t1/PI_math*180);
      Serial.print(' ');
      Serial.print(t2/PI_math*180);
      Serial.print(' ');
      Serial.println(t3/PI_math*180);

      Serial.println(micros());
      Serial.println(micros());
      leg_inverseKinematics(0,0,0,0.4,0.142,0.05,t1,t2,t3,FRONT_RIGHT_LEG);
      Serial.print(t1/PI_math*180);
      Serial.print(' ');
      Serial.print(t2/PI_math*180);
      Serial.print(' ');
      Serial.println(t3/PI_math*180);

      Serial.println(micros());
      Serial.println(micros());
      leg_inverseKinematics(0,0,0,0.4,0.142,0.05,t1,t2,t3,BACK_LEFT_LEG);
      Serial.print(t1/PI_math*180);
      Serial.print(' ');
      Serial.print(t2/PI_math*180);
      Serial.print(' ');
      Serial.println(t3/PI_math*180);

      Serial.println(micros());
      Serial.println(micros());
      leg_inverseKinematics(0,0,0,0.4,0.142,0.05,t1,t2,t3,BACK_RIGHT_LEG);
      Serial.print(t1/PI_math*180);
      Serial.print(' ');
      Serial.print(t2/PI_math*180);
      Serial.print(' ');
      Serial.println(t3/PI_math*180);

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
  // process state requests
  String serial_input;
  if (Serial.available() != 0)
  {
    serial_input = readString();
  }
  if (serial_input == "1step" && loop_state == STATE_IDLE)
  {
    loop_state = STATE_1_STEP;
    timer.reset();
  }
  else if (serial_input == "cont" && loop_state == STATE_IDLE)
  {
    loop_state = STATE_CONTINUOUS;
    requested_gait_state = GAIT_STARTING;
    timer.reset();
    timerrr = micros();
  }
  else if (serial_input == "end" && loop_state == STATE_CONTINUOUS)
  {
    requested_gait_state = GAIT_ENDING;
  }
  else if (serial_input == "dis")
  {
    disarmJoints();
    loop_state = STATE_IDLE;
  }
  else if (serial_input == "config")
  {
    loop_state = STATE_IDLE;
    config_sequence();
  }
  // else if (serial_input == "pos1" && loop_state == STATE_IDLE)
  // {
  //   loop_state = STATE_POS_1;
  //   timerrr = millis();
  // }
  // else if (serial_input == "pos2" && loop_state == STATE_IDLE)
  // {
  //   loop_state = STATE_POS_2;
  //   front_AB.update_target(LEFT, 5000000, -AB_POS_2);
  //   timerrr = millis();
  // }
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
  // state machine
  switch (loop_state)
  {
  case STATE_1_STEP:
    if (timer.check())
    {
      Serial.print(micros() - timerrr);
      joint.update_bsplineTraj(&traj_1step_forward_ab_pos, &traj_1step_forward_ab_vel, &traj_1step_forward_hip_pos, &traj_1step_forward_hip_vel,
                               &traj_1step_forward_knee_pos, &traj_1step_forward_knee_vel, &time_fL, &time_fR, &time_bL, &time_bR,
                               &flag_fL_end, &flag_fR_end, &flag_bL_end, &flag_bR_end);

      Serial.print(" | ");
      Serial.print(time_fL, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.fL_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fL_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fL_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fL_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fL_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fL_knee_velo);

      Serial.print(" | ");
      Serial.print(time_bR, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.bR_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bR_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bR_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bR_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bR_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bR_knee_velo);

      Serial.print(" | ");
      Serial.print(time_fR, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.fR_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fR_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fR_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fR_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fR_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fR_knee_velo);

      Serial.print(" | ");
      Serial.print(time_bL, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.bL_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bL_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bL_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bL_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bL_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bL_knee_velo);
      Serial.print(' ');
      Serial.println(micros() - timerrr);
      // update time stamps until the cycle end flag is set to true
      if (flag_end_cycle == false)
      {
        update_time(2.0, 0.8, CRAWL_GAIT, GAIT_ONESTEP);
      }
      // cycle end flag is set to true. gait cycle has completed.
      // set state machine back to idle
      else
      {
        reset_time();
        loop_state = STATE_IDLE;
      }
    }
    break;
  case STATE_CONTINUOUS:
    if (timer.check())
    {
      /**
       *  Continuous step cycle initiation is requested
       *  @param current_step_state is set to starting phase
       */
      if (requested_gait_state == GAIT_STARTING && gait_state == GAIT_IDLE)
      {
        // requested_gait_state = GAIT_STARTING;
        // // set the bspline pointers to the starting phase bsplines

        trajPtr_ab_pos = &traj_start_forward_ab_pos;
        trajPtr_ab_vel = &traj_start_forward_ab_vel;
        trajPtr_hip_pos = &traj_start_forward_hip_pos;
        trajPtr_hip_vel = &traj_start_forward_hip_vel;
        trajPtr_knee_pos = &traj_start_forward_knee_pos;
        trajPtr_knee_vel = &traj_start_forward_knee_vel;
        gait_state = GAIT_STARTING;
      }
      /**
       * Starting phase has completed. Continuous phase is initiated.
       */
      if (gait_state == GAIT_NORMAL)
      {
        Serial.print("normal bspline ");
        trajPtr_ab_pos = &traj_continuous_forward_ab_pos;
        trajPtr_ab_vel = &traj_continuous_forward_ab_vel;
        trajPtr_hip_pos = &traj_continuous_forward_hip_pos;
        trajPtr_hip_vel = &traj_continuous_forward_hip_vel;
        trajPtr_knee_pos = &traj_continuous_forward_knee_pos;
        trajPtr_knee_vel = &traj_continuous_forward_knee_vel;
      }
      /**
       *  Continuous step cycle termination is requested
       *  @param cycle_end_flag is set to true to schedule termination upon the completion of the current cycle
       */
      if (requested_gait_state == GAIT_ENDING && cycle_end_flag == false)
      {
        cycle_end_flag = true;
        Serial.print(" ending detected ");
      }
      /**
       * The current continuous cycle is completed but the cycle termination is not requested
       * @param flag_normal_cycle is set to false to reset the flag param
       */
      if (cycle_end_flag == false && flag_normal_cycle == true)
      {
        flag_normal_cycle = false;
        Serial.println("normal cycle completed");
      }
      /**
       * The current continuous cycle is completed and the cycle termination was requested
       * @param current_step_state is set to ending phase
       */
      if (cycle_end_flag == true && flag_normal_cycle == true)
      {
        Serial.print("end bspline ");
        // current_step_state = STEP_ENDING;
        trajPtr_ab_pos = &traj_end_forward_ab_pos;
        trajPtr_ab_vel = &traj_end_forward_ab_vel;
        trajPtr_hip_pos = &traj_end_forward_hip_pos;
        trajPtr_hip_vel = &traj_end_forward_hip_vel;
        trajPtr_knee_pos = &traj_end_forward_knee_pos;
        trajPtr_knee_vel = &traj_end_forward_knee_vel;
        gait_state = GAIT_ENDING;
      }
      /**
       * The ending phase is complted
       * Switch the state machine to IDLE
       */
      if (cycle_end_flag == true && flag_end_cycle == true)
      {
        reset_time();
        loop_state = STATE_IDLE;
        Serial.println("end cycle completed");
      }
      Serial.print(micros() - timerrr);
      joint.update_bsplineTraj(trajPtr_ab_pos, trajPtr_ab_vel, trajPtr_hip_pos, trajPtr_hip_vel,
                               trajPtr_knee_pos, trajPtr_knee_vel, &time_fL, &time_fR, &time_bL, &time_bR,
                               &flag_fL_end, &flag_fR_end, &flag_bL_end, &flag_bR_end);

      Serial.print(" | ");
      Serial.print(time_fL, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.fL_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fL_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fL_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fL_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fL_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fL_knee_velo);

      Serial.print(" | ");
      Serial.print(time_bR, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.bR_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bR_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bR_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bR_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bR_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bR_knee_velo);

      Serial.print(" | ");
      Serial.print(time_fR, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.fR_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fR_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fR_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fR_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.fR_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.fR_knee_velo);

      Serial.print(" | ");
      Serial.print(time_bL, 4);
      Serial.print(" | ");
      Serial.print(joint_ptr->joint_pos_target_deg.bL_ab_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bL_ab_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bL_hip_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bL_hip_velo);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_pos_target_deg.bL_knee_pos);
      Serial.print(' ');
      Serial.print(joint_ptr->joint_vel_target_deg.bL_knee_velo);
      Serial.print(' ');
      Serial.println(micros() - timerrr);
      update_time(2.0, 0.8, CRAWL_GAIT, gait_state);
    }
    break;
  // case STATE_POS_1:
  //   front_AB.moveTo_constVelo(LEFT, -AB_POS_1, 5.0);
  //   front_HIP.moveTo_constVelo(LEFT, -HIP_POS_1, 5.0);
  //   front_KNEE.moveTo_constVelo(LEFT, -KNEE_POS_1, 5.0);
  //   front_AB.moveTo_constVelo(RIGHT, AB_POS_2, 5.0);
  //   front_HIP.moveTo_constVelo(RIGHT, HIP_POS_2, 5.0);
  //   front_KNEE.moveTo_constVelo(RIGHT, KNEE_POS_2, 5.0);
  //   if (millis()-timerrr > 5*1000)
  //   {
  //   Serial.println("switching to idle");
  //     loop_state = STATE_IDLE;
  //   }
  //   break;
  // case STATE_POS_2:
  //   front_AB.moveTo_constVelo(LEFT, -AB_POS_2, 5.0);
  //   front_HIP.moveTo_constVelo(LEFT, -HIP_POS_2, 5.0);
  //   front_KNEE.moveTo_constVelo(LEFT, -KNEE_POS_2, 5.0);
  //   front_AB.moveTo_constVelo(RIGHT, AB_POS_1, 5.0);
  //   front_HIP.moveTo_constVelo(RIGHT, HIP_POS_1, 5.0);
  //   front_KNEE.moveTo_constVelo(RIGHT, KNEE_POS_1, 5.0);
  //   if (millis()-timerrr > 4.95*1000)
  //   {
  //   Serial.println("switching to idle");
  //     loop_state = STATE_IDLE;
  //   }
  //   break;
  case STATE_test:
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
    break;
  default:
    break;
  }
}
