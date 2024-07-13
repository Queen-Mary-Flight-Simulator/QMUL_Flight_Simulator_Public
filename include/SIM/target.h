#ifndef Target_H
#define Target_H

#include <stdbool.h>

#define Target_ON 0
#define Target_OFF 1

typedef unsigned char Target_Toggle;
extern float Target_TU;
extern float Target_TV;
extern float Target_TW;
extern float Target_TYaw;
extern float Target_TYawDot;
extern double Target_TLatitude;
extern double Target_TLongitude;
extern float Target_TPz;
extern float Target_TPitch;
extern float Target_TRoll;
extern Target_Toggle Target_TargSwitch, Target_HUDSwitch, Target_ConeSwitch;
extern bool Target_Conflict;

extern void Target_TargPursuit(float RelDist, double Latitude, double Longitude, float Pz, float Yaw);

extern void Target_TargetDynamics(float GroundLevel, float altitude, double Latitude, double Longitude);

extern void BEGIN_Target();

#endif
