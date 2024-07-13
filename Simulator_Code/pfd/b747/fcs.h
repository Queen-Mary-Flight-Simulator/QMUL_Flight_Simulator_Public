#ifndef FCS_H
#define FCS_H

#include <stdbool.h>

extern bool  FCS_APSpeedMode;
extern float FCS_ThrottlePosition;
extern bool  FCS_AutoTrimming;
extern float FCS_Kp;
extern float FCS_Ki;
extern float FCS_Kd;
extern int   FCS_FD_VBar;
extern int   FCS_FD_HBar;
extern bool  FCS_TOGAMode;

extern void  FCS_FCS(float *Elevator, float *Aileron, float *Rudder);
extern void  FCS_ResetFCS();

extern void  FCS_Engage_ALT();
extern void  FCS_Disengage_ALT();
extern bool  FCS_ALT_Engaged();
extern void  FCS_Set_ALT(float Href);
extern float FCS_ALT(float Href);

extern void  FCS_Engage_HDG();
extern void  FCS_Disengage_HDG();
extern bool  FCS_HDG_Engaged();
extern void  FCS_Set_HDG(float Href);
extern float FCS_HDG(float HdgRef);

extern void  FCS_Engage_SPD();
extern void  FCS_Disengage_SPD();
extern bool  FCS_SPD_Engaged();
extern void  FCS_Set_SPD(float Vref);
extern float FCS_SPD(float Vref);

extern void  FCS_Engage_VSPD();
extern void  FCS_Disengage_VSPD();
extern bool  FCS_VSPD_Engaged();
extern void  FCS_Set_VSPD(float Vref);
extern float FCS_VSPD(float Vref);

extern void  FCS_Engage_Pitch();
extern void  FCS_Disengage_Pitch();
extern bool  FCS_Pitch_Engaged();
extern void  FCS_Set_Pitch(float Theta);
extern float FCS_Pitch(float Vref);

extern void  FCS_Engage_FPA();
extern void  FCS_Disengage_FPA();
extern bool  FCS_FPA_Engaged();
extern void  FCS_Set_FPA(float Theta);
extern float FCS_FPA(float Vref);

extern void  FCS_Engage_BankAngle();
extern void  FCS_Disengage_BankAngle();
extern bool  FCS_BankAngle_Engaged();
extern void  FCS_Set_BankAngle(float b);
extern float FCS_BankAngle(float b);

extern void  FCS_Engage_TurnCoordinator();
extern void  FCS_Disengage_TurnCoordinator();
extern bool  FCS_TurnCoordinator_Engaged();
extern void  FCS_TurnCoordinator(float *Rudder);

extern void  FCS_Engage_YawDamper();
extern void  FCS_Disengage_YawDamper();
extern bool  FCS_YawDamper_Engaged();
extern void  FCS_YawDamper(float *Rudder);

extern void  FCS_Engage_Autoland();
extern void  FCS_Disengage_Autoland();
extern bool  FCS_Autoland_Engaged();
extern void  FCS_Autoland(float *Elevator, float *Aileron);

extern void  FCS_Engage_LOC();
extern void  FCS_Disengage_LOC();
extern bool  FCS_LOC_Engaged();
extern float FCS_LOC();

extern void  BEGIN_FCS();
#endif
