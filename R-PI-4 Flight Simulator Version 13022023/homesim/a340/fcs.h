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

extern void FCS_FCS(float *Elevator, float *Aileron, float *Rudder);
extern void FCS_EngageAutoland();
extern void FCS_DisengageAutoland();
extern bool FCS_AutolandEngaged();
extern void FCS_Autoland(float *Elevator, float *Aileron);
extern void FCS_EngageHeightHold();
extern void FCS_DisengageHeightHold();
extern bool FCS_HeightHoldEngaged();
extern void FCS_SetHeightHold(float Href);
extern float FCS_LOCHold();
extern float FCS_HeightHold(float Href);
extern void FCS_EngageHeadingHold();
extern void FCS_DisengageHeadingHold();
extern bool FCS_HeadingHoldEngaged();
extern void FCS_SetHeadingHold(float Href);
extern float FCS_HeadingHold(float HdgRef);
extern void FCS_EngageSpeedHold();
extern void FCS_DisengageSpeedHold();
extern bool FCS_SpeedHoldEngaged();
extern void FCS_SetSpeedHold(float Vref);
extern float FCS_SpeedHold(float Vref);
extern void FCS_EngageVSpeedHold();
extern void FCS_DisengageVSpeedHold();
extern bool FCS_VSpeedHoldEngaged();
extern void FCS_SetVSpeedHold(float Vref);
extern float FCS_VSpeedHold(float Vref);
extern void FCS_EngagePitchHold();
extern void FCS_DisengagePitchHold();
extern bool FCS_PitchHoldEngaged();
extern void FCS_SetPitchHold(float Theta);
extern void FCS_ResetFCS();
extern void BEGIN_FCS();
#endif

