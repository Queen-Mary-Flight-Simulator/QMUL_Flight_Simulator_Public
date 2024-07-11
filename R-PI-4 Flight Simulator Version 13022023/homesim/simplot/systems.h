/* SIMPLOT version */

#ifndef Systems_H
#define Systems_H

#include <stdbool.h>

#include <SIM/iodefn.h>

#define Systems_FlapUpRate      0.0015
#define Systems_FlapDownRate    0.0015
#define Systems_GearUpRate      0.002857 /* 7s */
#define Systems_GearDownRate    0.002857 /* 7s */

/* PFD */
extern bool                  Systems_Failures[51];
extern unsigned int          Systems_FlapSetting;
extern float                 Systems_FlapPosition;
extern float                 Systems_GearPosition;
extern IODefn_GearSelector   Systems_GearSelector;
extern bool                  Systems_ConfigWarning;

/* ENG */
extern bool                  Systems_RemoteHold;
extern bool                  Systems_Freezing;
extern bool                  Systems_EngineFireSound;

extern bool                  Systems_AttentionGetter;
extern bool                  Systems_EngineFire[4];

extern void  Systems_UpdateConfigWarning();
extern void  Systems_UpdateElevatorTrim();
extern float Systems_GetFlapPosition();
extern float Systems_GetGearPosition();
extern void  Systems_UpdateGearSelector(bool Airborne);
extern void  BEGIN_Systems();
#endif
