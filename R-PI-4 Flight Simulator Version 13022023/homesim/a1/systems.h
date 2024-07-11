#ifndef Systems_H
#define Systems_H

#include <stdbool.h>

#include <SIM/iodefn.h>

#define Systems_FlapUpRate   0.0015
#define Systems_FlapDownRate 0.0015
#define Systems_GearUpRate   0.002857 /* 7s */
#define Systems_GearDownRate 0.002857 /* 7s */

extern bool                  Systems_Failures[51];
extern unsigned int          Systems_FlapSetting;
extern float                 Systems_FlapPosition;
extern float                 Systems_GearPosition;
extern IODefn_GearSelector   Systems_GearSelector;
extern bool                  Systems_ConfigWarning;
extern bool                  Systems_ParkBrake;

extern bool                  Systems_RemoteHold;
extern bool                  Systems_Freezing;
extern unsigned int          Systems_SysTicks;
extern bool                  Systems_EngineFireSound;
extern bool                  Systems_AttentionGetter;
extern bool                  Systems_EngineFire[4];

extern bool                  Systems_AdfFixedCard;
extern bool                  Systems_AdfDip;
extern bool                  Systems_HSI_Installed;
extern bool                  Systems_VOR_Installed;
extern bool                  Systems_Radio_Installed;
extern float                 Systems_SelectedAltitude;
extern bool                  Systems_MarkerTest;
extern IODefn_SwitchPosition Systems_MasterSwitch;
extern IODefn_SwitchPosition Systems_KeySwitch;

extern void  Systems_UpdateConfigWarning();
extern void  Systems_UpdateElevatorTrim();
extern float Systems_GetFlapPosition();
extern float Systems_GetGearPosition();
extern void  Systems_UpdateGearSelector(bool Airborne);
extern void  Systems_UpdateAileronTrim();
extern void  Systems_UpdateRudderTrim();
extern float Systems_GetGearPosition();
extern float Systems_GetElevatorTrim();
extern float Systems_GetAileronTrim();
extern float Systems_GetRudderTrim();

extern void  Systems_UpdateSystems();

extern void  BEGIN_Systems();

#endif
