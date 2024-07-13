#ifndef IOLIB_H
#define IOLIB_H

#include <stdbool.h>
#include <SIM/iodefn.h>

/* analogue inputs */

extern float IOLib_GetElevator();
extern float IOLib_GetAileron();
extern float IOLib_GetRudder();
extern float IOLib_GetTiller();
extern float IOLib_GetElevatorTrim();
extern float IOLib_GetAileronTrim();
extern float IOLib_GetRudderTrim();
extern int IOLib_GetFlapSelector();
extern float IOLib_GetLeftBrake();
extern float IOLib_GetRightBrake();
extern float IOLib_GetEngineLever(unsigned int LeverNumber);
extern float IOLib_GetReverseEngineLever(unsigned int LeverNumber);

/* digital inputs */

extern IODefn_SwitchPosition IOLib_GetLampsTestButton();
extern IODefn_SwitchPosition IOLib_GetWarningCancelButton();
extern IODefn_GearSelector IOLib_GetGearSelector();
extern IODefn_RudderTrimSwitchPosition IOLib_GetRudderTrimSwitch();
extern IODefn_SwitchPosition IOLib_GetThrottlePushButton();
extern IODefn_SwitchPosition IOLib_GetThrottleSwitch();
extern IODefn_SwitchPosition IOLib_GetTriggerSwitch();
extern IODefn_AileronTrimSwitchPosition IOLib_GetAileronTrimSwitch();
extern IODefn_ElevatorTrimSwitchPosition IOLib_GetElevatorTrimSwitch();
extern bool IOLib_GetHoldButton();
extern bool IOLib_GetRestoreButton();
extern bool IOLib_GetFreezeButton();
extern bool IOLib_GetClockButton();
extern IODefn_SwitchPosition IOLib_GetRightBoostPumpSwitch();
extern IODefn_SwitchPosition IOLib_GetLeftBoostPumpSwitch();
extern IODefn_SwitchPosition IOLib_GetStarterSwitch(unsigned int EngineNumber);
extern IODefn_SwitchPosition IOLib_GetIgnitionSwitch(unsigned int EngineNumber);
extern bool IOLib_GetReverseSwitch();

extern bool IOLib_GetSideStick();
extern unsigned char IOLib_GetDigitalDataOutA();
extern unsigned char IOLib_GetDigitalDataOutB();
extern IODefn_SwitchPosition IOLib_GetAutomaticResetSwitch();
extern IODefn_SwitchPosition IOLib_GetMasterSwitch();
extern IODefn_SwitchPosition IOLib_GetColumnButton();
extern IODefn_SwitchPosition IOLib_GetParkBrake();
extern IODefn_SwitchPosition IOLib_GetKeySwitch();
extern void IOLib_StickShaker(bool On);
extern void IOLib_UpdateGearLamps(float gearposition);
void IOLib_SetSideStick(bool mode);

extern void BEGIN_IOLib();

#endif
