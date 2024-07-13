#ifndef IOLIB_H
#define IOLIB_H

#include <stdbool.h>
#include <SIM/iodefn.h>

#define BIT0    0x01
#define BIT1    0x02
#define BIT2    0x04
#define BIT3    0x08
#define BIT4    0x10
#define BIT5    0x20
#define BIT6    0x40
#define BIT7    0x80

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
extern void IOLib_SetHoldButton(IODefn_IODataPkt a, bool s);
extern bool IOLib_GetRestoreButton();
extern bool IOLib_GetFreezeButton();
extern bool IOLib_GetClockButton();
extern IODefn_SwitchPosition IOLib_GetRightBoostPumpSwitch();
extern IODefn_SwitchPosition IOLib_GetLeftBoostPumpSwitch();
extern IODefn_SwitchPosition IOLib_GetStarterSwitch(unsigned int EngineNumber);
extern IODefn_SwitchPosition IOLib_GetIgnitionSwitch(unsigned int EngineNumber);
extern IODefn_SwitchPosition IOLib_GetFuelSwitch(unsigned int EngineNumber);
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

extern IODefn_SwitchPosition IOLib_GetTOGASwitch();
extern IODefn_SwitchPosition IOLib_GetAPSwitch();
extern IODefn_SwitchPosition IOLib_GetATHRSwitch();

extern void BEGIN_IOLib();

#endif
