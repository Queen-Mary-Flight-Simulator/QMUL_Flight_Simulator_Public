#ifndef IOLib_H
#define IOLib_H

/* version for SIMPLOT */

#include <stdbool.h>
#include <SIM/iodefn.h>

extern unsigned char IOLib_DigitalDataOutA;
extern unsigned char IOLib_DigitalDataOutB;

/* analogue inputs */

extern float IOLib_GetElevator();

extern float IOLib_GetAileron();

extern float IOLib_GetRudder();

extern float IOLib_GetTiller();

extern float IOLib_GetElevatorTrim();

extern float IOLib_GetAileronTrim();

extern float IOLib_GetRudderTrim();

extern float IOLib_GetLeftWheel();

extern float IOLib_GetRightWheel();

extern float IOLib_GetLeftBrake();

extern float IOLib_GetRightBrake();

extern float IOLib_GetSpeedBrake();

extern float IOLib_GetFlapSelector();

/* digital inputs */

extern IODefn_ElevatorTrimSwitchPosition IOLib_GetElevatorTrimSwitch();

extern IODefn_AileronTrimSwitchPosition IOLib_GetAileronTrimSwitch();

extern IODefn_SwitchPosition IOLib_GetSpeedBrakeArmed();

extern IODefn_GearSelector IOLib_GetGearSelectorPosition();

extern IODefn_SwitchPosition IOLib_GetParkBrake();

extern bool IOLib_HoldButtonPressed();

extern bool IOLib_ResetButtonPressed();

extern bool IOLib_FreezeButtonPressed();

/* Digital Outputs */

extern void IOLib_SetTrimMotor(IODefn_ElevatorTrimSwitchPosition p);

extern void IOLib_EnableGearSelector(bool Armed);

extern void IOLib_GearDownLamp(bool On);

extern void IOLib_DoorsOpenLamp(bool On);

extern void IOLib_GearTransitLamp(bool On);

extern void IOLib_StickShaker(bool On);

extern void IOLib_EnableLeftBrakeReleaseLamp(bool On);

extern void IOLib_EnableRightBrakeReleaseLamp(bool On);

extern void IOLib_GroundIdleLamp(IODefn_SwitchPosition On);

extern void IOLib_GroundBandLamp(IODefn_SwitchPosition On);

void IOLib_EnableSpoiler(bool Armed);

/* the following functions are from ENG version of IOLIB */

extern float IOLib_GetEngineLever(unsigned int LeverNumber);

extern float IOLib_GetReverseEngineLever(unsigned int LeverNumber);

extern IODefn_SwitchPosition IOLib_GetEngineFltIgnitionSwitchPosition(unsigned int EngineNumber);

extern IODefn_SwitchPosition IOLib_GetEngineGndIgnitionSwitchPosition(unsigned int EngineNumber);

extern IODefn_SwitchPosition IOLib_GetFuelSwitch(unsigned int EngineNumber);

extern IODefn_SwitchPosition IOLib_GetFireHandle(unsigned int FireHandleNumber);

extern IODefn_SwitchPosition IOLib_GetEngineFuelSwitchPosition(unsigned int EngineNumber);

extern IODefn_APLSwitchPosition IOLib_GetAPLSwitchPosition();

extern IODefn_SwitchPosition IOLib_GetNo3StabTrimCutout();

extern IODefn_SwitchPosition IOLib_GetNo2StabTrimCutout();

extern IODefn_SwitchPosition IOLib_GetEngineLeverSwitch(unsigned int LeverNumber);

extern IODefn_SwitchPosition IOLib_GetATButton();

extern IODefn_SwitchPosition IOLib_GetAPButton();

extern IODefn_SwitchPosition IOLib_GetFDButton();

extern IODefn_SwitchPosition IOLib_GetGAButton();

extern IODefn_SwitchPosition IOLib_GetWarningHornCutout();

extern IODefn_SwitchPosition IOLib_GetAttentionGetterButton();

extern IODefn_SwitchPosition IOlib_GetAPDisconnect();

extern void IOLib_SetFireHandleLamp(unsigned int FireHandleNumber, bool On);

extern void IOLib_SetReverseSolenoid(unsigned int EngineNumber, bool On);

extern void IOLib_SetAttentionGetterLamp(bool On);

extern void IOLib_SetATMotor(IODefn_ATDirection d);

extern bool IOLib_HoldButtonPressed();

extern bool IOLib_ResetButtonPressed();

extern bool IOLib_FreezeButtonPressed();

extern IODefn_SwitchPosition IOLib_GetIgnitionSwitch(unsigned int EngineNumber);

extern IODefn_SwitchPosition IOLib_GetStarterSwitch(unsigned int EngineNumber);

extern void BEGIN_IOLib();

#endif