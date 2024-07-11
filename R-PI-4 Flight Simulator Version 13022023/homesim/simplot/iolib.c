#include <SIM/aerodefn.h>

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/iodefn.h>

#include "iolib.h"
#include "aerolink.h"
#include "englink.h"
#include "aero.h"

/* version for SIMPLOT */

unsigned char IOLib_DigitalDataOutA;  /* not used */
unsigned char IOLib_DigitalDataOutB;  /* not used */

/* ---------------------------------------------------- */
float IOLib_GetElevator()
{
    return AeroLink_AeroPkt.Elevator;  /* range -1.0->+1.0 */
}

/* ---------------------------------------------------- */
float IOLib_GetAileron()
{
    return AeroLink_AeroPkt.Aileron;  /* range -1.0->+1.0 */
}

/* ---------------------------------------------------- */
float IOLib_GetRudder()
{
    return AeroLink_AeroPkt.Rudder;  /* range -1.0->+1.0 */
}

/* ---------------------------------------------------- */
float IOLib_GetTiller()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetElevatorTrim()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetAileronTrim()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetRudderTrim()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetLeftWheel()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetRightWheel()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetLeftBrake()
{
    return AeroLink_AeroPkt.LeftBrake;
}

/* ---------------------------------------------------- */
float IOLib_GetRightBrake()
{
    return AeroLink_AeroPkt.RightBrake;
}

/* ---------------------------------------------------- */
float IOLib_GetSpeedBrake()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetFlapSelector()
{
    return 0.0;
}

/* ---------------------------------------------------- */
IODefn_ElevatorTrimSwitchPosition IOLib_GetElevatorTrimSwitch()
{
    return IODefn_ElevatorTrimOff;
}

/* ---------------------------------------------------- */
IODefn_AileronTrimSwitchPosition IOLib_GetAileronTrimSwitch()
{
    return IODefn_AileronTrimOff;
}

/* ---------------------------------------------------- */
IODefn_SwitchPosition IOLib_GetSpeedBrakeArmed()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */
IODefn_GearSelector IOLib_GetGearSelectorPosition()
{
    return IODefn_GearOff;
}

/* ---------------------------------------------------- */
IODefn_SwitchPosition IOLib_GetParkBrake()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */
bool IOLib_HoldButtonPressed()
{
    return false;
}

/* ---------------------------------------------------- */
bool IOLib_ResetButtonPressed()
{
    return false;
}

/* ---------------------------------------------------- */
bool IOLib_FreezeButtonPressed()
{
    return false;
}

/* ---------------------------------------------------- */
void IOLib_SetTrimMotor(IODefn_ElevatorTrimSwitchPosition p)
{
}

/* ---------------------------------------------------- */
void IOLib_EnableGearSelector(bool Armed)
{
}

/* ---------------------------------------------------- */
void IOLib_GearDownLamp(bool On)
{
}

/* ---------------------------------------------------- */
void IOLib_DoorsOpenLamp(bool On)
{
}

/* ---------------------------------------------------- */
void IOLib_GearTransitLamp(bool On)
{
}

/* ---------------------------------------------------- */
void IOLib_StickShaker(bool On)
{
}

/* ---------------------------------------------------- */
void IOLib_EnableEnableLeftBrakeRelease(bool Armed)
{
}

/* ---------------------------------------------------- */
void IOLib_EnableEnableRightBrakeRelease(bool Armed)
{
}

/* ---------------------------------------------------- */
void IOLib_GroundIdleLamp(IODefn_SwitchPosition On)
{
}

/* ---------------------------------------------------- */
void IOLib_GroundBandLamp(IODefn_SwitchPosition On)
{
}

/* ---------------------------------------------------- */
void IOLib_EnableSpoiler(bool Armed)
{
}

/* the following functions are from ENG version of IOLIB */

/* ---------------------------------------------------- */    
float IOLib_GetEngineLever(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
	return EngLink_EngPkt.EngineLevers[0];  /* range 0.0->+1.0 */
}

/* ---------------------------------------------------- */    
float IOLib_GetReverseEngineLever(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
    return 0.0;
}

/* Digital inputs */

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetEngineFltIgnitionSwitchPosition(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetEngineGndIgnitionSwitchPosition(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetStarterSwitch(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetFuelSwitch(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetFireHandle(unsigned int FireHandleNumber)  /* EngineNumber 0..3 */
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetEngineFuelSwitchPosition(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_APLSwitchPosition IOLib_GetAPLSwitchPosition()
{
    return IODefn_APLOff;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetNo3StabTrimCutout()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetNo2StabTrimCutout()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetEngineLeverSwitch(unsigned int EngineNumber)  /* EngineNumber 0..3 */
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOlib_GetATButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOlib_GetAPButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOlib_GetFDButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOlib_GetGAButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetWarningHornCutout()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetAttentionGetterButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOlib_GetAPDisconnect()
{
    return IODefn_Off;
}

/* Digital outputs */

/* ---------------------------------------------------- */    
void IOLib_SetFireHandleLamp(unsigned int FireHandleNumber, bool On)  /* FireHandleNumber 0..3 */
{
}

/* ---------------------------------------------------- */    
void IOLib_SetAttentionGetterLamp(bool On)
{
}

/* ---------------------------------------------------- */    
void IOLib_SetReverseSolenoid(unsigned int EngineNumber, bool On)  /* EngineNumber 0..3 */
{
}

/* ---------------------------------------------------- */    
void IOLib_SetATMotor(IODefn_ATDirection dir)
{
}

/* ---------------------------------------------------- */
void BEGIN_IOLib()
{
}
