/* EFS-500 version
   DJA 25 June 2017 */

#include <SIM/aerodefn.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/iodefn.h>

#include "iolib.h"
#include "aerolink.h"

#define BIT0    0x01
#define BIT1    0x02
#define BIT2    0x04
#define BIT3    0x08
#define BIT4    0x10
#define BIT5    0x20
#define BIT6    0x40
#define BIT7    0x80

void SetNoseGearLamp(bool Lamp);
void SetRightGearLamp(bool Lamp);
void SetLeftGearLamp(bool Lamp);
void SetGearTransitLamp(bool Lamp);
void SetGearDownLamps(bool On);
int convert(short unsigned int x);

static int                   FlapSelectorPosition = 0;
static IODefn_GearSelector   GearLeverPosition = IODefn_GearUp;
static bool                  HoldButton = false;
static bool                  ReverseMode = false;
static bool                  ParkBrake = false;
static unsigned char         OldHoldButtonSwitch = 0;
static unsigned char         OldFlapsUp = 0;
static unsigned char         OldFlapsDown = 0;
static unsigned char         OldReverseSwitch = 0;
static unsigned char         OldParkBrakeSwitch = 0;
static float                 ElectricTrim = 0.0;
static float                 RudderTrim = 0.0;

/* ---------------------------------------------------- */    
unsigned char IOLib_GetDigitalDataOutA()
{
    return 0;
}

/* ---------------------------------------------------- */    
unsigned char IOLib_GetDigitalDataOutB()
{
    return 0;
}

/* ---------------------------------------------------- */    
int convert(short unsigned int x)
{
    int a = x & 0xff;
    int b = (x >> 8) & 0xff;
    int r = (b << 8) | a;
    
    if ((r >> 15) != 0)
    {
        r |= 0xffff0000;
    }
    return r;
}

/* ---------------------------------------------------- */    
bool IOLib_GetReverseSwitch()
{
    unsigned char sw = AeroLink_IOPkt1.DigitalDataC & BIT5;

    if (sw != 0 && OldReverseSwitch == 0)
    {
        ReverseMode = !ReverseMode;
    }
    OldReverseSwitch = sw;
    return ReverseMode;
}

/* ---------------------------------------------------- */    
float IOLib_GetEngineLever(unsigned int LeverNumber)
{
    int e = convert(AeroLink_IOPkt1.AnalogueData[4]);

    return -0.5 * (float) e / 32767.0 + 0.5;  /* range 0->1.0 */
}

/* ---------------------------------------------------- */    
float IOLib_GetReverseEngineLever(unsigned int LeverNumber)
{
    if (ReverseMode)
    {
        int e = convert(AeroLink_IOPkt1.AnalogueData[4]);
        return (float) e / 32767.0;
    }
    else
    {
        return 0.0;
    }
}

/* ---------------------------------------------------- */    
float IOLib_GetElevator()
{
    int e = convert(AeroLink_IOPkt1.AnalogueData[1]);

    return (float) -e / 32767.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetAileron()
{
    int a = convert(AeroLink_IOPkt1.AnalogueData[0]);

    return (float) a / 32767.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetRudder()
{
    int r = convert(AeroLink_IOPkt1.AnalogueData[2]);

    return (float) -r / 32767.0;
}

/* ---------------------------------------------------- */
float IOLib_GetTiller()
{
    int t = convert(AeroLink_IOPkt1.AnalogueData[7]);




    return (float) t / 32767.0;
}

/* ---------------------------------------------------- */
float IOLib_GetElevatorTrim()
{
	float                               t = (float) convert(AeroLink_IOPkt1.AnalogueData[3]) / 32767.0;
	IODefn_ElevatorTrimSwitchPosition tsw = IOLib_GetElevatorTrimSwitch();
    
	if (tsw == IODefn_ElevatorTrimBackwards)
    {
	    ElectricTrim -= 0.002;
    }
	else if (tsw == IODefn_ElevatorTrimForwards)
    {
	    ElectricTrim += 0.002;
    }
	
	if (ElectricTrim < -1.0)
	{
	    ElectricTrim = -1.0;
	}
	else if (ElectricTrim > 1.0)
	{
	    ElectricTrim = 1.0;
	}

	return t + ElectricTrim;
    return t;
}

/* ---------------------------------------------------- */    
IODefn_ElevatorTrimSwitchPosition IOLib_GetElevatorTrimSwitch()
{
    int t = convert(AeroLink_IOPkt1.AnalogueData[9]);

    if (t > 1000)
    {
        return IODefn_ElevatorTrimBackwards;
    }
    else if (t < -1000)
    {
        return IODefn_ElevatorTrimForwards;
    }
    else
    {
        return IODefn_ElevatorTrimOff;
    }
}

/* ---------------------------------------------------- */
float IOLib_GetAileronTrim()
{
    return 0.0; /* no aileron trim */
}

/* ---------------------------------------------------- */
float IOLib_GetRudderTrim()
{
    IODefn_RudderTrimSwitchPosition rsw = IOLib_GetRudderTrimSwitch();
	
    if (rsw == IODefn_RudderTrimLeft)
	{
	    RudderTrim += 0.002;
	}
	else if (rsw == IODefn_RudderTrimRight)
	{
	    RudderTrim -= 0.002;
	}

	return RudderTrim;
}

/* ---------------------------------------------------- */    
IODefn_RudderTrimSwitchPosition IOLib_GetRudderTrimSwitch()
{
    int t = convert(AeroLink_IOPkt1.AnalogueData[9]);
	
    if (t < -1000)
    {
        return IODefn_RudderTrimLeft;
    }
    else if (t > 1000)
    {
        return IODefn_RudderTrimRight;
    }
    else
    {
        return IODefn_RudderTrimOff;
    }
}

/* ---------------------------------------------------- */    
int IOLib_GetFlapSelector()
{
    unsigned char FlapsUp = AeroLink_IOPkt1.DigitalDataC & BIT2;
    unsigned char FlapsDown = AeroLink_IOPkt1.DigitalDataC & BIT1;
    
    if (FlapsUp != 0 && OldFlapsUp == 0)
    {
        FlapSelectorPosition -= 1;
    }
    else if (FlapsDown != 0 && OldFlapsDown == 0)
    {
        FlapSelectorPosition += 1;
    }

    OldFlapsUp = FlapsUp;
    OldFlapsDown = FlapsDown;
    
	if (FlapSelectorPosition < 0)
    {
        FlapSelectorPosition = 0;
    }
    else if (FlapSelectorPosition > 6)
    {
        FlapSelectorPosition = 6;
    }
    return FlapSelectorPosition;
}

/* ---------------------------------------------------- */    
float IOLib_GetLeftBrake()
{
    int b = convert(AeroLink_IOPkt1.AnalogueData[6]);
    return (float) (32767 - b) / 65536.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetRightBrake()
{
    int b = convert(AeroLink_IOPkt1.AnalogueData[5]);
    return (float) (32767 - b) / 65536.0;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetLampsTestButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetWarningCancelButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_GearSelector IOLib_GetGearSelector()
{
    if ((BIT1 & AeroLink_IOPkt1.DigitalDataA) != 0)
    {
        GearLeverPosition = IODefn_GearUp;
    }
    else if ((BIT7 & AeroLink_IOPkt1.DigitalDataB) != 0)
    {
        GearLeverPosition = IODefn_GearDown;
    }
    return GearLeverPosition;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetThrottlePushButton()
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetThrottleSwitch()
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetTriggerSwitch()
{
    if ((BIT7 & AeroLink_IOPkt1.DigitalDataA) == 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_AileronTrimSwitchPosition IOLib_GetAileronTrimSwitch()
{
    return IODefn_AileronTrimOff;
}

/* ---------------------------------------------------- */    
bool IOLib_GetHoldButton()
{
    unsigned char sw = AeroLink_IOPkt1.DigitalDataC & BIT4;
    if (sw != 0 && OldHoldButtonSwitch == 0)
    {
        HoldButton = !HoldButton;
    }
    OldHoldButtonSwitch = sw;

    return HoldButton;
}

/* ---------------------------------------------------- */    
bool IOLib_GetRestoreButton()
{
    return ((BIT3 & AeroLink_IOPkt1.DigitalDataC) != 0);
}

/* ---------------------------------------------------- */    
bool IOLib_GetFreezeButton()
{
    return false;
}

/* ---------------------------------------------------- */    
bool IOLib_GetClockButton()
{
    return false;
}

/* ---------------------------------------------------- */    
bool IOLib_GetSideStick()
{
    return false;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetAutomaticResetSwitch()
{
    if ((BIT7 & AeroLink_IOPkt1.DigitalDataB) == 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetMasterSwitch()
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetColumnButton()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetParkBrake()
{
    unsigned char sw = AeroLink_IOPkt1.DigitalDataA & BIT5;
	
    if (sw != 0 && OldParkBrakeSwitch == 0)
    {
        ParkBrake = !ParkBrake;
    }
    OldParkBrakeSwitch = sw;

    return (ParkBrake) ? IODefn_On : IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetKeySwitch()
{
    return IODefn_On;
}

/* --------------------------------------------- */
void IOLib_UpdateGearLamps(float gearposition)
{
}

/* ---------------------------------------------------- */    
void SetNoseGearLamp(bool On)
{
}

/* ---------------------------------------------------- */    
void SetRightGearLamp(bool On)
{
}

/* ---------------------------------------------------- */    
void SetLeftGearLamp(bool On)
{
}

/* ---------------------------------------------------- */    
void SetGearTransitLamp(bool On)
{
}

/* --------------------------------------------- */
void SetGearDownLamps(bool On)
{
}

/* ---------------------------------------------------- */
void IOLib_StickShaker(bool On)
{
    return; /* no stick shaker */
}

/* ---------------------------------------------------- */    
void IOLib_SetSideStick(bool mode)
{
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetRightBoostPumpSwitch()
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetLeftBoostPumpSwitch()
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
extern IODefn_SwitchPosition IOLib_GetStarterSwitch(unsigned int EngineNumber)
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
extern IODefn_SwitchPosition IOLib_GetIgnitionSwitch(unsigned int EngineNumber)
{
    return IODefn_On;
}

/* ---------------------------------------------------- */    
void BEGIN_IOLib()
{
    SetGearTransitLamp(false);
    SetGearDownLamps(false);
}
