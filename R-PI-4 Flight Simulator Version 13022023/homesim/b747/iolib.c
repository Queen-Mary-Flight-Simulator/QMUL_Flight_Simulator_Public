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

static bool                ReverseMode = false;
static float               ElectricTrim = 0.0;
static float               RudderTrim = 0.0;
static bool                HoldButton = false;
static char                OldHoldSwitch = 0;

void SetNoseGearLamp(bool Lamp);
void SetRightGearLamp(bool Lamp);
void SetLeftGearLamp(bool Lamp);
void SetGearTransitLamp(bool Lamp);
void SetGearDownLamps(bool On);
int convert(short unsigned int x);

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
int convert(short unsigned int x)  /* sign extend to 32 bits */
{
    int r = (int) x;
    
    if (r & 0x8000)
    {
        r |= 0xffff0000;
    }
    return r;
}

/* ---------------------------------------------------- */    
bool IOLib_GetReverseSwitch()
{
    if ((AeroLink_IOPkt1.DigitalDataA & BIT3) != 0)
    {
        ReverseMode = true;
    }
    else if ((AeroLink_IOPkt1.DigitalDataA & BIT4) != 0)
    {
        ReverseMode = false;
    }
	
    return ReverseMode;
}

/* ---------------------------------------------------- */    
float IOLib_GetEngineLever(unsigned int LeverNumber)
{
    int e;
	
	if (LeverNumber < 2)
	{
        e = convert(AeroLink_IOPkt1.AnalogueData[5]);  /* port engine(s) */
	}
	else
	{
        e = convert(AeroLink_IOPkt1.AnalogueData[4]);  /* starboard engine(s) */
	}

    //printf("IOLib: lever=%d ch=%2x e=%d -> %f\n", LeverNumber, (LeverNumber < 2) ? (int) AeroLink_IOPkt1.AnalogueData[5] : (int) AeroLink_IOPkt1.AnalogueData[4], e, -0.5 * (float) e / 32767.0 + 0.5);
    return -0.5 * (float) e / 32767.0 + 0.5;  /* range 0->1.0 */
}

/* ---------------------------------------------------- */    
float IOLib_GetReverseEngineLever(unsigned int LeverNumber)
{
    int e;
		
	if (LeverNumber < 2)
	{
		e = convert(AeroLink_IOPkt1.AnalogueData[5]);  /* port engine(s) */
	}
	else
	{
		e = convert(AeroLink_IOPkt1.AnalogueData[4]);  /* starboard engine(s) */
	}
	return (float) e / 32767.0;
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
	return IOLib_GetElevatorTrim();
}

/* ---------------------------------------------------- */
float IOLib_GetElevatorTrim()
{
	float                               t = -(float) convert(AeroLink_IOPkt1.AnalogueData[3]) / 32767.0;
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
}

/* ---------------------------------------------------- */    
IODefn_ElevatorTrimSwitchPosition IOLib_GetElevatorTrimSwitch()
{
    int t = convert(AeroLink_IOPkt1.AnalogueData[8]);

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
    if (AeroLink_IOPkt1.DigitalDataB & BIT5)
	{
	    return 0;  /* flaps up */
	}
	else if (AeroLink_IOPkt1.DigitalDataB & BIT4)
	{
	    return 6;  /* flaps down */
	}
	else
	{
	    return 4; /* flaps 20 */
	}
}

/* ---------------------------------------------------- */    
float IOLib_GetLeftBrake()
{
    int b = convert(AeroLink_IOPkt1.AnalogueData[7]);
    return (float) (32767 - b) / 65536.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetRightBrake()
{
    int b = convert(AeroLink_IOPkt1.AnalogueData[6]);
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
    return ((BIT2 & AeroLink_IOPkt1.DigitalDataB) != 0) ? IODefn_GearUp : IODefn_GearDown;
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
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_AileronTrimSwitchPosition IOLib_GetAileronTrimSwitch()
{
    return IODefn_AileronTrimOff;
}

/* ---------------------------------------------------- */    
bool IOLib_GetHoldButton()
{
    unsigned char sw = AeroLink_IOPkt1.DigitalDataB & BIT0;
    if (sw != 0 && OldHoldSwitch == 0)
    {
        HoldButton = !HoldButton;
    }
    OldHoldSwitch = sw;

    return HoldButton;
}

/* ---------------------------------------------------- */
void IOLib_SetHoldButton(IODefn_IODataPkt a, bool s)
{
    if (s)
    { 
        a.DigitalDataD &= ~BIT7;
	}
	else
    { 
        a.DigitalDataD |= BIT7;
	}	
}

/* ---------------------------------------------------- */    
bool IOLib_GetRestoreButton()
{
    return ((BIT6 & AeroLink_IOPkt1.DigitalDataA) != 0);
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
    return IODefn_Off;
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
    if ((AeroLink_IOPkt1.DigitalDataB & BIT3) == 0)
	{
	    return IODefn_On;
	}
	else
	{
	    return IODefn_Off;
	}
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
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetLeftBoostPumpSwitch()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetStarterSwitch(unsigned int EngineNumber)
{
    if (EngineNumber < 2)
	{
	    if (AeroLink_IOPkt1.DigitalDataB & BIT7)
		{
            return IODefn_On;
		}
		else
		{
            return IODefn_Off;
		}
	}
	else
	{
	    if (AeroLink_IOPkt1.DigitalDataB & BIT6)
		{
            return IODefn_On;
		}
		else
		{
            return IODefn_Off;
		}
	}
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetFuelSwitch(unsigned int EngineNumber)
{
    if (EngineNumber < 2)  /* left engine */
	{
	    if (AeroLink_IOPkt1.DigitalDataA & BIT1)
		{
            return IODefn_On;
		}
		else
		{
            return IODefn_Off;
		}
	}
	else
	{
	    if (AeroLink_IOPkt1.DigitalDataA & BIT0)
		{
            return IODefn_On;
		}
		else
		{
            return IODefn_Off;
		}
	}
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetTOGASwitch()
{
    if (AeroLink_IOPkt1.DigitalDataA & BIT2)
	{
	    return IODefn_On;
	}
	else
	{
	    return IODefn_Off;
	}
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetAPSwitch()
{
    if ((AeroLink_IOPkt1.DigitalDataA & BIT5) == 0)
	{
	    return IODefn_On;
	}
	else
	{
	    return IODefn_Off;
	}
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetATHRSwitch()
{
    if ((AeroLink_IOPkt1.DigitalDataB & BIT1) == 0)
	{
	    return IODefn_On;
	}
	else
	{
	    return IODefn_Off;
	}
}

/* ---------------------------------------------------- */    
void BEGIN_IOLib()
{
    SetGearTransitLamp(false);
    SetGearDownLamps(false);
}
