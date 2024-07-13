/* EFS-500 version
   DJA 28 Nov 2022 */

#include <SIM/aerodefn.h>

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/iodefn.h>

#include "iolib.h"

#define BIT0     0x01
#define BIT1     0x02
#define BIT2     0x04
#define BIT3     0x08
#define BIT4     0x10
#define BIT5     0x20
#define BIT6     0x40
#define BIT7     0x80

#define MidPoint 1962  /* engine lever mid position */

bool             IOLib_SideStick;
int              ElevatorTrimPosition;
unsigned char    DigitalDataOut;
bool             OldClockButton;
bool             OldGearTransitLamp;
bool             OldGearDownLamp;
IODefn_IODataPkt *IOPkt;

void SetNoseGearLamp(bool Lamp);
void SetRightGearLamp(bool Lamp);
void SetLeftGearLamp(bool Lamp);
void SetGearTransitLamp(bool Lamp);
void SetGearDownLamps(bool On);

/* ---------------------------------------------------- */    
void IOLib_SetIOPkt(IODefn_IODataPkt *a)
{
    IOPkt = a;
}

/* ---------------------------------------------------- */    
float IOLib_GetElevator()
{
    unsigned int e1;
    unsigned int e2;
    float        sf;

    if (IOLib_SideStick)
    {
        e1 = IOPkt->AnalogueData[10];
        e2 = IOPkt->AnalogueData[11];
        if (e2 > e1)
        {
            e1 = e2;
            sf = -4096.0;
        }
        else
        {  
            sf = 4096.0;
        }
        return (float) ((int) e1) / sf;
    }
    else
    {
        e1 = IOPkt->AnalogueData[0];
    }
    return (float) (2048 - (int) (e1)) / 2048.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetAileron()
{
    unsigned int a1;
    unsigned int a2;
    float        sf;
  
    if (IOLib_SideStick)
    {
        a1 = IOPkt->AnalogueData[8];
        a2 = IOPkt->AnalogueData[9];
        if (a2 > a1)
        {
          a1 = a2;
          sf = -4096.0;
        }
        else
        {
            sf = 4096.0;
        }
        return (float) ((int) a1) / sf;
    }
    else
    {
        a1 = IOPkt->AnalogueData[1];
    }
    return (float) ((int) (a1) - 2048) / 2048.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetRudder()
{
    unsigned int RudderPosition;
  
    RudderPosition = IOPkt->AnalogueData[2];
    return (float) (2048 - (int) (RudderPosition)) / 2048.0;
}

/* ---------------------------------------------------- */
float IOLib_GetTiller()
{
    return 0.0;  /* no tiller */
}

/* ---------------------------------------------------- */
float IOLib_GetElevatorTrim()
{
    return 0.0;
}

/* ---------------------------------------------------- */
float IOLib_GetAileronTrim()
{
    return 0.0; /* for now */
}

/* ---------------------------------------------------- */
float IOLib_GetRudderTrim()
{
    return 0.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetFlapSelector()
{ 
    unsigned int FlapPosition;

    FlapPosition = IOPkt->AnalogueData[3];
    return (float) (FlapPosition) / 4096.0;
}


/* ---------------------------------------------------- */    
float IOLib_GetLeftBrake()
{
    unsigned int Brake;
  
    Brake = IOPkt->AnalogueData[6];
    return (float) (Brake) / 4096.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetRightBrake()
{
    unsigned int Brake;
  
    Brake = IOPkt->AnalogueData[7];
    return (float) (Brake) / 4096.0;
}

/* ---------------------------------------------------- */    
float IOLib_GetEngineLever(unsigned int LeverNumber)
{
    int LeverPosition;
    float lever;

    LeverPosition = (int) IOPkt->AnalogueData[4];
    if (LeverPosition >= MidPoint)
    {
        lever = (float) (LeverPosition - MidPoint) / (float) (4095 - MidPoint);
        if (lever < 0.0)
        {
            return 0.0;
        }
        else if (lever > 1.0)
        {
            return 1.0;
        }
        return lever;
    }
    else
    {
        return 0.0;
    }
}

/* ---------------------------------------------------- */    
float IOLib_GetReverseEngineLever(unsigned int LeverNumber)
{
    int LeverPosition;
    float lever;
  
    LeverPosition = (int) IOPkt->AnalogueData[4];
    if (LeverPosition <= MidPoint)
    {
        lever = (float) (MidPoint - LeverPosition) / (float) (MidPoint);
        if (lever < 0.0)
        {
            return 0.0;
        }
        else if (lever > 1.0)
        {
            return 1.0;
        }
        return lever;
    }
    else
    {
        return 0.0;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetLampsTestButton()
{
    if ((BIT7 & IOPkt->DigitalDataA) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetWarningCancelButton()
{
    if ((BIT6 & IOPkt->DigitalDataA) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_GearSelector IOLib_GetGearSelector()
{
    if ((BIT5 & IOPkt->DigitalDataA) == 0)
    {
        return IODefn_GearDown;
    }
    else if ((BIT4 & IOPkt->DigitalDataA) == 0)
    {
        return IODefn_GearUp;
    }
    else
    {
        return IODefn_GearOff;
    }
}

/* ---------------------------------------------------- */    
IODefn_RudderTrimSwitchPosition IOLib_GetRudderTrimSwitch()
{
    if ((BIT2 & IOPkt->DigitalDataA) == 0)
    {
        return IODefn_RudderTrimLeft;
    }
    else if ((BIT3 & IOPkt->DigitalDataA) == 0)
    {
        return IODefn_RudderTrimRight;
    }
    else
    {
        return IODefn_RudderTrimOff;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetThrottlePushButton()
{
    if ((BIT1 & IOPkt->DigitalDataA) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetThrottleSwitch()
{
    if ((BIT0 & IOPkt->DigitalDataA) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetTriggerSwitch()
{
    if ((BIT7 & IOPkt->DigitalDataB) != 0)
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
    if ((BIT5 & IOPkt->DigitalDataB) == 0)
    {
        return IODefn_AileronTrimLeftWingDown;
    }
    else if ((BIT6 & IOPkt->DigitalDataB) == 0)
    {
        return IODefn_AileronTrimRightWingDown;
    }
    else
    {
        return IODefn_AileronTrimOff;
    }
}

/* ---------------------------------------------------- */    
IODefn_ElevatorTrimSwitchPosition IOLib_GetElevatorTrimSwitch()
{
    if ((BIT4 & IOPkt->DigitalDataB) == 0)
    {
        return IODefn_ElevatorTrimBackwards;
    }
    else if ((BIT3 & IOPkt->DigitalDataB) == 0)
    {
        return IODefn_ElevatorTrimForwards;
    }
    else
    {
        return IODefn_ElevatorTrimOff;
    }
}

/* ---------------------------------------------------- */    
bool IOLib_GetHoldButton()
{
    return (BIT2 & IOPkt->DigitalDataB) == 0;
}

/* ---------------------------------------------------- */    
bool IOLib_GetRestoreButton()
{
    return (BIT1 & IOPkt->DigitalDataB) == 0;
}

/* ---------------------------------------------------- */    
bool IOLib_GetFreezeButton()
{
    return (BIT0 & IOPkt->DigitalDataB) == 0;
}

/* ---------------------------------------------------- */    
bool IOLib_GetClockButton()
{
    bool NewClockButton;
    bool Result;
  
    NewClockButton = (BIT7 & IOPkt->DigitalDataC) == 0;
    Result = NewClockButton && !OldClockButton;
    OldClockButton = NewClockButton;
    return Result;
}

/* ---------------------------------------------------- */    
unsigned char IOLib_GetDigitalDataOutA()
{
    return DigitalDataOut;
}

/* ---------------------------------------------------- */    
unsigned char IOLib_GetDigitalDataOutB()
{
    return 0;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetAutomaticResetSwitch()
{
    if ((BIT6 & IOPkt->DigitalDataC) != 0)
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
    if ((BIT1 & IOPkt->DigitalDataC) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetColumnButton()
{
    if ((BIT0 & IOPkt->DigitalDataC) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetParkBrake()
{
    if ((BIT7 & IOPkt->DigitalDataD) != 0)
    {
        return IODefn_Off;
    }
    else
    {
       return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetKeySwitch()
{
    if ((BIT6 & IOPkt->DigitalDataD) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */
void IOLib_StickShaker(bool On)
{
    return; /* no stick shaker */
}

/* --------------------------------------------- */
void IOLib_UpdateGearLamps(float gearposition)
{
    bool NewGearTransitLamp;
    bool NewGearDownLamp;

    NewGearTransitLamp = ((gearposition > 0.0) && (gearposition < 1.0)) || (IOLib_GetLampsTestButton() == IODefn_On);
    if (NewGearTransitLamp != OldGearTransitLamp)
    {
        SetGearTransitLamp(NewGearTransitLamp);
        OldGearTransitLamp = NewGearTransitLamp;
    }

    NewGearDownLamp = (gearposition >= 1.0) || (IOLib_GetLampsTestButton() == IODefn_On);
    if (NewGearDownLamp != OldGearDownLamp)
    {
        SetGearDownLamps(NewGearDownLamp);
        OldGearDownLamp = NewGearDownLamp;
    }
}

/* ---------------------------------------------------- */    
void IOLib_SetSideStick(bool mode)
{
    IOLib_SideStick = mode;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetRightBoostPumpSwitch()
{
    if ((BIT5 & IOPkt->DigitalDataC) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetLeftBoostPumpSwitch()
{
    if ((BIT4 & IOPkt->DigitalDataC) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetStarterSwitch(unsigned int EngineNumber)
{
    if ((BIT3 & IOPkt->DigitalDataC) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetIgnitionSwitch(unsigned int EngineNumber)
{
    if ((BIT2 & IOPkt->DigitalDataC) != 0)
    {
        return IODefn_Off;
    }
    else
    {
        return IODefn_On;
    }
}

/* ---------------------------------------------------- */    
void SetNoseGearLamp(bool On)
{
    if (On)
    {
        DigitalDataOut &= ~BIT0;
    }
    else
    {
        DigitalDataOut |= BIT0;
    }
}

/* ---------------------------------------------------- */    
void SetRightGearLamp(bool On)
{
    if (On)
    {
        DigitalDataOut &= ~BIT1;
    }
    else
    {
        DigitalDataOut |= BIT1;
    }
}

/* ---------------------------------------------------- */    
void SetLeftGearLamp(bool On)
{
    if (On)
    {
        DigitalDataOut &= ~BIT2;
    }
    else
    {
        DigitalDataOut |= BIT2;
    }
}

/* ---------------------------------------------------- */    
void SetGearTransitLamp(bool On)
{
    if (On)
    {
        DigitalDataOut &= ~BIT3;
    }
    else
    {
        DigitalDataOut |= BIT3;
    }
}

/* --------------------------------------------- */
void SetGearDownLamps(bool On)
{
    SetNoseGearLamp(On);
    SetLeftGearLamp(On);
    SetRightGearLamp(On);
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition IOLib_GetTOGASwitch()
{
    return IODefn_Off;  /* not implemented for EFS */
}

/* ---------------------------------------------------- */    
void BEGIN_IOLib()
{
    OldClockButton = false;
  
    DigitalDataOut = 0xf;  /* all lights off */
  
    OldGearTransitLamp = false;
    OldGearDownLamp = false;

    IOLib_SideStick = false;

    SetGearTransitLamp(false);
    SetGearDownLamps(false);
}
