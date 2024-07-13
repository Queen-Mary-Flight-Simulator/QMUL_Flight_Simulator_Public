/* +------------------------------+---------------------------------+
   | Module      : engines.c      | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-10      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 P&W JT9D Engine model             |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <SIM/iodefn.h>
#include <SIM/maths.h>

#include "systems.h"
#include "engines.h"
#include "aerolink.h"
#include "englink.h"
#include "navlink.h"
#include "ioslink.h"
#include "iolib.h"
#include "pfd.h"
#include "sounds.h"

static unsigned int ReverseDelay;
static bool         ReverseThrust;
static bool         OldReverseThrust;

static void Engine(float ThrottlePosition, float ReversePosition,
                   unsigned int EngineNumber, bool Failed);

Engines_EngineData    Engines_Engines[4];
float                 Engines_EngineThrustX;
float                 Engines_EngineThrustY;
float                 Engines_EngineThrustZ;
float                 Engines_EnginePMT;
float                 Engines_EngineRMT;
float                 Engines_EngineYMT;
bool                  Engines_SingleEngineMode;
float                 Engines_ThrottleLever[4];
float                 Engines_ReverseLever[4];
float                 Engines_FuelQuantityLeft;
float                 Engines_FuelQuantityRight;
float                 Engines_EprDemand[4];
IODefn_SwitchPosition Engines_EngineState[4];
EngDefn_Propulsion    Engines_EngineType;

const float           YEO = 21.15;
const float           YEI = 12.07;
const float           ZEO = 1.65;
const float           ZEI = 4.45;

/* --------------------------------------------------- */
void Engines_Update()
{
    if (EngLink_ReplayMode)
    {
        EngLink_Replay();
    }
    EngLink_FormPacket();
	
    memcpy(&AeroLink_EngPkt, &EngLink_EngPkt, sizeof(EngDefn_EngDataPkt));
    memcpy(&NavLink_EngPkt,  &EngLink_EngPkt, sizeof(EngDefn_EngDataPkt));
    memcpy(&IosLink_EngPkt,  &EngLink_EngPkt, sizeof(EngDefn_EngDataPkt));

    Engines_EngineModel(PFD_Held);
  	SoundSystem(PFD_Held);
    EngLink_RespondToIos();
}

/* ----------------------------------------------------------- */
void Engines_EngineModel(bool Held)
{
    unsigned int i;
    bool         Fail[4];
    float        WheelHeight;

    if (Held)
    {
        return;
    }

    WheelHeight = -EngLink_AeroPkt.Pz + (float) (EngLink_NavPkt.GroundLevel) + EngLink_AeroPkt.CGHeight;

    for (i = 0; i <= 3; i += 1)
    {
        Engines_ThrottleLever[i] = IOLib_GetEngineLever(i);
        Engines_ReverseLever[i] = IOLib_GetReverseEngineLever(i);

        if (EngLink_OctaveMode)
        {
            Engines_ThrottleLever[i] = EngLink_ProtoPkt.Data.Matlab.Throttle;
        }

        Fail[i] = Systems_Failures[i + 10];
    }

    if (EngLink_AeroPkt.APSpeedMode)
    {
        if ((WheelHeight * 3.280840) > 50.0)
        {
            unsigned int i;
            
            for (i = 0; i <= 3; i += 1)
            {
                Engines_ThrottleLever[i] = EngLink_AeroPkt.APThrottlePosition;
            }
        }
    }

    for (i = 0; i <= 3; i += 1)
    {
         Engine(Engines_ThrottleLever[i], Engines_ReverseLever[i], i, Fail[i]);
    }
    
    Engines_EngineThrustX = Engines_Engines[0].Thrust + Engines_Engines[1].Thrust +
                            Engines_Engines[2].Thrust + Engines_Engines[3].Thrust;
    Engines_EngineThrustY = 0.0349 * (Engines_Engines[0].Thrust + Engines_Engines[1].Thrust -
                                      Engines_Engines[2].Thrust - Engines_Engines[3].Thrust);
    Engines_EngineThrustZ = -0.0436 * Engines_EngineThrustX;

    Engines_EnginePMT = ZEO * (Engines_Engines[0].Thrust + Engines_Engines[3].Thrust) +
                        ZEI * (Engines_Engines[1].Thrust + Engines_Engines[2].Thrust);
    Engines_EngineYMT = YEO * (Engines_Engines[0].Thrust - Engines_Engines[3].Thrust) +
                        YEI * (Engines_Engines[1].Thrust - Engines_Engines[2].Thrust);
    Engines_EngineRMT = 0.0436 * Engines_EngineYMT;
}

/* ----------------------------------------------------------- */
static void Engine(float ThrottlePosition, float ReversePosition,
                   unsigned int EngineNumber, bool Failed)
{
    float e;
    bool  StartSelected = false;
    float m = EngLink_AeroPkt.MachNumber;
    bool  IgnitionSwitch = (IOLib_GetIgnitionSwitch(EngineNumber) == IODefn_On);
    bool  StarterSwitch = IOLib_GetStarterSwitch(EngineNumber) == IODefn_On;
    
    e = Engines_Engines[EngineNumber].Epr;
    StartSelected = IgnitionSwitch && StarterSwitch;

    if (!IgnitionSwitch || Failed)
    {
        Engines_EngineState[EngineNumber] = IODefn_Off;
    }
    else if ((Engines_EngineState[EngineNumber] == IODefn_Off) && StartSelected)
    {
        Engines_EngineState[EngineNumber] = IODefn_On;
    }
    
    if (Engines_EngineState[EngineNumber] == IODefn_On)
    {
        ReverseThrust = ReversePosition > 0.06 && EngLink_AeroPkt.OnTheGround;
        if (ReverseThrust != OldReverseThrust)
        {
            ReverseDelay     = 100;
            OldReverseThrust = ReverseThrust;
        }
        if (ReverseDelay > 0)
        {
            ReverseDelay     = ReverseDelay - 1;
            ThrottlePosition = 0.0;
            ReversePosition  = 0.0;
        }
        if (ReverseThrust)
        {
            ThrottlePosition = ReversePosition;
        }
        if (ThrottlePosition <= 0.45)
        {
            e = 1.02 + (ThrottlePosition - 0.2) * 0.62;
        }
        else
        {
            e = 1.175 + (ThrottlePosition - 0.45) * 0.554545;
        }
    }
    else
    {
        e = 0.0;
    }

    if ((Engines_Engines[EngineNumber].Epr < 0.5) && !StartSelected)
    {
        e = 0.0;
    }

    e = e - m * 0.09667 * m - 0.3833 * m * m;
    /* previously e = e - m * 0.475; */

    Engines_EprDemand[EngineNumber] = e;
    Maths_Integrate(&Engines_Engines[EngineNumber].Epr,
                    0.2 * (e - Engines_Engines[EngineNumber].Epr));

    if (ReverseThrust)
    {
        float t = -((Engines_Engines[EngineNumber].Epr - 1.0) * 29286.0 + m * 50000.0) * 4.4482;
        
        if (t > 0.0)
        {
            t = 0.0;
        }
        Engines_Engines[EngineNumber].Thrust = t;
    }
    else
    {
        float t = (Engines_Engines[EngineNumber].Epr - 1.0) * 413685.0;

        if (m <= 0.4)
        {
            t += (108732.0 - 469530.0 * m + 494244.0 * m * m) * (Engines_Engines[EngineNumber].Epr - 1.05);
        }
        else
        {
            t += (m - 0.4) * 62791.0;
        }
        if (t < 0.0)
        {
            t = 0.0;
        }
        Engines_Engines[EngineNumber].Thrust = t;
    }

    if (Engines_Engines[EngineNumber].Epr >= 1.0)
    {
        Engines_Engines[EngineNumber].Rpm = 70.0 + 50.0 * (Engines_Engines[EngineNumber].Epr - 1.0);
    }
    else
    {
        Engines_Engines[EngineNumber].Rpm = Engines_Engines[EngineNumber].Epr * 70.0;
    }
    
    Engines_Engines[EngineNumber].Egt      = 8.8 * Engines_Engines[EngineNumber].Rpm;

    Engines_Engines[EngineNumber].FuelFlow = Engines_Engines[EngineNumber].Thrust * 0.0408;
    if (Engines_Engines[EngineNumber].FuelFlow < 0.0)
    {
        Engines_Engines[EngineNumber].FuelFlow = 0.0;
    }
}

/* ----------------------------------------------------------- */
void Engines_Init()
{
}

/* ----------------------------------------------------------- */
void BEGIN_Engines()
{
    int i;

    printf("ENG starting\n");
    
    Engines_SingleEngineMode = false;
    
    for (i = 0; i <= 3; i += 1)
    {
        Engines_Engines[i].Thrust   = 0.0;
        Engines_Engines[i].Epr      = 0.0;
        Engines_Engines[i].Rpm      = 0.0;
        Engines_Engines[i].FuelFlow = 0.0;
        Engines_Engines[i].Egt      = 0.0;

        Engines_ThrottleLever[i]    = 0.0;
        Engines_EprDemand[i]        = 0.0;
        Engines_EngineState[i]      = IODefn_Off;
    }
    
    Engines_FuelQuantityLeft  = Engines_MaxFuel;
    Engines_FuelQuantityRight = Engines_MaxFuel;
    Engines_EngineThrustX     = 0.0;
    Engines_EngineThrustY     = 0.0;
    Engines_EngineThrustZ     = 0.0;
    Engines_EnginePMT         = 0.0;
    Engines_EngineRMT         = 0.0;
    Engines_EngineYMT         = 0.0;
    Engines_EngineType        = EngDefn_Turbofan;

    ReverseDelay              = 0;
    ReverseThrust             = false;
    OldReverseThrust          = false;
}
