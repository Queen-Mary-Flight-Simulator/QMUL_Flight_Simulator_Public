/* +------------------------------+---------------------------------+
   | Module      : EngLink.c      | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-02      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/lfs/eicas/               |
   | Compiler    : gcc 5.4                                          |
   | OS          : Windows10                                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Flight model packet management                   |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <SIM/clocks.h>
#include <SIM/iodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/protodefn.h>
#include <SIM/maths.h>

#include "iolib.h"
#include "englink.h"
#include "engines.h"
#include "systems.h"

IODefn_IODataPkt       EngLink_IOPkt1;
IODefn_IODataPkt       EngLink_IOPkt2;
AeroDefn_AeroDataPkt   EngLink_AeroPkt;
EngDefn_EngDataPkt     EngLink_EngPkt;
NavDefn_NavDataPkt     EngLink_NavPkt;
IosDefn_IosDataPkt     EngLink_IosPkt;
ProtoDefn_ProtoDataPkt EngLink_ProtoPkt;

bool                   EngLink_OctaveMode;
bool                   EngLink_ReplayMode;
unsigned int           EngLink_CameraPosition;
bool                   EngLink_Freezing;
bool                   EngLink_RemoteHold;

bool                   EngLink_Reloading;
char                   EngLink_ReloadFilename[50];
bool                   EngLink_Stopping;

static unsigned int    FrameNumber;
static float           WindSpeed;
static float           WindDirection;
static bool            OldOctaveMode;
static bool            OldReplayMode;
static unsigned int    CmdPtr;
static float           Alt_QNH;

static unsigned char GetByte();
static float GetReal();
static unsigned int GetWord();
static int GetInt();
static bool GetBoolean();
static void GetFilename(char Str[]);

/* ------------------------------------------------ */
bool EngLink_RespondToIos()
{
    float t;

    CmdPtr = 0;

    while (1)
    {
        switch (GetWord())
        {
            case IosDefn_EndOfPkt:
                return false;

            case IosDefn_Exit:
                EngLink_Stopping = true;
				return true;
                break;

            case IosDefn_SetRunHoldFreeze:
                switch (GetInt())
                {
                    case 0:
                        EngLink_RemoteHold = false;
                        EngLink_Freezing   = false;
                        break;
                    case 1:
                        EngLink_RemoteHold = true;
                        EngLink_Freezing   = false;
                        break;
                    case 2:
                        EngLink_RemoteHold = false;
                        EngLink_Freezing   = true;
                        break;
                }
                break;

            case IosDefn_SetOctaveMode:
                EngLink_OctaveMode = GetBoolean();
                break;

            case IosDefn_LoadFlightPlan:
                GetByte();
                GetByte();
                GetByte();
                GetByte();
                GetReal();
                GetReal();
                GetInt();
                GetInt();
                break;

            case IosDefn_SetFlightPlanMode:
                GetBoolean();
                break;

            case IosDefn_FailFlaps:
                Systems_Failures[1] = GetBoolean();
                break;

            case IosDefn_FailGear:
                Systems_Failures[2] = GetBoolean();
                break;

            case IosDefn_FailNav1Localiser:
                Systems_Failures[3] = GetBoolean();
                break;
            case IosDefn_FailNav1GlideSlope:
                Systems_Failures[4] = GetBoolean();
                break;

            case IosDefn_FailNav2Localiser:
                Systems_Failures[5] = GetBoolean();
                break;

            case IosDefn_FailNav2GlideSlope:
                Systems_Failures[6] = GetBoolean();
                break;
            case IosDefn_FailRMI1:
                Systems_Failures[7] = GetBoolean();
                break;

            case IosDefn_FailRMI2:
                Systems_Failures[8] = GetBoolean();
                break;
            case IosDefn_FailDME:
                Systems_Failures[9] = GetBoolean();
                break;

            case IosDefn_FailEngine1:
                Systems_Failures[10] = GetBoolean();
                break;
            case IosDefn_FailEngine2:
                Systems_Failures[11] = GetBoolean();
                break;

            case IosDefn_FailEngine3:
                Systems_Failures[12] = GetBoolean();
                break;

            case IosDefn_FailEngine4:
                Systems_Failures[13] = GetBoolean();
                break;
            case IosDefn_FailASI:
                Systems_Failures[14] = GetBoolean();
                break;
            case IosDefn_FailAI:
                Systems_Failures[15] = GetBoolean();
                break;
            case IosDefn_FailVSI:
                Systems_Failures[16] = GetBoolean();
                break;
            case IosDefn_FailAltimeter:
                Systems_Failures[17] = GetBoolean();
                break;

            case IosDefn_FailTurn:
                Systems_Failures[18] = GetBoolean();
                break;

            case IosDefn_FailCompass:
                Systems_Failures[19] = GetBoolean();
                break;
            
			case IosDefn_FailFD:
                Systems_Failures[20] = GetBoolean();
                break;

            case IosDefn_Engine1Fire:
                Systems_Failures[21] = GetBoolean();
				Systems_EngineFire[0] = true;
                break;
			
            case IosDefn_Engine2Fire:
                Systems_Failures[22] = GetBoolean();
				Systems_EngineFire[1] = true;
                break;
			
            case IosDefn_Engine3Fire:
                Systems_Failures[23] = GetBoolean();
				Systems_EngineFire[2] = true;
                break;
			
            case IosDefn_Engine4Fire:
                Systems_Failures[24] = GetBoolean();
				Systems_EngineFire[3] = true;
                break;
			
            case IosDefn_Restore:
                break;

            case IosDefn_SetAircraftAltitude:
                GetReal();
                break;

            case IosDefn_SetAircraftHeading:
                GetReal();
                break;

            case IosDefn_SetAircraftSpeed:
                GetReal();
                break;

            case IosDefn_SetCgPosition:
                GetReal();
                break;

            case IosDefn_SetLeftFuelQuantity:
                Engines_FuelQuantityLeft = GetReal();
                break;
            case IosDefn_SetRightFuelQuantity:
                Engines_FuelQuantityRight = GetReal();
                break;
            case IosDefn_SetSingleEngineMode:
                Engines_SingleEngineMode = GetBoolean();
                break;

            case IosDefn_SetMorseMode:
                GetInt();
                break;

            case IosDefn_SetRMICardType:
                GetBoolean();
                break;

            case IosDefn_SetAdfDip:
                GetBoolean();
                break;

            case IosDefn_SetTurbulence:
                GetReal();
                break;

            case IosDefn_SetWindSpeed:
                GetReal();
                break;

            case IosDefn_SetWindDir:
                GetReal();
                break;

            case IosDefn_SetQNH:
                Alt_QNH = GetReal();
                break;

            case IosDefn_SetMagneticVariation:
                GetReal();
                break;

            case IosDefn_SetTimeOfDay:
                t                 = GetReal();
                Clocks_ClockHours = (unsigned int) t;
                Clocks_ClockMins  = (unsigned int) ((t - (float) Clocks_ClockHours) * 60.0);
                break;

            case IosDefn_RePositionAircraft:
                GetReal();
                GetReal();
                break;

            case IosDefn_SetGroundTemperature:
                GetReal();
                break;

            case IosDefn_SetTargetPosition:
                GetReal();
                GetReal();
                break;

            case IosDefn_SetTargetDistance:
                GetReal();
                break;

            case IosDefn_SetTargetSpeed:
                GetReal();
                break;

            case IosDefn_SetTargetHeading:
                GetReal();
                break;

            case IosDefn_SetTargetTurnRate:
                GetReal();
                break;

            case IosDefn_SetTargetAltitude:
                GetReal();
                break;

            case IosDefn_SetTargetClimbRate:
                GetReal();
                break;

            case IosDefn_SetTargetConflict:
                break;

            case IosDefn_SetAutopilotAltitude:
                GetReal();
                break;

            case IosDefn_AutopilotAltitudeOn:
                GetBoolean();
                break;

            case IosDefn_SetAutopilotHeading:
                GetReal();
                break;

            case IosDefn_AutopilotHeadingOn:
                GetBoolean();
                break;

            case IosDefn_SetAutopilotSpeed:
                GetReal();
                break;

            case IosDefn_AutopilotSpeedOn:
                GetBoolean();
                break;

            case IosDefn_SetAutopilotVSpeed:
                GetReal();
                break;

            case IosDefn_AutopilotVSpeedOn:
                GetBoolean();
                break;

            case IosDefn_AutolandOn:
                GetBoolean();
                break;

            case IosDefn_SetKp:
                GetReal();
                break;

            case IosDefn_SetKi:
                GetReal();
                break;

            case IosDefn_SetKd:
                GetReal();
                break;

            case IosDefn_PlaybackReplay:
                EngLink_ReplayMode = GetBoolean();
                if (EngLink_ReplayMode != OldReplayMode)
                {
                    if (EngLink_ReplayMode)
                    {
                        OldOctaveMode       = EngLink_OctaveMode;
                        EngLink_OctaveMode = false;
                    }
                    else
                    {
                        EngLink_OctaveMode = OldOctaveMode;
                    }
                    OldReplayMode = EngLink_ReplayMode;
                }
                break;

            case IosDefn_PlaybackCamera:
                GetInt();
                break;

            case IosDefn_Models:
                EngLink_Reloading = true;
                GetFilename(EngLink_ReloadFilename);
                break;

            case IosDefn_SetDate:
                GetByte(); /* ignore 32 bit arg (only needed for RPi) */
                GetByte();
                GetByte();
                GetByte();
                break;

            default:
                return false;
        }
    }
}

/* ------------------------------------------------ */
static unsigned char GetByte()
{
    unsigned char x;

    x      = EngLink_IosPkt.CmdBuff[CmdPtr];
    CmdPtr = CmdPtr + 1;
    return x;
}

/* ------------------------------------------------ */
static float GetReal()
{
    union
    {
        float r;
        char  b[4];
    } x32;

    x32.b[0] = GetByte();
    x32.b[1] = GetByte();
    x32.b[2] = GetByte();
    x32.b[3] = GetByte();
    return x32.r;
}

/* ------------------------------------------------ */
static unsigned int GetWord()
{
    union
    {
        unsigned short int c;
        char               b[2];
    } x16;

    x16.b[0] = GetByte();
    x16.b[1] = GetByte();
    return (unsigned int) x16.c;
}

/* ------------------------------------------------ */
static int GetInt()
{
    union
    {
        short int i;
        char      b[2];
    } x16;

    x16.b[0] = GetByte();
    x16.b[1] = GetByte();
    return (int) x16.i;
}

/* ------------------------------------------------ */
static bool GetBoolean()
{
    unsigned char x;

    x = GetByte();
    return(x != 0);
}

/* ------------------------------------------------ */
static void GetFilename(char Str[])
{
    unsigned int i;

    i = 0;
    do
    {
        Str[i] = GetByte();
        i      = i + 1;
    } while (!(Str[i - 1] == 0));
}

/* ------------------------------------------------ */
void EngLink_FormPacket()
{
    unsigned int i;

    EngLink_EngPkt.PktNumber       = FrameNumber;
    FrameNumber                    = FrameNumber + 1;

    EngLink_EngPkt.NumberOfEngines = 4;
    for (i = 0; i <= 3; i += 1)
    {
        EngLink_EngPkt.Engines[i].Thrust           = Engines_Engines[i].Thrust;
        EngLink_EngPkt.Engines[i].Epr              = Engines_Engines[i].Epr;
        EngLink_EngPkt.Engines[i].Rpm              = Engines_Engines[i].Rpm;
        EngLink_EngPkt.Engines[i].FuelFlow         = Engines_Engines[i].FuelFlow;
        EngLink_EngPkt.Engines[i].Egt              = Engines_Engines[i].Egt;
        EngLink_EngPkt.Engines[i].Beta             = Engines_Engines[i].Beta;
        EngLink_EngPkt.Engines[i].ManifoldPressure = Engines_Engines[i].ManifoldPressure;
        
		EngLink_EngPkt.EngineLevers[i]             = Engines_ThrottleLever[i];
		EngLink_EngPkt.ReverseLevers[i]            = Engines_ReverseLever[i];
    }
    EngLink_EngPkt.FuelQuantityLeft  = Engines_FuelQuantityLeft;
    EngLink_EngPkt.FuelQuantityRight = Engines_FuelQuantityRight;

    EngLink_EngPkt.EngineThrustX     = Engines_EngineThrustX;
    EngLink_EngPkt.EngineThrustY     = Engines_EngineThrustY;
    EngLink_EngPkt.EngineThrustZ     = Engines_EngineThrustZ;
    EngLink_EngPkt.EnginePMT         = Engines_EnginePMT;
    EngLink_EngPkt.EngineRMT         = Engines_EngineRMT;
    EngLink_EngPkt.EngineYMT         = Engines_EngineYMT;

    EngLink_EngPkt.EngineType        = Engines_EngineType;
    EngLink_EngPkt.EngineFireSound   = Systems_EngineFireSound;

    EngLink_EngPkt.DigitalDataOutA   = 0;
    EngLink_EngPkt.DigitalDataOutB   = 0;
}

/* ------------------------------------------------ */
void EngLink_AddFlightData(unsigned int n, float x)
{
    EngLink_AeroPkt.FlightData[n - 1] = x;
}

/* ------------------------------------------------ */
void EngLink_Replay()
{
    Engines_EngineThrustX   = EngLink_IosPkt.PlaybackDataPkt.EngPkt.EngineThrustX;
    Engines_EngineYMT       = EngLink_IosPkt.PlaybackDataPkt.EngPkt.EngineYMT;
    Systems_EngineFireSound = EngLink_IosPkt.PlaybackDataPkt.EngPkt.EngineFireSound;

    Engines_FuelQuantityLeft  = EngLink_IosPkt.PlaybackDataPkt.EngPkt.FuelQuantityLeft;
    Engines_FuelQuantityRight = EngLink_IosPkt.PlaybackDataPkt.EngPkt.FuelQuantityRight;
}

/* ------------------------------------------------ */
void BEGIN_EngLink()
{
    unsigned int i;
	
    CmdPtr                         = 0;
    FrameNumber                    = 0;
	
    EngLink_Stopping = false;
    EngLink_NavPkt.GroundLevel = 0.0;
	
    EngLink_EngPkt.PktNumber = 0;
    EngLink_EngPkt.NumberOfEngines = 4;
	
    for (i = 0; i <= 3; i += 1)
    {
        EngLink_EngPkt.Engines[i].Thrust           = 0.0;
        EngLink_EngPkt.Engines[i].Epr              = 0.0;
        EngLink_EngPkt.Engines[i].Rpm              = 0.0;
        EngLink_EngPkt.Engines[i].FuelFlow         = 0.0;
        EngLink_EngPkt.Engines[i].Egt              = 0.0;
        EngLink_EngPkt.Engines[i].Beta             = 0.0;
        EngLink_EngPkt.Engines[i].ManifoldPressure = 0.0;
        
		EngLink_EngPkt.EngineLevers[i]             = 0.0;
		EngLink_EngPkt.ReverseLevers[i]            = 0.0;
    }
	
    EngLink_EngPkt.FuelQuantityLeft  = 0.0;
    EngLink_EngPkt.FuelQuantityRight = 0.0;

    EngLink_EngPkt.EngineThrustX     = 0.0;
    EngLink_EngPkt.EngineThrustY     = 0.0;
    EngLink_EngPkt.EngineThrustZ     = 0.0;
    EngLink_EngPkt.EnginePMT         = 0.0;
    EngLink_EngPkt.EngineRMT         = 0.0;
    EngLink_EngPkt.EngineYMT         = 0.0;

    EngLink_EngPkt.EngineType        = EngDefn_Turbofan;
    EngLink_EngPkt.EngineFireSound   = false;

    EngLink_EngPkt.DigitalDataOutA   = 0xff;
    EngLink_EngPkt.DigitalDataOutB   = 0xff;

    EngLink_OctaveMode = false;
    EngLink_Freezing   = false;
    EngLink_RemoteHold = false;
    EngLink_ReplayMode = false;
    EngLink_Reloading  = false;

    OldOctaveMode      = false;
    OldReplayMode      = false;
}
