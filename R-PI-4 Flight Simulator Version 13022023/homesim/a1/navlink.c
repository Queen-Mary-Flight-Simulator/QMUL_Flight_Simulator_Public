/* +------------------------------+---------------------------------+
   | Module      : navlink.c      | Version         : 1.1           | 
   | Last Edit   : 09-01-2017     | Reference Number: 03-01-07      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/lfs/nfd/b747             |
   | Compiler    : gcc 5.4                                          |
   | OS          : Windows10                                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Management of packet transfers                   |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <SIM/iosdefn.h>
#include <SIM/iodefn.h>
#include <SIM/navdefn.h>
#include <SIM/maths.h>
#include <SIM/clocks.h>
#include <SIM/navlib.h>

#include "systems.h"
#include "nav.h"
#include "radio.h"
#include "fcu.h"
#include "nfd.h"
#include "navlink.h"
#include "radio.h"

IODefn_IODataPkt       NavLink_IOPkt1;
IODefn_IODataPkt       NavLink_IOPkt2;
AeroDefn_AeroDataPkt   NavLink_AeroPkt;
EngDefn_EngDataPkt     NavLink_EngPkt;
NavDefn_NavDataPkt     NavLink_NavPkt;
IosDefn_IosDataPkt     NavLink_IosPkt;
ProtoDefn_ProtoDataPkt NavLink_ProtoPkt;

bool                   NavLink_OctaveMode;
unsigned int           NavLink_CmdPtr;
bool                   NavLink_Freezing;
bool                   NavLink_RemoteHold;
bool                   NavLink_Reloading;
char                   NavLink_ReloadFilename[50];
bool                   NavLink_Restored;
bool                   NavLink_Stopping;

static int             CurrentRadio;
static unsigned int    FrameNumber;

static unsigned char GetByte();
static float         GetReal();
static unsigned int  GetWord();
static int           GetInt();
static bool          GetBoolean();
static void          GetFilename(char Str[]);
static void          RestoreState(IosDefn_RestoreVectorRecord v);

void NavLink_RespondToIos()
{
    char         Filename[80];
    unsigned int wpn;
    int          cmd;

    NavLink_CmdPtr = 0;

    while (1)
    {
        cmd = GetWord();

        switch (cmd)
        {
            case IosDefn_EndOfPkt:
                return;

            case IosDefn_Exit:
                NavLink_Stopping = true;
                break;

            case IosDefn_SetRunHoldFreeze:
                switch (GetInt())
                {
                    case 0:
                        NavLink_RemoteHold = false;
                        NavLink_Freezing   = false;
                        break;
                    case 1:
                        NavLink_RemoteHold = true;
                        NavLink_Freezing   = false;
                        break;
                    case 2:
                        NavLink_RemoteHold = false;
                        NavLink_Freezing   = true;
                        break;
                }
                break;

            case IosDefn_SetOctaveMode:
                NavLink_OctaveMode = GetBoolean();
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
                break;
            
            case IosDefn_Engine2Fire:
                Systems_Failures[22] = GetBoolean();
                break;
            
            case IosDefn_Engine3Fire:
                Systems_Failures[23] = GetBoolean();
                break;
            
            case IosDefn_Engine4Fire:
                Systems_Failures[24] = GetBoolean();
                break;
            
            case IosDefn_Restore:
                RestoreState(NavLink_IosPkt.RestoreVector);
                NavLink_Restored = true;
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

            case IosDefn_SetFlightControls:
                GetBoolean();
                break;
            case IosDefn_SetLeftFuelQuantity:
                GetReal();
                break;

            case IosDefn_SetRightFuelQuantity:
                GetReal();
                break;

            case IosDefn_SetSingleEngineMode:
                GetBoolean();
                break;

            case IosDefn_SetMorseMode:
                Nav_MorseChannel = GetInt();
                break;

            case IosDefn_SetRMICardType:
                Systems_AdfFixedCard = GetBoolean();
                break;

            case IosDefn_SetAdfDip:
                Systems_AdfDip = GetBoolean();
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
                Nav_RegionalQNH = (unsigned int) (GetReal());
                break;

            case IosDefn_SetMagneticVariation:
                Nav_MagneticVariation = GetReal();
                break;

            case IosDefn_SetDate:
                GetByte(); /* ignore - only used by RPi */
                GetByte();
                GetByte();
                GetByte();
                break;

            case IosDefn_SetCloudbase:
                GetReal();
                break;

            case IosDefn_SetVisibility:
                GetReal();
                break;

            case IosDefn_RePositionAircraft:
                GetReal();
                GetReal();
                break;

            case IosDefn_SetGroundTemperature:
                GetReal();
                break;

            case IosDefn_LoadTargetFile:
                GetFilename(Filename);
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

            case IosDefn_SwitchTargetOff:
                break;

            case IosDefn_SwitchHUDOff:
                break;

            case IosDefn_SetFlightPlanMode:
                Nav_NumberOfWayPoints = GetInt();
                if (Nav_NumberOfWayPoints == 0)
                {
                    Nav_FlightPlan_Segment = 0;
                }
                break;

            case IosDefn_LoadFlightPlan:
                wpn = GetInt();
                if (wpn <= Nav_NumberOfWayPoints)
                {
                    Nav_WayPoints[wpn].WayPointID[0]     = GetByte();
                    Nav_WayPoints[wpn].WayPointID[1]     = GetByte();
                    Nav_WayPoints[wpn].WayPointID[2]     = GetByte();
                    Nav_WayPoints[wpn].WayPointID[3]     = GetByte();
                    Nav_WayPoints[wpn].WayPointLatitude  = Maths_Rads(GetReal());  /* convert from degs in database */
                    Nav_WayPoints[wpn].WayPointLongitude = Maths_Rads(GetReal());  /* convert from degs in database */
                    Nav_WayPoints[wpn].WayPointAltitude  = GetInt();
                    Nav_WayPoints[wpn].WayPointSpeed     = GetInt();
                    if (wpn == Nav_NumberOfWayPoints)
                    {
                        Nav_FlightPlan_Segment = 1;
                    }

                    if (wpn == Nav_NumberOfWayPoints)
                    {
                        //for (i = 1; i <= Nav_NumberOfWayPoints; i += 1)
                        //    printf("wp%d: %c%c%c%c %f %f %d %d\n", i, Nav_WayPoints[i].WayPointID[0],
                        //           Nav_WayPoints[i].WayPointID[1], Nav_WayPoints[i].WayPointID[2],
                        //           Nav_WayPoints[i].WayPointID[3], Nav_WayPoints[i].WayPointLatitude,
                        //           Nav_WayPoints[i].WayPointLongitude, Nav_WayPoints[i].WayPointAltitude,
                        //           Nav_WayPoints[i].WayPointSpeed);
                    }
                }
                break;

            case IosDefn_SetAutopilotAltitude:
                FCU_ALT = (unsigned int) GetReal();  /* Ft */
                break;

            case IosDefn_SetAutopilotHeading:
                FCU_HDG = (unsigned int) GetReal();  /* Degrees */
                
                break;

            case IosDefn_SetAutopilotSpeed:  /* Kts */
                FCU_SPD = (unsigned int) GetReal();
                break;

            case IosDefn_SetAutopilotVSpeed:  /* fpm */
                FCU_VS = (int) GetReal();
                break;

            case IosDefn_AutopilotAltitudeOn:
                FCU_ALTKnob = (GetBoolean()) ? NavDefn_Pulled : NavDefn_Middle;
                break;

            case IosDefn_AutopilotHeadingOn:
                FCU_HDGKnob = (GetBoolean()) ? NavDefn_Pulled : NavDefn_Middle;
                break;

            case IosDefn_AutopilotSpeedOn:
                FCU_SPDKnob = (GetBoolean()) ? NavDefn_Pulled : NavDefn_Middle;
                break;

            case IosDefn_AutopilotVSpeedOn:
                FCU_VSKnob = (GetBoolean()) ? NavDefn_Pulled : NavDefn_Middle;
                break;

            case IosDefn_AutolandOn:
                FCU_APPR = GetBoolean();
                break;

            case IosDefn_Models:
                NavLink_Reloading = true;
                GetFilename(NavLink_ReloadFilename);
                break;

            case IosDefn_Radio:
                CurrentRadio = GetInt();
                break;
            
            case IosDefn_ILS1:
            case IosDefn_ILS2:
            case IosDefn_VOR1:
            case IosDefn_VOR2:
            case IosDefn_ADF1:
            case IosDefn_ADF2:
                Radio_SetRadio(CurrentRadio, cmd-IosDefn_ILS1, GetReal());
                break;
                
            default:
                return;
        }
    }
}

static unsigned char GetByte()
{
    unsigned char x;

    x              = NavLink_IosPkt.CmdBuff[NavLink_CmdPtr];
    NavLink_CmdPtr = NavLink_CmdPtr + 1;
    return x;
}

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

static bool GetBoolean()
{
    unsigned char x;

    x = GetByte();
    return(x != 0);
}

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

static void RestoreState(IosDefn_RestoreVectorRecord v)
{
    if (Nav_CurrentRunway != 0) 
    {
        Nav_GroundLevel = -(float) NavLib_Runways[Nav_CurrentRunway].Elevation;
    } 
    else 
    {
       Nav_GroundLevel = 0.0;
    }
    Radio_RestoreRMP(v);
    FCU_RestoreFCU(v);
}

void NavLink_FormPacket()
{
    NavLink_NavPkt.PktNumber         = FrameNumber;
    FrameNumber                      = FrameNumber + 1;
    
    NavLink_NavPkt.BaroPressure1     = (short unsigned int) Nav_BaroPressure1;
    NavLink_NavPkt.BaroPressure2     = (short unsigned int) Nav_BaroPressure2;
    NavLink_NavPkt.MagneticVariation = Nav_MagneticVariation;
    
    NavLink_NavPkt.RMI_Dir1          = Nav_Rmi_Dir1;
    NavLink_NavPkt.RMI_Dir2          = Nav_Rmi_Dir2;
    
    NavLink_NavPkt.CurrentRunway     = (short unsigned int) Nav_CurrentRunway;
    if (Nav_CurrentRunway > 0)
    {
        NavLink_NavPkt.RunwayLatitude  = NavLib_Runways[Nav_CurrentRunway].RunwayLatitude;
        NavLink_NavPkt.RunwayLongitude = NavLib_Runways[Nav_CurrentRunway].RunwayLongitude;
        NavLink_NavPkt.RunwayQDM       = NavLib_Runways[Nav_CurrentRunway].Qdm;
    }
    else
    {
        NavLink_NavPkt.RunwayLatitude  = 0.0;
        NavLink_NavPkt.RunwayLongitude = 0.0;
        NavLink_NavPkt.RunwayQDM       = 0.0;
    }
    NavLink_NavPkt.GroundLevel  = Nav_GroundLevel;

    NavLink_NavPkt.OuterMarker  = Nav_OuterMarker;
    NavLink_NavPkt.MiddleMarker = Nav_MiddleMarker;
    NavLink_NavPkt.InnerMarker  = Nav_InnerMarker;
    NavLink_NavPkt.MarkerTest   = Systems_MarkerTest;

    NavLink_NavPkt.Track        = Nav_Track;
    NavLink_NavPkt.TrackRange   = (short unsigned int) Nav_TrackRange;

    NavLink_NavPkt.NAV1.LocaliserError   = Nav_VOR1.LocaliserError;
    NavLink_NavPkt.NAV1.GlideSlopeError  = Nav_VOR1.GlideSlopeError;
    NavLink_NavPkt.NAV1.SlantDistance    = Nav_VOR1.SlantDistance;
    NavLink_NavPkt.NAV1.BearingToStation = Nav_VOR1.BearingToStation;
    NavLink_NavPkt.NAV1.RunwayQdm        = Nav_VOR1.RunwayQdm;
    NavLink_NavPkt.NAV1.ILSBeacon        = Nav_VOR1.IlsFrequency;
    NavLink_NavPkt.NAV1.BeaconStatus     = Nav_VOR1.BeaconStatus;

    NavLink_NavPkt.NAV2.LocaliserError   = Nav_VOR2.LocaliserError;
    NavLink_NavPkt.NAV2.GlideSlopeError  = Nav_VOR2.GlideSlopeError;
    NavLink_NavPkt.NAV2.SlantDistance    = Nav_VOR2.SlantDistance;
    NavLink_NavPkt.NAV2.BearingToStation = Nav_VOR2.BearingToStation;
    NavLink_NavPkt.NAV2.RunwayQdm        = Nav_VOR2.RunwayQdm;
    NavLink_NavPkt.NAV2.ILSBeacon        = Nav_VOR2.IlsFrequency;
    NavLink_NavPkt.NAV2.BeaconStatus     = Nav_VOR2.BeaconStatus;

    NavLink_NavPkt.ADF1.LocaliserError   = Nav_ADF1.LocaliserError;
    NavLink_NavPkt.ADF1.GlideSlopeError  = Nav_ADF1.GlideSlopeError;
    NavLink_NavPkt.ADF1.SlantDistance    = Nav_ADF1.SlantDistance;
    NavLink_NavPkt.ADF1.BearingToStation = Nav_ADF1.BearingToStation;
    NavLink_NavPkt.ADF1.RunwayQdm        = Nav_ADF1.RunwayQdm;
    NavLink_NavPkt.ADF1.ILSBeacon        = Nav_ADF1.IlsFrequency;
    NavLink_NavPkt.ADF1.BeaconStatus     = Nav_ADF1.BeaconStatus;

    NavLink_NavPkt.ADF2.LocaliserError   = Nav_ADF2.LocaliserError;
    NavLink_NavPkt.ADF2.GlideSlopeError  = Nav_ADF2.GlideSlopeError;
    NavLink_NavPkt.ADF2.SlantDistance    = Nav_ADF2.SlantDistance;
    NavLink_NavPkt.ADF2.BearingToStation = Nav_ADF2.BearingToStation;
    NavLink_NavPkt.ADF2.RunwayQdm        = Nav_ADF2.RunwayQdm;
    NavLink_NavPkt.ADF2.ILSBeacon        = Nav_ADF2.IlsFrequency;
    NavLink_NavPkt.ADF2.BeaconStatus     = Nav_ADF2.BeaconStatus;

    NavLink_NavPkt.ILS1.LocaliserError   = Nav_ILS1.LocaliserError;
    NavLink_NavPkt.ILS1.GlideSlopeError  = Nav_ILS1.GlideSlopeError;
    NavLink_NavPkt.ILS1.SlantDistance    = Nav_ILS1.SlantDistance;
    NavLink_NavPkt.ILS1.BearingToStation = Nav_ILS1.BearingToStation;
    NavLink_NavPkt.ILS1.RunwayQdm        = Nav_ILS1.RunwayQdm;
    NavLink_NavPkt.ILS1.ILSBeacon        = Nav_ILS1.IlsFrequency;
    NavLink_NavPkt.ILS1.BeaconStatus     = Nav_ILS1.BeaconStatus;

    NavLink_NavPkt.WayPoint.LocaliserError   = Nav_WayPoint.LocaliserError;
    NavLink_NavPkt.WayPoint.GlideSlopeError  = Nav_WayPoint.GlideSlopeError;
    NavLink_NavPkt.WayPoint.SlantDistance    = Nav_WayPoint.SlantDistance;
    NavLink_NavPkt.WayPoint.BearingToStation = Nav_WayPoint.BearingToStation;
    NavLink_NavPkt.WayPoint.RunwayQdm        = Nav_WayPoint.RunwayQdm;
    NavLink_NavPkt.WayPoint.ILSBeacon        = Nav_WayPoint.IlsFrequency;
    NavLink_NavPkt.WayPoint.BeaconStatus     = Nav_WayPoint.BeaconStatus;

    NavLink_NavPkt.MorseChannel = Nav_MorseChannel;
    switch (Nav_MorseChannel)
    {
        default:
            NavLink_NavPkt.MorseIdent[0] = '\0';
            break;
        case 1:
            memcpy(NavLink_NavPkt.MorseIdent, NavLib_Beacons[Nav_ILS1.SelectedBeacon].Ident, 4);
            break;
        case 2:
            memcpy(NavLink_NavPkt.MorseIdent, NavLib_Beacons[Nav_VOR1.SelectedBeacon].Ident, 4);
            break;
        case 3:
            memcpy(NavLink_NavPkt.MorseIdent, NavLib_Beacons[Nav_VOR2.SelectedBeacon].Ident, 4);
            break;
        case 4:
            memcpy(NavLink_NavPkt.MorseIdent, NavLib_Beacons[Nav_ADF1.SelectedBeacon].Ident, 4);
            break;
        case 5:
            memcpy(NavLink_NavPkt.MorseIdent, NavLib_Beacons[Nav_ADF2.SelectedBeacon].Ident, 4);
            break;
    }
  
    NavLink_NavPkt.HSI_Crs = (short int) Nav_HSI_Crs;
    NavLink_NavPkt.HSI_Hdg = (short int) Nav_HSI_Hdg;
    NavLink_NavPkt.VOR_Obs = (short int) Nav_VOR_Obs;

    Radio_SaveRMP(&NavLink_NavPkt);

    NavLink_NavPkt.FCU_BaroPressure = FCU_BaroPressure;
    NavLink_NavPkt.FCU_HDG          = FCU_HDG;
    NavLink_NavPkt.FCU_ALT          = FCU_ALT;
    NavLink_NavPkt.FCU_SPD          = FCU_SPD;
    NavLink_NavPkt.FCU_VS           = FCU_VS;

    NavLink_NavPkt.FCU_FD           = FCU_FD;
    NavLink_NavPkt.FCU_LS           = FCU_LS;
    NavLink_NavPkt.FCU_LOC          = FCU_LOC;
    NavLink_NavPkt.FCU_AP1          = FCU_AP1;
    NavLink_NavPkt.FCU_AP2          = FCU_AP2;
    NavLink_NavPkt.FCU_ATHR         = FCU_ATHR;
    NavLink_NavPkt.FCU_EXPED        = FCU_EXPED;
    NavLink_NavPkt.FCU_APPR         = FCU_APPR;
    NavLink_NavPkt.FCU_SPD_MACH     = FCU_SPD_MACH_Button;
    NavLink_NavPkt.FCU_HDG_TRK      = FCU_HDG_TRK_Button;

    NavLink_NavPkt.FCU_BaroHg       = FCU_BaroHg;
    NavLink_NavPkt.FCU_BaroKnob     = FCU_BaroKnob;
    NavLink_NavPkt.FCU_HDGKnob      = FCU_HDGKnob;
    NavLink_NavPkt.FCU_ALTKnob     = FCU_ALTKnob;
    NavLink_NavPkt.FCU_SPDKnob     = FCU_SPDKnob;
    NavLink_NavPkt.FCU_VSKnob      = FCU_VSKnob;
    NavLink_NavPkt.FCU_Metric_Button  = FCU_Metric_Button;
    
    NavLink_NavPkt.NavAid1          = FCU_NavSwitch1;
    NavLink_NavPkt.NavAid2          = FCU_NavSwitch2;
    NavLink_NavPkt.Mode             = FCU_ModeSelector;
    NavLink_NavPkt.Data             = FCU_DataMode;
}

void BEGIN_NavLink()
{
    unsigned int i;
    
    NavLink_CmdPtr     = 0;
    FrameNumber        = 0;
    
    NavLink_Stopping = false;
    
    NavLink_NavPkt.PktNumber = 0;

    NavLink_NavPkt.BaroPressure1 = 1013;
    NavLink_NavPkt.BaroPressure2 = 1013;
    NavLink_NavPkt.MagneticVariation = 0.0;

    NavLink_NavPkt.RMI_Dir1 = 0.0;
    NavLink_NavPkt.RMI_Dir2 = 0.0;

    NavLink_NavPkt.CurrentRunway = 0;
    NavLink_NavPkt.RunwayQDM = 0;
    NavLink_NavPkt.RunwayLatitude = 0.0;
    NavLink_NavPkt.RunwayLongitude = 0.0;
    NavLink_NavPkt.GroundLevel = 0.0;

    NavLink_NavPkt.OuterMarker = false;
    NavLink_NavPkt.MiddleMarker = false;
    NavLink_NavPkt.InnerMarker = false;
    NavLink_NavPkt.MarkerTest = false;

    NavLink_NavPkt.Track = 0.0;
    NavLink_NavPkt.TrackRange = 80;

    NavLink_NavPkt.NAV1.LocaliserError   = 0.0;
    NavLink_NavPkt.NAV1.GlideSlopeError  = 0.0;
    NavLink_NavPkt.NAV1.SlantDistance    = 0.0;
    NavLink_NavPkt.NAV1.BearingToStation = 0.0;
    NavLink_NavPkt.NAV1.RunwayQdm        = 0.0;
    NavLink_NavPkt.NAV1.ILSBeacon        = false;
    NavLink_NavPkt.NAV1.BeaconStatus     = false;

    NavLink_NavPkt.NAV2.LocaliserError   = 0.0;
    NavLink_NavPkt.NAV2.GlideSlopeError  = 0.0;
    NavLink_NavPkt.NAV2.SlantDistance    = 0.0;
    NavLink_NavPkt.NAV2.BearingToStation = 0.0;
    NavLink_NavPkt.NAV2.RunwayQdm        = 0.0;
    NavLink_NavPkt.NAV2.ILSBeacon        = false;
    NavLink_NavPkt.NAV2.BeaconStatus     = false;

    NavLink_NavPkt.ADF1.LocaliserError   = 0.0;
    NavLink_NavPkt.ADF1.GlideSlopeError  = 0.0;
    NavLink_NavPkt.ADF1.SlantDistance    = 0.0;
    NavLink_NavPkt.ADF1.BearingToStation = 0.0;
    NavLink_NavPkt.ADF1.RunwayQdm        = 0.0;
    NavLink_NavPkt.ADF1.ILSBeacon        = false;
    NavLink_NavPkt.ADF1.BeaconStatus     = false;

    NavLink_NavPkt.ADF2.LocaliserError   = 0.0;
    NavLink_NavPkt.ADF2.GlideSlopeError  = 0.0;
    NavLink_NavPkt.ADF2.SlantDistance    = 0.0;
    NavLink_NavPkt.ADF2.BearingToStation = 0.0;
    NavLink_NavPkt.ADF2.RunwayQdm        = 0.0;
    NavLink_NavPkt.ADF2.ILSBeacon        = false;
    NavLink_NavPkt.ADF2.BeaconStatus     = false;

    NavLink_NavPkt.ILS1.LocaliserError   = 0.0;
    NavLink_NavPkt.ILS1.GlideSlopeError  = 0.0;
    NavLink_NavPkt.ILS1.SlantDistance    = 0.0;
    NavLink_NavPkt.ILS1.BearingToStation = 0.0;
    NavLink_NavPkt.ILS1.RunwayQdm        = 0.0;
    NavLink_NavPkt.ILS1.ILSBeacon        = false;
    NavLink_NavPkt.ILS1.BeaconStatus     = false;

    NavLink_NavPkt.WayPoint.LocaliserError   = 0.0;
    NavLink_NavPkt.WayPoint.GlideSlopeError  = 0.0;
    NavLink_NavPkt.WayPoint.SlantDistance    = 0.0;
    NavLink_NavPkt.WayPoint.BearingToStation = 0.0;
    NavLink_NavPkt.WayPoint.RunwayQdm        = 0.0;
    NavLink_NavPkt.WayPoint.ILSBeacon        = false;
    NavLink_NavPkt.WayPoint.BeaconStatus     = false;

    NavLink_NavPkt.MorseChannel = 0;
    NavLink_NavPkt.MorseIdent[0] = '\0';
    
    NavLink_NavPkt.HSI_Crs = 0;
    NavLink_NavPkt.HSI_Hdg = 0;
    NavLink_NavPkt.VOR_Obs = 0;

    for (i=0; i<=0; i+=1)  /* only 1 radio for the LFS */
    {
        NavLink_NavPkt.SavedRadios[i].NavVOR.Active  = 112300;
        NavLink_NavPkt.SavedRadios[i].NavVOR.Stby    = 116400;
        NavLink_NavPkt.SavedRadios[i].NavILS.Active  = 108900;
        NavLink_NavPkt.SavedRadios[i].NavILS.Stby    = 111700;
        NavLink_NavPkt.SavedRadios[i].NavADF.Active  = 200;
        NavLink_NavPkt.SavedRadios[i].NavADF.Stby    = 1157;
        NavLink_NavPkt.SavedRadios[i].ComHF1.Active  = 3100;
        NavLink_NavPkt.SavedRadios[i].ComHF1.Stby    = 30000;
        NavLink_NavPkt.SavedRadios[i].ComHF2.Active  = 3200;
        NavLink_NavPkt.SavedRadios[i].ComHF2.Stby    = 30000;
        NavLink_NavPkt.SavedRadios[i].ComVHF1.Active = 120100;
        NavLink_NavPkt.SavedRadios[i].ComVHF1.Stby   = 136975;
        NavLink_NavPkt.SavedRadios[i].ComVHF2.Active = 120200;
        NavLink_NavPkt.SavedRadios[i].ComVHF2.Stby   = 136975;
        NavLink_NavPkt.SavedRadios[i].ComVHF3.Active = 120300;
        NavLink_NavPkt.SavedRadios[i].ComVHF3.Stby   = 136975;
        NavLink_NavPkt.SavedRadios[i].CrsKnob        = 100;
    }
    
    NavLink_NavPkt.FCU_BaroPressure = 1013;
    NavLink_NavPkt.FCU_HDG = 0;
    NavLink_NavPkt.FCU_ALT = 0;
    NavLink_NavPkt.FCU_SPD = 0;
    NavLink_NavPkt.FCU_VS = 0;

    NavLink_NavPkt.FCU_FD = false;
    NavLink_NavPkt.FCU_LS = false;
    NavLink_NavPkt.FCU_LOC = false;
    NavLink_NavPkt.FCU_AP1 = false;
    NavLink_NavPkt.FCU_AP2 = false;
    NavLink_NavPkt.FCU_ATHR = false;
    NavLink_NavPkt.FCU_EXPED = false;
    NavLink_NavPkt.FCU_APPR = false;
    NavLink_NavPkt.FCU_SPD_MACH = false;
    NavLink_NavPkt.FCU_HDG_TRK = false;

    NavLink_NavPkt.FCU_BaroHg = false;
    NavLink_NavPkt.FCU_BaroKnob = NavDefn_Middle;
    NavLink_NavPkt.FCU_HDGKnob = NavDefn_Middle;
    NavLink_NavPkt.FCU_ALTKnob = NavDefn_Middle;
    NavLink_NavPkt.FCU_SPDKnob = NavDefn_Middle;
    NavLink_NavPkt.FCU_VSKnob = NavDefn_Middle;

    NavLink_NavPkt.NavAid1 = NavDefn_NavOFF;
    NavLink_NavPkt.NavAid2 = NavDefn_NavOFF;
    NavLink_NavPkt.Mode = NavDefn_ModeILS;
    NavLink_NavPkt.Data = NavDefn_DataOFF;

    NavLink_Reloading  = false;
    NavLink_OctaveMode = false;
    NavLink_Restored   = false;

    CurrentRadio = 0;
}
