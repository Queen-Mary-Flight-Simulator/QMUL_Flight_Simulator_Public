/* +------------------------------+---------------------------------+
   | Module      : aerolink.c     | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-02      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
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
#include <SIM/navdefn.h>
#include <SIM/protodefn.h>
#include <SIM/maths.h>
#include <SIM/weather.h>
#include <SIM/target.h>
#include <SIM/dted.h>
#include <SIM/igdefn.h>
#include <SIM/navlib.h>

#include "aerolink.h"
#include "fcs.h"
#include "model.h"
#include "systems.h"
#include "aero.h"
#include "iolib.h"
#include "nav.h"
#include "fcu.h"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

IODefn_IODataPkt            AeroLink_IOPkt1;
IODefn_IODataPkt            AeroLink_IOPkt2;
AeroDefn_AeroDataPkt        AeroLink_AeroPkt;
EngDefn_EngDataPkt          AeroLink_EngPkt;
NavDefn_NavDataPkt          AeroLink_NavPkt;
IosDefn_IosDataPkt          AeroLink_IosPkt;
ProtoDefn_ProtoDataPkt      AeroLink_ProtoPkt;
IGDefn_IGDataPkt            AeroLink_IGPkt;

bool                        AeroLink_OctaveMode;
bool                        AeroLink_ReplayMode;
unsigned int                AeroLink_CameraPosition;
bool                        AeroLink_Freezing;
bool                        AeroLink_RemoteHold;

bool                        AeroLink_Reloading;
char                        AeroLink_ReloadFilename[50];
bool                        AeroLink_Stopping;

static unsigned int                FrameNumber;
static float                       WindSpeed;
static float                       WindDirection;
static IosDefn_RestoreVectorRecord LastRestore;
static bool                        Restored;
static bool                        OldOctaveMode;
static bool                        OldReplayMode;
static unsigned int                CmdPtr;
static char                        Filename[50];
static char                        DTEDFilename[50];
static float                       AirportOffset;
/*                                            Heathrow    Gatwick  Manchester   Toulouse   Cranfield   Hong Kong
                                                 EGLL,      EGKK,      EGCC,      LFBO,       EGTC,       VHHH  */
static double AirportLatitudes[]  = {  0.0, 51.477500, 51.148056, 53.353889, 43.635000,  52.072222,  22.308889 };
static double AirportLongitudes[] = {  0.0, -0.461389, -0.190278, -2.275000,  1.367778,  -0.616667, 113.914722 };
static float  AirportCorrection[] = {  0.0,     -0.75,       0.0,      12.5,       0.0,        0.0 };

float FindAirport(double latx, double longx);
static unsigned char GetByte();
static float GetReal();
static unsigned int GetWord();
static int GetInt();
static bool GetBoolean();
static void GetFilename(char Str[]);
static void RememberState(IosDefn_RestoreVectorRecord v);
static void RestoreState(IosDefn_RestoreVectorRecord v);

/* ------------------------------------------------ */
float FindAirport(double latx, double longx)
{
    unsigned NumberOfAirports = (sizeof(AirportCorrection) / sizeof(float));
	unsigned int i;
	
    for (i=1; i<NumberOfAirports; i+=1)
	{
		if (NavLib_Distance(latx, longx, AirportLatitudes[i] * DEG2RAD, AirportLongitudes[i] * DEG2RAD) < 30000.0)
		{
		    //printf("CGI: Airport %f,%f\n", AirportLatitudes[i], AirportLongitudes[i]);
			return AirportCorrection[i];   /* up component database adjustment */
		}
	}
	
	printf("No Airport found\n");
	return 0.0;
}

/* ------------------------------------------------ */
bool AeroLink_RespondToIos()
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
                AeroLink_Stopping = true;
                return true;
                break;

            case IosDefn_SetRunHoldFreeze:
                switch (GetInt())
                {
                    case 0:
                        AeroLink_RemoteHold = false;
                        AeroLink_Freezing   = false;
                        break;
                    case 1:
                        AeroLink_RemoteHold = true;
                        AeroLink_Freezing   = false;
                        break;
                    case 2:
                        AeroLink_RemoteHold = false;
                        AeroLink_Freezing   = true;
                        break;
                }
                break;

            case IosDefn_SetOctaveMode:
                AeroLink_OctaveMode = GetBoolean();
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

            case  IosDefn_Engine4Fire:
                Systems_Failures[24] = GetBoolean();
                break;
            
            case IosDefn_Restore:
                RestoreState(AeroLink_IosPkt.RestoreVector);
                RememberState(AeroLink_IosPkt.RestoreVector);
                if (!Model_OnTheGround)
                {
                    FCS_ResetFCS();
                }
                Weather_SetWind(WindSpeed, WindDirection, true);
				AirportOffset = FindAirport(Model_Latitude, Model_Longitude);
				//printf("Airport Offset = %f\n", AirportOffset); // ***
                /* Engines_EngineModel(); */
                /* Model_FlightModel(); */
                break;

            case IosDefn_SetAircraftAltitude:
                Model_Pz = GetReal();
                break;

            case IosDefn_SetAircraftHeading:
                Model_Yaw = GetReal() + (float) AeroLink_NavPkt.MagneticVariation;
                Model_SetQuarternions();
                Model_SetDCM();
                break;

            case IosDefn_SetAircraftSpeed:
                Model_U = GetReal() / sqrt(Weather_DensityRatio);
                Model_V = 0.0;
                Model_W = Model_U * tan(Model_Alpha);
                Model_ResetFlightModel();
                break;

            case IosDefn_SetCgPosition:
                Aero_CgPosition = GetReal();
                break;

            case IosDefn_Pushback:
				Model_Pushback(Maths_Rads(GetReal() + (float) AeroLink_NavPkt.MagneticVariation), true);
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
                GetInt();
                break;

            case IosDefn_SetRMICardType:
                GetBoolean();
                break;

            case IosDefn_SetAdfDip:
                GetBoolean();
                break;

            case IosDefn_SetTurbulence:
                Weather_Turbulence_Level = GetReal();
                break;

            case IosDefn_SetWindSpeed:
                WindSpeed = GetReal();
                Weather_SetWind(WindSpeed, WindDirection, false);
                break;

            case IosDefn_SetWindDir:
                WindDirection = GetReal();
                Weather_SetWind(WindSpeed, WindDirection, false);
                break;

            case IosDefn_SetQNH:
                Weather_RegionalQNH = intround(GetReal());
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
                Model_Latitude  = GetReal();
                Model_Longitude = GetReal();
                break;

            case IosDefn_SetGroundTemperature:
                Weather_ISADeviation = GetReal();
                break;

            case IosDefn_SetTargetPosition:
                Target_TLatitude  = GetReal();
                Target_TLongitude = GetReal();
                break;

            case IosDefn_SetTargetDistance:
                Target_TargPursuit(GetReal(), Model_Latitude, Model_Longitude, Model_Pz, Model_Yaw);
                break;

            case IosDefn_SetTargetSpeed:
                Target_TU = GetReal();
                break;

            case IosDefn_SetTargetHeading:
                Target_TYaw = GetReal() + (float) AeroLink_NavPkt.MagneticVariation;
                break;

            case IosDefn_SetTargetTurnRate:
                Target_TYawDot = GetReal();
                break;

            case IosDefn_SetTargetAltitude:
                Target_TPz = GetReal();
                break;

            case IosDefn_SetTargetClimbRate:
                Target_TW = GetReal();
                break;

            case IosDefn_SetTargetConflict:
                Target_Conflict = true;
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
                if (GetBoolean())
                {
                    FCS_Engage_Autoland();
                }
                else
                {
                    FCS_Disengage_Autoland();
                }
                break;

            case IosDefn_SetKp:
                FCS_Kp = GetReal();
                break;

            case IosDefn_SetKi:
                FCS_Ki = GetReal();
                break;

            case IosDefn_SetKd:
                FCS_Kd = GetReal();
                break;

            case IosDefn_PlaybackReplay:
                AeroLink_ReplayMode = GetBoolean();
                if (AeroLink_ReplayMode != OldReplayMode)
                {
                    if (AeroLink_ReplayMode)
                    {
                        OldOctaveMode       = AeroLink_OctaveMode;
                        AeroLink_OctaveMode = false;
                    }
                    else
                    {
                        AeroLink_OctaveMode = OldOctaveMode;
                    }
                    OldReplayMode = AeroLink_ReplayMode;
                }
                break;

            case IosDefn_PlaybackCamera:
                AeroLink_CameraPosition = GetInt();
                break;

            case IosDefn_Models:
                AeroLink_Reloading = true;
                GetFilename(AeroLink_ReloadFilename);
                break;

            case IosDefn_LoadDTED:
                GetFilename(Filename);
                strcpy(DTEDFilename, "../files/");
                strcat(DTEDFilename, Filename);
                DTED_LoadDTED(DTEDFilename);
                break;

            case IosDefn_SetDate:
                GetByte(); /* ignore 32 bit arg (only needed for RPi) */
                GetByte();
                GetByte();
                GetByte();
                break;

            case IosDefn_SetFlaps:                      // added for desktop version *** 
			    Systems_FlapSetting = (int) GetReal(); // added for desktop version ***
				break;
				
			case IosDefn_SetGear:
			{   int p = GetInt();                         // added for desktop version ***
			    Systems_GearSelector = (p == 0) ? 1 : 2;  // added for desktop version ***
				break;
			}
			
			case IosDefn_SetParkBrake:
			{   int p = GetInt();                         // added for desktop version ***
			    Systems_ParkBrake = (p != 0);             // added for desktop version ***
				break;
			}
			
			case IosDefn_Autotrim:
			    //Model_SetTrim(GetReal());
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

    x      = AeroLink_IosPkt.CmdBuff[CmdPtr];
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
static void RememberState(IosDefn_RestoreVectorRecord v)
{
    memcpy(&LastRestore, &v, sizeof(IosDefn_RestoreVectorRecord));
    Restored = true;
}

/* ------------------------------------------------ */
void AeroLink_RestoreLast()
{
    if (Restored)
    {
        RestoreState(LastRestore);
        Weather_SetWind(WindSpeed, WindDirection, true);
    }
}

/* ------------------------------------------------ */
static void RestoreState(IosDefn_RestoreVectorRecord v)
{
    Model_Latitude  = v.Latitude;
    Model_Longitude = v.Longitude;
    Model_Pz        = v.Pz + Aero_CGHeight;
    Model_Pitch     = v.Pitch;
    Model_Roll      = v.Roll;
    Model_Yaw       = v.Yaw;
    Model_U         = v.U;
    Model_V         = v.V;
    Model_W         = v.W;
    Model_P         = v.P;
    Model_Q         = v.Q;
    Model_R         = v.R;
    Model_Vc        = sqrt(Model_U * Model_U + Model_V * Model_V + Model_W * Model_W);
    Model_SetQuarternions();
    Model_SetDCM();
    Model_ResetFlightModel();
}

/* ------------------------------------------------ */
void AeroLink_FormPacket()
{
    AeroLink_AeroPkt.PktNumber = FrameNumber;
    FrameNumber                = FrameNumber + 1;
    
    AeroLink_AeroPkt.Pitch     = Model_Pitch;
    AeroLink_AeroPkt.Roll      = Model_Roll;
    AeroLink_AeroPkt.Yaw       = Model_Yaw;

    AeroLink_AeroPkt.P = Model_P;
    AeroLink_AeroPkt.Q = Model_Q;
    AeroLink_AeroPkt.R = Model_R;

    AeroLink_AeroPkt.PDot = Model_PDot;
    AeroLink_AeroPkt.QDot = Model_QDot;
    AeroLink_AeroPkt.RDot = Model_RDot;

    AeroLink_AeroPkt.Vn = Model_Vn;
    AeroLink_AeroPkt.Ve = Model_Ve;
    AeroLink_AeroPkt.Vd = Model_Vd;

    AeroLink_AeroPkt.Pz        = Model_Pz;
    AeroLink_AeroPkt.Latitude  = Model_Latitude;
    AeroLink_AeroPkt.Longitude = Model_Longitude;

    AeroLink_AeroPkt.U = Model_U;
    AeroLink_AeroPkt.V = Model_V;
    AeroLink_AeroPkt.W = Model_W;

    AeroLink_AeroPkt.UDot = Model_UDot;
    AeroLink_AeroPkt.VDot = Model_VDot;
    AeroLink_AeroPkt.WDot = Model_WDot;

    AeroLink_AeroPkt.Pmt = Model_Pmt;
    AeroLink_AeroPkt.Rmt = Model_Rmt;
    AeroLink_AeroPkt.Ymt = Model_Ymt;

    AeroLink_AeroPkt.Alpha    = Model_Alpha;
    AeroLink_AeroPkt.Beta     = Model_Beta;
    AeroLink_AeroPkt.AlphaDot = Model_AlphaDot;
    AeroLink_AeroPkt.BetaDot  = Model_BetaDot;

    AeroLink_AeroPkt.Cl = Model_Cl;
    AeroLink_AeroPkt.Cd = Model_Cd;

    AeroLink_AeroPkt.Lift      = Model_Lift;
    AeroLink_AeroPkt.Thrust    = Model_Thrust;
    AeroLink_AeroPkt.Drag      = Model_Drag;
    AeroLink_AeroPkt.SideForce = Model_SideForce;

    AeroLink_AeroPkt.XForce = Model_XForce;
    AeroLink_AeroPkt.YForce = Model_YForce;
    AeroLink_AeroPkt.ZForce = Model_ZForce;

    AeroLink_AeroPkt.Vc         = Model_Vc;
    AeroLink_AeroPkt.MachNumber = Model_MachNumber;

    AeroLink_AeroPkt.WindSpeed  = WindSpeed;
    AeroLink_AeroPkt.WindDir    = WindDirection;

    AeroLink_AeroPkt.Ixx  = Aero_Ixx;
    AeroLink_AeroPkt.Iyy  = Aero_Iyy;
    AeroLink_AeroPkt.Izz  = Aero_Izz;
    AeroLink_AeroPkt.Ixz  = Aero_Ixz;
    AeroLink_AeroPkt.Iyz  = Aero_Iyz;
    AeroLink_AeroPkt.Ixy  = Aero_Ixy;
    AeroLink_AeroPkt.Mass = Aero_Mass;

    AeroLink_AeroPkt.Ex     = Model_Ex;
    AeroLink_AeroPkt.Ey     = Model_Ey;
    AeroLink_AeroPkt.Ez     = Model_Ez;
    AeroLink_AeroPkt.Wheelz = Model_Pz - Aero_CGHeight;

    AeroLink_AeroPkt.TPitch     = Target_TPitch;
    AeroLink_AeroPkt.TRoll      = Target_TRoll;
    AeroLink_AeroPkt.TYaw       = Target_TYaw;
    AeroLink_AeroPkt.TPz        = Target_TPz;
    AeroLink_AeroPkt.TLatitude  = Target_TLatitude;
    AeroLink_AeroPkt.TLongitude = Target_TLongitude;

    AeroLink_AeroPkt.Rho        = Weather_Rho;
    AeroLink_AeroPkt.OAT        = Weather_GroundTemperature;

    AeroLink_AeroPkt.TimeOfDay  = (unsigned short int) Clocks_ClockHours * 60 + Clocks_ClockMins;

    AeroLink_AeroPkt.Stalling      = Model_Stalling;
    AeroLink_AeroPkt.ConfigWarning = Systems_ConfigWarning;
    AeroLink_AeroPkt.OnTheGround   = Model_OnTheGround;
    AeroLink_AeroPkt.OctaveMode    = AeroLink_OctaveMode;

    AeroLink_AeroPkt.APSpeedMode        = FCS_SPD_Engaged();
    AeroLink_AeroPkt.APThrottlePosition = FCS_ThrottlePosition;
    AeroLink_AeroPkt.TOGAMode           = FCS_TOGAMode;
    AeroLink_AeroPkt.FlapPosition = Systems_FlapPosition;
    AeroLink_AeroPkt.FlapSetting  = Systems_FlapSetting;
    AeroLink_AeroPkt.GearPosition = Systems_GearPosition;
    AeroLink_AeroPkt.GearSelector = Systems_GearSelector;
  
    AeroLink_AeroPkt.DigitalDataOutA = IOLib_GetDigitalDataOutA();
    AeroLink_AeroPkt.DigitalDataOutB = IOLib_GetDigitalDataOutB();
    
    AeroLink_AeroPkt.Elevator     = Model_Elevator * Aero_ElevatorGain;
    AeroLink_AeroPkt.Aileron      = Model_Aileron * Aero_AileronGain;
    AeroLink_AeroPkt.Rudder       = Model_Rudder * Aero_RudderGain;

    AeroLink_AeroPkt.ElevatorTrim = Model_ElevatorTrim * Aero_ElevatorTrimGain;
    AeroLink_AeroPkt.AileronTrim  = Model_AileronTrim * Aero_AileronTrimGain;
    AeroLink_AeroPkt.RudderTrim   = Model_RudderTrim * Aero_RudderTrimGain;

    AeroLink_AeroPkt.LeftBrake    = Model_LeftBrake;
    AeroLink_AeroPkt.RightBrake   = Model_RightBrake;
    AeroLink_AeroPkt.CGHeight     = Aero_CGHeight;

    AeroLink_AeroPkt.Stopping     = AeroLink_Stopping;
	
    AeroLink_AeroPkt.TimeStamp    = ((Clocks_ClockHours * 60 + Clocks_ClockMins) * 60 +
                                     Clocks_ClockSecs) * Clocks_FrameRate + Clocks_ClockTicks;

/* ------------------------------------------------- */
    AeroLink_IGPkt.PktNumber  = FrameNumber;
    FrameNumber               = FrameNumber + 1;
    
    AeroLink_IGPkt.Pitch      = Model_Pitch;
    AeroLink_IGPkt.Roll       = Model_Roll;
    AeroLink_IGPkt.Yaw        = Model_Yaw;

    AeroLink_IGPkt.U          = Model_U;
    AeroLink_IGPkt.Rho        = Weather_Rho;

    AeroLink_IGPkt.Pz         = Model_Pz;
    AeroLink_IGPkt.Latitude   = Model_Latitude;
    AeroLink_IGPkt.Longitude  = Model_Longitude;

    AeroLink_IGPkt.Alpha      = Model_Alpha;
    AeroLink_IGPkt.Beta       = Model_Beta;
    AeroLink_IGPkt.Q          = Model_Q;

    AeroLink_IGPkt.AlphaDot   = Model_AlphaDot;
    AeroLink_IGPkt.BetaDot    = Model_BetaDot;
    AeroLink_IGPkt.UDot       = Model_UDot;

    AeroLink_IGPkt.Stopping   = AeroLink_Stopping;  /* later? */

    AeroLink_IGPkt.TPitch     = Target_TPitch;
    AeroLink_IGPkt.TRoll      = Target_TRoll;
    AeroLink_IGPkt.TYaw       = Target_TYaw;
    AeroLink_IGPkt.TPz        = Target_TPz;
    AeroLink_IGPkt.TLatitude  = Target_TLatitude;
    AeroLink_IGPkt.TLongitude = Target_TLongitude;

    AeroLink_IGPkt.Ex         = Model_Ex;
    AeroLink_IGPkt.Ey         = Model_Ey;
    AeroLink_IGPkt.Ez         = Model_Ez;

    AeroLink_IGPkt.TimeOfDay  = (unsigned short int) Clocks_ClockHours * 60 + Clocks_ClockMins;

    AeroLink_IGPkt.MagneticVariation = Nav_MagneticVariation;
    AeroLink_IGPkt.FCU_BaroPressure  = FCU_BaroPressure;
	AeroLink_IGPkt.CurrentRunway     = Nav_CurrentRunway;
    AeroLink_IGPkt.RunwayLatitude    = NavLib_Runways[Nav_CurrentRunway].RunwayLatitude;
    AeroLink_IGPkt.RunwayLongitude   = NavLib_Runways[Nav_CurrentRunway].RunwayLongitude;
    AeroLink_IGPkt.RunwayQDM         = NavLib_Runways[Nav_CurrentRunway].Qdm;
    AeroLink_IGPkt.GroundLevel       = Nav_GroundLevel;
	AeroLink_IGPkt.AirportOffset     = AirportOffset;
	
    memcpy(&AeroLink_IGPkt.CmdBuff, &AeroLink_IosPkt.CmdBuff, sizeof(AeroLink_IosPkt.CmdBuff)); /* pass over IOS commands to IG */	
}

/* ------------------------------------------------ */
void AeroLink_AddFlightData(unsigned int n, float x)
{
    AeroLink_AeroPkt.FlightData[n - 1] = x;
}

/* ------------------------------------------------ */
void AeroLink_Replay()
{
    Model_Pitch               = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Pitch;
    Model_Roll                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Roll;
    Model_Yaw                 = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Yaw;
    Model_P                   = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.P;
    Model_Q                   = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Q;
    Model_R                   = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.R;
    Model_PDot                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.PDot;
    Model_QDot                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.QDot;
    Model_RDot                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.RDot;
    Model_Vn                  = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Vn;
    Model_Ve                  = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Ve;
    Model_Vd                  = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Vd;
    Model_Pz                  = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Pz;
    Model_Latitude            = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Latitude;
    Model_Longitude           = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Longitude;
    Model_U                   = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.U;
    Model_V                   = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.V;
    Model_W                   = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.W;
    Model_UDot                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.UDot;
    Model_VDot                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.VDot;
    Model_WDot                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.WDot;
    Model_Pmt                 = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Pmt;
    Model_Rmt                 = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Rmt;
    Model_Ymt                 = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Ymt;
    Model_Alpha               = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Alpha;
    Model_Beta                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Beta;
    Model_AlphaDot            = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.AlphaDot;
    Model_BetaDot             = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.BetaDot;
    Model_Stalling            = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Stalling;
    Model_Cl                  = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Cl;
    Model_Cd                  = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Cd;
    Model_Lift                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Lift;
    Model_Thrust              = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Thrust;
    Model_Drag                = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Drag;
    Model_SideForce           = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.SideForce;
    Model_XForce              = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.XForce;
    Model_YForce              = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.YForce;
    Model_ZForce              = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.ZForce;
    Model_Vc                  = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Vc;
    Model_MachNumber          = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.MachNumber;
    WindSpeed                 = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.WindSpeed;
    WindDirection             = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.WindDir;
    Weather_Rho               = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.Rho;
    Weather_GroundTemperature = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.OAT;
    Systems_ConfigWarning     = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.ConfigWarning;
    Systems_FlapPosition      = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.FlapPosition;
    Systems_GearPosition      = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.GearPosition;
    Model_OnTheGround         = AeroLink_IosPkt.PlaybackDataPkt.AeroPkt.OnTheGround;
}

/* ------------------------------------------------ */
void BEGIN_AeroLink()
{
    CmdPtr                      = 0;
    FrameNumber                 = 0;
    WindSpeed                   = 0.0;
    WindDirection               = 0.0;
    AirportOffset               = 0.0;

    AeroLink_Freezing           = false;
    AeroLink_RemoteHold         = false;
    Restored                    = false;
    AeroLink_ReplayMode         = false;
    AeroLink_CameraPosition     = 1;
    AeroLink_OctaveMode         = false;
    OldOctaveMode               = false;
    OldReplayMode               = false;
    AeroLink_Reloading          = false;
    AeroLink_NavPkt.GroundLevel = 0.0;

    AeroLink_Stopping           = false;
    AeroLink_AeroPkt.Pitch      = 0.0;
    AeroLink_AeroPkt.Roll       = 0.0;
    AeroLink_AeroPkt.Yaw        = 0.0;

    AeroLink_AeroPkt.P = 0.0;
    AeroLink_AeroPkt.Q = 0.0;
    AeroLink_AeroPkt.R = 0.0;

    AeroLink_AeroPkt.PDot = 0.0;
    AeroLink_AeroPkt.QDot = 0.0;
    AeroLink_AeroPkt.RDot = 0.0;

    AeroLink_AeroPkt.Vn = 0.0;
    AeroLink_AeroPkt.Ve = 0.0;
    AeroLink_AeroPkt.Vd = 0.0;

    AeroLink_AeroPkt.Pz        = 0.0;
    AeroLink_AeroPkt.Latitude  = 0.0;
    AeroLink_AeroPkt.Longitude = 0.0;

    AeroLink_AeroPkt.U = 0.0;
    AeroLink_AeroPkt.V = 0.0;
    AeroLink_AeroPkt.W = 0.0;

    AeroLink_AeroPkt.UDot = 0.0;
    AeroLink_AeroPkt.VDot = 0.0;
    AeroLink_AeroPkt.WDot = 0.0;

    AeroLink_AeroPkt.Pmt = 0.0;
    AeroLink_AeroPkt.Rmt = 0.0;
    AeroLink_AeroPkt.Ymt = 0.0;

    AeroLink_AeroPkt.Alpha    = 0.0;
    AeroLink_AeroPkt.Beta     = 0.0;
    AeroLink_AeroPkt.AlphaDot = 0.0;
    AeroLink_AeroPkt.BetaDot  = 0.0;

    AeroLink_AeroPkt.Cl = 0.0;
    AeroLink_AeroPkt.Cd = 0.0;

    AeroLink_AeroPkt.Lift      = 0.0;
    AeroLink_AeroPkt.Thrust    = 0.0;
    AeroLink_AeroPkt.Drag      = 0.0;
    AeroLink_AeroPkt.SideForce = 0.0;

    AeroLink_AeroPkt.XForce = 0.0;
    AeroLink_AeroPkt.YForce = 0.0;
    AeroLink_AeroPkt.ZForce = 0.0;

    AeroLink_AeroPkt.Vc         = 0.0;
    AeroLink_AeroPkt.MachNumber = 0.0;

    AeroLink_AeroPkt.WindSpeed  = 0.0;
    AeroLink_AeroPkt.WindDir    = 0.0;

    AeroLink_AeroPkt.Ixx  = 0.0;
    AeroLink_AeroPkt.Iyy  = 0.0;
    AeroLink_AeroPkt.Izz  = 0.0;
    AeroLink_AeroPkt.Ixz  = 0.0;
    AeroLink_AeroPkt.Iyz  = 0.0;
    AeroLink_AeroPkt.Ixy  = 0.0;
    AeroLink_AeroPkt.Mass = 0.0;

    AeroLink_AeroPkt.Ex     = 0.0;
    AeroLink_AeroPkt.Ey     = 0.0;
    AeroLink_AeroPkt.Ez     = 0.0;
    AeroLink_AeroPkt.Wheelz = 0.0;

    AeroLink_AeroPkt.TPitch     = 0.0;
    AeroLink_AeroPkt.TRoll      = 0.0;
    AeroLink_AeroPkt.TYaw       = 0.0;
    AeroLink_AeroPkt.TPz        = 0.0;
    AeroLink_AeroPkt.TLatitude  = 0.0;
    AeroLink_AeroPkt.TLongitude = 0.0;

    AeroLink_AeroPkt.Rho = 1.225;
    AeroLink_AeroPkt.OAT = 15.0;

    AeroLink_AeroPkt.TimeOfDay = 0;

    AeroLink_AeroPkt.Stalling = false;
    AeroLink_AeroPkt.ConfigWarning = false;
    AeroLink_AeroPkt.OnTheGround = true;
    AeroLink_AeroPkt.OctaveMode  = false;

    AeroLink_AeroPkt.APSpeedMode = false;
    AeroLink_AeroPkt.APThrottlePosition = 0.0;
    
    AeroLink_AeroPkt.FlapPosition = 0.0;
    AeroLink_AeroPkt.FlapSetting = 0.0;
    AeroLink_AeroPkt.GearPosition = 0.0;
    AeroLink_AeroPkt.GearSelector = IODefn_GearDown;

    AeroLink_AeroPkt.DigitalDataOutA = 0xff;
    AeroLink_AeroPkt.DigitalDataOutB = 0xff;

    AeroLink_AeroPkt.Elevator = 0.0;
    AeroLink_AeroPkt.Aileron  = 0.0;
    AeroLink_AeroPkt.Rudder   = 0.0;

    AeroLink_AeroPkt.ElevatorTrim = 0.0;
    AeroLink_AeroPkt.AileronTrim  = 0.0;
    AeroLink_AeroPkt.RudderTrim   = 0.0;

    AeroLink_AeroPkt.LeftBrake = 0.0;
    AeroLink_AeroPkt.RightBrake = 0.0;
    AeroLink_AeroPkt.CGHeight = Aero_CGHeight;

    AeroLink_AeroPkt.TimeStamp   = 0;
}
