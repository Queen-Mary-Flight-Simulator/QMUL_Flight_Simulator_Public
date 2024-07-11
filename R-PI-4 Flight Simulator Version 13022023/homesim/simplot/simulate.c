#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <SIM/iodefn.h>
#include <SIM/navdefn.h>
#include <SIM/weather.h>
#include <SIM/clocks.h>
#include <SIM/maths.h>

#include "model.h"
#include "engines.h"
#include "aero.h"
#include "systems.h"
#include "fcs.h"
#include "simulate.h"
#include "aerolink.h"
#include "englink.h"
#include "gear.h"
#include "stab.h"
#include "iolib.h"

unsigned int       Simulate_SimSteps;
bool               Simulate_AutoTrimMode;
float              Simulate_Altitude;
float              Simulate_Speed;
float              Simulate_Heading;
float              Simulate_VSpeed;
float              Simulate_Elevator;
float              Simulate_Aileron;
float              Simulate_Rudder;
float              Simulate_EngineLever;
bool               Simulate_IAS;
float              Simulate_Spoiler;
float              Simulate_ParkBrake;
float              Simulate_LeftBrake;
float              Simulate_RightBrake;
float              Simulate_Latitude;
float              Simulate_Longitude;
IosDefn_PlaybackDataPktRecord *Simulate_FlightDataBlk;
int                Simulate_FlightDataSize;
float              Simulate_Distance;
float              Simulate_Mass;

unsigned short int Simulate_FCU_HDG      = 0;
unsigned short int Simulate_FCU_ALT      = 0;
unsigned short int Simulate_FCU_SPD      = 0;
short int          Simulate_FCU_VS       = 0;
bool               Simulate_FCU_HDG_Hold = false;
bool               Simulate_FCU_ALT_Hold = false;
bool               Simulate_FCU_SPD_Hold = false;
bool               Simulate_FCU_VS_Hold  = false;

unsigned int       FrameNumber;
InputType          Inputs[Simulate_MaxInputs+1];
bool               ParkBrakeOn;

void AutoTrim(float Height, float Speed, float VSpeed, float *TrimAlpha, float *TrimPitch, float *De, float *Dp);
float TAS(float v);
void ReadBlk(IosDefn_PlaybackDataPktRecord *dblk, IosDefn_PlaybackDataPktRecord *sblk, unsigned int slot);

/* -------------------------------------------------------------------- */
extern void Simulate_SetInputs(unsigned int inputno, unsigned int s, float d, float w, float a)
{
    Inputs[inputno].active    = true;
    Inputs[inputno].shape     = s;
    Inputs[inputno].delay     = d;
    Inputs[inputno].width     = w;
    Inputs[inputno].amplitude = a;
}

/* -------------------------------------------------------------------- */
void Simulate_Simulate(PtrProc PrintOutputs)
{
    float        TrimAlpha;
    float        TrimPitch;
    float        de;
    float        da;
    float        dr;
    float        dp;
    //int          PktNo = 0;
	
    Model_ResetFlightModel();
    Weather_WeatherModel(false, Simulate_Altitude, 0); /* set rho and air density ratio */

    if (Simulate_IAS)
    {
        Simulate_Speed = TAS(Simulate_Speed);
    }
    
    Aero_Mass = Simulate_Mass;

    if (Simulate_AutoTrimMode)
    {
        AutoTrim(Simulate_Altitude, Simulate_Speed, Simulate_VSpeed, &TrimAlpha, &TrimPitch, &de, &dp);
        Model_Alpha = TrimAlpha;
        Model_Pitch = TrimPitch;
        printf("trim: alpha=%5.2f deg, pitch=%5.2f deg, de=%5.2f deg, dp=%5.2f EPR=%5.2f\n", Maths_Degrees(TrimAlpha), Maths_Degrees(TrimPitch), Maths_Degrees(de), dp, Engines_Engines[0].Epr);
        de = de / Aero_ElevatorGain;  /* normalise for IOLib */
        stability_derivatives();
    }
    else
    {
        Model_Alpha = 0.0;
        Model_Pitch = 0.0;
        de          = Simulate_Elevator;
        dp          = Simulate_EngineLever;
    }

	da = Simulate_Aileron;
	dr = Simulate_Rudder;

	Model_Pz = Simulate_Altitude;
	Model_U  = Simulate_Speed;
	Model_V  = 0.0;
	Model_W  = Model_U * tan(Model_Alpha);
	Model_ResetFlightModel();
	Model_Vc   = sqrt(Model_U * Model_U + Model_V * Model_V + Model_W * Model_W);
	Model_Yaw  = Simulate_Heading;
	Model_Vn   = Model_U;
	Model_Ve   = 0.0;
	Model_Roll = 0.0;
	Model_P    = 0.0;
	//Model_Spoiler = Simulate_Spoiler;
	Model_Latitude = Simulate_Latitude;
	Model_Longitude = Simulate_Longitude;
	
    Model_SetQuarternions();
    Model_SetDCM();

    for (FrameNumber=1; FrameNumber<=Simulate_SimSteps; FrameNumber+=1)
    {
        unsigned int p;
        
        for (p=1; p<=(unsigned int) Simulate_MaxInputs; p+=1)
        {
            //if (Simulate_FlightDataSize > 0)
            //{
			//    IosDefn_PlaybackDataPktRecord *a = NULL;
			//	
            //    ReadBlk(a, Simulate_FlightDataBlk, PktNo);
            //    AeroLink_AeroPkt.Elevator = a->AeroPkt.Elevator;
            //    AeroLink_AeroPkt.Aileron = a->AeroPkt.Aileron;
            //    AeroLink_AeroPkt.Rudder = a->AeroPkt.Rudder;
            //    EngLink_EngPkt.EngineLevers[0] = a->EngPkt.EngineLevers[0];
			//}
			
            //else if (Inputs[p].active)
			if (Inputs[p].active)
            {
                unsigned int StartFrame = (unsigned int) (Inputs[p].delay * 50.0);
                unsigned int EndFrame   = StartFrame + (unsigned int) (Inputs[p].width * 50.0);
                unsigned int MidFrame   = (StartFrame + EndFrame) / 2;
                float        x = 0.0;
                
                switch (p)
                {
                    case 1:
                        x = de;
                        break;
                    case 2:
                        x = da;
                        break;
                    case 3:
                        x = dr;
                        break;
                    case 4:
                        x = dp;
                        break;
                    case 5:
                        x = Simulate_Spoiler;
                        break;
                    case 6:
                        x = Simulate_ParkBrake;
                        break;
                    case 7:
                        x = Simulate_LeftBrake;
                        break;
                    case 8:
                        x = Simulate_RightBrake;
                        break;
                }
                
                switch (Inputs[p].shape)
                {
                    case Simulate_Step_Shape:
                        if (FrameNumber >= StartFrame)
                        {
                            x += Inputs[p].amplitude;
                        }
                        break;

                    case Simulate_Pulse_Shape:
                        if (FrameNumber >= StartFrame && FrameNumber <= EndFrame)
                        {
                            x += Inputs[p].amplitude;
                        }
                        break;

                    case Simulate_Doublet_Shape:
                        if (FrameNumber >= StartFrame && FrameNumber <= MidFrame)
                        {
                            x += Inputs[p].amplitude;
                        }
                        else if (FrameNumber >= MidFrame && FrameNumber <= EndFrame)
                        {
                            x -= Inputs[p].amplitude;
                        }
                        break;

                    case Simulate_Ramp_Shape:
                        if (FrameNumber >= StartFrame && FrameNumber <= MidFrame)
                        {
                            x *= (float) (FrameNumber - StartFrame) / (float) (MidFrame - StartFrame);
                        }
                        else if (FrameNumber >= MidFrame && FrameNumber <= EndFrame)
                        {
                            x *= (float) (EndFrame - FrameNumber) / (float) (EndFrame - MidFrame);
                        }
                        break;

                    case Simulate_Sine_Shape:
                        if (FrameNumber >= StartFrame && FrameNumber <= EndFrame)
                        {
                            float wt = (float) (FrameNumber - StartFrame) / (float) (EndFrame - StartFrame) * 2.0 * M_PI;

                            x *= sin(wt);
                        }
                        break;

                    default:
                        break;
                }
                
                switch (p)
                {
                    case 1:
                        AeroLink_AeroPkt.Elevator = x;
                        break;
                    case 2:
                        AeroLink_AeroPkt.Aileron = x;
                        break;
                    case 3:
                        AeroLink_AeroPkt.Rudder = x;
                        break;
                    case 4:
                        EngLink_EngPkt.EngineLevers[0] = x;
                        break;
                    case 5:
                        //AeroLink_AeroPkt.Spoiler = x; // *** to be written
                        break;
                    case 6:
                        ParkBrakeOn = (x > 0.5);
                        break;
                    case 7:
                        if (ParkBrakeOn)
                        {
                            AeroLink_AeroPkt.LeftBrake = 1.0;
                        }
                        else
                        {
                            AeroLink_AeroPkt.LeftBrake = x;
                        }
                        break;
                    case 8:
                        if (ParkBrakeOn)
                        {
                            AeroLink_AeroPkt.RightBrake = 1.0;
                        }
                        else
                        {
                            AeroLink_AeroPkt.RightBrake = x;
                        }
                        break;
                }
            }
            else
            {
                switch (p)
                {
                    case 1:
                        AeroLink_AeroPkt.Elevator = de;
                        break;
                    case 2:
                        AeroLink_AeroPkt.Aileron = da;
                        break;
                    case 3:
                        AeroLink_AeroPkt.Rudder = dr;
                        break;
                    case 4:
                        EngLink_EngPkt.EngineLevers[0] = dp;
                        break;
                    case 5:
                        //AeroLink_AeroPkt.Spoiler = Simulate_Spoiler; // *** to be written
                        break;
                    case 7:
                        if (ParkBrakeOn)
                        {
                            AeroLink_AeroPkt.LeftBrake = 1.0;
                        }
                        else
                        {
                            AeroLink_AeroPkt.LeftBrake = Simulate_LeftBrake;
                        }
                        break;
                    case 8:
                        if (ParkBrakeOn)
                        {
                            AeroLink_AeroPkt.RightBrake = 1.0;
                        }
                        else
                        {
                            AeroLink_AeroPkt.RightBrake = Simulate_RightBrake;
                        }
                        break;
                }
            }
        }

        /* simulate the packet transfers */
        EngLink_AeroPkt.Pz                      = Model_Pz;
        EngLink_AeroPkt.OnTheGround             = Model_OnTheGround;
        EngLink_AeroPkt.APSpeedMode             = FCS_APSpeedMode;
        EngLink_AeroPkt.APThrottlePosition      = FCS_ThrottlePosition;
        EngLink_AeroPkt.MachNumber              = Model_MachNumber;

        AeroLink_EngPkt.EngineThrustX           = Engines_EngineThrustX;
        AeroLink_EngPkt.EngineThrustY           = Engines_EngineThrustY;
        AeroLink_EngPkt.EngineThrustZ           = Engines_EngineThrustZ;
        AeroLink_EngPkt.EnginePMT               = Engines_EnginePMT;
        AeroLink_EngPkt.EngineRMT               = Engines_EngineRMT;
        AeroLink_EngPkt.EngineYMT               = Engines_EngineYMT;

        AeroLink_NavPkt.FCU_ALT                 = Simulate_FCU_ALT;  /* override NavPkt for simulate settings */
        AeroLink_NavPkt.FCU_HDG                 = Simulate_FCU_HDG;
        AeroLink_NavPkt.FCU_SPD                 = Simulate_FCU_SPD;
        AeroLink_NavPkt.FCU_VS                  = Simulate_FCU_VS;
        AeroLink_NavPkt.FCU_ALT_Hold            = Simulate_FCU_ALT_Hold;
        AeroLink_NavPkt.FCU_HDG_Hold            = Simulate_FCU_HDG_Hold;
        AeroLink_NavPkt.FCU_SPD_Hold            = Simulate_FCU_SPD_Hold;
        AeroLink_NavPkt.FCU_VS_Hold             = Simulate_FCU_VS_Hold;
        AeroLink_NavPkt.FCU_AP1                 = true;
        AeroLink_NavPkt.FCU_AP2                 = true;
        AeroLink_NavPkt.FCU_ATHR                = true;
        AeroLink_NavPkt.FCU_SPD_MACH            = true;
        AeroLink_NavPkt.FCU_Metric_ALT          = false;
        AeroLink_NavPkt.FCU_APPR                = false;
        AeroLink_NavPkt.WayPoint.BeaconStatus   = false;
        AeroLink_NavPkt.FCU_LOC                 = false;
        AeroLink_NavPkt.FCU_FD                  = false;
        AeroLink_NavPkt.ILS1.ILSBeacon          = false;
        AeroLink_NavPkt.ILS1.RunwayQdm          = 0.0;
        AeroLink_NavPkt.GroundLevel             = 0.0;
        AeroLink_NavPkt.ILS1.LocaliserError     = 0.0;
        AeroLink_NavPkt.ILS1.GlideSlopeError    = 0.0;
        AeroLink_NavPkt.MagneticVariation       = 0.0;
        AeroLink_NavPkt.WayPoint.RunwayQdm      = 0.0;
        AeroLink_NavPkt.WayPoint.LocaliserError = 0.0;
        AeroLink_NavPkt.HSI_Crs                 = 0;
        AeroLink_NavPkt.NAV1.LocaliserError     = 0.0;

        Clocks_UpdateClocks(false, false);

        Weather_WeatherModel(true, Model_Pz, Model_U);
        Engines_EngineModel(false);
        Model_FlightModel();

        Simulate_Distance = Maths_Integrate(Simulate_Distance, Model_U);
		
        PrintOutputs(FrameNumber);
    }
    PrintOutputs(0);
}

/* -------------------------------------------------------------------- */
void AutoTrim(float h, float u, float VSpeed, float *TrimAlpha, float *TrimPitch, float *Elevator, float *PowerLever)
{
    double       Cl;
    double       ClTail;
    double       Lift;
    double       Cd;
    double       Drag;
    double       Vc2;
    double       Thrust;
    double       Alpha;
    double       AlphaWing;
    double       Cm0;
    double       CmAlpha;
    double       CmDe;
    double       de;
    double       dp;
    double       xforce;
    double       zforce;
    double       w;
    double       Gamma;
    double       Pitch;
    unsigned int tcount;

    FCS_AutoTrimming = true;  /* inhibit transients in engine model */

    de          = 0.0;
    dp          = 0.6;
    Alpha       = 0.0;
    tcount      = 0;

    Model_Pz    = h;
    Model_U     = u;
    Model_Flaps = Systems_GetFlapPosition();
    Model_Gear  = Systems_GetGearPosition();
    Gamma       = atan2(VSpeed, Model_U);

    do
    {
        Weather_WeatherModel(false, h, u);  /* set rho and air density ratio */
        Model_Pz = h;
        Model_U = u;

        Model_Alpha = (float) Alpha;                       /* next lines identical to longitudinal force computation in model.c */
        AlphaWing = Alpha + (double) Aero_WingIncidence;
        Model_AlphaWing = (float) AlphaWing;
        Pitch = Alpha + Gamma;
        Model_Pitch = (float) Pitch;
        w = (double) u * tan(Alpha);
        Vc2 = (double) u * (double) u + (double) w * (double) w;

        Model_MachNumber = sqrt(Vc2) / Weather_SpeedOfSound;
        EngLink_AeroPkt.MachNumber = Model_MachNumber;     /* machNumber and Lever needed by engine model */
        EngLink_EngPkt.EngineLevers[0] = (float) dp;
        Engines_EngineModel(false);                        /* compute thrust from engine model */
        Thrust = (double) Engines_EngineThrustX;

        Cd = (double) Aero_AeroCd();
        Drag = 0.5 * (double) Weather_Rho * Vc2 * (double) Aero_s * Cd;

        Cl = (double) Aero_AeroCl();
        ClTail = (double) Aero_AeroClTail();
        Lift = 0.5 * (double) Weather_Rho * Vc2 * (double) Aero_s * (Cl + ClTail * de) + (double) Engines_EngineThrustZ;

        xforce = Thrust - Drag * cos(Alpha) + Lift * sin(Alpha) - (double) Aero_Mass * (double) Model_G * sin(Pitch) + Gear_Fx;
        zforce = -Lift * cos(Alpha) - Drag * sin(Alpha) + (double) Aero_Mass * (double) Model_G * cos(Pitch) + Gear_Fz;

        dp = dp - xforce / (double) 10E6;
        if (dp < 0.2)
		{
		    dp = 0.2;
		}
		else if (dp > 1.0)
		{
		    dp = 1.0;
		}
		
        Alpha = Alpha + zforce / (double) 10E8;
		if (Alpha < -0.5)
		{
		    Alpha = -0.5;
		}
		else if (Alpha > 0.5)
		{
		    Alpha = 0.5;
		}
        AlphaWing = Alpha + (double) Aero_WingIncidence;

        Cm0 = (double) Aero_AeroCm0();                     /* next lines identical to pitching moment computation in model.c */
        CmAlpha = (double) Aero_AeroCmAlpha();
        CmDe = (double) Aero_AeroCmDe();

		de = ((-Lift * ((double) Aero_CgPosition - 0.25) * (double) Aero_CBar * cos(Alpha) -
		     Drag * ((double) Aero_CgPosition - 0.25) * (double) Aero_CBar * sin(Alpha) -
			 (double) Engines_EnginePMT) / (0.5 * (double) Weather_Rho * Vc2 * (double) Aero_s * (double) Aero_CBar) - Cm0 -CmAlpha * AlphaWing) / CmDe;

        tcount = tcount + 1;
        if (tcount > 100000)
        {
            printf("Trim failure\n");
            printf("xforce=%7.0f zforce=%7.0f alpha=%5.2f de=%5.2f dp=%5.2f\n", xforce, zforce, Alpha, de, dp);
            printf("Thrust=%7.0f Drag=%7.0f Lift=%7.0f mg=%7.0f\n", Thrust, Drag, Lift, (double) Aero_Mass * (double) Model_G);
            exit(1);
        }
        //printf("%i %f %f %f %f\n", tcount, xforce, zforce, de, dp); // ***
    } while (!((fabs(xforce) < 0.5) && (fabs(zforce) < 0.5)));

    printf("Autotrim: converged after %d steps\n", tcount);
    printf("Thrust=%7.0f Drag=%7.0f Lift=%7.0f mg=%7.0f\n", Thrust, Drag, Lift, (double) Aero_Mass * (double) Model_G);
	
    *TrimAlpha = (float) Alpha;
    *TrimPitch = (float) Alpha + Gamma;
    *Elevator = (float) de;
    *PowerLever = (float) dp;
    FCS_AutoTrimming     = false;
}

/* -------------------------------------------------------------------- */
float TAS(float v)
{
    return v / sqrt(Weather_DensityRatio);
}

/* --------------------------------------------- */
void ReadBlk(IosDefn_PlaybackDataPktRecord *dblk, IosDefn_PlaybackDataPktRecord *sblk, unsigned int slot)
{
    /* dest, source, size */
    memcpy(dblk, sblk + slot, sizeof(IosDefn_PlaybackDataPktRecord));
}

/* -------------------------------------------------------------------- */
void BEGIN_Simulate()
{
    unsigned int i;
    unsigned int MemSize;
    unsigned int MaxPlotRecords;
    
    Simulate_SimSteps        = 12 * 50;
    Simulate_AutoTrimMode    = false;
    Simulate_Altitude        = -5000.0 * 0.3048;
    Simulate_Speed           = 180.0 * 0.5144;
    Simulate_VSpeed          = 0.0;
    Simulate_Heading         = 0.0;
    Simulate_Elevator        = 0.0;
    Simulate_Aileron         = 0.0;
    Simulate_Rudder          = 0.0;
    Simulate_EngineLever     = 0.0;
    Simulate_IAS             = false;
    Simulate_Spoiler         = 0.0;
    Simulate_ParkBrake       = 0.0;
    Simulate_LeftBrake       = 0.0;
    Simulate_RightBrake      = 0.0;
    Simulate_Latitude        = 1.0;
    Simulate_Longitude       = 0.0;
    Simulate_FlightDataSize  = 0;
    Simulate_Mass            = Aero_Mass;
	
    for (i = 0; i <= 3; i += 1)  /* start with engines running */
    {
        Engines_Engines[i].Thrust   = 0.0;
        Engines_Engines[i].Epr      = 1.0;
        Engines_Engines[i].Rpm      = 0.0;
        Engines_Engines[i].FuelFlow = 0.0;
        Engines_Engines[i].Egt      = 0.0;

        Engines_ThrottleLever[i]    = 0.0;
        Engines_EngineState[i]      = IODefn_On;
    }

    AeroLink_NavPkt.GroundLevel = 0.0;
    for (i=1; i<=Simulate_MaxInputs; i+=1)
    {
        Inputs[i].active  = false;
    }
    ParkBrakeOn = false;

    // Allocate 1GB for plot data
    MemSize        = 1073741824;
    MaxPlotRecords = (unsigned int) (MemSize / sizeof(IosDefn_PlaybackDataPktRecord));
    Simulate_FlightDataBlk = malloc(sizeof(IosDefn_PlaybackDataPktRecord) * MaxPlotRecords);
    if (Simulate_FlightDataBlk == NULL)
    {
        printf("Memory error : Flight data storage allocation failed\n");
    }
}
