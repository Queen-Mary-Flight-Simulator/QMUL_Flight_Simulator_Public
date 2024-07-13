/* +---------------------------+---------------------------------+
| Module      : fcs.c          | Version         : 1.1           |
| Last Edit   : 30-03-2016     | Reference Number: 02-01-11      |
|+-----------------------------+---------------------------------+
| Computer    : DELL1                                            |
| Directory   : /dja/aerosoft/cranfield/software/pfd/            |
| Compiler    : gcc 4.8.1                                        |
| OS          : Windows7                                         |
|+---------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
|+---------------------------------------------------------------+
| Description : Boeing 747-400 Flight Control System (FCS)       |
|                                                                |
|+---------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/weather.h>

#include "aero.h"
#include "aerolink.h"
#include "model.h"
#include "fcs.h"
#include "iolib.h"

bool  FCS_APSpeedMode;
float FCS_ThrottlePosition;
bool  FCS_AutoTrimming;
float FCS_Kp;
float FCS_Ki;
float FCS_Kd;
int   FCS_FD_VBar;
int   FCS_FD_HBar;
float SPD_s;

#define YawDamper          true
#define DEG1               (1.0 / Maths_ONERAD)
#define DEG3               (3.0 / Maths_ONERAD)
#define DEG25              (25.0 / Maths_ONERAD)
#define DEG60              (60.0 / Maths_ONERAD)
#define DEG85              (85.0 / Maths_ONERAD)
#define DEG95              (95.0 / Maths_ONERAD)

#define NumberOfEngines    4

static float        APAlt;
static bool         APAltMode;
static float        APPitch;
static bool         APPitchMode;
static float        APHdg;
static bool         APHdgMode;
static float        APSpeed;
static bool         APSpeedMode;
static float        APVS;
static bool         APVSMode;
static bool         AutolandMode;
static unsigned int OldSpeed;
static unsigned int OldHdg;
static unsigned int OldAlt;
static int          OldVS;
static bool         OldMetric;
static float        de_fcs;
static float        YawRate_t;
static float        Roll_t;
static float        RollRate_t;
static float        YawDamper_t;
static float        YawRate_d;
static float        Roll_d;
static float        RollRate_d;
static float        YawDamper_d;
static float        ydamp    = 0.0;
static float        ydampint = 0.0;

static float        fdhx;
static float        fdhy;

void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv);
void CheckFCU();
float CheckFMS();
float PitchHold(float ThetaC);

float BankAngleHold(float BRef);
float FPAHold(float GammaC);

/* --------------------------------------------------------- */
void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv)
{
    float de;
    float da;
    float dh;
    float dv;

    FCS_Autoland(&de, &da);

    dh = (Elevator - de) * 140.0;
    Maths_Limit(&dh, -140.0, 140.);
    Maths_Integrate(&fdhx, 0.5 * (dh - fdhx));
    *fdh = intround(fdhx);

    dv = (da - Aileron) * 140.0;
    Maths_Limit(&dv, -140.0, 140.0);
    Maths_Integrate(&fdhy, 0.5 * (dv - fdhy));
    *fdv = intround(fdhy);
}

/* --------------------------------------------------------- */
void FCS_TurnCoordinator(float *Rudder)
{
    *Rudder = -Model_Beta * 20.0;
}

/* --------------------------------------------------------- */
void FCS_YawDamper(float *Rudder)
{
    Maths_Integrate(&ydampint, ydamp);
    ydamp   = Model_R * 20.0 - ydampint / 1.0;
    *Rudder = *Rudder + ydamp;
}

/* --------------------------------------------------------- */
void CheckFCU()
{
    bool NewSpeedMode;
    bool NewHdgMode;
    bool NewAltMode;
    bool NewVSMode;
    int  at_speed;
	
    NewSpeedMode = (AeroLink_NavPkt.FCU_AP1 | AeroLink_NavPkt.FCU_AP2) && AeroLink_NavPkt.FCU_ATHR && AeroLink_NavPkt.FCU_SPD_Hold;
    if (APSpeedMode != NewSpeedMode)
    {
        if (NewSpeedMode)
        {
            FCS_EngageSpeedHold();
        }
        else
        {
            FCS_DisengageSpeedHold();
        }
        APSpeedMode = NewSpeedMode;
		FCS_APSpeedMode = APSpeedMode;  /* needed for engines module */
    }
	if (AeroLink_NavPkt.FCU_SPD_MACH)
	{
    	at_speed = AeroLink_NavPkt.FCU_SPD;
	}
	else
	{
	    at_speed = (int) Weather_Mach_to_Kts(-Model_Pz, (float) AeroLink_NavPkt.FCU_SPD / 100.0);
	}
    if (OldSpeed != at_speed)
    {
        FCS_SetSpeedHold((float) (at_speed) / 1.944);
        OldSpeed = at_speed;
    }

    NewHdgMode = AeroLink_NavPkt.FCU_HDG_Hold;
    if (APHdgMode != NewHdgMode)
    {
        if (NewHdgMode)
        {
            FCS_EngageHeadingHold();
        }
        else
        {
            FCS_DisengageHeadingHold();
        }
        APHdgMode = NewHdgMode;
    }
    if (OldHdg != AeroLink_NavPkt.FCU_HDG)
    {
        FCS_SetHeadingHold(Maths_Rads((float) (AeroLink_NavPkt.FCU_HDG)));
        OldHdg = AeroLink_NavPkt.FCU_HDG;
    }

    NewAltMode = AeroLink_NavPkt.FCU_ALT_Hold;
    if (APAltMode != NewAltMode)
    {
        if (NewAltMode)
        {
            FCS_EngageHeightHold();
        }
        else
        {
            FCS_DisengageHeightHold();
        }
        APAltMode = NewAltMode;
    }
    if ((OldAlt != AeroLink_NavPkt.FCU_ALT) || (OldMetric != AeroLink_NavPkt.FCU_Metric_ALT))
    {
	    if (AeroLink_NavPkt.FCU_Metric_ALT)
		{
            FCS_SetHeightHold((float) (-AeroLink_NavPkt.FCU_ALT));
		}
		else
		{
            FCS_SetHeightHold(Maths_Metres((float) (-AeroLink_NavPkt.FCU_ALT)));
		}
        OldAlt = AeroLink_NavPkt.FCU_ALT;
		OldMetric = AeroLink_NavPkt.FCU_Metric_ALT;
    }

    NewVSMode = AeroLink_NavPkt.FCU_VS_Hold;
    if (APVSMode != NewVSMode)
    {
        if (NewVSMode)
        {
            FCS_EngageVSpeedHold();
        }
        else
        {
            FCS_DisengageVSpeedHold();
        }
        APVSMode = NewVSMode;
    }
    if (OldVS != AeroLink_NavPkt.FCU_VS)
    {
        FCS_SetVSpeedHold((float) (AeroLink_NavPkt.FCU_VS) * 0.508);  /* FPM * 100 -> m/s */
        OldVS = AeroLink_NavPkt.FCU_VS;
    }

    if (AutolandMode != AeroLink_NavPkt.FCU_APPR)
    {
        //printf("AutolandMode : %d AeroLink_NavPkt.APPR : %d\n", AutolandMode, AeroLink_NavPkt.APPR);
        //printf("ModelLat : %f ModelLong : %f\n", Model_Latitude, Model_Longitude);
        if (AeroLink_NavPkt.FCU_APPR)
        {
            FCS_EngageAutoland();
        }
        else
        {
            FCS_DisengageAutoland();
        }
    }
}

/* --------------------------------------------------------- */
void FCS_ResetFCS()
{
    if (APAltMode || APPitchMode || APVSMode || AutolandMode)
    {
        Model_Pitch = Model_Autotrim(&de_fcs);
    }
}

/* --------------------------------------------------------- */
void FCS_FCS(float *Elevator, float *Aileron, float *Rudder)
{
    CheckFCU();

    if (APAltMode)
    {
        *Elevator = FCS_HeightHold(APAlt);
    }

    if (AeroLink_NavPkt.WayPoint.BeaconStatus)
    {   /* flight plan active? */
        *Aileron = CheckFMS();
    }
    else if (AeroLink_NavPkt.FCU_LOC)
    {
        *Aileron = FCS_LOCHold();
    }
    else if (APHdgMode)
    {
        *Aileron = FCS_HeadingHold(APHdg);
    }

    if (APSpeedMode)
    {
        FCS_ThrottlePosition = FCS_SpeedHold(APSpeed);
    }

    if (APVSMode)
    {
        *Elevator = FCS_VSpeedHold(APVS);
    }

    if (AutolandMode)
    {
        FCS_Autoland(Elevator, Aileron);
    }

    if (AeroLink_NavPkt.FCU_FD)
    {
        UpdateFlightDirector(*Elevator, *Aileron, &FCS_FD_HBar, &FCS_FD_VBar);
    }
}

/* --------------------------------------------------------- */
void FCS_EngagePitchHold()
{
    APPitchMode = true;
    FCS_DisengageVSpeedHold();
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengagePitchHold()
{
    APPitchMode = false;
}

/* --------------------------------------------------------- */
bool FCS_PitchHoldEngaged()
{
    return APPitchMode;
}

/* --------------------------------------------------------- */
float PitchHold(float ThetaC)
{
    const float Pitch_Kp = 0.4;
    const float Pitch_Ki = 5.0;

    float       Qc;
    float       dedot;

    Qc    = (ThetaC - Model_Pitch) * Pitch_Kp;
    dedot = -Pitch_Ki * (Qc - Model_Q);
    Maths_Integrate(&de_fcs, dedot);
    Maths_Limit(&de_fcs, -1.0, 1.0);
    return de_fcs;
}

/* --------------------------------------------------------- */
void FCS_SetPitchHold(float Theta)
{
    APPitch = Theta;
}

/* --------------------------------------------------------- */
void FCS_EngageAutoland()
{
    AutolandMode = true;
    FCS_DisengageHeightHold();
    FCS_DisengageHeadingHold();
    FCS_DisengageVSpeedHold();
}

/* --------------------------------------------------------- */
void FCS_DisengageAutoland()
{
    AutolandMode = false;
}

/* --------------------------------------------------------- */
bool FCS_AutolandEngaged()
{
    return AutolandMode;
}

/* --------------------------------------------------------- */
void FCS_Autoland(float *Elevator, float *Aileron)
{
    float Qdm;
    float h;
    float fpa;
    float gserr;

    //printf("ILSBeacon : %d AL RWQDM : %f\n", AeroLink_NavPkt.ILS1.ILSBeacon, AeroLink_NavPkt.ILS1.RunwayQdm);

    if (AeroLink_NavPkt.ILS1.ILSBeacon)
    {
        Qdm = AeroLink_NavPkt.ILS1.RunwayQdm - AeroLink_NavPkt.MagneticVariation;
		Maths_Normalise(&Qdm);
    }
    else
    {
        return;
    }
    h = -Model_Pz + (float) (AeroLink_NavPkt.GroundLevel) + Aero_CGHeight;
    if (h < 15.0)
    {
        *Aileron = FCS_HeadingHold(Qdm + 1.0 * (float) (AeroLink_NavPkt.ILS1.LocaliserError));
        if (Model_OnTheGround)
        {
            *Elevator = PitchHold(Model_Pitch * 0.5);
        }
        else
        {
            *Elevator = FPAHold(-0.005);
        }
    }
    else
    {
        gserr = (float) (AeroLink_NavPkt.ILS1.GlideSlopeError);
        Maths_Limit(&gserr, -DEG1, DEG1);
        fpa = -DEG3 + 5.0 * gserr;
        if (fpa > 0.0)
        {
            fpa = 0.0;
        }
        *Aileron  = FCS_HeadingHold(Qdm + 10.0 * (float) (AeroLink_NavPkt.ILS1.LocaliserError));
        *Elevator = FPAHold(fpa);
    }
}

/* --------------------------------------------------------- */
void FCS_EngageHeightHold()
{
    APAltMode = true;
    FCS_DisengageVSpeedHold();
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengageHeightHold()
{
    APAltMode = false;
}

/* --------------------------------------------------------- */
bool FCS_HeightHoldEngaged()
{
    return APAltMode;
}

/* --------------------------------------------------------- */
void FCS_SetHeightHold(float Href)
{
    APAlt = Href;
}

/* --------------------------------------------------------- */
float FCS_HeightHold(float Href)
{
    float vs;
    float h;

    h  = Model_Pz; // - Aero_CGHeight;
    vs = -(Href - h) * 0.083333;
    if (vs > 5.08)
    {
        vs = 5.08;
    }
    else if (vs < -5.08)
    {
        vs = -5.08;
    }
    return FCS_VSpeedHold(vs);
}

/* --------------------------------------------------------- */
void FCS_EngageHeadingHold()
{
    APHdgMode = true;
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengageHeadingHold()
{
    APHdgMode = false;
}

/* --------------------------------------------------------- */
bool FCS_HeadingHoldEngaged()
{
    return APHdgMode;
}

/* --------------------------------------------------------- */
void FCS_SetHeadingHold(float Href)
{
    APHdg = Href;
}

/* --------------------------------------------------------- */
float FCS_HeadingHold(float HdgRef)
{
    float dHdg;
    float trc;

    HdgRef = HdgRef + (float) (AeroLink_NavPkt.MagneticVariation);
    dHdg   = HdgRef - Model_Yaw;
    Maths_Normalise(&dHdg);
    trc = dHdg * 0.15;  /* 20 deg -> 3 deg/s */
    if (trc > DEG3)
    {
        trc = DEG3;
    }
    else if (trc < -DEG3)
    {
        trc = -DEG3;
    }
    return BankAngleHold(trc * Model_U / 9.81);
}

/* --------------------------------------------------------- */
float CheckFMS()
{
    float h;
    float dh;
    float error;

    h     = AeroLink_NavPkt.WayPoint.RunwayQdm;
    error = AeroLink_NavPkt.WayPoint.LocaliserError;

    dh = error * 6.0;
    Maths_Limit(&dh, -DEG60, DEG60);
    h = h + dh;
    Maths_Normalise(&h);
    return FCS_HeadingHold(h);
}

/* --------------------------------------------------------- */
float FCS_LOCHold()
{
    float h;
    float dh;
    float error;

    h     = Maths_Rads((float) AeroLink_NavPkt.HSI_Crs);
    error = AeroLink_NavPkt.NAV1.LocaliserError;

    if ((error >= -DEG85) && (error <= DEG85))   /* TO */
    {
        /* do nothing */
    }
    else if (error <= -DEG95)   /* FROM */
    {
        error = -Maths_PI - error;
    }
    else if (error >= DEG95)    /* FROM */
    {
        error = Maths_PI - error;
    }
    else
    {
        error = 0.0;  /* cone of confusion */
    }

    dh = error * 6.0;
    Maths_Limit(&dh, -DEG60, DEG60);
    h = h + dh;
    Maths_Normalise(&h);
    return FCS_HeadingHold(h);
}

/* --------------------------------------------------------- */
void FCS_EngageSpeedHold()
{
    APSpeedMode = true;
}

/* --------------------------------------------------------- */
void FCS_DisengageSpeedHold()
{
    APSpeedMode = false;
}

/* --------------------------------------------------------- */
bool FCS_SpeedHoldEngaged()
{
    return APSpeedMode;
}

/* --------------------------------------------------------- */
void FCS_SetSpeedHold(float Vref)
{
    APSpeed = Vref;
}

/* --------------------------------------------------------- */
float FCS_SpeedHold(float Vref)
{
    const float SPD_Kp    = 10.0;
    const float SPD_Ki    = 0.0;
    const float SPD_Kudot = 2.0;

    float       tp;
    float       e;
    float       IAS;

    IAS    = Model_U * sqrt(Weather_DensityRatio);
    e      = Vref - IAS - Model_UDot * SPD_Kudot;
    SPD_s += e;
    tp     = SPD_Kp * e + SPD_Ki * SPD_s;
    Maths_Limit(&tp, 0.2, 1.0);
    return tp;
}

/* --------------------------------------------------------- */
void FCS_EngageVSpeedHold()
{
    APVSMode = true;
    FCS_DisengageHeightHold();
    FCS_DisengageAutoland();
}

/* --------------------------------------------------------- */
void FCS_DisengageVSpeedHold()
{
    APVSMode = false;
}

/* --------------------------------------------------------- */
bool FCS_VSpeedHoldEngaged()
{
    return APVSMode;
}

/* --------------------------------------------------------- */
void FCS_SetVSpeedHold(float Vref)
{
    APVS = Vref;
}

/* --------------------------------------------------------- */
float FCS_VSpeedHold(float Vref)
{
    return FPAHold(Vref / Model_Vc);
}

/* --------------------------------------------------------- */
float BankAngleHold(float RollC)
{
    const float Bank_Kp = 3.0;
    const float Bank_Kd = 3.0;

    float       da;

    Maths_Limit(&RollC, -DEG25, DEG25);
    da = (RollC - Model_Roll) * Bank_Kp - Model_P * Bank_Kd;
    Maths_Limit(&da, -1.0, 1.0);
    return da;
}

/* --------------------------------------------------------- */
float FPAHold(float GammaC)
{
    const float FPA_Kp = 0.2;
    const float FPA_Ki = 20.0;

    float       Qc;
    float       dedot;
    float       Gamma;

    Gamma = Model_Pitch - Model_Alpha;
    Qc    = (GammaC - Gamma) * FPA_Kp;
    dedot = -FPA_Ki * (Qc - (Model_Q * cos(Model_Roll) - Model_R * sin(Model_Roll)));
    Maths_Integrate(&de_fcs, dedot);
    Maths_Limit(&de_fcs, -1.0, 1.0);
    return de_fcs;
}

/* --------------------------------------------------------- */
void BEGIN_FCS()
{
    APAlt           = 0.0;
    APAltMode       = false;
    APHdg           = 0.0;
    APHdgMode       = false;
    APSpeed         = 0.0;
    APSpeedMode     = false;
    APVS            = 0.0;
    APVSMode        = false;
    AutolandMode    = false;
    APPitchMode     = false;

    OldSpeed = 0;
    OldHdg = 0;
    OldAlt = 0;
    OldVS = 0;
    OldMetric = false;

    FCS_Kp = 1.0;
    FCS_Ki = 0.0;
    FCS_Kd = 0.0;

    de_fcs = 0.0;

    YawRate_t   = 0.0;
    Roll_t      = 0.0;
    RollRate_t  = 0.0;
    YawDamper_t = 0.0;
    YawRate_d   = 0.0;
    Roll_d      = 0.0;
    RollRate_d  = 0.0;
    YawDamper_d = 0.0;

    FCS_ThrottlePosition = 0.0;
    FCS_APSpeedMode = false;

    fdhx = 0.0;
    fdhy = 0.0;
}
