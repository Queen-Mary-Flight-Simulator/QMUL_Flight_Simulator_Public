/* +---------------------------+---------------------------------+
| Module      : fcs.c          | Version         : 1.2           |
| Last Edit   : 17-02-2022     | Reference Number: 02-01-11      |
|+-----------------------------+---------------------------------+
| Computer    : DELL1                                            |
| Directory   : /dja/aerosoft/cranfield/software/pfd/            |
| Compiler    : gcc 10.2.0                                       |
| OS          : Windows10                                        |
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
#include <math.h>

#include <SIM/maths.h>
#include <SIM/weather.h>

#include "aero.h"
#include "aerolink.h"
#include "model.h"
#include "fcs.h"
#include "iolib.h"

#define YAWDAMPER       true
#define TURNCOORDINATOR true
#define NumberOfEngines 4

#define ONERAD      (180.0 / M_PI)
#define DEG1        (1.0 / ONERAD)
#define DEG3        (3.0 / ONERAD)
#define DEG25       (25.0 / ONERAD)
#define DEG60       (60.0 / ONERAD)
#define DEG85       (85.0 / ONERAD)
#define DEG95       (95.0 / ONERAD)

bool                FCS_AP_SPD_Mode;
float               FCS_ThrottlePosition;
bool                FCS_AutoTrimming;
float               FCS_Kp;
float               FCS_Ki;
float               FCS_Kd;
int                 FCS_FD_VBar;
int                 FCS_FD_HBar;
bool                FCS_TOGAMode;

static float        AP_ALT;
static bool         AP_ALT_Mode;
static float        AP_Pitch;
static bool         AP_Pitch_Mode;
static float        AP_FPA;
static bool         AP_FPA_Mode;
static float        AP_BankAngle;
static bool         AP_BankAngle_Mode;
static float        AP_HDG;
static bool         AP_HDG_Mode;
static float        AP_SPD;
static bool         AP_SPD_Mode;
static float        AP_VSPD;
static bool         AP_VSPD_Mode;

static bool         AP_TurnCoordinator_Mode;
static bool         AP_YawDamper_Mode;
static bool         AP_Autoland_Mode;
static bool         AP_LOC_Mode;

static unsigned int OldSpeed;
static unsigned int OldHdg;
static unsigned int OldAlt;
static int          OldVS;
static bool         OldMetricButton;
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

static float        SPD_s = 0.0;
static float        Pitch_s = 0.0;
static float        FPA_s = 0.0;

void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv);
float CheckFMS();
void CheckFCU();

/* --------------------------------------------------------- */
void FCS_FCS(float *Elevator, float *Aileron, float *Rudder)
{
    CheckFCU();

    if (IOLib_GetTOGASwitch() == IODefn_On)
	{
	    FCS_TOGAMode = true;
	}
	
    if (AP_Autoland_Mode)
    {
        FCS_Autoland(Elevator, Aileron);
    }
    else
    {
        if (AP_ALT_Mode)
        {
            *Elevator = FCS_ALT(AP_ALT);
        }
        else if (AP_VSPD_Mode)
        {
            *Elevator = FCS_VSPD(AP_VSPD);
        }
        else if (AP_Pitch_Mode)
        {
            *Elevator = FCS_Pitch(AP_Pitch);
        }
        else if (AP_FPA_Mode)
        {
            *Elevator = FCS_FPA(AP_FPA);
        }

        
        if (AP_HDG_Mode)
        {
            *Aileron = FCS_HDG(AP_HDG);
        }
        else if (AeroLink_NavPkt.FCU_LOC)
        {
            *Aileron = FCS_LOC();
        }
        else if (AP_BankAngle_Mode)
        {
            *Aileron = FCS_BankAngle(AP_BankAngle);
        }
        else if (AeroLink_NavPkt.WayPoint.BeaconStatus)
        {   /* flight plan active? */
            *Aileron = CheckFMS();
        }
    }
    
    if (AP_SPD_Mode)
    {
        FCS_ThrottlePosition = FCS_SPD(AP_SPD);
    }

    if (AeroLink_NavPkt.FCU_FD)
    {
        UpdateFlightDirector(*Elevator, *Aileron, &FCS_FD_HBar, &FCS_FD_VBar);
    }

    //*Elevator = FCS_Pitch(10.0 / 57.29577951);  // *** only used for pitch hold test
    //*Elevator = FCS_FPA(5.08 / 164.62);         // *** only used for FPA hold test

    if (AP_TurnCoordinator_Mode)
    {
        FCS_TurnCoordinator(Rudder);
    }
    
    if (AP_YawDamper_Mode)
    {
        FCS_YawDamper(Rudder);
    }
}

/* --------------------------------------------------------- */
void FCS_ResetFCS()
{
    if (AP_ALT_Mode || AP_Pitch_Mode || AP_VSPD_Mode || AP_Autoland_Mode)
    {
        Model_Pitch = Model_Autotrim(&de_fcs);
    }
}

/* --------------------------------------------------------- */
void FCS_Engage_ALT()
{
    FCS_Disengage_VSPD();
    FCS_Disengage_Autoland();
    AP_ALT_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_ALT()
{
    AP_ALT_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_ALTEngaged()
{
    return AP_ALT_Mode;
}

/* --------------------------------------------------------- */
void FCS_Set_ALT(float Href)
{
    AP_ALT = Href;
}

/* --------------------------------------------------------- */
float FCS_ALT(float Href)  /* -ve up, metres */
{
    float vs = (-Href + Model_Pz) * 0.083333;

    vs = Maths_Limit(vs, -5.08, 5.08);

    return FCS_VSPD(vs);
}

/* --------------------------------------------------------- */
void FCS_Engage_HDG()
{
    FCS_Disengage_Autoland();
    AP_HDG_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_HDG()
{
    AP_HDG_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_HDG_Engaged()
{
    return AP_HDG_Mode;
}

/* --------------------------------------------------------- */
void FCS_Set_HDG(float Href)
{
    AP_HDG = Href;
}

/* --------------------------------------------------------- */
float FCS_HDG(float HdgRef)
{
    float dHdg;
    float trc;

    HdgRef = HdgRef + (float) (AeroLink_NavPkt.MagneticVariation);
    dHdg   = HdgRef - Model_Yaw;
    dHdg = Maths_Normalise(dHdg);
    trc = dHdg * 0.15;  /* (20 deg -> 3 deg/s */
    trc = Maths_Limit(trc, -DEG3, DEG3);

    return FCS_BankAngle(trc * Model_U / 9.81);
}

/* --------------------------------------------------------- */
void FCS_Engage_SPD()
{
    AP_SPD_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_SPD()
{
    AP_SPD_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_SPD_Engaged()
{
    return AP_SPD_Mode;
}

/* --------------------------------------------------------- */
void FCS_Set_SPD(float Vref)
{
    AP_SPD = Vref;
}

/* --------------------------------------------------------- */
float FCS_SPD(float Vref)
{
    const float SPD_Kp = 1.0;
    const float SPD_Ki = 0.02;
    const float K1     = 5.0;
    const float K2     = 1.0;

    float       IAS    = Model_U * sqrt(Weather_DensityRatio);
    float       e      = (Vref - IAS) * K2 - Model_UDot * K1;
    float       s      = SPD_s + e;
    float       tp     = SPD_Kp * e + SPD_Ki * s;
    
    if (tp > 0.2 && tp < 1.0)
    {
        SPD_s = s;
    }
    return Maths_Limit(tp, 0.2, 1.0);
}

/* --------------------------------------------------------- */
void FCS_Engage_VSPD()
{
    FCS_Disengage_ALT();
    FCS_Disengage_Autoland();
    AP_VSPD_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_VSPD()
{
    AP_VSPD_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_VSPD_Engaged()
{
    return AP_VSPD_Mode;
}

/* --------------------------------------------------------- */
void FCS_Set_VSPD(float Vref)
{
    AP_VSPD = Vref;
}

/* --------------------------------------------------------- */
float FCS_VSPD(float Vref)
{
    return FCS_FPA(Vref / Model_Vc);
}

/* --------------------------------------------------------- */
void FCS_Engage_Pitch()
{
    FCS_Disengage_VSPD();
    FCS_Disengage_Autoland();
    AP_Pitch_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_Pitch()
{
    AP_Pitch_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_Pitch_Engaged()
{
    return AP_Pitch_Mode;
}

/* --------------------------------------------------------- */
void FCS_Set_Pitch(float Theta)
{
    AP_Pitch = Theta;
}

/* --------------------------------------------------------- */
float FCS_Pitch(float ThetaC)
{
    const float Pitch_Kp = -50.0;
    const float Pitch_Ki = -5.0;

    float Qc             = Maths_Limit(ThetaC - Model_Pitch, -2.0 / ONERAD, 2.0 / ONERAD);
    float e              = Qc - Model_Q * cos(Model_Roll) + Model_R * sin(Model_Roll);
    float s              = Pitch_s + e;
    float de             = Pitch_Kp * e + Pitch_Ki * s;
    
//    Qc = Maths_Limit(Qc, -2.0 / ONERAD, 2.0 / ONERAD);
    if (de > -1.0 && de < 1.0)
    {
        Pitch_s = s;
    }
    de = Maths_Limit(de, -1.0, 1.0);
    return de;
}

/* --------------------------------------------------------- */
void FCS_Engage_FPA()
{
    FCS_Disengage_VSPD();
    FCS_Disengage_Autoland();
    AP_FPA_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_FPA()
{
    AP_FPA_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_FPA_Engaged()
{
    return AP_FPA_Mode;
}

/* --------------------------------------------------------- */
void FCS_Set_FPA(float Theta)
{
    AP_FPA = Theta;
}

/* --------------------------------------------------------- */
float FCS_FPA(float GammaC)
{
    const float Kp = -50.0; // -50.0
    const float Ki = -5.0;  // -5.0

    float Gamma    = Model_Pitch - Model_Alpha;
    float Qc       = (GammaC - Gamma) * 0.2;
    float e        = Qc - Model_Q * cos(Model_Roll) + Model_R * sin(Model_Roll) - Model_UDot * 0.0015;
    float s        = FPA_s + e;
    float de       = Kp * e + Ki * s;

    if (de > -1.0 && de < 1.0)
    {
        FPA_s = s;
    }
    de = Maths_Limit(de, -1.0, 1.0);
    return de;
}

/* --------------------------------------------------------- */
void FCS_Engage_BankAngle()
{
    AP_BankAngle_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_BankAngle()
{
    AP_BankAngle_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_BankAngle_Engaged()
{
    return AP_BankAngle_Mode;
}

/* --------------------------------------------------------- */
void FCS_Set_BankAngle(float Theta)
{
    AP_BankAngle = Theta;
}

/* --------------------------------------------------------- */
float FCS_BankAngle(float RollC)
{
    const float Bank_Kp = 3.0;
    const float Bank_Kd = 3.0;

    float       da;

    RollC = Maths_Limit(RollC, -DEG25, DEG25);
    da = (RollC - Model_Roll) * Bank_Kp - Model_P * Bank_Kd;
    da = Maths_Limit(da, -1.0, 1.0);
    return da;
}

/* --------------------------------------------------------- */
void FCS_Engage_TurnCoordinator()
{
    AP_TurnCoordinator_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_TurnCoordinator()
{
    AP_TurnCoordinator_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_TurnCoordinator_Engaged()
{
    return AP_TurnCoordinator_Mode;
}

/* --------------------------------------------------------- */
void FCS_TurnCoordinator(float *Rudder)
{
    *Rudder = *Rudder - Model_Beta * 20.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_YawDamper()
{
    AP_YawDamper_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_YawDamper()
{
    AP_YawDamper_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_YawDamper_Engaged()
{
    return AP_YawDamper_Mode;
}

/* --------------------------------------------------------- */
void FCS_YawDamper(float *Rudder)
{
    ydampint = Maths_Integrate(ydampint, ydamp);
    ydamp   = Model_R * 20.0 - ydampint / 1.0; // was 20 1
    *Rudder = *Rudder + ydamp;
}

/* --------------------------------------------------------- */
void FCS_Engage_Autoland()
{
    FCS_Disengage_ALT();
    FCS_Disengage_HDG();
    FCS_Disengage_VSPD();
    AP_Autoland_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_Autoland()
{
    AP_Autoland_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_Autoland_Engaged()
{
    return AP_Autoland_Mode;
}

/* --------------------------------------------------------- */
void FCS_Autoland(float *Elevator, float *Aileron)
{
    float Qdm;
    float h;
    float fpa;
    float gserr;

    if (AeroLink_NavPkt.ILS1.ILSBeacon)
    {
        Qdm = AeroLink_NavPkt.ILS1.RunwayQdm - AeroLink_NavPkt.MagneticVariation; /* Heading-hold uses deg magnetic */
        Qdm = Maths_Normalise(Qdm);
    }
    else
    {
        return;
    }
    h = -Model_Pz + (float) (AeroLink_NavPkt.GroundLevel) + Aero_CGHeight;
    if (h < 15.0)
    {
        *Aileron = FCS_HDG(Qdm + 1.0 * (float) (AeroLink_NavPkt.ILS1.LocaliserError));
        if (Model_OnTheGround)
        {
            *Elevator = FCS_Pitch(Model_Pitch * 0.5);
        }
        else
        {
            *Elevator = FCS_FPA(-0.005);
        }
    }
    else
    {
        gserr = (float) (AeroLink_NavPkt.ILS1.GlideSlopeError);
        gserr = Maths_Limit(gserr, -DEG1, DEG1);
        fpa = -DEG3 + 5.0 * gserr;
        if (fpa > 0.0)
        {
            fpa = 0.0;
        }
        *Aileron  = FCS_HDG(Qdm + 10.0 * (float) (AeroLink_NavPkt.ILS1.LocaliserError));
        *Elevator = FCS_FPA(fpa);
    }
}

/* --------------------------------------------------------- */
void FCS_Engage_LOC()
{
    FCS_Disengage_HDG();
    AP_LOC_Mode = true;
}

/* --------------------------------------------------------- */
void FCS_Disengage_LOC()
{
    AP_LOC_Mode = false;
}

/* --------------------------------------------------------- */
bool FCS_LOC_Engaged()
{
    return AP_LOC_Mode;
}

/* --------------------------------------------------------- */
float FCS_LOC()
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
        error = -M_PI - error;
    }
    else if (error >= DEG95)    /* FROM */
    {
        error = M_PI - error;
    }
    else
    {
        error = 0.0;  /* cone of confusion */
    }

    dh = error * 6.0;
    dh = Maths_Limit(dh, -DEG60, DEG60);
    h = h + dh;
    h = Maths_Normalise(h);
    return FCS_HDG(h);
}

/* --------------------------------------------------------- */
void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv)
{
    float de;
    float da;
    float dh;
    float dv;

    FCS_Autoland(&de, &da);

    dh = (Elevator - de) * 140.0;
    dh = Maths_Limit(dh, -140.0, 140.);
    fdhx = Maths_Integrate(fdhx, 0.5 * (dh - fdhx));
    *fdh = intround(fdhx);

    dv = (da - Aileron) * 140.0;
    dv = Maths_Limit(dv, -140.0, 140.0);
    fdhy = Maths_Integrate(fdhy, 0.5 * (dv - fdhy));
    *fdv = intround(fdhy);
}

/* --------------------------------------------------------- */
void CheckFCU()
{
    bool NewSpeedMode;
    bool NewHdgMode;
    bool NewAltMode;
    bool NewVSMode;
    int  at_speed;

    NewSpeedMode = AeroLink_NavPkt.FCU_ATHR && (AeroLink_NavPkt.FCU_SPDKnob == NavDefn_Pulled);
    if (AP_SPD_Mode != NewSpeedMode)
    {
        if (NewSpeedMode)
        {
            FCS_Engage_SPD();
        }
        else
        {
            FCS_Disengage_SPD();
        }
        AP_SPD_Mode = NewSpeedMode;
        FCS_AP_SPD_Mode = AP_SPD_Mode;  /* needed for engines module */
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
        FCS_Set_SPD((float) (at_speed) / 1.944);
        OldSpeed = at_speed;
    }

    NewHdgMode = (AeroLink_NavPkt.FCU_HDGKnob == NavDefn_Pulled);
    if (AP_HDG_Mode != NewHdgMode)
    {
        if (NewHdgMode)
        {
            FCS_Engage_HDG();
        }
        else
        {
            FCS_Disengage_HDG();
        }
        AP_HDG_Mode = NewHdgMode;
    }
    if (OldHdg != AeroLink_NavPkt.FCU_HDG)
    {
        FCS_Set_HDG(Maths_Rads((float) (AeroLink_NavPkt.FCU_HDG)));
        OldHdg = AeroLink_NavPkt.FCU_HDG;
    }

    NewAltMode = (AeroLink_NavPkt.FCU_ALTKnob == NavDefn_Pulled);
    if (AP_ALT_Mode != NewAltMode)
    {
        if (NewAltMode)
        {
            FCS_Engage_ALT();
        }
        else
        {
            FCS_Disengage_ALT();
        }
        AP_ALT_Mode = NewAltMode;
    }
    if ((OldAlt != AeroLink_NavPkt.FCU_ALT) || (OldMetricButton != AeroLink_NavPkt.FCU_Metric_Button))
    {
        if (AeroLink_NavPkt.FCU_Metric_Button)
        {
            FCS_Set_ALT((float) (-AeroLink_NavPkt.FCU_ALT));
        }
        else
        {
            FCS_Set_ALT(Maths_Metres((float) (-AeroLink_NavPkt.FCU_ALT)));
        }
        OldAlt = AeroLink_NavPkt.FCU_ALT;
        OldMetricButton = AeroLink_NavPkt.FCU_Metric_Button;
    }

    NewVSMode = (AeroLink_NavPkt.FCU_VSKnob == NavDefn_Pulled);
    if (AP_VSPD_Mode != NewVSMode)
    {
        if (NewVSMode)
        {
            FCS_Engage_VSPD();
        }
        else
        {
            FCS_Disengage_VSPD();
        }
        AP_VSPD_Mode = NewVSMode;
    }
    if (OldVS != AeroLink_NavPkt.FCU_VS)
    {
        FCS_Set_VSPD((float) (AeroLink_NavPkt.FCU_VS) * 0.00508);  /* FPM * 100 -> m/s */
        OldVS = AeroLink_NavPkt.FCU_VS;
    }

    if (AP_Autoland_Mode != AeroLink_NavPkt.FCU_APPR)
    {
        //printf("AP_Autoland_Mode : %d AeroLink_NavPkt.APPR : %d\n", AP_Autoland_Mode, AeroLink_NavPkt.APPR);
        //printf("ModelLat : %f ModelLong : %f\n", Model_Latitude, Model_Longitude);
        if (AeroLink_NavPkt.FCU_APPR)
        {
            FCS_Engage_Autoland();
        }
        else
        {
            FCS_Disengage_Autoland();
        }
    }
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
    dh = Maths_Limit(dh, -DEG60, DEG60);
    h = h + dh;
    h = Maths_Normalise(h);
    return FCS_HDG(h);
}

/* --------------------------------------------------------- */
void BEGIN_FCS()
{
    AP_ALT                  = 0.0;
    AP_ALT_Mode             = false;
    AP_HDG                  = 0.0;
    AP_HDG_Mode             = false;
    AP_SPD                  = 0.0;
    AP_SPD_Mode             = false;
    AP_VSPD                 = 0.0;
    AP_VSPD_Mode            = false;
    
    AP_Autoland_Mode        = false;
    AP_Pitch                = 0.0;
    AP_Pitch_Mode           = false;
    AP_FPA                  = 0.0;
    AP_FPA_Mode             = false;
    AP_BankAngle            = 0.0;
    AP_BankAngle_Mode       = false;
    AP_TurnCoordinator_Mode = TURNCOORDINATOR;
    AP_YawDamper_Mode       = YAWDAMPER;
    AP_LOC_Mode             = false;

    OldSpeed = 0;
    OldHdg = 0;
    OldAlt = 0;
    OldVS = 0;
    OldMetricButton = false;

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
    FCS_AP_SPD_Mode = false;
    FCS_TOGAMode = false;

    fdhx = 0.0;
    fdhy = 0.0;
}
