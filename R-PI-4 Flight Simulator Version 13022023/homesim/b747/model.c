/*
+------------------------------+---------------------------------+
| Module      : model.c        | Version         : 1.1           |
| Last Edit   : 30-03-2016     | Reference Number: 02-01-13      |
+------------------------------+---------------------------------+
| Computer    : DELL1                                            |
| Directory   : /dja/aerosoft/cranfield/software/pfd/            |
| Compiler    : gcc 4.8.1                                        |
| OS          : Windows7                                         |
+----------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
+----------------------------------------------------------------+
| Description : Boeing 747-400 Equations of Motion               |
|                                                                |
+----------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include <SIM/iodefn.h>
#include <SIM/engdefn.h>
#include <SIM/weather.h>
#include <SIM/maths.h>
#include <SIM/clocks.h>
#include <SIM/dted.h>

#include "iolib.h"
#include "aero.h"
#include "systems.h"
#include "aerolink.h"
#include "fcs.h"
#include "model.h"
#include "gear.h"

double       Model_Latitude, Model_Longitude;
double       Model_dLatitude, Model_dLongitude;
float        Model_Pitch;
float        Model_Roll;
float        Model_Yaw;
float        Model_Vn, Model_Ve, Model_Vd;
float        Model_Pz;
float        Model_U, Model_V, Model_W;
float        Model_UDot, Model_VDot, Model_WDot;
float        Model_P, Model_Q, Model_R;
float        Model_PDot, Model_QDot, Model_RDot;
float        Model_Pmt, Model_Rmt, Model_Ymt;
float        Model_Alpha, Model_Beta;
float        Model_AlphaWing;
float        Model_AlphaDot, Model_BetaDot;
float        Model_Cl;
float        Model_ClTail;
float        Model_Clfw;
float        Model_Cd;
float        Model_CyBeta;
float        Model_CyDr;
float        Model_Cz0;
float        Model_Cz1;
float        Model_Cm0;
float        Model_CmAlpha;
float        Model_CmDe;
float        Model_CmQ;
float        Model_CmAlphaDot;
float        Model_ClBeta;
float        Model_ClDr;
float        Model_ClDa;
float        Model_ClP;
float        Model_ClR;
float        Model_CnBeta;
float        Model_CnBetaDot;
float        Model_CnDr;
float        Model_CnDa;
float        Model_CnP;
float        Model_CnR;
float        Model_Lift, Model_Thrust, Model_Drag, Model_SideForce;
float        Model_DiffBraking;
float        Model_GForce, Model_GMin, Model_GMax;
float        Model_XForce, Model_YForce, Model_ZForce;
float        Model_A11, Model_A12, Model_A13;
float        Model_A21, Model_A22, Model_A23;
float        Model_A31, Model_A32, Model_A33;
float        Model_Vc, Model_VcDot;
float        Model_MachNumber;
float        Model_Elevator;
float        Model_Aileron;
float        Model_Rudder;
float        Model_Tiller;
float        Model_ElevatorTrim;
float        Model_AileronTrim;
float        Model_RudderTrim;
float        Model_LeftBrake, Model_RightBrake;
float        Model_Flaps;
float        Model_Gear;
bool         Model_Stalling;
bool         Model_OnTheGround;
float        Model_e0, Model_e1, Model_e2, Model_e3;
float        Model_Ex, Model_Ey, Model_Ez;
float        Model_Incline;

float        e0Dot, e1Dot, e2Dot, e3Dot;
float        Pstab, Rstab;
float        U_aero, V_aero, W_aero;

double       PushbackDistance;
float        PushbackMaxDistance;
float        PushbackSpeed;
float        PushbackHdg;
bool         Pushingback;
float        PushbackInitHdg; 
float        PushbackHdgRate;

void SetEyePosition();
void Pushback(float hdg);

/* ----------------------------------------------- */
void Model_ResetFlightModel()
{
    U_aero = Model_U;
    V_aero = Model_V;
    W_aero = Model_W;
}

/* ----------------------------------------------- */
void Model_FlightModel()
{
    float Vc2;
    float MaxAlpha;
    float OneG;
    float De, Da, Dr;
    float CosAlpha, SinAlpha;
    float Pm, Rm, Ym;
    float Lambda;
    float SinPitch;
    float CosPitch;
    float SinRoll;
    float CosRoll;
    float t;
    float uw2;

    if (Pushingback)
    {
        Model_Pushback(PushbackHdg, false);
        return;
    }
    
    Model_Elevator = IOLib_GetElevator();
    Model_Aileron  = IOLib_GetAileron();
    Model_Rudder   = IOLib_GetRudder();
    Model_Tiller   = IOLib_GetElevatorTrim();   // (Thrustmaster only) IOLib_GetTiller();

    Model_ElevatorTrim = IOLib_GetElevatorTrim();
    Model_AileronTrim  = IOLib_GetAileronTrim();
    Model_RudderTrim   = IOLib_GetRudderTrim();
    Model_LeftBrake    = IOLib_GetLeftBrake();
    Model_RightBrake   = IOLib_GetRightBrake();
    if (IOLib_GetParkBrake() == IODefn_On)
    {
        Model_LeftBrake  = 1.0;
        Model_RightBrake = 1.0;
    }

    Model_Flaps = Systems_GetFlapPosition();
    Model_Gear  = Systems_GetGearPosition();
    OneG        = Aero_Mass * Model_G;
    FCS_FCS(&Model_Elevator, &Model_Aileron, &Model_Rudder);

    if (AeroLink_OctaveMode)
    {
        De = AeroLink_ProtoPkt.Data.Matlab.Elevator;
        Da = AeroLink_ProtoPkt.Data.Matlab.Aileron;
        Dr = AeroLink_ProtoPkt.Data.Matlab.Rudder;
        /*printf("De : %f\n", De);*/
        /*printf("Da : %f\n", Da);*/
        /*printf("Dr : %f\n", Dr);*/
    }
    else
    {
        De = Model_Elevator * Aero_ElevatorGain + Model_ElevatorTrim * Aero_ElevatorTrimGain;
        Da = Model_Aileron * Aero_AileronGain - Model_AileronTrim * Aero_AileronTrimGain;
        Dr = Model_Rudder * Aero_RudderGain + Model_RudderTrim * Aero_RudderTrimGain;
    }

    Model_Stalling    = false;
    Model_OnTheGround = Gear_WeightOnWheels();
    Vc2               = Model_U * Model_U + Model_V * Model_V + Model_W * Model_W;
    Model_Vc          = sqrt(Vc2);
    Model_MachNumber  = Model_Vc / Weather_SpeedOfSound;
    if (Model_Vc < 5.0)
    {
        Model_Alpha    = 0.0;
        Model_Beta     = 0.0;
        Model_AlphaDot = 0.0;
        Model_BetaDot  = 0.0;
    }
    else
    {
        uw2            = Model_U * Model_U + Model_W * Model_W;
        Model_Alpha    = atan2(Model_W, Model_U);
        Model_Beta     = atan2(Model_V, sqrt(uw2));
        Model_AlphaDot = (Model_U * Model_WDot - Model_W * Model_UDot) / uw2;
        Model_BetaDot  = (uw2 * Model_VDot - Model_V * (Model_U * Model_UDot + Model_W * Model_WDot)) /
                         ((Model_U * Model_U + Model_V * Model_V + Model_W * Model_W) * sqrt(uw2));
    }
    Model_AlphaWing = Model_Alpha + Aero_WingIncidence;
    MaxAlpha        = Aero_AeroMaxAlpha();
    if (fabs(Model_Alpha) > MaxAlpha - 0.05)
    {
        Model_Stalling = true;
    }
    IOLib_StickShaker(Model_Stalling);

    Model_Clfw       = Aero_AeroClfw();
    Model_Cl         = Aero_AeroCl();
    Model_ClTail     = Aero_AeroClTail();
    Model_Cd         = Aero_AeroCd();
    Model_CyBeta     = Aero_AeroCyBeta();
    Model_CyDr       = Aero_AeroCyDr();
    Model_Cm0        = Aero_AeroCm0();
    Model_CmAlpha    = Aero_AeroCmAlpha();
    Model_CmDe       = Aero_AeroCmDe();
    Model_CmQ        = Aero_AeroCmQ();
    Model_CmAlphaDot = Aero_AeroCmAlphaDot();
    Model_ClBeta     = Aero_AeroClBeta();
    Model_ClDr       = Aero_AeroClDr();
    Model_ClDa       = Aero_AeroClDa();
    Model_ClP        = Aero_AeroClP();
    Model_ClR        = Aero_AeroClR();
    Model_CnBeta     = Aero_AeroCnBeta();
    Model_CnBetaDot  = Aero_AeroCnBetaDot();
    Model_CnDr       = Aero_AeroCnDr();
    Model_CnDa       = Aero_AeroCnDa();
    Model_CnP        = Aero_AeroCnP();
    Model_CnR        = Aero_AeroCnR();

    Model_Thrust = AeroLink_EngPkt.EngineThrustX;
    Model_Drag   = 0.5 * Weather_Rho * Vc2 * Aero_s * Model_Cd;
    if (Model_Drag < 0.0)
    {
        Model_Drag = 0.0;
    }
    Model_Lift = 0.5 * Weather_Rho * Vc2 * Aero_s * (Model_Cl + Model_ClTail * De) +
                 AeroLink_EngPkt.EngineThrustZ;
    Model_SideForce = 0.5 * Weather_Rho * Vc2 * Aero_s * (Model_CyDr * Dr + Model_CyBeta * Model_Beta) +
                      AeroLink_EngPkt.EngineThrustY;

    Model_GForce = Model_Lift / OneG;
    Model_GForce = Maths_Limit(Model_GForce, -6.0, 10.0);
    if (Model_GForce > Model_GMax)
    {
        Model_GMax = Model_GForce;
    }
    else if (Model_GForce < Model_GMin)
    {
        Model_GMin = Model_GForce;
    }

    SinAlpha = sin(Model_Alpha);
    CosAlpha = cos(Model_Alpha);
    SinPitch = sin(Model_Pitch);
    CosPitch = cos(Model_Pitch);
    SinRoll  = sin(Model_Roll);
    CosRoll  = cos(Model_Roll);

    Model_XForce = Model_Thrust - Model_Drag * CosAlpha + Model_Lift * SinAlpha -
                   Aero_Mass * Model_G * SinPitch + Gear_Fx;

    if (Model_OnTheGround)
    {
        Model_XForce -= Aero_Mass * Model_G * sin(DTED_Incline());
    }

    Model_YForce = Model_SideForce + Aero_Mass * Model_G * SinRoll * CosPitch + Gear_Fy;
    Model_ZForce = -Model_Lift * CosAlpha - Model_Drag * SinAlpha +
                   Aero_Mass * Model_G * CosPitch * CosRoll + Gear_Fz;

    Model_UDot = Model_XForce / Aero_Mass - Model_Q * Model_W + Model_R * Model_V;
    Model_VDot = Model_YForce / Aero_Mass - Model_R * Model_U + Model_P * Model_W;
    Model_WDot = Model_ZForce / Aero_Mass - Model_P * Model_V + Model_Q * Model_U;

    U_aero = Maths_Integrate(U_aero, Model_UDot);
    V_aero = Maths_Integrate(V_aero, Model_VDot);
    W_aero = Maths_Integrate(W_aero, Model_WDot);

    if (U_aero < 0.0)
    {
        U_aero = 0.0;
    }

    Model_U = U_aero + Weather_WindN * Model_A11 + Weather_WindE * Model_A21;
    Model_V = V_aero + Weather_WindN * Model_A12 + Weather_WindE * Model_A22;
    Model_W = W_aero + Weather_WindN * Model_A13 + Weather_WindE * Model_A23;

    Model_U = Model_U + Weather_UTurb * Model_A11 + Weather_VTurb * Model_A21 +
              Weather_WTurb * Model_A31;
    Model_V = Model_V + Weather_UTurb * Model_A12 + Weather_VTurb * Model_A22 +
              Weather_WTurb * Model_A32;
    Model_W = Model_W + Weather_UTurb * Model_A13 + Weather_VTurb * Model_A23 +
              Weather_WTurb * Model_A33;

    if (Model_U < 0.1)
    {
        Model_V = 0.0;
        V_aero  = 0.0;
    }

    Model_Vn = Model_U * Model_A11 + Model_V * Model_A12 + Model_W * Model_A13 - Weather_WindN;
    Model_Ve = Model_U * Model_A21 + Model_V * Model_A22 + Model_W * Model_A23 - Weather_WindE;
    Model_Vd = Model_U * Model_A31 + Model_V * Model_A32 + Model_W * Model_A33;

    if (!AeroLink_Freezing)
    {
        Model_dLongitude = Model_Ve / ((Model_EarthRadius - Model_Pz) * cos(Model_Latitude));
        Model_dLatitude  = Model_Vn / (Model_EarthRadius - Model_Pz);
        Model_Latitude = Maths_Double_Integrate(Model_Latitude, Model_dLatitude);
        Model_Longitude = Maths_Double_Integrate(Model_Longitude, Model_dLongitude);
        Model_Pz = Maths_Integrate(Model_Pz, Model_Vd);
    }

    Pstab     = Model_P * CosAlpha + Model_R * SinAlpha;
    Rstab     = Model_R * CosAlpha - Model_P * SinAlpha;

    Model_Pmt = 0.5 * Weather_Rho * Vc2 * Aero_s * Aero_CBar *
                (Model_Cm0 + Model_CmAlpha * Model_AlphaWing + Model_CmDe * De) +
                0.25 * Weather_Rho * Model_Vc * Aero_s * Aero_CBar * Aero_CBar *
                (Model_CmQ * Model_Q + Model_CmAlphaDot * Model_AlphaDot);
    Model_Rmt = 0.5 * Weather_Rho * Vc2 * Aero_s * Aero_b *
                (Model_ClBeta * Model_Beta + Model_ClDa * Da + Model_ClDr * Dr) +
                0.25 * Weather_Rho * Model_Vc * Aero_s * Aero_b * Aero_b *
                (Model_ClP * Pstab + Model_ClR * Rstab);
    Model_Ymt = 0.5 * Weather_Rho * Vc2 * Aero_s * Aero_b *
                (Model_CnBeta * Model_Beta + Model_CnDr * Dr + Model_CnDa * Da) +
                0.25 * Weather_Rho * Model_Vc * Aero_s * Aero_b * Aero_b *
                (Model_CnBetaDot * Model_BetaDot + Model_CnP * Pstab + Model_CnR * Rstab);

    Pm = Model_Pmt + Model_Lift * (Aero_CgPosition - 0.25) * Aero_CBar * CosAlpha +
         Model_Drag * (Aero_CgPosition - 0.25) * Aero_CBar * SinAlpha +
         AeroLink_EngPkt.EnginePMT + Gear_My;
    Rm = Model_Rmt * CosAlpha - Model_Ymt * SinAlpha +
         AeroLink_EngPkt.EngineRMT + Gear_Mx;
    Ym = Model_Ymt * CosAlpha + Model_Rmt * SinAlpha +
         Model_SideForce * (Aero_CgPosition - 0.25) * Aero_CBar +
         AeroLink_EngPkt.EngineYMT + Gear_Mz;

    Model_PDot = (Rm + (Aero_Iyy - Aero_Izz) * Model_Q * Model_R +
                  Aero_Ixz * (Model_RDot + Model_P * Model_Q)) / Aero_Ixx;
    Model_QDot = (Pm + (Aero_Izz - Aero_Ixx) * Model_R * Model_P +
                  Aero_Ixz * (Model_R * Model_R - Model_P * Model_P)) / Aero_Iyy;
    Model_RDot = (Ym + (Aero_Ixx - Aero_Iyy) * Model_P * Model_Q +
                  Aero_Ixz * (Model_PDot - Model_Q * Model_R)) / Aero_Izz;
    Model_P = Maths_Integrate(Model_P, Model_PDot);
    Model_Q = Maths_Integrate(Model_Q, Model_QDot);
    Model_R = Maths_Integrate(Model_R, Model_RDot);

    Model_P = Maths_Limit(Model_P, -M_PI, M_PI);
    Model_P = Maths_Normalise(Model_P);
    Model_Q = Maths_Normalise(Model_Q);
    Model_R = Maths_Normalise(Model_R);

    if (Model_U < 1.0) 
    {
        Model_R = 0.0;
    }

    Lambda = 25.0 * (1.0 - (Model_e0 * Model_e0 + Model_e1 * Model_e1 +
                            Model_e2 * Model_e2 + Model_e3 * Model_e3));
    e0Dot = -0.5 * (Model_e1 * Model_P + Model_e2 * Model_Q + Model_e3 * Model_R) +
            Lambda * Model_e0;
    e1Dot = 0.5 * (Model_e0 * Model_P + Model_e2 * Model_R -
                   Model_e3 * Model_Q) + Lambda * Model_e1;
    e2Dot = 0.5 * (Model_e0 * Model_Q + Model_e3 * Model_P - Model_e1 * Model_R) +
            Lambda * Model_e2;
    e3Dot = 0.5 * (Model_e0 * Model_R + Model_e1 * Model_Q -
                   Model_e2 * Model_P) + Lambda * Model_e3;
    Model_e0 = Maths_Integrate(Model_e0, e0Dot);
    Model_e1 = Maths_Integrate(Model_e1, e1Dot);
    Model_e2 = Maths_Integrate(Model_e2, e2Dot);
    Model_e3 = Maths_Integrate(Model_e3, e3Dot);
    Model_SetDCM();
    SetEyePosition();
    t = -Model_A31;
    t = Maths_Limit(t, -1.0, 1.0);
    Model_Pitch = asin(t);
    Model_Roll  = atan2(Model_A32, Model_A33);
    Model_Yaw   = atan2(Model_A21, Model_A11);

    Gear_GearModel();
}

/* ----------------------------------------------- */
void Model_SetDCM()
{
    float e00, e11, e22, e33;

    e00       = Model_e0 * Model_e0;
    e11       = Model_e1 * Model_e1;
    e22       = Model_e2 * Model_e2;
    e33       = Model_e3 * Model_e3;
    Model_A11 = e00 + e11 - e22 - e33;
    Model_A12 = 2.0 * (Model_e1 * Model_e2 - Model_e0 * Model_e3);
    Model_A13 = 2.0 * (Model_e0 * Model_e2 + Model_e1 * Model_e3);
    Model_A21 = 2.0 * (Model_e1 * Model_e2 + Model_e0 * Model_e3);
    Model_A22 = e00 - e11 + e22 - e33;
    Model_A23 = 2.0 * (Model_e2 * Model_e3 - Model_e0 * Model_e1);
    Model_A31 = 2.0 * (Model_e1 * Model_e3 - Model_e0 * Model_e2);
    Model_A32 = 2.0 * (Model_e2 * Model_e3 + Model_e0 * Model_e1);
    Model_A33 = e00 - e11 - e22 + e33;
}

/* ----------------------------------------------- */
void SetEyePosition()
{
    Model_Ex = Model_A11 * Aero_EyeXStation + Model_A12 * Aero_EyeYStation + Model_A13 * Aero_EyeZStation;
    Model_Ey = Model_A21 * Aero_EyeXStation + Model_A22 * Aero_EyeYStation + Model_A23 * Aero_EyeZStation;
    Model_Ez = Model_A31 * Aero_EyeXStation + Model_A32 * Aero_EyeYStation + Model_A33 * Aero_EyeZStation;
}

/* ----------------------------------------------- */
void Model_SetQuarternions()
{
    float p, r, y;
    float sp, cp, sr, cr, sy, cy;

    p        = Model_Pitch * 0.5;
    r        = Model_Roll * 0.5;
    y        = Model_Yaw * 0.5;
    sp       = sin(p);
    cp       = cos(p);
    sr       = sin(r);
    cr       = cos(r);
    sy       = sin(y);
    cy       = cos(y);
    Model_e0 = cr * cp * cy + sr * sp * sy;
    Model_e1 = sr * cp * cy - cr * sp * sy;
    Model_e2 = cr * sp * cy + sr * cp * sy;
    Model_e3 = cr * cp * sy - sr * sp * cy;
}

/* ----------------------------------------------- */
bool oldModel_Autotrim(float *de)
{
    float cm0;
    float cmalpha;
    float cmde;
    float alphawing;

    cm0       = Aero_AeroCm0();
    cmalpha   = Aero_AeroCmAlpha();
    cmde      = Aero_AeroCmDe();
    alphawing = Model_Alpha + Aero_WingIncidence;
    *de       = (-cm0 - cmalpha * alphawing) / cmde / Aero_ElevatorGain;
    return true;
}

/* ----------------------------------------------- */
void Model_SetTrim(float VSpeed)
{
    Model_Autotrim(&Model_ElevatorTrim);
}

/* ----------------------------------------------- */
bool Model_Autotrim(float *elevator)
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
    double       xforce;
    double       zforce;
    double       w;
    double       Gamma;
    double       Pitch;
    unsigned int tcount;
    float        h;
	float        u;
	float        VSpeed = 0.0;  // for now ***

return false;
printf("AutoTrim starting\n"); // ***
fflush(stdout); // ***
    FCS_AutoTrimming = true;  /* inhibit transients in engine model */

    h           = -Model_Pz;
	u           = Model_U;
    de          = 0.0;
    Alpha       = 0.0;
    Gamma       = atan2(VSpeed, Model_U);
    tcount      = 0;

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

        Thrust = (double) AeroLink_EngPkt.EngineThrustX;

        Cd = (double) Aero_AeroCd();
        Drag = 0.5 * (double) Weather_Rho * Vc2 * (double) Aero_s * Cd;

        Cl = (double) Aero_AeroCl();
        ClTail = (double) Aero_AeroClTail();
        Lift = 0.5 * (double) Weather_Rho * Vc2 * (double) Aero_s * (Cl + ClTail * de) + (double) AeroLink_EngPkt.EngineThrustZ;

        xforce = Thrust - Drag * cos(Alpha) + Lift * sin(Alpha) - (double) Aero_Mass * (double) Model_G * sin(Pitch) + Gear_Fx;
        zforce = -Lift * cos(Alpha) - Drag * sin(Alpha) + (double) Aero_Mass * (double) Model_G * cos(Pitch) + Gear_Fz;

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
			 (double) AeroLink_EngPkt.EnginePMT) / (0.5 * (double) Weather_Rho * Vc2 * (double) Aero_s * (double) Aero_CBar) - Cm0 -CmAlpha * AlphaWing) / CmDe;

        tcount = tcount + 1;
        if (tcount > 100000)
        {
            printf("Trim failure\n");
            printf("xforce=%7.0f zforce=%7.0f alpha=%5.2f de=%5.2f\n", xforce, zforce, Alpha, de);
            printf("Thrust=%7.0f Drag=%7.0f Lift=%7.0f mg=%7.0f\n", Thrust, Drag, Lift, (double) Aero_Mass * (double) Model_G);
			*elevator = 0.0;
fflush(stdout); // ***
            return false;
        }
        //printf("%i %f %f %f\n", tcount, xforce, zforce, de); // ***
    } while (!((fabs(xforce) < 0.5) && (fabs(zforce) < 0.5)));

    printf("Autotrim: converged after %d steps\n", tcount);
    printf("Thrust=%7.0f Drag=%7.0f Lift=%7.0f mg=%7.0f\n", Thrust, Drag, Lift, (double) Aero_Mass * (double) Model_G);
fflush(stdout); // ***
	
    FCS_AutoTrimming     = false;
	*elevator = de / Aero_ElevatorGain;
	
	return true;
}

/* -------------------------------------------------------------------- */
void Model_Pushback(float hdg, bool reset)
/*
    pushback distance 82m
    pushback speed 1.5 m/s (3 Kt)
    pushback rate 3 deg/s
*/

{
    if (reset)
    {
        Pushingback = true;
        PushbackDistance = 0.0;
        PushbackMaxDistance = 60.0;
        PushbackSpeed = 0.0;
        PushbackHdg = hdg;
        PushbackHdg = Maths_Normalise(PushbackHdg);
        PushbackInitHdg = Model_Yaw;
        PushbackHdgRate = 0.0;
    }
    
    if (PushbackDistance < 2.25)
    {
        PushbackSpeed += 0.01;
    }
    else if (PushbackDistance > (PushbackMaxDistance - 2.25))
    {
        PushbackSpeed -= 0.01;
        if (PushbackSpeed < 0.0)
        {
            float a = Model_Yaw - PushbackHdg;
            float b = Model_Yaw - PushbackInitHdg;
            float c = PushbackHdg - PushbackInitHdg;
            
            PushbackSpeed = 0.0;

            a = Maths_Normalise(a);
            b = Maths_Normalise(b);
            c = Maths_Normalise(c);
            
            if (fabs(a) < 0.001)
            {
                Pushingback = false;
                return;
            }
            else if (fabs(b) < 0.0785)  /* first 4.5 deg */
            {
                PushbackHdgRate += 0.0003491;  /* 0 -> 3 deg/s in 3s */
            }
            else if (fabs(a) < 0.0785)  /* last 4.5 deg */
            {
                PushbackHdgRate -= 0.0003491;  /* 3 deg/s -> 0 in 3s */
            }
            else
            {
                PushbackHdgRate = 0.0524;  /* othewrwise 3 deg/s */
            }

            Model_Yaw = Maths_Integrate(Model_Yaw, (c > 0.0) ? PushbackHdgRate : -PushbackHdgRate);
            Model_Yaw = Maths_Normalise(Model_Yaw);
        }
    }
    else
    {
        PushbackSpeed = 1.5;
    }

    PushbackDistance = Maths_Double_Integrate(PushbackDistance, PushbackSpeed);

    Model_U = -PushbackSpeed;   /* code identical to model.c except allow backward motion */
    Model_V = 0.0;              /* and wheel motion rather than CG motion */
    Model_W = 0.0;
    Model_Pitch = 0.0;
    Model_Roll=0.0;
    Model_P = 0.0;
    Model_Q = 0.0;
    Model_R = 0.0;
    Model_Vn = Model_U * Model_A11 + Model_V * Model_A12 + Model_W * Model_A13;
    Model_Ve = Model_U * Model_A21 + Model_V * Model_A22 + Model_W * Model_A23;

    Model_dLongitude = (double) Model_Ve / (Model_EarthRadius * cos(Model_Latitude));
    Model_dLatitude  = (double) Model_Vn / Model_EarthRadius;
    Model_Latitude = Maths_Double_Integrate(Model_Latitude, Model_dLatitude);
    Model_Longitude = Maths_Double_Integrate(Model_Longitude, Model_dLongitude);
    Model_SetQuarternions();
    Model_SetDCM();
    SetEyePosition();
}

/* ----------------------------------------------- */
void BEGIN_Model()
{
    Model_Pitch        = 0.0;
    Model_Roll         = 0.0;
    Model_Yaw          = 0.0;
    Model_P            = 0.0;
    Model_Q            = 0.0;
    Model_R            = 0.0;
    Model_PDot         = 0.0;
    Model_QDot         = 0.0;
    Model_RDot         = 0.0;
    Model_Vn           = 0.0;
    Model_Ve           = 0.0;
    Model_Vd           = 0.0;
    Model_Pz           = Aero_CGHeight;
    Model_Latitude     = 0.0;
    Model_Longitude    = 0.0;
    Model_U            = 0.0;
    Model_V            = 0.0;
    Model_W            = 0.0;
    Model_UDot         = 0.0;
    Model_VDot         = 0.0;
    Model_WDot         = 0.0;
    Model_Pmt          = 0.0;
    Model_Rmt          = 0.0;
    Model_Ymt          = 0.0;
    Model_Alpha        = 0.0;
    Model_Beta         = 0.0;
    Model_AlphaDot     = 0.0;
    Model_BetaDot      = 0.0;
    Model_Stalling     = false;
    Model_Cl           = 0.0;
    Model_Cd           = 0.0;
    Model_Lift         = 0.0;
    Model_Thrust       = 0.0;
    Model_Drag         = 0.0;
    Model_SideForce    = 0.0;
    Model_XForce       = 0.0;
    Model_YForce       = 0.0;
    Model_ZForce       = 0.0;
    Model_Vc           = 0.0;
    Model_MachNumber   = 0.0;
    Model_Elevator     = 0.0;
    Model_Aileron      = 0.0;
    Model_Rudder       = 0.0;
    Model_Tiller       = 0.0;
    Model_ElevatorTrim = 0.0;
    Model_AileronTrim  = 0.0;
    Model_RudderTrim   = 0.0;
    Model_LeftBrake    = 0.0;
    Model_RightBrake   = 0.0;
    Model_Flaps        = 0.0;
    Model_Gear         = 0.0;
    Model_A11          = 1.0;
    Model_A12          = 0.0;
    Model_A13          = 0.0;
    Model_A21          = 0.0;
    Model_A22          = 1.0;
    Model_A23          = 0.0;
    Model_A31          = 0.0;
    Model_A32          = 0.0;
    Model_A33          = 1.0;
    Model_GMin         = 1.0;
    Model_GMax         = 1.0;
    Model_OnTheGround  = false;
    Model_e0           = 1.0;
    Model_e1           = 0.0;
    Model_e2           = 0.0;
    Model_e3           = 0.0;

    Model_Incline      = 0.0;
    Pushingback = false;

    U_aero = 0.0;
    V_aero = 0.0;
    W_aero = 0.0;
}
