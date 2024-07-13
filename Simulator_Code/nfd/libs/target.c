/* +------------------------------+---------------------------------+
   | Module      : target.c       | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-11              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : target management library                        |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */


#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/target.h>
#include <SIM/navlib.h>

float         Target_TU;
float         Target_TV;
float         Target_TW;
float         Target_TYaw;
float         Target_TYawDot;
double        Target_TLatitude;
double        Target_TLongitude;
float         Target_TPz;
float         Target_TPitch;
float         Target_TRoll;
Target_Toggle Target_TargSwitch, Target_HUDSwitch, Target_ConeSwitch;
bool          Target_Conflict;

#define EarthRadius    6378137.0

static float TA11, TA12, TA13;
static float TA21, TA22, TA23;
static float TA31, TA32, TA33;
static float TVn, TVe, TVd;
static float TRollDem, TRollDot;

static void Transformation();

/* ------------------------------------------------------ */
static void Transformation()
{
    float TSinPitch, TCosPitch;
    float TSinRoll, TCosRoll;
    float TCosYaw, TSinYaw;
    float Tspsr, Tspcr;
    float Tcpsr, Tcpcr;
    float Tcpsy, Tcpcy;
    float Tsrsy, Tsrcy;
    float Tcrsy, Tcrcy;

    TSinPitch = sin(Target_TPitch);
    TCosPitch = cos(Target_TPitch);
    TSinRoll  = sin(Target_TRoll);
    TCosRoll  = cos(Target_TRoll);
    TSinYaw   = sin(Target_TYaw);
    TCosYaw   = cos(Target_TYaw);
    Tspsr     = TSinPitch * TSinRoll;
    Tspcr     = TSinPitch * TCosRoll;
    Tcpsr     = TCosPitch * TSinRoll;
    Tcpcr     = TCosPitch * TCosRoll;
    Tcpsy     = TCosPitch * TSinYaw;
    Tcpcy     = TCosPitch * TCosYaw;
    Tsrsy     = TSinRoll * TSinYaw;
    Tsrcy     = TSinRoll * TCosYaw;
    Tcrsy     = TCosRoll * TSinYaw;
    Tcrcy     = TCosRoll * TCosYaw;
    TA11      = Tcpcy;
    TA12      = Tspsr * TCosYaw - Tcrsy;
    TA13      = Tspcr * TCosYaw + Tsrsy;
    TA21      = Tcpsy;
    TA22      = Tspsr * TSinYaw + Tcrcy;
    TA23      = Tspcr * TSinYaw - Tsrcy;
    TA31      = -TSinPitch;
    TA32      = Tcpsr;
    TA33      = Tcpcr;
}

/* ------------------------------------------------------ */
void Target_TargPursuit(float RelDist, double Latitude, double Longitude, float Pz, float Yaw)
{
    float dnorth;
    float deast;

    dnorth            = RelDist * cos(Yaw);
    deast             = RelDist * sin(Yaw);
    Target_TLatitude  = Latitude + (double) (dnorth / EarthRadius);
    Target_TLongitude = Longitude + (double) (deast / (EarthRadius * cos(Latitude)));
    Target_TPz = Pz;
    Target_TYaw = Yaw;
    Target_TRoll = 0.0;
}

/* ------------------------------------------------------ */
void Target_TargetDynamics(float GroundLevel, float altitude, double Latitude, double Longitude)
{
    float dLatitude;
    float dLongitude;
    float TVc;

    float e;
    float b;

    if (Target_Conflict)
    {
        b = (float) NavLib_Bearing(Target_TLatitude, Target_TLongitude, Latitude, Longitude);
        e = (float) (b - Target_TYaw);
        e = Maths_Normalise(e);
        Target_TYawDot = e * 0.3;
        Target_TYawDot = Maths_Limit(Target_TYawDot, -0.1, 0.1);
        TVd = (altitude - Target_TPz) * 10.0 / 200.0;
        TVd = Maths_Limit(TVd, -10.0, 10.0);

        if (NavLib_Distance(Latitude, Longitude, Target_TLatitude, Target_TLongitude) < 500.0)
        {
            Target_Conflict = false;
            Target_TYawDot  = 0.0;
        }
    }

    Transformation();
    TVc = sqrt(Target_TU * Target_TU + Target_TV * Target_TV + Target_TW * Target_TW);
    TVn = Target_TU * TA11 + Target_TV * TA12 + Target_TW * TA13;
    TVe = Target_TU * TA21 + Target_TV * TA22 + Target_TW * TA23;
    if (!Target_Conflict)
    {
        TVd = Target_TU * TA31 + Target_TV * TA32 + Target_TW * TA33;
    }
    dLongitude = TVe / ((EarthRadius - Target_TPz) * cos(Target_TLatitude));
    dLatitude  = TVn / (EarthRadius - Target_TPz);
    Target_TLatitude = Maths_Double_Integrate(Target_TLatitude, dLatitude);
    Target_TLongitude = Maths_Double_Integrate(Target_TLongitude, dLongitude);
    Target_TPz = Maths_Integrate(Target_TPz, TVd);
    if (Target_TPz >= GroundLevel - 0.001f)
    {
        Target_TPz = GroundLevel;
    }
    TRollDem = atan(TVc * Target_TYawDot / 9.81);
    if (fabs(Target_TRoll - TRollDem) <= 0.01)
    {
        TRollDot = 0.0;
    }
    else
    {
        if (TRollDem < Target_TRoll)
        {
            TRollDot = Maths_Rads(-120.00);
        }
        else
        {
            TRollDot = Maths_Rads(120.00);
        }
    }
    Target_TRoll = Maths_Integrate(Target_TRoll, TRollDot);
    Target_TYaw = Maths_Integrate(Target_TYaw, Target_TYawDot);
}

/* ------------------------------------------------------ */
void BEGIN_Target()
{
    Target_TargSwitch = Target_OFF;
    Target_HUDSwitch  = Target_OFF;
    Target_ConeSwitch = Target_OFF;
    Target_TRoll      = 0.0;
    Target_TPitch     = 0.0;
    Target_TYaw       = 0.0;
    TRollDot          = 0.0;
    Target_TYawDot    = 0.0;
    Target_TLatitude  = 0.0;
    Target_TLongitude = 0.0;
    Target_TPz        = 0.0;
    Target_TU         = 0.0;
    Target_TV         = 0.0;
    Target_TW         = 0.0;
    Target_Conflict   = false;
}
