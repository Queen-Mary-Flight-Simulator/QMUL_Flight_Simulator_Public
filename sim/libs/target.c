/* +------------------------------+---------------------------------+
   | Module      : target.c       | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-02-05      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
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

static double Distance(double Lat1, double Long1, double Lat2, double Long2);
static double Bearing(double Lat1, double Long1, double Lat2, double Long2);
static void Transformation();

/* ------------------------------------------------------ */
static double Distance(double Lat1, double Long1, double Lat2, double Long2)
{
    double dLat;
    double dLong;
    double d;

    dLat  = Lat2 - Lat1;
    dLong = Long2 - Long1;
    d     = EarthRadius * sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
    return d;
}

/* ------------------------------------------------------ */
static double Bearing(double Lat1, double Long1, double Lat2, double Long2)
{
    double x, y;
    double ax, ay;
    double dLat;
    double dLong;
    double d;
    double psi;

    dLat  = Lat2 - Lat1;
    dLong = Long2 - Long1;
    d     = sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
    x     = sin(Lat2) - sin(Lat1) * cos(d);
    ax    = fabs(x);
    y     = cos(Lat2) * sin(dLong) * cos(Lat1);
    ay    = fabs(y);
    psi   = atan(ay / ax);
    if (x < 0.0)
    {
        psi = M_PI - psi;
    }
    if (y < 0.0)
    {
        return -psi;
    }
    else
    {
        return psi;
    }
}

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

void Target_TargPursuit(float RelDist, double Latitude, double Longitude, float Pz, float Yaw)
{
    float dnorth;
    float deast;

    dnorth            = RelDist * cos(Yaw);
    deast             = RelDist * sin(Yaw);
    Target_TLatitude  = Latitude + (double) (dnorth / EarthRadius);
    Target_TLongitude = Longitude + (double) (deast / (EarthRadius * cos(Latitude)));
    //Target_TPz = Pz;
    //Target_TYaw = Yaw;
    Target_TRoll = 0.0;
}

void Target_TargetDynamics(float GroundLevel, float altitude, double Latitude, double Longitude)
{
    float dLatitude;
    float dLongitude;
    float TVc;

    float e;
    float b;

    if (Target_Conflict)
    {
        b = (float) Bearing(Target_TLatitude, Target_TLongitude, Latitude, Longitude);
        e = (float) (b - Target_TYaw);
        Maths_Normalise(&e);
        Target_TYawDot = e * 0.3;
        Maths_Limit(&Target_TYawDot, -0.1, 0.1);
        TVd = (altitude - Target_TPz) * 10.0 / 200.0;
        Maths_Limit(&TVd, -10.0, 10.0);

        if (Distance(Latitude, Longitude, Target_TLatitude, Target_TLongitude) < 500.0)
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
    Maths_Double_Integrate(&Target_TLatitude, dLatitude);
    Maths_Double_Integrate(&Target_TLongitude, dLongitude);
    Maths_Integrate(&Target_TPz, TVd);
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
    Maths_Integrate(&Target_TRoll, TRollDot);
    Maths_Integrate(&Target_TYaw, Target_TYawDot);
}

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
