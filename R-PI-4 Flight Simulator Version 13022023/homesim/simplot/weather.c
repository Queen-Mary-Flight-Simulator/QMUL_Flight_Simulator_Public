/* +------------------------------+---------------------------------+
   | Module      : weather.c      | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-02-08      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : weather model library                            |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/weather.h>

float        Weather_Rho;
float        Weather_Pressure;
float        Weather_PressureAltitude;
float        Weather_DensityRatio;
float        Weather_Turbulence_Level;
float        Weather_Turbulence_Intermittency;
float        Weather_WindN, Weather_WindE;
unsigned int Weather_RegionalQNH;
float        Weather_GroundTemperature;
float        Weather_SpeedOfSound;
float        Weather_ISADeviation;
float        Weather_CloudBase;
bool         Weather_InCloud, Weather_OldInCloud;
bool         Weather_OutCloud, Weather_OldOutCloud;
float        Weather_Visibility, Weather_CloudVis;
float        Weather_LightsIntensity;
float        Weather_SkyLightLevel;
float        Weather_GroundLightLevel;
float        Weather_FogBrightness;
float        Weather_SkyRedLevel;
float        Weather_SkyGreenLevel;
float        Weather_SkyBlueLevel;
float        Weather_UTurb, Weather_VTurb, Weather_WTurb;
bool         Weather_DayMode;

const float  f2m = 0.3048;      /* ft -> m */
const float  m2f = 3.28084;     /* m -> ft */

const double Rs    = 8314.32;   /* Nm/(kmol K), gas constant */
const double M0    = 28.9644;   /* kg/kmol, mean molecular weight of air */
const double g0    = 9.80665;   /* m/s^2, acceleration of gravity at 45.5425 deg lat. */
const double r0    = 6356766.0; /* m, Earth radius at g0 */
const double P0    = 101325.0;  /* Pa, air pressure at g0 */
const double T0    = 288.15;    /* K, standard sea-level temperature */
const double Td    = 273.15;    /* K, 0 degrees C */
const double Gamma = 1.40;      /* Ratio of Specific heats for ideal diatomic gas */

static int   Seed   = 123456789;
static float frt    = 20.0;                    /* frametime ms */
static float r      = 0.7;                     /* decay rate */
static int   ngusts = 0;                       /* number of gusts per sec */

static int   old_ngusts                   = 0; /* remember changes in no. of gusts */
static float old_Turbulence_Level         = 0.0;
static float old_Turbulence_Intermittency = 0.0;

static float wspeed = 0.0;
static float wdir   = 0.0;

static float VSTEP(float P, float ff, float xmu);
static void FDC(int *IDC3, int MDC3, float *F3, float *DF3, int *MSW3);
static float RandomNumber(void);
static void UpdateWind();

/* --------------------------------------------------------- */
float Weather_Mach_to_Kts(float Z, float MachNo) /* Z geometric height */
{
    double H;                        /* geopotential height m */
    double T;                        /* temperature K */
    double sos;                      /* speed of sound m/s */
    
    H = r0 * (double) Z / (r0 + (double) Z);
    T = T0 - 0.0065 * H;

    sos = (float) sqrt((Gamma * Rs / M0) * T);
    return (float) (MachNo * sos * 1.943844);
}

/* -------------------------------------------------------------- */
float Weather_Temperature(float Z) /* ambient temperature deg K, Z geometric height m */
{
    double H;                      /* geopotential height m */

    H = r0 * (double) Z / (r0 + (double) Z);
    return T0 - 0.0065 * H;
}

/* -------------------------------------------------------------- */
float Weather_Mach(float Z, float V) /* Z geometric height m, V true airspeed m/s */
{
    double H;                        /* geopotential height m */
    double T;                        /* temperature K */
    double sos;                      /* speed of sound m/s */

    H = r0 * (double) Z / (r0 + (double) Z);
    T = T0 - 0.0065 * H;

    sos = (float) sqrt((Gamma * Rs / M0) * T);
    return V / sos;
}

/* -------------------------------------------------------------- */
void Weather_WeatherModel(bool turbulence, float Pz, float U)
{
    double As; /* constant */
    float  Z;  /* geometric height m */
    double H;  /* geopotential height m */
    double P;  /* pressure Pa */
    double T;  /* temperature K */

    As = g0 * M0 / Rs;
    Z  = -Pz;

    H = r0 * (double) Z / (r0 + (double) Z);
    T = T0 - 0.0065 * H;
    P = P0 * exp(As * log(T0 / T) / (-0.0065));

    Weather_Rho              = (float) ((P / T) * (M0 / Rs));
    Weather_DensityRatio     = Weather_Rho / 1.225;
    Weather_SpeedOfSound     = (float) sqrt((Gamma * Rs / M0) * T);
    Weather_Pressure         = P;
    Weather_PressureAltitude = (float) (P * 0.00029536); /* In Hg */
    UpdateWind();
    if (turbulence)
    {
        Weather_Gusts(&Weather_UTurb, &Weather_VTurb, &Weather_WTurb, U, Z);
    }
}

/* -------------------------------------------------------------- */
void Weather_SetWind(float WindSpeed, float WindDirection, bool reset)
{
    wspeed = WindSpeed;
    wdir   = WindDirection;
    wdir = Maths_Normalise(wdir);
    if (reset)
    {
        Weather_WindN = 0.0;
        Weather_WindE = 0.0;
    }
}

/* -------------------------------------------------------------- */
static void UpdateWind()
{
    float ws;

    ws = sqrt(Weather_WindN * Weather_WindN + Weather_WindE * Weather_WindE);
    if (ws < (wspeed - 0.02))
    {
        ws = ws + 0.02;
    }
    else if (ws > (wspeed + 0.02))
    {
        ws = ws - 0.02;
    }
    Weather_WindN = ws * cos(wdir);
    Weather_WindE = ws * sin(wdir);
}

/* -------------------------------------------------------------- */
void Weather_Gusts(float *ug, float *vg, float *wg, float AirSpeed, float h)
{
    static float p, a1, a2, a3, h0, h1, h2, h3, r1, r2, r3, v1, v2, v3;
    static float df1, df2, ff1, ff2, fh1, fh2, fh3, ff3, df3;
    static float f1x, f2x, f3x, f1y, f2y, f3y, f1z, f3z;
    static float f2z, vv1, vv2, vv3;
    static float rh01, rh12, rh23, xmu;
    static int   mdc1, mdc2, mdc3, ibp1, ibp2, ibp3, mbp1, mbp2, mbp3;
    static float df1x, df1y, df1z, df2x, df2y, df2z, df3x, df3y, df3z;
    static int   mrt1, mrt2, mrt3, idc1x, idc1y, idc1z, idc2x, idc2y, idc2z;
    static int   idc3x, idc3y, idc3z;
    static float delv3, delv2, delv1;
    static int   msw1x, msw1y, msw1z, msw2x, msw2y, msw2z, msw3x, msw3y, msw3z;
    static float sigma;
    static float frtime;

    if (Weather_Turbulence_Level < 0.05)
    {
        return;
    }

    AirSpeed = AirSpeed * m2f;
    h        = h * m2f;

    ngusts = (int) (AirSpeed / 50.0);
    if ((ngusts != old_ngusts) ||
        (fabs(Weather_Turbulence_Level - old_Turbulence_Level) > 0.01) ||
        (fabs(Weather_Turbulence_Intermittency - old_Turbulence_Intermittency) > 0.01))
    {
        old_ngusts                   = ngusts;
        old_Turbulence_Level         = Weather_Turbulence_Level;
        old_Turbulence_Intermittency = Weather_Turbulence_Intermittency;

        h0   = 2500.0;
        h1   = 800.0;
        h2   = 200.0;
        h3   = 50.0;
        rh01 = 1.0 / (h0 - h1);
        rh12 = 1.0 / (h1 - h2);
        rh23 = 1.0 / (h2 - h3);

        fh1 = 1.0;
        fh2 = 1.0;
        fh3 = 1.0;

        sigma = Weather_Turbulence_Level * 10.0; /* turb 0 to 1.0 -> sigma 0 .. 10.0 */
        xmu   = sqrt(2.0 * (1.0 - Weather_Turbulence_Intermittency)) / sigma;

        ff3 = 1.0 - Weather_Turbulence_Intermittency;
        ff2 = ff3 / sqrt(2.0);
        ff1 = ff3 / 2.0;

        frtime = frt / 1000.0;
        mbp1   = (int) (16.0 / (frtime * (float) ngusts));
        mbp2   = (int) (4.0 / (frtime * (float) ngusts));
        mbp3   = (int) (1.0 / (frtime * (float) ngusts));

        mdc1 = (int) (240.0 / (frtime * (float) ngusts));
        mdc2 = (int) (60.0 / (frtime * (float) ngusts));
        mdc3 = (int) (15.0 / (frtime * (float) ngusts));

        mrt1 = (int) (80.0 / (frtime * (float) ngusts));
        mrt2 = (int) (20.0 / (frtime * (float) ngusts));
        mrt3 = (int) (5.0 / (frtime * (float) ngusts));

        df1  = 2.0 / mrt1;
        df2  = 2.0 / mrt2;
        df3  = 2.0 / mrt3;
        df1x = df1;
        df1y = df1;
        df1z = df1;
        df2x = df2;
        df2y = df2;
        df2z = df2;
        df3x = df3;
        df3y = df3;
        df3z = df3;

        a1 = 1.0;
        a2 = a1 * powf((float) mbp2 / (float) mbp1, 0.333333);
        a3 = a1 * powf((float) mbp3 / (float) mbp1, 0.333333);

        f1x = 0.0;
        f2x = 0.0;
        f3x = 0.0;
        f1y = 0.0;
        f2y = 0.0;
        f3y = 0.0;
        f1z = 0.0;
        f2y = 0.0;
        f3z = 0.0;

        idc1x = 0;
        idc1y = 0;
        idc1z = 0;
        idc2x = 0;
        idc2y = 0;
        idc2z = 0;
        idc3x = 0;
        idc3y = 0;
        idc3z = 0;

        msw1x = 0;
        msw1y = 0;
        msw1z = 0;
        msw2x = 0;
        msw2y = 0;
        msw2z = 0;
        msw3x = 0;
        msw3y = 0;
        msw3z = 0;

        ibp1 = 0;
        ibp2 = 0;
        ibp3 = 0;

        v1 = 0.0;
        v2 = 0.0;
        v3 = 0.0;

        r3 = powf(r, 1.0 / (float) mbp3);
        r2 = powf(r, 1.0 / (float) mbp2);
        r1 = powf(r, 1.0 / (float) mbp1);
    }

    ibp3 = ibp3 % mbp3 + 1;
    if (ibp3 == 1)
    {
        p     = RandomNumber();
        delv3 = VSTEP(p, ff3, xmu) / mbp3;
        vv3   = v3;
    }
    v3 = vv3 * powf(r3, (float) ibp3) + delv3 * ibp3;

    ibp2 = ibp2 % mbp2 + 1;
    if (ibp2 == 1)
    {
        p     = RandomNumber();
        delv2 = VSTEP(p, ff2, xmu) / mbp2;
        vv2   = v2;
    }
    v2 = vv2 * powf(r2, (float) ibp2) + delv2 * ibp2;

    ibp1 = ibp1 % mbp1 + 1;
    if (ibp1 == 1)
    {
        p     = RandomNumber();
        delv1 = VSTEP(p, ff1, xmu) / mbp1;
        vv1   = v1;
    }
    v1 = vv1 * powf(r1, (float) ibp1) + delv1 * ibp1;

    FDC(&idc1x, mdc1, &f1x, &df1x, &msw1x);
    FDC(&idc1y, mdc1, &f1y, &df1y, &msw1y);
    FDC(&idc1z, mdc1, &f1z, &df1z, &msw1z);
    FDC(&idc2x, mdc2, &f2x, &df2x, &msw2x);
    FDC(&idc2y, mdc2, &f2y, &df2y, &msw2y);
    FDC(&idc2z, mdc2, &f2z, &df2z, &msw2z);
    FDC(&idc3x, mdc3, &f3x, &df3x, &msw3x);
    FDC(&idc3y, mdc3, &f3y, &df3y, &msw3y);
    FDC(&idc3z, mdc3, &f3z, &df3z, &msw3z);

    if (h >= h0)
    {
        fh1 = 1.0;
        fh2 = 1.0;
        fh3 = 1.0;
    }
    else
    {
        fh1 = (h - h1) * rh01;
        if (h < h1)
        {
            fh1 = 0.0;
            fh2 = (h - h2) * rh12;
            if (h < h2)
            {
                fh1 = 0.0;
                fh2 = 0.0;
                fh3 = (h - h3) * rh23;
                if (h < h3)
                {
                    fh1 = 0.0;
                    fh2 = 0.0;
                    fh3 = 0.0;
                }
            }
        }
    }

    *ug = f2m * (a1 * f1x * v1 + a2 * f2x * v2 + a3 * f3x * v3);
    *vg = f2m * (a1 * f1y * v1 + a2 * f2y * v2 + a3 * f3y * v3);
    *wg = f2m * (a1 * f1z * v1 * fh1 + a2 * f2z * v2 * fh2 + a3 * f3z * v3 * fh3);
}

/* -------------------------------------------------------------- */
static float VSTEP(float p, float ff, float xmu)
{
    static float x, dp, pp, eps, sign, epsm;

    eps = (1.0 - ff) * 0.5;
    dp  = p - 0.5;
    if (dp > eps)
    {
        sign = -1.0;
        pp   = 1.0 - p;
    }
    else
    {
        epsm = -eps;
        if (dp < epsm)
        {
            sign = 1.0;
            pp   = p;
        }
        else
        {
            return 0.0;
        }
    }

    x = pp * 2.0 / ff;
    if (x < 1.0E-10)
    {
        x = 1.0E-10;
    }
    return sign * log(x) / xmu;
}

/* -------------------------------------------------------------- */
static void FDC(int *idc3, int mdc3, float *f3, float *df3, int *msw3)
{
    static float p;

    *idc3 = *idc3 % mdc3 + 1;
    if (*idc3 == 1)
    {
        p = RandomNumber();
        if (p > 0.5)
        {
            *msw3 = 0;
            *df3  = -(*df3);
        }
        else
        {
            *msw3 = *msw3 + 1;
            if (*msw3 == 3)
            {
                *msw3 = 0;
                *df3  = -(*df3);
            }
        }
    }

    *f3 = *f3 + *df3;
    if (*f3 > 1.0)
    {
        *f3 = 1.0;
    }
    if (*f3 < -1.0)
    {
        *f3 = -1.0;
    }
}

/* -------------------------------------------------------------- */
float RandomNumber(void)
{
    int low, hi, test;

    hi   = Seed / 127773;
    low  = Seed % 127773;
    test = (16807 * low) - (2836 * hi);
    if (test > 0)
    {
        Seed = test;
    }
    else
    {
        Seed = test + 2147483647;
    }
    return (float) Seed / 2147483647.0;
}

/* -------------------------------------------------------------- */
void BEGIN_Weather()
{
    Weather_WeatherModel(false, 0.0, 0.0);
    Weather_Turbulence_Level         = 0.0;
    Weather_Turbulence_Intermittency = 0.7;
    Weather_WindN                    = 0.0;
    Weather_WindE                    = 0.0;
    Weather_GroundTemperature        = 15.0;
    Weather_ISADeviation             = 0.0;
    Weather_RegionalQNH              = 1013;
    Weather_CloudBase                = -Maths_Metres(30000.0);
    Weather_OldInCloud               = false;
    Weather_OldOutCloud              = true;
    Weather_Visibility               = 50000.0;
    Weather_CloudVis                 = 0.0;
    Weather_LightsIntensity          = 0.5;
    Weather_SkyLightLevel            = 1.0;
    Weather_GroundLightLevel         = 1.0;
    Weather_SkyRedLevel              = 0.25;
    Weather_SkyGreenLevel            = 0.50;
    Weather_SkyBlueLevel             = 0.44;
    Weather_FogBrightness            = 1.0;
    Weather_DayMode                  = true;
    Weather_UTurb                    = 0.0;
    Weather_VTurb                    = 0.0;
    Weather_WTurb                    = 0.0;
}
