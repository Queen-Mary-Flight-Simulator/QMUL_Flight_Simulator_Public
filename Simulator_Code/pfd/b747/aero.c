/* +---------------------------+---------------------------------+
| Module      : aero.c         | Version         : 1.1           |
| Last Edit   : 30-03-2016     | Reference Number: 02-01-01      |
|+-----------------------------+---------------------------------+
| Computer    : DELL1                                            |
| Directory   : /dja/aerosoft/cranfield/software/pfd/            |
| Compiler    : gcc 4.8.1                                        |
| OS          : Windows7                                         |
|+---------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
|+---------------------------------------------------------------+
| Description : Boeing 747-400 Aerodynamic Data                  |
|                                                                |
|+---------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#include <math.h>
#include "model.h"
#include "aero.h"

#define YawDamperEngaged true

float        Aero_Ixx, Aero_Iyy, Aero_Izz;
float        Aero_Ixz, Aero_Iyz, Aero_Ixy;
float        Aero_Mass;
float        Aero_CgPosition;

float        IxxMoment(float Mass);
float        IyyMoment(float Mass);
float        IzzMoment(float Mass);
float        IxzMoment(float Mass);

/* ------------------------------------------------------------------ */
float Aero_AeroMaxAlpha()
{
    return 0.262 + Model_Flaps * 0.1;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCz0()
{
    return -0.09 + 0.13 * Model_Flaps + 0.84 * Model_Flaps * Model_Flaps;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCz1()
{
    return 5.5 + 2.18 * Model_Flaps - 1.722 * Model_Flaps * Model_Flaps;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCl()
{
    float Cz0, Cz1, MaxAlpha;
    float Cl, DeltaAlpha, DeltaCl, AbsAlpha;

    Cz0      = Aero_AeroCz0();
    Cz1      = Aero_AeroCz1();
    MaxAlpha = Aero_AeroMaxAlpha();
    Cl       = Cz0 + Cz1 * Model_AlphaWing;
    AbsAlpha = fabs(Model_AlphaWing);
    if (AbsAlpha > MaxAlpha)
    {
        DeltaAlpha = AbsAlpha - MaxAlpha;
        DeltaCl    = 20.0 * DeltaAlpha * DeltaAlpha;
        if (Model_AlphaWing > 0.0)
        {
            DeltaCl = -DeltaCl;
        }
        Cl = Cl + DeltaCl;
    }
    return Cl;
}

/* ------------------------------------------------------------------ */
float Aero_AeroClTail()
{
    return 0.404;
}

/* ------------------------------------------------------------------ */
float Aero_AeroClfw()
{
    return 0.0;  /* N/A */
}

/* ------------------------------------------------------------------ */
float Aero_AeroCd()
{
    float Cd;

    Cd = 0.061 - 0.12 * Model_AlphaWing + 3.0 * Model_AlphaWing * Model_AlphaWing;
    if (Model_Flaps < 0.667)
    {
        Cd = Cd - (0.667 - Model_Flaps) * 0.05;
    }
    else
    {
        Cd = Cd + (Model_Flaps - 0.667) * 0.288;
    }
    return Cd + (0.028 - Model_Flaps * 0.021) * Model_Gear;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCyBeta()
{
    return -0.974;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCyDr()
{
    return 0.078;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCm0()
{
    return 0.15 - 0.16 * Model_Flaps - Model_Gear * 0.1;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCmAlpha()
{
    return -1.528;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCmDe()
{
    return -1.375;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCmQ()
{
    return -22.6 + 8.2 * Model_MachNumber * (1.0 + Model_Pz / 12192.0);
}

/* ------------------------------------------------------------------ */
float Aero_AeroCmAlphaDot()
{
    return -3.4;
}

/* ------------------------------------------------------------------ */
float Aero_AeroClBeta()
{
    return -(0.143 + Model_Flaps * 0.086 + Model_Alpha * 0.344);
}

/* ------------------------------------------------------------------ */
float Aero_AeroClDr()
{
    return 0.01 - Model_Alpha * 0.048;
}

/* ------------------------------------------------------------------ */
float Aero_AeroClDa()
{
    if (Model_Flaps < 0.05)
    {
        return 0.0403;
    }
    else
    {
        return 0.0619 + Model_Flaps * 0.0125;
    }
}

/* ------------------------------------------------------------------ */
float Aero_AeroClP()
{
    return -(0.4 + Model_Flaps * 0.13);
}

/* ------------------------------------------------------------------ */
float Aero_AeroClR()
{
    return 0.05 + 0.589 * Model_Alpha;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCnBeta()
{
    return 0.172;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCnBetaDot()
{
    return -0.025;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCnDr()
{
    return -0.045;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCnDa()
{
    return 0.0024;
}

/* ------------------------------------------------------------------ */
float Aero_AeroCnP()
{
    if (YawDamperEngaged)
    {
        return 0.034;
    }
    else
    {
        return 0.03 - Model_Flaps * 0.07 - Model_Alpha * 0.802;
    }
}

/* ------------------------------------------------------------------ */
float Aero_AeroCnR()
{
    return -(0.18 + Model_Flaps * 0.1);
}

/* ------------------------------------------------------------------ */
float IxxMoment(float Mass)
{
    if (Mass < 259000.0)
    {
        return 15.9E6 + 34.0 * (Mass - 159000.0);
    }
    else if (Mass < 286000.0)
    {
        return 19.3E6 + 188.9 * (Mass - 259000.0);
    }
    else
    {
        return 24.4E6 + 44.4 * (Mass - 286000.0);
    }
}

/* ------------------------------------------------------------------ */
float IyyMoment(float Mass)
{
    if (Mass < 254000.0)
    {
        return 29.8E6 + 136.5 * (Mass - 159000.0);
    }
    else if (Mass < 298000.0)
    {
        return 43.8E6 + 27.3 * (Mass - 254000.0);
    }
    else
    {
        return 45.0E6 + 79.7 * (Mass - 298000.0);
    }
}

/* ------------------------------------------------------------------ */
float IzzMoment(float Mass)
{
    if (Mass < 283000.0)
    {
        return 44.0E6 + 180.6 * (Mass - 159000.0);
    }
    else
    {
        return 66.4E6 + 86.1 * (Mass - 283000.0);
    }
}

/* ------------------------------------------------------------------ */
float IxzMoment(float Mass)
{
    if (Mass < 277000.0)
    {
        return 0.94E6 + 2.2 * (Mass - 159000.0);
    }
    else
    {
        return 1.21E6 + 5.88 * (Mass - 277000.0);
    }
}

/* ------------------------------------------------------------------ */
void BEGIN_Aero()
{
    Aero_Mass       = 260000.0;
    Aero_Ixx        = IxxMoment(Aero_Mass);
    Aero_Iyy        = IyyMoment(Aero_Mass);
    Aero_Izz        = IzzMoment(Aero_Mass);
    Aero_Ixz        = IxzMoment(Aero_Mass);
    Aero_Iyz        = 0.0;
    Aero_Ixy        = 0.0;
    Aero_CgPosition = 0.25;
}
