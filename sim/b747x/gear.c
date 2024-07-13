/* +---------------------------+---------------------------------+
| Module      : gear.c         | Version         : 1.1           |
| Last Edit   : 30-03-2016     | Reference Number: 02-01-12      |
|+-----------------------------+---------------------------------+
| Computer    : DELL1                                            |
| Directory   : /dja/aerosoft/cranfield/pfd/b747                 |
| Compiler    : gcc 6.3.0                                        |
| OS          : Windows10                                        |
|+---------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
|+---------------------------------------------------------------+
| Description : Boeing 747-400 Undercarriage model               |
|                                                                |
|+---------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/dted.h>

#include "gear.h"
#include "aero.h"
#include "model.h"
#include "aerolink.h"

typedef struct
{
    float xc;
    float yc;
} vertex;

float  Gear_Fx, Gear_Fy, Gear_Fz;       /* gear force components */
float  Gear_Mx, Gear_My, Gear_Mz;       /* gear moment components */
float  dst1, dst2, dst3;                /* oleo strut compressions */

vertex damping_nose[] =
{ {  0.0,  7.0 }, {  6.0,  7.0 }, { 10.0, 10.0 }, { 14.0, 15.0 },
  { 16.0, 29.0 }, { 20.0, 38.0 }, { 25.0, 67.0 } };

vertex damping_main[] =
{ {  0.0, 10.0 }, {  7.0, 10.0 }, { 10.0, 15.0 }, { 15.0, 27.0 }, { 20.0, 47.0 },
  { 25.0, 72.0 }, { 27.0, 81.0 } };

vertex spring_nose[] =
{ {  0.0,     0.0 }, { 5.0, 4000.0 }, { 10.0, 8000.0 }, { 15.0, 13000.0 }, { 20.0, 23000.0 },
  { 25.0, 53000.0 } };

vertex spring_main[] =
{ {  0.0,      0.0 }, {  5.0,  10000.0 }, { 10.0, 23000.0 }, { 15.0, 46000.0 }, { 20.0, 84000.0 },
  { 25.0, 200000.0 }, { 27.0, 258000.0 } };

/* ------------------------------------------------------------------- */
float lookup(float x, vertex a[], unsigned int n)
{
    unsigned int p0, p1;

    if (x <= a[0].xc)
    {
        return a[0].yc;
    }
    if (x >= a[n - 1].xc)
    {
        return a[n - 1].yc;
    }
    for (p1 = 1; p1 < n; p1 = p1 + 1)
    {
        if (x < a[p1].xc)
        {
            p0 = p1 - 1;
            return a[p0].yc + (x - a[p0].xc) * (a[p1].yc - a[p0].yc) / (a[p1].xc - a[p0].xc);
        }
    }
    return 0.0; /* cannot reach here */
    printf("lookup error:\n");
}

/* ------------------------------------------------------------------- */
bool Gear_WeightOnWheels()
{
    return((dst1 < 0.0) || (dst2 < 0.0) || (dst3 < 0.0));
}

/* ------------------------------------------------------------------- */
void Gear_GearModel()
{
    float       XL1, YL1, ZL1;  /* nose gear position (ft) */
    float       XL2, YL2, ZL2;  /* left main gear position (ft) */
    float       XL3, YL3, ZL3;  /* right main gear position (ft) */

    float       h;              /* cg height above the runway (ft) */
    float       h_dot;          /* vertical speed (ft/s) */

    float       spitch, cpitch;
    float       sroll, croll;
    float       spsr, spcr, cpsr, cpcr;

    float       c1, c2, c3;                /* damping constants */
    float       vf1, vf2, vf3;             /* spring forces */

    float       dforce1, dforce2, dforce3; /* damping forces */
    float       fgz1, fgz2, fgz3;          /* oleo strut forces */
    float       fng1, fng2, fng3;          /* tyre normal forces */
    float       fs1, fs2, fs3;             /* tyre side forces */
    float       fmu1, fmu2, fmu3;          /* wheel force drag */
    float       fx1, fx2, fx3;             /* total wheel forces */
    float       fy1, fy2, fy3;
    float       fz1, fz2, fz3;
    float       mx1, mx2, mx3;             /* total wheel moments */
    float       my1, my2, my3;
    float       mz1, mz2, mz3;

    const float gt1 = 3834.0;
    const float gt2 = 15336.0;
    const float gt3 = 15336.0;

    const float ht1 = 619.0;
    const float ht2 = 2477.0;
    const float ht3 = 2477.0;

    const float kt1 = -0.093 / 2000.0;
    const float kt2 = -0.077 / 8000.0;
    const float kt3 = -0.077 / 8000.0;

    const float g = 32.17405;

    float       dt1, dt2, dt3;                /* tyre deflection  (in)*/
    float       bg1, bg2, bg3;                /* ground track angle (rads) */
    float       ds;                           /* pedal nose wheel steering angle (deg) */

    float       fbmax;                        /* max braking force (lbf) */
    float       fb2, fb3;                     /* braking force (main gear only) */
    float       bof;                          /* break out friction */

    float       w;                            /* aircraft weight (lbs)*/
    float       gs;                           /* ground speed (n/s) */
    float       ug, vg;                       /* gear velocity in body axes (m/s) */
    float       ng, eg;                       /* gear velocity in world axes (m/s) */
    float       gd;                           /* aircraft heading (rads) */
    float       gp;                           /* track angle (rads) */

    float       dst1_dot, dst2_dot, dst3_dot; /* oleo struct compression rates (in/s) */

    double      hat;                          /* height above terrain (ft) */
    
    spitch = sin(Model_Pitch);
    cpitch = cos(Model_Pitch);
    sroll  = sin(Model_Roll);
    croll  = cos(Model_Roll);
    spsr   = spitch * sroll;
    spcr   = spitch * croll;
    cpsr   = cpitch * sroll;
    cpcr   = cpitch * croll;

    w     = Aero_Mass * 2.204623; /* lbs */
    fbmax = 2.0 * (0.834 + 4.167 * 0.4) * w / g;
    gs    = sqrt(Model_Vn * Model_Vn + Model_Ve * Model_Ve);

    if (gs < 5.0)
    {
        bof = 0.014 - gs * 0.0028;
    }
    else
    {
        bof = 0.0;
    }

    if (gs < 1.0)
    {
        ds = 0.0;
    }
    else
    {
        float dnw = 0.0;
        ds = -Model_Rudder * 10.0; /* max deflection 10 degs */
        if (gs < 35.0) /* max nose wheel defelection 75 deg at 0 Kt reducing to zero at 70 Kt */
        {
            dnw = Model_Tiller * 75.0 * (35.0 - gs) / 35.0;
        }
        ds = ds + dnw;
    }

/* nb. dimensions in ft */

    XL1 = 77.0 + 0.01 * Maths_Feet((Aero_CgPosition - 0.25) * Aero_CBar);
    YL1 = 0.0;
    ZL1 = 17.0;

    XL2 = -7.0 + 0.01 * Maths_Feet((Aero_CgPosition - 0.25) * Aero_CBar);
    YL2 = -12.0;
    ZL2 = 17.0;

    XL3 = -7.0 + 0.01 * Maths_Feet((Aero_CgPosition - 0.25) * Aero_CBar);
    YL3 = 12.0;
    ZL3 = 17.0;

    hat = DTED_PostHeight(Model_Latitude, Model_Longitude, AeroLink_NavPkt.GroundLevel);
    h = Maths_Feet(-Model_Pz + (float) hat);
    h_dot = Maths_Feet(-Model_Vd);

    dst1 = (h + XL1 * spitch - YL1 * cpsr - ZL1 * cpcr) * 12.0;
    dst2 = (h + XL2 * spitch - YL2 * cpsr - ZL2 * cpcr) * 12.0;
    dst3 = (h + XL3 * spitch - YL3 * cpsr - ZL3 * cpcr) * 12.0;

    dst1_dot = (h_dot + XL1 * cpitch * Model_Q +
                YL1 * (spsr * Model_Q - cpcr * Model_P) +
                ZL1 * (spcr * Model_Q + cpsr * Model_P)) * 12.0;
    dst2_dot = (h_dot + XL2 * cpitch * Model_Q +
                YL2 * (spsr * Model_Q - cpcr * Model_P) +
                ZL2 * (spcr * Model_Q + cpsr * Model_P)) * 12.0;
    dst3_dot = (h_dot + XL3 * cpitch * Model_Q +
                YL3 * (spsr * Model_Q - cpcr * Model_P) +
                ZL3 * (spcr * Model_Q + cpsr * Model_P)) * 12.0;

    if (dst1 < 0.0)              /* nose gear */
    {
        ug  = Model_U;
        vg  = Model_V + Model_R * Maths_Metres(XL1);
        ng  = ug * Model_A11 + vg * Model_A12;
        eg  = ug * Model_A21 + vg * Model_A22;
        gp  = (Model_U < 1.0) ? Model_Yaw : atan2(eg, ng);
        gd  = Model_Yaw;
        bg1 = gp - gd;
        Maths_Normalise(&bg1);

        c1      = lookup(-dst1, damping_nose, 7);
        vf1     = 2.0 * lookup(-dst1, spring_nose, 6);
        dforce1 = -2.0 * c1 * dst1_dot * fabs(dst1_dot);
        fgz1    = -(vf1 + dforce1);
        fng1    = fgz1 / cpcr;
        dt1     = fng1 * kt1;
        fs1     = (dt1 * gt1 - (dt1 * dt1 * ht1)) * (ds - Maths_Degrees(bg1));
        Maths_Limit(&fs1, -fabs(0.6 * fng1), fabs(0.6 * fng1));
        fmu1 = -((0.015 + bof) * fabs(fng1)); /* no braking on nose wheel */
        fx1  = fmu1 - fng1 * Model_Pitch - fs1 * Maths_Rads(ds);
        fy1  = fs1 + fng1 * Model_Roll;
        fz1  = fmu1 * Model_Pitch - fs1 * Model_Roll + fng1;
    }
    else
    {
        fx1 = 0.0;
        fy1 = 0.0;
        fz1 = 0.0;
    }

    if (dst2 < 0.0)              /* left main gear */
    {
        ug  = Model_U;
        vg  = Model_V + Model_R * Maths_Metres(XL2);
        ng  = ug * Model_A11 + vg * Model_A12;
        eg  = ug * Model_A21 + vg * Model_A22;
        gp  = (Model_U < 1.0) ? Model_Yaw : atan2(eg, ng);
        gd  = Model_Yaw;
        bg2 = gp - gd;
        Maths_Normalise(&bg2);

        c2      = lookup(-dst2, damping_main, 7);
        vf2     = 2.0 * lookup(-dst2, spring_main, 7);
        dforce2 = -2.0 * c2 * dst2_dot * fabs(dst2_dot);
        fgz2    = -(vf2 + dforce2);
        fng2    = fgz2 / cpcr;
        dt2     = fng2 * kt2;
        fs2     = (dt2 * gt2 - (dt2 * dt2 * ht2)) * Maths_Degrees(-bg2);
        Maths_Limit(&fs2, -fabs(0.6 * fng2), fabs(0.6 * fng2));
        fb2 = Model_LeftBrake * 10.0 * 2.0 * 0.263 * w / g;
        Maths_Limit(&fb2, 0.0, fbmax);
        fmu2 = -((0.015 + bof) * fabs(fng2) + fb2);
        fx2  = fmu2 - fng2 * Model_Pitch;
        fy2  = fs2 + fng2 * Model_Roll;
        fz2  = fmu2 * Model_Pitch - fs2 * Model_Roll + fng2;
    }
    else
    {
        fx2 = 0.0;
        fy2 = 0.0;
        fz2 = 0.0;
    }

    if (dst3 < 0.0)              /* right main gear */
    {
        ug  = Model_U;
        vg  = Model_V + Model_R * Maths_Metres(XL3);
        ng  = ug * Model_A11 + vg * Model_A12;
        eg  = ug * Model_A21 + vg * Model_A22;
        gp  = (Model_U < 1.0) ? Model_Yaw : atan2(eg, ng);
        gd  = Model_Yaw;
        bg3 = gp - gd;
        Maths_Normalise(&bg3);

        c3      = lookup(-dst3, damping_main, 7);
        vf3     = 2.0 * lookup(-dst3, spring_main, 7);
        dforce3 = -2.0 * c3 * dst3_dot * fabs(dst3_dot);
        fgz3    = -(vf3 + dforce3);
        fng3    = fgz3 / cpcr;
        dt3     = fng3 * kt3;
        fs3     = (dt3 * gt3 - (dt3 * dt3 * ht3)) * Maths_Degrees(-bg3);
        Maths_Limit(&fs3, -fabs(0.6 * fng3), fabs(0.6 * fng3));
        fb3 = Model_RightBrake * 10.0 * 2.0 * 0.263 * w / g;
        Maths_Limit(&fb3, 0.0, fbmax);
        fmu3 = -((0.015 + bof) * fabs(fng3) + fb3);
        fx3  = fmu3 - fng3 * Model_Pitch;
        fy3  = fs3 + fng3 * Model_Roll;
        fz3  = fmu3 * Model_Pitch - fs3 * Model_Roll + fng3;
    }
    else
    {
        fx3 = 0.0;
        fy3 = 0.0;
        fz3 = 0.0;
    }

    Gear_Fx = (fx1 + fx2 + fx3) * 4.448222; /* convert to N */
    Gear_Fy = (fy1 + fy2 + fy3) * 4.448222;
    Gear_Fz = (fz1 + fz2 + fz3) * 4.448222;

    mx1 = fz1 * YL1 - fy1 * (-dst1 / 12.0);
    mx2 = fz2 * YL2 - fy2 * (-dst2 / 12.0);
    mx3 = fz3 * YL3 - fy3 * (-dst3 / 12.0);

    my1 = -fz1 * XL1 + fx1 * (-dst1 / 12.0);
    my2 = -fz2 * XL2 + fx2 * (-dst2 / 12.0);
    my3 = -fz3 * XL3 + fx3 * (-dst3 / 12.0);

    mz1 = fy1 * XL1 - fx1 * YL1;
    mz2 = fy2 * XL2 - fx2 * YL2;
    mz3 = fy3 * XL3 - fx3 * YL3;

    Gear_Mx = (mx1 + mx2 + mx3) * 1.35581795; /* convert to Nm */
    Gear_My = (my1 + my2 + my3) * 1.35581795;
    Gear_Mz = (mz1 + mz2 + mz3) * 1.35581795;

/*  AeroLink_AddFlightData(1, -dst1);
    AeroLink_AddFlightData(2, -dst2);
    AeroLink_AddFlightData(3, -dst3); */
}

/* ------------------------------------------------------------------- */
void BEGIN_Gear()
{
    Gear_Fx = 0.0;
    Gear_Fy = 0.0;
    Gear_Fz = 0.0;

    Gear_Mx = 0.0;
    Gear_My = 0.0;
    Gear_Mz = 0.0;
}
