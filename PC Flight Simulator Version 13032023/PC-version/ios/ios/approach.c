/*
    approach.c
    approach mode plotting window
 */

#define GlideslopeOffset 416.0f

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/udplib.h>
#include <SIM/glib.h>
#include <SIM/navlib.h>

#include "approach.h"
#include "ioslink.h"

#define x0        0
#define xaxis     (Glib_SCREENWIDTH - 70)
#define ygap      20
#define yaxis     ((Glib_SCREENHEIGHT - 16 - 4 * ygap) / 3)
#define y1axis    ygap
#define y2axis    (y1axis + yaxis + ygap + yaxis / 2)
#define y3axis    (y2axis + yaxis / 2 + ygap)
#define MaxPts    10000

typedef struct
{
    float x;
    float y;
    float z;
    float u;
} ApproachData;

static float        CurrentRange   = 10.0;
static unsigned int ApproachPts    = 0;
static ApproachData ApproachPlot[MaxPts];
static float        lastd          = 0.0;
static float        ApproachDistance = 10.0E6;

/* --------------------------------------------- */
void Approach_Reset()
{
    ApproachPts = 0;
    ApproachDistance = 10.0E6;
}

/* --------------------------------------------- */
void Approach_SetApproachRange(float d)
{
    CurrentRange = d;
}

/* --------------------------------------------- */
unsigned int SpeedScale(unsigned int n)  /* max +- 200 kts */
{
    return (unsigned int) intround((float) n * (float) yaxis / 200.0);
}

/* --------------------------------------------- */
unsigned int HeightScale(unsigned int n)  /* ft to pix */
{
    float h;

    h = CurrentRange * 6076.0 * 0.0524; /* ht in ft for 3 deg g/s at max range */
    return (unsigned int) intround((float) n * (float) yaxis / h);
}

/* --------------------------------------------- */
void UpdateApproach(void)
{
    float        d, b, q;
    float        Range;
    float        OldDistance   = 0.0;
    float        OldLocaliser  = 0.0;
    float        OldGlideslope = 0.0;
    float        OldSpeed      = 0.0;
    
    unsigned int i;
    unsigned int r = IosLink_NavPkt.CurrentRunway;
    bool         drawing;

    if (r == 0)
    {
        return;
    }

    q = NavLib_Runways[r].Qdm + IosLink_NavPkt.MagneticVariation;
    q = Maths_Normalise(q);
    d = NavLib_Distance(IosLink_AeroPkt.Latitude, IosLink_AeroPkt.Longitude,
                        (double) NavLib_Runways[r].RunwayLatitude, (double) NavLib_Runways[r].RunwayLongitude);
    b = NavLib_Bearing(IosLink_AeroPkt.Latitude, IosLink_AeroPkt.Longitude,
                       (double) NavLib_Runways[r].RunwayLatitude, (double) NavLib_Runways[r].RunwayLongitude);
    if (fabs(d - lastd) > 5.0)  /* only update 5m change */
    {
        lastd = d;
        
        if (ApproachPts < MaxPts && d < ApproachDistance)
        {
            ApproachPlot[ApproachPts].x = d * tan(q - b);
            ApproachPlot[ApproachPts].y = d + GlideslopeOffset;  /* distance from G/S transmitter */
            ApproachPlot[ApproachPts].z = -(float) NavLib_Runways[IosLink_NavPkt.CurrentRunway].Elevation - IosLink_AeroPkt.Pz;
            ApproachPlot[ApproachPts].u = IosLink_AeroPkt.U;
            ApproachPts                 += 1;
            ApproachDistance            = d;
        }
    }

    Range = CurrentRange * 1852.0;  /* range in m */
    Glib_Colour(Glib_BLACK);
    Glib_LineWidth(1.0);

    Glib_ClipWindow(x0, y1axis, xaxis, yaxis);  /* speed */
    drawing = false;
    
    for (i=0; i<ApproachPts; i+=1)
    {
        float d = (float) x0 + (ApproachPlot[i].y * (float) xaxis) / Range;
        float v = (float) y1axis + ApproachPlot[i].u * ((float) yaxis / 102.88);

        if ((int) d != (int) OldDistance)
        {
            if (drawing)
            {
                Glib_Draw(OldDistance, OldSpeed, d, v);
            }
            OldSpeed = v;
            OldDistance = d;
            drawing = true;
        }
    }

    Glib_ClipWindow(x0, y2axis - yaxis / 2, xaxis, yaxis);  /* localiser */
    drawing = false;
    
    for (i=0; i<ApproachPts; i+=1)
    {
        float d = (float) x0 + (ApproachPlot[i].y * (float) xaxis) / Range;
        float loc = (float) y2axis + (ApproachPlot[i].x * (float) (yaxis / 2)) / (Range * 0.04366);

        if ((int) d != (int) OldDistance)
        {
            if (drawing)
            {
                Glib_Draw(OldDistance, OldLocaliser, d, loc);
            }
            OldLocaliser = loc;
            OldDistance = d;
            drawing = true;
        }
    }

    Glib_ClipWindow(x0, y3axis, xaxis, yaxis);  /* glide slope */
    drawing = false;
    
    for (i=0; i<ApproachPts; i+=1)
    {
        float d = (float) x0 + (ApproachPlot[i].y * (float) xaxis) / Range;
        float gs = (float) y3axis + (ApproachPlot[i].z * (float) yaxis) / (Range * 0.05241);

        if ((int) d != (int) OldDistance)
        {
            if (drawing)
            {
                Glib_Draw(OldDistance, OldGlideslope, d, gs);
            }
            OldDistance = d;
            OldGlideslope = gs;
            drawing = true;
        }
    }
    Glib_RemoveClipWindow(); 
}

/* --------------------------------------------- */
void Approach_ShowApproach(void)
{
    unsigned int xunits;
    unsigned int i;
    unsigned int x;
    unsigned int h;
    char         str[21];

    Glib_Colour(Glib_GREY);
    Glib_SetFont(Glib_GFONT10, 6);
    Glib_LineWidth(1.0);

    Glib_Draw(x0, y1axis, x0 + xaxis, y1axis);  /* speed axes */
    Glib_Draw(x0, y1axis, x0, y1axis + yaxis);

    Glib_Draw(x0, y2axis - yaxis / 2, x0, y2axis + yaxis / 2); /* localiser axes */
    Glib_Draw(x0, y2axis, x0 + xaxis, y2axis);
    Glib_Draw(x0, y2axis, x0 + xaxis, y2axis + yaxis / 2);     /* +2.5 deg */
    Glib_Draw(x0, y2axis, x0 + xaxis, y2axis - yaxis / 2);     /* -2.5 deg */


    Glib_Draw(x0, y3axis, x0, y3axis + yaxis);  /* Glideslope axes */
    Glib_Draw(x0, y3axis, x0 + xaxis, y3axis);
    Glib_Draw(x0, y3axis, x0 + xaxis, y3axis + yaxis);
    Glib_Draw(x0, y3axis, x0 + xaxis, y3axis + (float) ((yaxis) * 2.3 / 3.0));  /* 2.3 deg g/s */
    Glib_Draw(x0, y3axis, x0 + (float) ((xaxis) * 3.0 / 3.7), y3axis + yaxis);  /* 3.7 deg g/s */

    xunits = floor(CurrentRange);

    for (i = 1; i <= xunits; i += 1)
    {
        sprintf(str, "%2.1f", (float) i);
        x = x0 + (float) i * (float) xaxis / CurrentRange;
        Glib_Colour(Glib_GREY);
        Glib_Draw(x, y1axis, x, y1axis + 5);
        Glib_Draw(x, y2axis, x, y2axis + 5);
        Glib_Draw(x, y3axis, x, y3axis + 5);
        Glib_Colour(Glib_BLUE);
        Glib_Chars(str, x - Glib_StringSize(str) / 2, y1axis - 12);
        Glib_Chars(str, x - Glib_StringSize(str) / 2, y2axis - 12);
        Glib_Chars(str, x - Glib_StringSize(str) / 2, y3axis - 12);
    }

    for (i = 0; i <= 200; i += 50)
    {
        Glib_Colour(Glib_GREY);
        Glib_Draw(x0, y1axis + SpeedScale(i), x0 + 5, y1axis + SpeedScale(i));
        sprintf(str, "%2.0f", (float) i);
        Glib_Colour(Glib_BLUE);
        Glib_Chars(str, x0 - Glib_StringSize(str) - 2, y1axis + SpeedScale(i) - 4);
    }

    h = (unsigned int) (CurrentRange * 6076.0 * 0.0524); /* altitude in ft for 3 deg g/s at max range */
    h = (h / 100) * 100;

    for (i = 0; i <= h; i += 500)
    {
        Glib_Colour(Glib_GREY);
        Glib_Draw(x0, y3axis + HeightScale(i), x0 + 5, y3axis + HeightScale(i));
        sprintf(str, "%2.0f", (float) i);
        Glib_Colour(Glib_BLUE);
        Glib_Chars(str, x0 - Glib_StringSize(str) - 2, y3axis + HeightScale(i) - 4);
    }

    Glib_Colour(Glib_BLUE);
    Glib_Chars("kt", x0 + 10, y1axis + yaxis - 4);
    Glib_Chars("nm", x0 + xaxis - 10, y1axis + 10);
    Glib_Chars("nm", x0 + xaxis - 10, y2axis + 10);
    Glib_Chars("nm", x0 + xaxis - 10, y3axis + 10);
    Glib_Chars("ft", x0 + 10, y3axis + yaxis - 4);

    UpdateApproach();
}

/* --------------------------------------------- */
void BEGIN_Approach(void)
{
}
