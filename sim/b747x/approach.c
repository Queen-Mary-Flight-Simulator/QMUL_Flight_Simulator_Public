/*
    approach.c
    approach mode plotting window
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/udplib.h>
#include <SIM/glibx.h>
#include <SIM/navlib.h>

#include "map.h"        /* Map_GetMagneticVariation */
#include "approach.h"
#include "ioslink.h"

#define x0        (Glibx_SCREENWIDTH / 2 + 50)
#define xaxis     (Glibx_SCREENWIDTH / 2 - 70)
#define ygap      20
#define yaxis     ((Glibx_SCREENHEIGHT - 16 - 4 * ygap) / 3)
#define y1axis    ygap
#define y2axis    (y1axis + yaxis + ygap + yaxis / 2)
#define y3axis    (y2axis + yaxis / 2 + ygap)
#define MaxPts    10000

#define FontWidth 9  /* fon12.fnt */

typedef struct
{
    float x;
    float y;
    float z;
    float u;
} ApproachData;

float        CurrentRange   = 10.0;
unsigned int ApproachPts    = 0;
ApproachData ApproachPlot[MaxPts];

/* --------------------------------------------- */
void Approach_Reset()
{
    ApproachPts = 0;
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
    float        aircraftx;     /* localiser offset (m) */
    float        aircrafty;     /* distance (m) */
    float        aircraftz;     /* g/s height above ground (m) */
    float        xx, yy;

    float        Range;
    float        Distance;
    float        Localiser;
    float        GlideSlope;
    float        Speed;
    float        OldDistance   = 0.0;
    float        OldLocaliser  = 0.0;
    float        OldGlideSlope = 0.0;
    float        OldSpeed      = 0.0;

    unsigned int i;
    bool         drawing = false;
    unsigned int r = IosLink_NavPkt.CurrentRunway;
	
    if (r == 0)
    {
        return;
    }

    q = -Maths_Rads((float) NavLib_Runways[r].Qdm + Map_GetMagneticVariation());
    Maths_Normalise(&q);
    d = NavLib_Distance(IosLink_AeroPkt.Latitude, IosLink_AeroPkt.Longitude,
                        (double) NavLib_Runways[r].RunwayLatitude, (double) NavLib_Runways[r].RunwayLongitude);
    b = NavLib_Bearing(IosLink_AeroPkt.Latitude, IosLink_AeroPkt.Longitude,
                       (double) NavLib_Runways[r].RunwayLatitude, (double) NavLib_Runways[r].RunwayLongitude);
    xx = -d * sin(b);
    yy = -d * cos(b);

    aircraftx = xx * cos(q) + yy * sin(q);
    //aircrafty = yy * cos(q) - xx * sin(q) - 300.0;  /* allow for G/S transmitter */
    aircrafty = yy * cos(q) - xx * sin(q);
    aircraftz = -(float) NavLib_Runways[IosLink_NavPkt.CurrentRunway].Elevation - IosLink_AeroPkt.Pz;

    if ((ApproachPts == 0) || ((ApproachPts > 0) && fabs(ApproachPlot[ApproachPts - 1].y - aircrafty) > 1.0))
    {
        if (ApproachPts < MaxPts)
        {
            ApproachPlot[ApproachPts].x = aircraftx;
            ApproachPlot[ApproachPts].y = aircrafty;
            ApproachPlot[ApproachPts].z = aircraftz;
            ApproachPlot[ApproachPts].u = IosLink_AeroPkt.U;
            ApproachPts                 = ApproachPts + 1;
        }
    }

    Range = CurrentRange * 1852.0;  /* range in m */
    Glibx_Colour(Glibx_BLACK);
    Glibx_LineWidth(1.0);

    for (i = 0; i < ApproachPts; i += 1)
    {
        Distance   = (float) x0 + (-ApproachPlot[i].y * (float) xaxis) / Range;
        GlideSlope = (float) y3axis + (ApproachPlot[i].z * (float) (yaxis)) / (Range * 0.05241);
        Localiser  = (float) y2axis + (ApproachPlot[i].x * (float) (yaxis / 2)) / (Range * 0.04366);
        Speed      = (float) y1axis + ApproachPlot[i].u * ((float) yaxis / 102.88);

        if ((int) Distance != (int) OldDistance)
        {
            if (drawing)
            {
                Glibx_ClipWindow(x0, y2axis - yaxis / 2, xaxis, yaxis);
                Glibx_Draw(OldDistance, OldLocaliser, Distance, Localiser);
                Glibx_ClipWindow(x0, y3axis, xaxis, yaxis);
                Glibx_Draw(OldDistance, OldGlideSlope, Distance, GlideSlope);
                Glibx_ClipWindow(x0, y1axis, xaxis, yaxis);
                Glibx_Draw(OldDistance, OldSpeed, Distance, Speed);
                glDisable(GL_SCISSOR_TEST);
            }
            OldDistance   = Distance;
            OldGlideSlope = GlideSlope;
            OldLocaliser  = Localiser;
            OldSpeed      = Speed;
            drawing       = true;
        }
    }
}

/* --------------------------------------------- */
void Approach_ShowApproach(void)
{
    unsigned int xunits;
    unsigned int i;
    unsigned int x;
    unsigned int h;
    char         str[21];

    Glibx_Colour(Glibx_GREY);
    //Glibx_SetFont(Glibx_GFONT10, 6);
    Glibx_LineWidth(1.0);

    Glibx_Draw(x0, y1axis, x0 + xaxis, y1axis);  /* speed axes */
    Glibx_Draw(x0, y1axis, x0, y1axis + yaxis);

    Glibx_Draw(x0, y2axis - yaxis / 2, x0, y2axis + yaxis / 2); /* localiser axes */
    Glibx_Draw(x0, y2axis, x0 + xaxis, y2axis);
    Glibx_Draw(x0, y2axis, x0 + xaxis, y2axis + yaxis / 2);     /* +2.5 deg */
    Glibx_Draw(x0, y2axis, x0 + xaxis, y2axis - yaxis / 2);     /* -2.5 deg */


    Glibx_Draw(x0, y3axis, x0, y3axis + yaxis);  /* glideslope axes */
    Glibx_Draw(x0, y3axis, x0 + xaxis, y3axis);
    Glibx_Draw(x0, y3axis, x0 + xaxis, y3axis + yaxis);
    Glibx_Draw(x0, y3axis, x0 + xaxis, y3axis + (float) ((yaxis) * 2.3 / 3.0));  /* 2.3 deg g/s */
    Glibx_Draw(x0, y3axis, x0 + (float) ((xaxis) * 3.0 / 3.7), y3axis + yaxis);  /* 3.7 deg g/s */

    xunits = floor(CurrentRange);

    for (i = 1; i <= xunits; i += 1)
    {
        sprintf(str, "%2.1f", (float) i);
        x = x0 + (float) i * (float) xaxis / CurrentRange;
        Glibx_Colour(Glibx_GREY);
        Glibx_Draw(x, y1axis, x, y1axis + 5);
        Glibx_Draw(x, y2axis, x, y2axis + 5);
        Glibx_Draw(x, y3axis, x, y3axis + 5);
        Glibx_Colour(Glibx_BLUE);
        Map_Chars(str, x - strlen(str) * FontWidth / 2, y1axis - 16);
        Map_Chars(str, x - strlen(str) * FontWidth / 2, y2axis - 16);
        Map_Chars(str, x - strlen(str) * FontWidth / 2, y3axis - 16);
    }

    for (i = 0; i <= 200; i += 50)
    {
        Glibx_Colour(Glibx_GREY);
        Glibx_Draw(x0, y1axis + SpeedScale(i), x0 + 5, y1axis + SpeedScale(i));
        sprintf(str, "%2.0f", (float) i);
        Glibx_Colour(Glibx_BLUE);
        Map_Chars(str, x0 - strlen(str) * FontWidth - 2, y1axis + SpeedScale(i) - 6);
    }

    h = (unsigned int) (CurrentRange * 6076.0 * 0.0524); /* altitude in ft for 3 deg g/s at max range */
    h = (h / 100) * 100;

    for (i = 0; i <= h; i += 500)
    {
        Glibx_Colour(Glibx_GREY);
        Glibx_Draw(x0, y3axis + HeightScale(i), x0 + 5, y3axis + HeightScale(i));
        sprintf(str, "%2.0f", (float) i);
        Glibx_Colour(Glibx_BLUE);
        Map_Chars(str, x0 - strlen(str) * FontWidth - 2, y3axis + HeightScale(i) - 6);
    }

    Glibx_Colour(Glibx_BLUE);
    Map_Chars("kt", x0 + 8, y1axis + yaxis - 4);
    Map_Chars("nm", x0 + xaxis - 12, y1axis + 10);
    Map_Chars("nm", x0 + xaxis - 12, y2axis + 10);
    Map_Chars("nm", x0 + xaxis - 12, y3axis + 10);
    Map_Chars("ft", x0 + 10, y3axis + yaxis - 4);

    UpdateApproach();
}

/* --------------------------------------------- */
void BEGIN_Approach(void)
{
}
