/* +------------------------------+---------------------------------+
   | Module      : navinfo.c      | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-06      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Navigation Information for EFIS display          |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>
#include <GL/gl.h>
#include <stdbool.h>

#include <SIM/glib.h>
#include <SIM/iodefn.h>
#include <SIM/navdefn.h>
#include <SIM/maths.h>
#include <SIM/navlib.h>

#include "nfd.h"
#include "navlink.h"
#include "fcu.h"
#include "nav.h"
#include "radio.h"
#include "nfd-compass.h"
#include "navinfo.h"

#define PI       3.141592654
#define DEG45    (PI / 4.0)

static void WriteDme(unsigned int d, unsigned int x, unsigned int y);
static void FormFrequency(unsigned int f, NavDefn_FCUNav Mode, char v[]);


static void WriteDme(unsigned int d, unsigned int x, unsigned int y)
{
    if (d >= 1000)
    {
        Glib_Char(d / 1000 + '0', x, y);
        d = d % 1000;
        x = x + 10;
    }
    if (d >= 100)
    {
        Glib_Char(d / 100 + '0', x, y);
        d = d % 100;
        x = x + 10;
    }
    Glib_Char(d / 10 + '0', x, y);
    Glib_Char('.', x + 10, y);
    Glib_Char(d % 10 + '0', x + 20, y);
}

void NavInfo_UpdateLeftNavInfo(NavDefn_FCUNav Mode)
{
    Nav_NavRecord *w;
    unsigned int  f = 0;
    unsigned int  d;
    char          Str[10];
    bool          DmeOK;

    Glib_SetFont(Glib_GFONT16, 11);
    DmeOK = false;
    d     = 0;
    if (Mode == NavDefn_NavVOR)
    {
        f = Radio_Radios[0].NavVOR.Active;
        w = &Nav_VOR1;

        if (w->BeaconStatus)
        {
            if (w->SelectedBeacon > 0)
            {
                if (NavLib_DME & NavLib_Beacons[w->SelectedBeacon].Navaid)
                {
                    d     = (unsigned int) (w->SlantDistance * 0.0053966);
                    DmeOK = true;
                }
            }
        }
    }
    else if (Mode == NavDefn_NavADF)
    {
        f = Radio_Radios[0].NavADF.Active;
    }
    FormFrequency(f, Mode, Str);
    if (Mode == NavDefn_NavVOR)
    {
        Glib_Colour(Glib_GREEN);
        Glib_Chars("VOR L", NFD_LeftNavInfoX, NFD_LeftNavInfoY + 40);
        Glib_Chars(Str, NFD_LeftNavInfoX, NFD_LeftNavInfoY + 20);
        if (DmeOK)
        {
            Glib_SetFont(Glib_GFONT12, 10);
            Glib_Chars("DME", NFD_LeftNavInfoX, NFD_LeftNavInfoY);
            Glib_SetFont(Glib_GFONT16, 11);
            WriteDme(d, NFD_LeftNavInfoX + 40, NFD_LeftNavInfoY);
        }
    }
    else if (Mode == NavDefn_NavADF)
    {
        Glib_Colour(Glib_BLUE);
        Glib_Chars("ADF L", NFD_LeftNavInfoX, NFD_LeftNavInfoY + 40);
        Glib_Chars(Str, NFD_LeftNavInfoX, NFD_LeftNavInfoY + 20);
    }
}

void NavInfo_UpdateRightNavInfo(NavDefn_FCUNav Mode)
{
    Nav_NavRecord *w;
    unsigned int  f = 0;
    unsigned int  d;
    char          Str[10];
    bool          DmeOK;

    Glib_SetFont(Glib_GFONT16, 11);
    DmeOK = false;
    d     = 0;

    if (Mode == NavDefn_NavVOR)
    {
        f = Radio_Radios[0].NavVOR.Active;
        w = &Nav_VOR2;

        if (w->BeaconStatus)
        {
            if (w->SelectedBeacon > 0)
            {
                if (NavLib_DME & NavLib_Beacons[w->SelectedBeacon].Navaid)
                {
                    d     = (unsigned int) (w->SlantDistance * 0.0053966);
                    DmeOK = true;
                }
            }
        }
    }
    else if (Mode == NavDefn_NavADF)
    {
        f = Radio_Radios[0].NavADF.Active;
    }

    FormFrequency(f, Mode, Str);
    if (Mode == NavDefn_NavVOR)
    {
        Glib_Colour(Glib_GREEN);
        Glib_Chars("VOR R", NFD_RightNavInfoX, NFD_RightNavInfoY + 40);
        Glib_Chars(Str, NFD_RightNavInfoX, NFD_RightNavInfoY + 20);
        if (DmeOK)
        {
            Glib_SetFont(Glib_GFONT12, 10);
            Glib_Chars("DME", NFD_RightNavInfoX, NFD_RightNavInfoY);
            Glib_SetFont(Glib_GFONT16, 11);
            WriteDme(d, NFD_RightNavInfoX + 40, NFD_RightNavInfoY);
        }
    }
    else if (Mode == NavDefn_NavADF)
    {
        Glib_Colour(Glib_BLUE);
        Glib_Chars("ADF R", NFD_RightNavInfoX, NFD_RightNavInfoY + 40);
        Glib_Chars(Str, NFD_RightNavInfoX, NFD_RightNavInfoY + 20);
    }
}

void NavInfo_UpdateTopNavInfo(NavDefn_FCUMode Mode)
{
    Nav_NavRecord *w;
    unsigned int  f;
    unsigned int  d;
    unsigned int  Crs;
    char          Str[10];
    bool          DmeOK;
    bool          IlsFreq;

    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_GFONT12, 10);
    Glib_Chars("GS", NFD_GsX, NFD_GsY);
    Glib_Chars("TAS", NFD_TasX, NFD_TasY);

    Glib_SetFont(Glib_GFONT16, 11);
    DmeOK   = false;
    IlsFreq = false;
    d       = 0;

    if (Mode == NavDefn_ModeILS)
    {
        f   = Radio_Radios[0].NavILS.Active;
        Crs = Radio_Radios[0].CrsKnob;
        w   = &Nav_ILS1;

        if (w->BeaconStatus)
        {
            if (w->SelectedBeacon > 0)
            {
                if (NavLib_DME & NavLib_Beacons[w->SelectedBeacon].Navaid)
                {
                    d     = (unsigned int) (w->SlantDistance * 0.0053966);
                    DmeOK = true;
                }
                IlsFreq = NavLib_ILS & NavLib_Beacons[w->SelectedBeacon].Navaid;
            }
        }
    }
    else
    {
        f   = Radio_Radios[0].NavVOR.Active;
        Crs = Radio_Radios[0].CrsKnob;
        w   = &Nav_VOR1;

        if (w->BeaconStatus)
        {
            if (w->SelectedBeacon > 0)
            {
                if (NavLib_DME & NavLib_Beacons[w->SelectedBeacon].Navaid)
                {
                    d     = (unsigned int) (w->SlantDistance * 0.0053966);
                    DmeOK = true;
                }
                IlsFreq = NavLib_ILS & NavLib_Beacons[w->SelectedBeacon].Navaid;
            }
        }
    }

    Glib_SetFont(Glib_GFONT16, 11);
    Glib_Colour(Glib_GREEN);
    if (IlsFreq)
    {
        Glib_Chars("ILS L", NFD_TopNavInfoX - 75, NFD_TopNavInfoY + 40);
    }
    else
    {
        Glib_Chars("VOR L", NFD_TopNavInfoX - 75, NFD_TopNavInfoY + 40);
    }
    FormFrequency(f, NavDefn_NavVOR, Str);
    Glib_Chars(Str, NFD_TopNavInfoX, NFD_TopNavInfoY + 40);
    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_GFONT12, 10);
    Glib_Chars("CRS", NFD_TopNavInfoX, NFD_TopNavInfoY + 20);
    Glib_SetFont(Glib_GFONT16, 11);
    Compass_Wrn(NFD_TopNavInfoX + 40, NFD_TopNavInfoY + 20, Crs, 3, true);
    Glib_SetFont(Glib_GFONT12, 10);
    Glib_Chars("DME", NFD_TopNavInfoX, NFD_TopNavInfoY);
    Glib_SetFont(Glib_GFONT16, 11);
    if (DmeOK)
    {
        WriteDme(d, NFD_TopNavInfoX + 40, NFD_TopNavInfoY);
    }
    else
    {
        Glib_Chars("--.-", NFD_TopNavInfoX + 40, NFD_TopNavInfoY);
    }
}

static void FormFrequency(unsigned int f, NavDefn_FCUNav Mode, char v[])
{
    char d1, d2, d3, d4;

    d1 = f % 10000 / 1000 + '0';
    d2 = f % 1000 / 100 + '0';
    d3 = f % 100 / 10 + '0';
    d4 = f % 10 + '0';
    if (Mode == NavDefn_NavADF)
    {
        v[0] = d1;
        v[1] = d2;
        v[2] = d3;
        v[3] = '.';
        v[4] = d4;
        v[5] = 0;
    }
    else if (Mode == NavDefn_NavVOR)
    {
        v[0] = '1';
        v[1] = d1;
        v[2] = d2;
        v[3] = '.';
        v[4] = d3;
        v[5] = d4;
        v[6] = 0;
    }
    else
    {
        v[0] = 0;
    }
}

void NavInfo_UpdateGS(float v)
{
    unsigned int g;

    g = intround(v * 1.9440124);
    Glib_SetFont(Glib_GFONT16, 11);
    Glib_Colour(Glib_WHITE);
    if (g > 30)
    {
        Glib_Char(g / 100 + '0', NFD_GsX + 20, NFD_GsY);
        Glib_Char(g / 10 % 10 + '0', NFD_GsX + 30, NFD_GsY);
        Glib_Char(g % 10 + '0', NFD_GsX + 40, NFD_GsY);
    }
}

void NavInfo_UpdateTAS(float v)
{
    unsigned int t;

    t = intround(v * 1.9440124);
    Glib_SetFont(Glib_GFONT16, 11);
    Glib_Colour(Glib_WHITE);
    if (t > 30)
    {
        Glib_Char(t / 100 + '0', NFD_TasX + 30, NFD_TasY);
        Glib_Char(t / 10 % 10 + '0', NFD_TasX + 40, NFD_TasY);
        Glib_Char(t % 10 + '0', NFD_TasX + 50, NFD_TasY);
    }
}

void NavInfo_UpdateWindVector(float WindSpeed, float WindDir)
{
    int   d;
    int   s;
    float a;

    d = intround(Maths_Degrees(WindDir));
    while (d <= 0)
    {
        d = d + 360;
    }
    while (d > 360)
    {
        d = d - 360;
    }
    s = intround(WindSpeed * 1.944);
    a = WindDir - PI - ((float) NavLink_AeroPkt.Yaw - Nav_MagneticVariation);
    Maths_Normalise(&a);
    Glib_SetFont(Glib_GFONT16, 11);
    Glib_Colour(Glib_WHITE);
    Compass_Wrn(NFD_WindX, NFD_WindY, d, 3, true);
    Glib_Char(7, NFD_WindX + 36, NFD_WindY);
    Glib_Char('/', NFD_WindX + 48, NFD_WindY);
    if (s > 99)
    {
        Compass_Wrn(NFD_WindX + 60, NFD_WindY, s, 3, false);
    }
    else
    {
        Compass_Wrn(NFD_WindX + 60, NFD_WindY, s, 2, false);
    }
    Glib_Colour(Glib_WHITE);
    glPushMatrix();
    glTranslatef(-(NFD_WindY - 25), NFD_WindX, 0.0);
    glRotatef(Maths_Degrees(-a), 0.0, 0.0, 1.0);
    Glib_Draw(0, 20, 0, -20);
    Glib_Draw(0, 20, -4, 16);
    Glib_Draw(0, 20, 4, 16);
    glPopMatrix();
}

void BEGIN_NavInfo()
{
}
