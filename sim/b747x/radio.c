/* +------------------------------+---------------------------------+
   | Module      : radio.c        | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-11      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Radio Management                                 |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <GL/gl.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <SIM/iosdefn.h>
#include <SIM/navdefn.h>
#include <SIM/glib.h>
#include <SIM/navlib.h>

#include "navlink.h"
#include "nfd.h"
#include "panellib.h"
#include "nav.h"
#include "radio.h"

NavDefn_RadioPanel Radio_Radios[2];

typedef struct
{
    int xc;
    int yc;
} Vertex;

#define SwNone    0
#define SwNAV     1
#define SwVOR     2
#define SwILS     3
#define SwMLS     4
#define SwADF     5
#define SwBFO     6
#define SwHF1     7
#define SwHF2     8
#define SwAM      9
#define SwVHF1    10
#define SwVHF2    11
#define SwVHF3    12
#define SwSEL     13

typedef unsigned char   SwitchType;

Vertex SwitchCoords[14] = {
    {                0,                0 },
    { NFD_RadioX + 59,  NFD_RadioY + 32  },  /* NAV */
    { NFD_RadioX + 136, NFD_RadioY + 32  },  /* VOR */
    { NFD_RadioX + 219, NFD_RadioY + 32  },  /* ILS */
    { NFD_RadioX + 297, NFD_RadioY + 32  },  /* MLS */
    { NFD_RadioX + 376, NFD_RadioY + 32  },  /* ADF */
    { NFD_RadioX + 455, NFD_RadioY + 32  },  /* BFO */
    { NFD_RadioX + 64,  NFD_RadioY + 126 },  /* HF1 */
    { NFD_RadioX + 219, NFD_RadioY + 126 },  /* HF2 */
    { NFD_RadioX + 297, NFD_RadioY + 126 },  /* AM */
    { NFD_RadioX + 64,  NFD_RadioY + 182 },  /* VHF1 */
    { NFD_RadioX + 136, NFD_RadioY + 182 },  /* VHF2 */
    { NFD_RadioX + 219, NFD_RadioY + 182 },  /* VFH3 */
    { NFD_RadioX + 158, NFD_RadioY + 139 }
};                                           /* SEL */

#define ToggleX        (NFD_RadioX + 546)
#define ToggleY        (NFD_RadioY + 63)
#define GuardX         (NFD_RadioX + 59)
#define GuardY         (NFD_RadioY + 14)
#define KnobX          (NFD_RadioX + 449)
#define KnobY          (NFD_RadioY + 147)
#define LeftPanelX     (NFD_RadioX + 46)
#define LeftPanelY     (NFD_RadioY + 238)
#define RightPanelX    (NFD_RadioX + 361)
#define RightPanelY    (NFD_RadioY + 238)
#define ChangeoverX    (NFD_RadioX + 254)
#define ChangeoverY    (NFD_RadioY + 241)

#define VhfBeacon      NavLib_DME | NavLib_ILS | NavLib_VOR
#define AdfBeacon      NavLib_NDB

float               KnobList[2][2];

static unsigned int DigitsMin;
static unsigned int DigitsMax;
static unsigned int DigitsInc;
static unsigned int PanelDigits;

static int          oldleftb;
static unsigned int NavMode;
static int          InnerKnobAngle;
static int          OuterKnobAngle;
static char         LeftPanelStr[20];
static char         RightPanelStr[20];
static unsigned int ticks;
static unsigned int ticklimit;
static bool         CrsMode;

static unsigned int OldIls1;
static unsigned int OldIls2;
static unsigned int OldVor1;
static unsigned int OldVor2;
static unsigned int OldAdf1;
static unsigned int OldAdf2;

static bool InsideCircle(int x, int y, int x0, int y0, int r);
static bool InsideBox(int x, int y, int x0, int y0, int xs, int ys);
static void FormCrsString(unsigned int n);
static void FormFreqString(unsigned int f, unsigned int mode, char v[]);
static void SetParameters();
static void SetFreq(unsigned int f, unsigned int mode);
static unsigned int GetStbyFreq(unsigned int mode);
static unsigned int GetActiveFreq(unsigned int mode);
static void ChangeoverSwitch();
static void DisplayPanel(char str[], unsigned int x, unsigned int y);
static void LCDChar(char Ch, int x, int y);
static void LED(int x, int y);
static void Segment(unsigned int n, int x, int y);

/* --------------------------------------------------- */
void Radio_RMP()
{
    unsigned int i1;
    unsigned int v1;
    unsigned int a1;

    i1 = Radio_Radios[0].NavILS.Active;
    if (i1 != OldIls1)
    {
        Nav_ILS1.SelectedBeacon = NavLib_LookupChannel(NavLink_AeroPkt.Latitude, NavLink_AeroPkt.Longitude, i1, VhfBeacon);
        /*
           printf("Radio_RMP: i1(%d) != OldIls1(%d) SB = %d\n", i1, OldIls1, Nav_ILS1.SelectedBeacon);
           printf("Radio_RMP: %f %f\n",NavLink_AeroPkt.Latitude, NavLink_AeroPkt.Longitude);
           fflush(stdout);
        */
        OldIls1 = i1;
    }
    v1 = Radio_Radios[0].NavVOR.Active;
    if (v1 != OldVor1)
    {
        Nav_VOR1.SelectedBeacon = NavLib_LookupChannel(NavLink_AeroPkt.Latitude, NavLink_AeroPkt.Longitude, v1, VhfBeacon);
        OldVor1                 = v1;
    }
    a1 = Radio_Radios[0].NavADF.Active;
    if (a1 != OldAdf1)
    {
        Nav_ADF1.SelectedBeacon = NavLib_LookupChannel(NavLink_AeroPkt.Latitude, NavLink_AeroPkt.Longitude, a1, AdfBeacon);
        OldAdf1                 = a1;
    }
}

/* --------------------------------------------------- */
static bool InsideCircle(int x, int y, int x0, int y0, int r)
{
    return (x - x0) * (x - x0) + (y - y0) * (y - y0) < r * r;
}

/* --------------------------------------------------- */
static bool InsideBox(int x, int y, int x0, int y0, int xs, int ys)
{
    return x >= x0 && x <= x0 + xs && y >= y0 && y <= y0 + ys;
}

/* --------------------------------------------------- */
void Radio_UpdateRMP(int n, int leftb, int middleb, int rightb, int x, int y)
{
    unsigned int i;
    unsigned int j;
    unsigned int PowerCode;
    bool         InnerKnob;

    if (Radio_Radios[0].PowerSwitch)
    {
        PowerCode = 10;
    }
    else
    {
        PowerCode = 11;
    }

    if (leftb & oldleftb)  /* check if left button held down */
    {
        ticks += 1;
        if (ticks >= 150)  /* speed up after 3s */
        {
            ticklimit = 2;  /* set to fast mode - update every 2nd frame */
        }
    }
    else
    {
        ticklimit = 20;  /* set to slow mode - update every 10th frame */
        ticks = 0;
    }
    
    if (leftb)
    {
        if (InsideCircle(x, y, KnobX, KnobY, 47))
        {
            InnerKnob = InsideCircle(x, y, KnobX, KnobY, 32);
            if (InnerKnob)
            {
                if (CrsMode)
                {
                    PanelDigits = Radio_Radios[0].CrsKnob;
                    if (x < KnobX)
                    {
                        InnerKnobAngle += 1;
                        if (ticks % ticklimit == 0)
                        {
                            PanelDigits = (PanelDigits + 358) % 360 + 1;  /* backwards 1 */
                        }
                    }
                    else
                    {
                        InnerKnobAngle -= 1;
                        if (ticks % ticklimit == 0)
                        {
                            PanelDigits = PanelDigits % 360 + 1;  /* forwards 1 */
                        }
                    }
                    if (InnerKnobAngle < 0)
                    {
                        InnerKnobAngle += 360;
                    }
                    else if (InnerKnobAngle > 360)
                    {
                        InnerKnobAngle -= 360;
                    }
                    FormCrsString(PanelDigits);
                    Radio_Radios[0].CrsKnob = PanelDigits;
                }
                else
                {
                    PanelDigits = GetStbyFreq(NavMode);
                    if (x < KnobX)
                    {
                        InnerKnobAngle += 1;
                        if (ticks % ticklimit == 0)
                        {
                            if ((PanelDigits % 100) != 0)
                            {
     							PanelDigits -= 5;
                            }
                        }
                    }
                    else
                    {
                        InnerKnobAngle -= 1;
                        if (ticks % ticklimit == 0)
                        {
                            if ((PanelDigits % 100) != 95)
                            {
                                PanelDigits += 5;
                            }
                        }
                    }
                    if (InnerKnobAngle < 0)
                    {
                        InnerKnobAngle += 360;
                    }
                    else if (InnerKnobAngle > 360)
                    {
                        InnerKnobAngle -= 360;
                    }
                    FormFreqString(PanelDigits, NavMode, RightPanelStr);
                    SetFreq(PanelDigits, NavMode);
                }
            }
            else
            {
                PanelDigits = GetStbyFreq(NavMode);
                if (x < KnobX)
                {
                    OuterKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits -= 100;
                    }
                }
                else
                {
                    OuterKnobAngle -= 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits += 100;
                    }
                }
                if (OuterKnobAngle < 0)
                {
                    OuterKnobAngle += 360.0;
                }
                else if (OuterKnobAngle > 360)
                {
                    OuterKnobAngle -= 360;
                }
                
                if (PanelDigits > DigitsMax)
                {
                    PanelDigits = DigitsMax;
                }
                else if (PanelDigits < DigitsMin)
                {
                    PanelDigits = DigitsMin;
                }
                
                FormFreqString(PanelDigits, NavMode, RightPanelStr);
                SetFreq(PanelDigits, NavMode);
            }
        }
    }
    
    if (leftb && !oldleftb)
    {
        if (InsideCircle(x, y, ToggleX, ToggleY, 15))
        {
            Radio_Radios[0].PowerSwitch = !Radio_Radios[0].PowerSwitch;
            PowerCode      = 21 - PowerCode;
            if (!Radio_Radios[0].PowerSwitch)
            {
                for (i = 1; i <= 13; i += 1)
                {
                    Radio_Radios[0].PushSwitches[i] = false;
                }
            }
            else
            {
                strcpy(LeftPanelStr, "88888.88");
                strcpy(RightPanelStr, LeftPanelStr);
            }
        }
        else if (Radio_Radios[0].NavGuard && InsideBox(x, y, GuardX, GuardY, 45, 80))
        {
            Radio_Radios[0].NavGuard = false;
        }
        else if (!Radio_Radios[0].NavGuard && InsideBox(x, y, GuardX, GuardY + 40, 45, 30))
        {
            Radio_Radios[0].NavGuard = true;
        }
        else if (Radio_Radios[0].PowerSwitch)
        {
            if (InsideBox(x, y, ChangeoverX, ChangeoverY, 92, 45))
            {
                ChangeoverSwitch();
                CrsMode = !CrsMode;
            }
            else
            {
                for (i = 1; i <= 13; i += 1)
                {
                    if (i == SwSEL)
                    {
                        if (InsideCircle(x, y, NFD_RadioX + 158, NFD_RadioY + 139, 29))
                        {
                            Radio_Radios[0].PushSwitches[i] = !Radio_Radios[0].PushSwitches[i];
                        }
                    }
                    else if (InsideBox(x, y, SwitchCoords[i].xc, SwitchCoords[i].yc, 40, 30))
                    {
                        if (!((i == SwNAV) && Radio_Radios[0].NavGuard))
                        {
                            if (i == SwSEL || i == SwNAV)
                            {
                                Radio_Radios[0].PushSwitches[i] = !Radio_Radios[0].PushSwitches[i];
                            }
                            else
                            {
                                for (j = 2; j <= 6; j += 1)
                                {
                                    Radio_Radios[0].PushSwitches[j] = false;
                                }
                                for (j = 7; j <= 12; j += 1)
                                {
                                    Radio_Radios[0].PushSwitches[j] = false;
                                }
                                Radio_Radios[0].PushSwitches[i] = true;
                                NavMode              = i;
                                Radio_Radios[0].Mode = NavMode;
                                SetParameters();
                                PanelDigits = GetActiveFreq(NavMode);
                                FormFreqString(PanelDigits, NavMode, LeftPanelStr);
                                PanelDigits = GetStbyFreq(NavMode);
                                FormFreqString(PanelDigits, NavMode, RightPanelStr);
                            }
                        }
                    }
                }
            }
        }
    }
    
    oldleftb  = leftb;

    PanelLib_DisplayRadio(Radio_Radios[0].PushSwitches[SwVHF1],
                          Radio_Radios[0].PushSwitches[SwVHF2],
                          Radio_Radios[0].PushSwitches[SwVHF3],
                          Radio_Radios[0].PushSwitches[SwHF1],
                          Radio_Radios[0].PushSwitches[SwSEL],
                          Radio_Radios[0].PushSwitches[SwHF2],
                          Radio_Radios[0].PushSwitches[SwAM],
                          Radio_Radios[0].PushSwitches[SwNAV],
                          Radio_Radios[0].PushSwitches[SwVOR],
                          Radio_Radios[0].PushSwitches[SwILS],
                          Radio_Radios[0].PushSwitches[SwMLS],
                          Radio_Radios[0].PushSwitches[SwADF],
                          Radio_Radios[0].PushSwitches[SwBFO],
                          Radio_Radios[0].NavGuard,
                          PowerCode,
                          (float) InnerKnobAngle,
                          (float) OuterKnobAngle);

    if (Radio_Radios[0].PowerSwitch)
    {
        glDisable(GL_TEXTURE_2D);
        DisplayPanel(LeftPanelStr, LeftPanelX + 10, LeftPanelY + 9);
        DisplayPanel(RightPanelStr, RightPanelX + 10, RightPanelY + 9);
    }
}

/* --------------------------------------------------- */
static void FormCrsString(unsigned int n)
{
    RightPanelStr[0] = ' ';
    RightPanelStr[1] = ' ';
    RightPanelStr[2] = n / 100 + '0';
    n                = n % 100;
    RightPanelStr[3] = n / 10 + '0';
    RightPanelStr[4] = n % 10 + '0';
    RightPanelStr[5] = 0;
}

/* --------------------------------------------------- */
static void FormFreqString(unsigned int f, unsigned int mode, char v[])
{
    char d1, d2, d3, d4, d5;

    d1 = f / 10000 + '0';
    d2 = f % 10000 / 1000 + '0';
    d3 = f % 1000 / 100 + '0';
    d4 = f % 100 / 10 + '0';
    d5 = f % 10 + '0';
    if (mode == SwADF)
    {
        v[0] = ' ';
        v[1] = ' ';
        v[2] = d2;
        v[3] = d3;
        v[4] = d4;
        v[5] = '.';
        v[6] = d5;
        v[7] = '0';
        v[8] = 0;
    }
    else
    {
        v[0] = ' ';
        v[1] = ' ';
        v[2] = d1;
        v[3] = d2;
        v[4] = d3;
        v[5] = '.';
        v[6] = d4;
        v[7] = d5;
        v[8] = 0;
    }
}

/* --------------------------------------------------- */
static void SetParameters()
{
    switch (NavMode)
    {
        case SwVOR:
            DigitsMin   = 10800;
            DigitsMax   = 11795;
            DigitsInc   = 5;
            PanelDigits = Radio_Radios[0].NavVOR.Stby;
            break;
        case SwILS:
            DigitsMin   = 10800;
            DigitsMax   = 11195;
            DigitsInc   = 5;
            PanelDigits = Radio_Radios[0].NavILS.Stby;
            break;
        case SwADF:
            DigitsMin   = 1900;
            DigitsMax   = 5350;
            DigitsInc   = 5;
            PanelDigits = Radio_Radios[0].NavADF.Stby;
            break;
        case SwVHF1:
            DigitsMin   = 11800;
            DigitsMax   = 13595;
            DigitsInc   = 5;
            PanelDigits = Radio_Radios[0].ComVHF1.Stby;
            break;
        case SwVHF2:
            DigitsMin   = 11800;
            DigitsMax   = 13595;
            DigitsInc   = 5;
            PanelDigits = Radio_Radios[0].ComVHF2.Stby;
            break;
        case SwVHF3:
            DigitsMin   = 11800;
            DigitsMax   = 13595;
            DigitsInc   = 5;
            PanelDigits = Radio_Radios[0].ComVHF3.Stby;
            break;
        default:
            DigitsMin   = 0;
            DigitsMax   = 0;
            DigitsInc   = 0;
            PanelDigits = 0;
            break;
    }
}

/* --------------------------------------------------- */
static void SetFreq(unsigned int f, unsigned int mode)
{
    switch (mode)
    {
        case SwVOR:
            Radio_Radios[0].NavVOR.Stby = f;
            break;
        case SwILS:
            Radio_Radios[0].NavILS.Stby = f;
            break;
        case SwADF:
            Radio_Radios[0].NavADF.Stby = f;
            break;
        case SwVHF1:
            Radio_Radios[0].ComVHF1.Stby = f;
            break;
        case SwVHF2:
            Radio_Radios[0].ComVHF2.Stby = f;
            break;
        case SwVHF3:
            Radio_Radios[0].ComVHF3.Stby = f;
            break;
        default:
            return;
            break;
    }
}

/* --------------------------------------------------- */
static unsigned int GetStbyFreq(unsigned int mode)
{
    switch (mode)
    {
        case SwVOR:
            return Radio_Radios[0].NavVOR.Stby;
            break;
        case SwILS:
            return Radio_Radios[0].NavILS.Stby;
            break;
        case SwADF:
            return Radio_Radios[0].NavADF.Stby;
            break;
        case SwVHF1:
            return Radio_Radios[0].ComVHF1.Stby;
            break;
        case SwVHF2:
            return Radio_Radios[0].ComVHF2.Stby;
            break;
        case SwVHF3:
            return Radio_Radios[0].ComVHF3.Stby;
            break;
        default:
            return 0;
            break;
    }
}

/* --------------------------------------------------- */
static unsigned int GetActiveFreq(unsigned int mode)
{
    switch (mode)
    {
        case SwVOR:
            return Radio_Radios[0].NavVOR.Active;
            break;
        case SwILS:
            return Radio_Radios[0].NavILS.Active;
            break;
        case SwADF:
            return Radio_Radios[0].NavADF.Active;
            break;
        case SwVHF1:
            return Radio_Radios[0].ComVHF1.Active;
            break;
        case SwVHF2:
            return Radio_Radios[0].ComVHF2.Active;
            break;
        case SwVHF3:
            return Radio_Radios[0].ComVHF3.Active;
            break;
        default:
            return 0;
            break;
    }
}

/* --------------------------------------------------- */
static void ChangeoverSwitch()
{
    unsigned int       t;
    NavDefn_RadioPanel *w;

    {
        w = &Radio_Radios[0];

        switch (NavMode)
        {
            case SwVOR:
                t                = w->NavVOR.Active;
                w->NavVOR.Active = w->NavVOR.Stby;
                w->NavVOR.Stby   = t;
                FormFreqString(w->NavVOR.Active, NavMode, LeftPanelStr);
                FormFreqString(w->NavVOR.Stby, NavMode, RightPanelStr);
                break;
            case SwILS:
                t                = w->NavILS.Active;
                w->NavILS.Active = w->NavILS.Stby;
                w->NavILS.Stby   = t;
                FormFreqString(w->NavILS.Active, NavMode, LeftPanelStr);
                FormFreqString(w->NavILS.Stby, NavMode, RightPanelStr);
                break;
            case SwADF:
                t                = w->NavADF.Active;
                w->NavADF.Active = w->NavADF.Stby;
                w->NavADF.Stby   = t;
                FormFreqString(w->NavADF.Active, NavMode, LeftPanelStr);
                FormFreqString(w->NavADF.Stby, NavMode, RightPanelStr);
                break;
            case SwVHF1:
                t                 = w->ComVHF1.Active;
                w->ComVHF1.Active = w->ComVHF1.Stby;
                w->ComVHF1.Stby   = t;
                FormFreqString(w->ComVHF1.Active, NavMode, LeftPanelStr);
                FormFreqString(w->ComVHF1.Stby, NavMode, RightPanelStr);
                break;
            case SwVHF2:
                t                 = w->ComVHF2.Active;
                w->ComVHF2.Active = w->ComVHF2.Stby;
                w->ComVHF2.Stby   = t;
                FormFreqString(w->ComVHF2.Active, NavMode, LeftPanelStr);
                FormFreqString(w->ComVHF2.Stby, NavMode, RightPanelStr);
                break;
            case SwVHF3:
                t                 = w->ComVHF3.Active;
                w->ComVHF3.Active = w->ComVHF3.Stby;
                w->ComVHF3.Stby   = t;
                FormFreqString(w->ComVHF3.Active, NavMode, LeftPanelStr);
                FormFreqString(w->ComVHF3.Stby, NavMode, RightPanelStr);
                break;
            default:
                return;
                break;
        }
    }
}

/* --------------------------------------------------- */
static void DisplayPanel(char str[], unsigned int x, unsigned int y)
{
    unsigned int i;
    char         Ch;

    Glib_Colour(Glib_RED);
    for (i = 0; i <= 7; i += 1)
    {
        Ch = str[i];
        if (Ch == 0)
        {
            return;
        }
        if (Ch == '.')
        {
            LCDChar('.', x - 3, y);
            x = x + 10;
        }
        else
        {
            LCDChar(Ch, x, y);
            x = x + 25;
        }
    }
}

/* --------------------------------------------------- */
static void LCDChar(char Ch, int x, int y)
{
    unsigned int Segments[10][8] = {
        { 1, 2, 3, 4, 5, 6, 0, 0 }, /* 0 */
        { 2, 3, 0, 0, 0, 0, 0, 0 }, /* 1 */
        { 1, 6, 7, 3, 4, 0, 0, 0 }, /* 2 */
        { 1, 2, 3, 4, 7, 0, 0, 0 }, /* 3 */
        { 2, 3, 5, 7, 0, 0, 0, 0 }, /* 4 */
        { 4, 5, 7, 2, 1, 0, 0, 0 }, /* 5 */
        { 4, 5, 1, 2, 6, 7, 0, 0 }, /* 6 */
        { 2, 3, 4, 0, 0, 0, 0, 0 }, /* 7 */
        { 1, 2, 3, 4, 5, 6, 7, 0 }, /* 8 */
        { 1, 2, 3, 4, 5, 7, 0, 0 }
    };                              /* 9 */

    unsigned int p;
    unsigned int i;
    unsigned int s;

    if (Ch == '.')
    {
        LED(x, y);
        return;
    }
    if (Ch >= '0' && Ch <= '9')
    {
        p = Ch - '0';
        for (i = 0; i <= 8; i += 1)
        {
            s = Segments[p][i];
            if (s == 0)
            {
                return;
            }
            else
            {
                Segment(s, x, y);
            }
        }
    }
}

/* --------------------------------------------------- */
static void LED(int x, int y)
{
    Glib_Draw(x + 2, y, x + 5, y);
    Glib_Draw(x + 1, y + 1, x + 6, y + 1);
    Glib_Draw(x, y + 2, x + 7, y + 2);
    Glib_Draw(x, y + 3, x + 7, y + 3);
    Glib_Draw(x, y + 4, x + 7, y + 4);
    Glib_Draw(x, y + 5, x + 7, y + 5);
    Glib_Draw(x + 1, y + 6, x + 6, y + 6);
    Glib_Draw(x + 2, y + 7, x + 5, y + 7);
}

/* --------------------------------------------------- */
static void Segment(unsigned int n, int x, int y)
{
    int i;

    if (n == 7)
    {
        Glib_Draw(x + 1, y + 15, x + 14, y + 15);
        Glib_Draw(x + 2, y + 16, x + 13, y + 16);
        Glib_Draw(x + 2, y + 14, x + 13, y + 14);
    }
    else
    {
        for (i = 0; i <= 2; i += 1)
        {
            switch (n)
            {
                case 1:
                    Glib_Draw(x + i + 1, y + i, x + 14 - i, y + i);
                    break;
                case 2:
                    Glib_Draw(x + 15 - i, y + 1 + i, x + 15 - i, y + 14 - i);
                    break;
                case 3:
                    Glib_Draw(x + 15 - i, y + 16 + i, x + 15 - i, y + 29 - i);
                    break;
                case 4:
                    Glib_Draw(x + i + 1, y + 30 - i, x + 14 - i, y + 30 - i);
                    break;
                case 5:
                    Glib_Draw(x + i, y + 16 + i, x + i, y + 29 - i);
                    break;
                case 6:
                    Glib_Draw(x + i, y + 1 + i, x + i, y + 14 - i);
                    break;
            }
        }
    }
}

/* --------------------------------------------------- */
void Radio_SaveRMP(NavDefn_NavDataPkt *pkt)
{
    memcpy(&pkt->SavedRadios[0], &Radio_Radios[0], sizeof(NavDefn_RadioPanel));
}

/* --------------------------------------------------- */
void Radio_RestoreRMP(IosDefn_RestoreVectorRecord v)
{
    unsigned int i;

    memcpy(&Radio_Radios[0], &v.SavedRadios[0], sizeof(NavDefn_RadioPanel));
    NavMode = 0;
    for (i = 2; i <= 13; i += 1)
    {
        if (Radio_Radios[0].PushSwitches[i])
        {
            NavMode = i;
        }
    }

    SetParameters();
    PanelDigits = GetActiveFreq(NavMode);
    FormFreqString(PanelDigits, NavMode, LeftPanelStr);
    PanelDigits = GetStbyFreq(NavMode);
    FormFreqString(PanelDigits, NavMode, RightPanelStr);

    OldIls1 = 0;
    OldIls2 = 0;
    OldVor1 = 0;
    OldVor2 = 0;
    OldAdf1 = 0;
    OldAdf2 = 0;
}

/* --------------------------------------------------- */
void BEGIN_Radio()
{
    int j;
    NavDefn_RadioPanel *w;

    w                 = &Radio_Radios[0];
    w->NavVOR.Active  = 11640;
    w->NavVOR.Stby    = 10800;
    w->NavILS.Active  = 11090;
    w->NavILS.Stby    = 10800;
    w->NavADF.Active  = 1950;
    w->NavADF.Stby    = 1950;
    w->ComVHF1.Active = 11800;
    w->ComVHF1.Stby   = 11800;
    w->ComVHF2.Active = 11800;
    w->ComVHF2.Stby   = 11800;
    w->ComVHF3.Active = 11800;
    w->ComVHF3.Stby   = 11800;
    w->ComHF1.Active  = 0;
    w->ComHF1.Stby    = 0;
    w->ComHF2.Active  = 0;
    w->ComHF2.Stby    = 0;
    w->ComAM.Active   = 0;
    w->ComAM.Stby     = 0;
    w->CrsKnob        = 360;
    w->NavGuard       = true;
    w->PowerSwitch    = false;
    for (j=1; j<=13; j+=1)
    {
        w->PushSwitches[j] = false;
    }
    w->Mode           = 0;

    NavMode          = 0;
    InnerKnobAngle   = 0;
    OuterKnobAngle   = 0;
    oldleftb         = false;
    LeftPanelStr[0]  = 0;
    RightPanelStr[0] = 0;
    ticks            = 0;
    DigitsMin        = 11800;
    DigitsMax        = 13595;
    DigitsInc        = 5;
    OldIls1          = 0;
    OldIls2          = 0;
    OldVor1          = 0;
    OldVor2          = 0;
    OldAdf1          = 0;
    OldAdf2          = 0;
    CrsMode          = false;
}
