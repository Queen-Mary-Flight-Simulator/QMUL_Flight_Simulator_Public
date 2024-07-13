/* +------------------------------+---------------------------------+
   | Module      : fcu.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-04      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : FCU Panel                                        |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdbool.h>
#include <math.h>

#include <GL/gl.h>

#include <SIM/maths.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/glib.h>

#include "navlink.h"
#include "panellib.h"
#include "nfd.h"
#include "fcu.h"

bool                   FCU_FD;
bool                   FCU_LS;
bool                   FCU_LOC;
bool                   FCU_AP1;
bool                   FCU_AP2;
bool                   FCU_ATHR;
bool                   FCU_EXPED;
bool                   FCU_APPR;
bool                   FCU_SPD_MACH_Button;
unsigned int           FCU_BaroPressure;
bool                   FCU_BaroHg;
bool                   FCU_BaroSTD;
bool                   FCU_HDG_TRK_Button;
unsigned int           FCU_MACH;
unsigned int           FCU_HDG;
unsigned int           FCU_TRK;
unsigned int           FCU_ALT;
unsigned int           FCU_SPD;
int                    FCU_VS;
int                    FCU_FPA;
bool                   FCU_Metric_Alt_Button;
bool                   FCU_HDG_Hold;
bool                   FCU_ALT_Hold;
bool                   FCU_SPD_Hold;
bool                   FCU_VS_Hold;
unsigned int           FCU_ALT_Range;

NavDefn_FCUNav         FCU_NavSwitch1;
NavDefn_FCUNav         FCU_NavSwitch2;
NavDefn_FCUMode        FCU_ModeSelector;
NavDefn_FCUData        FCU_DataMode;
unsigned int           FCU_RangeSelector;

bool                   FCU_CSTR_Button;
bool                   FCU_WPT_Button;
bool                   FCU_VORD_Button;
bool                   FCU_NDB_Button;
bool                   FCU_ARPT_Button;

#define ONERAD    (180.0 / Maths_PI)
#define DEG30     (30.0 / ONERAD)
#define InHg      0
#define hPa       1

typedef struct
{
    int xc;
    int yc;
} Vertex;

typedef struct
{
    int xc;
    int yc;
    int xs;
    int ys;
} Rect;

#define SwFD       0
#define SwLS       1
#define SwCSTR     2
#define SwWPT      3
#define SwVORD     4
#define SwNDB      5
#define SwARPT     6
#define SwLOC      7
#define SwAP1      8
#define SwAP2      9
#define SwATHR     10
#define SwEXPED    11
#define SwAPPR     12
typedef unsigned char   PushSwitchType;

static Rect   SwitchCoords[13] = {
    { NFD_FCUX + 28,  NFD_FCUY + 11,  29, 24 },  /* FD */
    { NFD_FCUX + 66,  NFD_FCUY + 11,  29, 24 },  /* LS */
    { NFD_FCUX + 114, NFD_FCUY + 146, 32, 24 },  /* CSTR */
    { NFD_FCUX + 151, NFD_FCUY + 146, 32, 24 },  /* WPT */
    { NFD_FCUX + 189, NFD_FCUY + 146, 32, 24 },  /* VORD */
    { NFD_FCUX + 225, NFD_FCUY + 146, 32, 24 },  /* NDB */
    { NFD_FCUX + 262, NFD_FCUY + 146, 32, 24 },  /* ARPT */
    { NFD_FCUX + 438, NFD_FCUY + 14,  29, 24 },  /* LOC */
    { NFD_FCUX + 502, NFD_FCUY + 46,  29, 29 },  /* AP1 */
    { NFD_FCUX + 545, NFD_FCUY + 46,  29, 29 },  /* AP" */
    { NFD_FCUX + 523, NFD_FCUY + 8,   29, 29 },  /* ATHR */
    { NFD_FCUX + 608, NFD_FCUY + 11,  29, 24 },  /* EXPED */
    { NFD_FCUX + 698, NFD_FCUY + 11,  29, 24 }   /* APPR */
};

static Vertex KnobCoords[5] = {
    { NFD_FCUX + 60,  NFD_FCUY + 67 },
    { NFD_FCUX + 383, NFD_FCUY + 83 },
    { NFD_FCUX + 452, NFD_FCUY + 83 },
    { NFD_FCUX + 622, NFD_FCUY + 83 },
    { NFD_FCUX + 713, NFD_FCUY + 83 }
};

#define Nav1X             NFD_FCUX + 162  /* NAV1 selector */
#define Nav1Y             NFD_FCUY + 44
#define Nav2X             NFD_FCUX + 253  /* NAV2 selector */
#define Nav2Y             NFD_FCUY + 44
#define NavSelectorX      NFD_FCUX + 162  /* MAV mode selector */
#define NavSelectorY      NFD_FCUY + 93
#define RangeSelectorX    NFD_FCUX + 253  /* range selector */
#define RangeSelectorY    NFD_FCUY + 93

bool                          PushSwitchList[13];
float                         KnobAngle[5];

static NavDefn_FCUNav NavSwitch1;
static NavDefn_FCUNav NavSwitch2;
static NavDefn_FCUMode    NavSelector;
static int                    RangeSelector;

static unsigned int           CurrentKnob;
static unsigned int           CurrentSwitch;
static unsigned int           CurrentSelector;
static unsigned int           PanelMode;
static bool                   MetricMode;

static unsigned int           i;
static int                    oldleftb;
static int                    oldmiddleb;
static int                    oldrightb;
static unsigned int           ticks;
static unsigned int           ticklimit;
static bool                   OldBaroHg;

static void DisplayBaroPanel(unsigned int p, unsigned int m);
static bool InsideCircle(int x, int y, int x0, int y0, int r);
static bool InsideBox(int x, int y, int x0, int y0, int xs, int ys);
static void GetFCU();
static void SetFCU();

/* static void LCDChars(char Str[], int x, int y); */
static void LCDChar(char Ch, int x, int y);
static void LED(int x, int y);
static void DisplayPanel(int n, int f);
static void DisplayLCDPanel(int x, int y, unsigned int n, unsigned int f,
                            unsigned int ndigits, bool MinusSign);

/* ------------------------------------------------------ */
static void DisplayBaroPanel(unsigned int p, unsigned int m)
{
    Glib_SetFont(Glib_GFONT16, 10);
    Glib_Colour(Glib_WHITE);
    if (m < 2)
    {
        Glib_Chars("STD", NFD_FCUX + 45, NFD_FCUY + 142);
    }
    if (m > 0)
    {
        DisplayLCDPanel(NFD_FCUX + 23, NFD_FCUY + 119, 1, p, 4, false);
    }
}

/* ------------------------------------------------------ */
static bool InsideCircle(int x, int y, int x0, int y0, int r)
{
    return (x - x0) * (x - x0) + (y - y0) * (y - y0) < r * r;
}

/* ------------------------------------------------------ */
static bool InsideBox(int x, int y, int x0, int y0, int xs, int ys)
{
    return x >= x0 && x <= x0 + xs && y >= y0 && y <= y0 + ys;
}

/* ------------------------------------------------------ */
void FCU_InitialiseFCU()
{
    return;
}

/* ------------------------------------------------------ */
void FCU_StopFCU()
{
    return;
}

/* ------------------------------------------------------ */
void FCU_UpdateFCU(int leftb, int middleb, int rightb, int x, int y)
{
    unsigned int i;

    GetFCU();
    
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
    
    if (leftb && !oldleftb)
    {
        if (InsideCircle(x, y, Nav1X, Nav1Y, 20))
        {
            if (x <= Nav1X)
            {
                if (NavSwitch1 == NavDefn_NavOFF)
                {
                    NavSwitch1 = NavDefn_NavADF;
                }
                else if (NavSwitch1 == NavDefn_NavVOR)
                {
                    NavSwitch1 = NavDefn_NavOFF;
                }
            }
            else
            {
                if (NavSwitch1 == NavDefn_NavOFF)
                {
                    NavSwitch1 = NavDefn_NavVOR;
                }
                else if (NavSwitch1 == NavDefn_NavADF)
                {
                    NavSwitch1 = NavDefn_NavOFF;
                }
            }
        }
        else if (InsideCircle(x, y, Nav2X, Nav2Y, 20))
        {
            if (x < Nav2X)
            {
                if (NavSwitch2 == NavDefn_NavOFF)
                {
                    NavSwitch2 = NavDefn_NavADF;
                }
                else if (NavSwitch2 == NavDefn_NavVOR)
                {
                    NavSwitch2 = NavDefn_NavOFF;
                }
            }
            else
            {
                if (NavSwitch2 == NavDefn_NavOFF)
                {
                    NavSwitch2 = NavDefn_NavVOR;
                }
                else if (NavSwitch2 == NavDefn_NavADF)
                {
                    NavSwitch2 = NavDefn_NavOFF;
                }
            }
        }
        else if (InsideCircle(x, y, NavSelectorX, NavSelectorY, 17))
        {
            if (x < NavSelectorX)
            {
                switch (NavSelector)
                {
                    case NavDefn_ModeVOR:
                        NavSelector = NavDefn_ModeILS;
                        break;
                    case NavDefn_ModeNAV:
                        NavSelector = NavDefn_ModeVOR;
                        break;
                    case NavDefn_ModeARC:
                        NavSelector = NavDefn_ModeNAV;
                        break;
                    case NavDefn_ModePLAN:
                        NavSelector = NavDefn_ModeARC;
                        break;
                    default:
                        break;
                }
            }
            else
            {
                switch (NavSelector)
                {
                    case NavDefn_ModeILS:
                        NavSelector = NavDefn_ModeVOR;
                        break;
                    case NavDefn_ModeVOR:
                        NavSelector = NavDefn_ModeNAV;
                        break;
                    case NavDefn_ModeNAV:
                        NavSelector = NavDefn_ModeARC;
                        break;
                    case NavDefn_ModeARC:
                        NavSelector = NavDefn_ModePLAN;
                        break;
                    default:
                        break;
                }
            }
        }
        else if (InsideCircle(x, y, RangeSelectorX, RangeSelectorY, 50))
        {
            if (x < RangeSelectorX)
            {
                if (RangeSelector > 0)
                {
                    RangeSelector = RangeSelector - 1;
                }
            }
            else
            {
                if (RangeSelector < 5)
                {
                    RangeSelector = RangeSelector + 1;
                }
            }
        }

        for (i = 0; i <= 12; i += 1)
        {
            if (InsideBox(x, y, SwitchCoords[i].xc, SwitchCoords[i].yc,
                          SwitchCoords[i].xs, SwitchCoords[i].ys))
            {
                PushSwitchList[i] = !PushSwitchList[i];
            }
        }
        if (InsideCircle(x, y, NFD_FCUX + 333, NFD_FCUY + 98, 20))
        {
            FCU_SPD_MACH_Button = !FCU_SPD_MACH_Button;
        }
        else if (InsideCircle(x, y, NFD_FCUX + 539, NFD_FCUY + 98, 20))
        {
            FCU_HDG_TRK_Button = !FCU_HDG_TRK_Button;
        }
        else if (InsideCircle(x, y, NFD_FCUX + 669, NFD_FCUY + 98, 20))
        {
            FCU_Metric_Alt_Button = !FCU_Metric_Alt_Button;
        }
        else if (InsideCircle(x, y, NFD_FCUX + 40, NFD_FCUY + 91, 20))
        {
            FCU_BaroHg = true;
            if (!OldBaroHg)
            {
                FCU_BaroPressure = intround((float) FCU_BaroPressure * 2992.0 / 1013.0);
                OldBaroHg        = true;
            }
        }
        else if (InsideCircle(x, y, NFD_FCUX + 86, NFD_FCUY + 91, 10))
        {
            FCU_BaroHg = false;
            if (OldBaroHg)
            {
                FCU_BaroPressure = intround((float) FCU_BaroPressure * 1013.0 / 2992.0);
                OldBaroHg        = false;
            }
        }
        else
        {
            for (i = 0; i <= 4; i += 1)
            {
                if (InsideCircle(x, y, KnobCoords[i].xc, KnobCoords[i].yc, 5))
                {
                    switch (i)
                    {
                        case 0:
                            FCU_BaroSTD = (FCU_BaroSTD + 1) % 3;
                            if (FCU_BaroSTD == 0)
                            {
                                if (FCU_BaroHg)
                                {
                                    FCU_BaroPressure = 2992;
                                }
                                else
                                {
                                    FCU_BaroPressure = 1013;
                                }
                            }
                            break;
                        case 1:
                            FCU_SPD_Hold = !FCU_SPD_Hold;
                            break;
                        case 2:
                            FCU_HDG_Hold = !FCU_HDG_Hold;
                            break;
                        case 3:
                            FCU_ALT_Hold = !FCU_ALT_Hold;
                            if (FCU_ALT_Hold)
                            {
                                FCU_VS_Hold = false;
                            }
                            break;
                        case 4:
                            FCU_VS_Hold = !FCU_VS_Hold;
                            if (FCU_VS_Hold)
                            {
                                FCU_ALT_Hold = false;
                            }
                            break;
                    }
                }
            }
        }
    }

    if (leftb)
    {
        for (i = 0; i <= 4; i += 1)
        {
            if (InsideCircle(x, y, KnobCoords[i].xc, KnobCoords[i].yc, 30))
            {
                if (x < KnobCoords[i].xc)
                {
                    KnobAngle[i] = KnobAngle[i] + 1.0;
                    if (KnobAngle[i] > 360.0)
                    {
                        KnobAngle[i] = KnobAngle[i] - 360.0;
                    }
                    if (ticks % ticklimit == 0)
                    {
                        switch (i)
                        {
                            case 0:
                                FCU_BaroPressure = FCU_BaroPressure - 1;
                                if (FCU_BaroSTD == 0)
                                {
                                    FCU_BaroSTD = 1;
                                }
                                if (FCU_BaroHg)
  							    {
                                    if (FCU_BaroPressure < 2800)
                                    {
                                        FCU_BaroPressure = 2800;
                                    }
                                }
                                else
                                {
                                    if (FCU_BaroPressure < 950)
                                    {
                                        FCU_BaroPressure = 950;
                                    }
                                }
                                break;
                            case 1:
                                if (FCU_SPD > 0)
                                {
                                    FCU_SPD = FCU_SPD - 1;
                                }
                                break;
                            case 2:
                                if (FCU_HDG == 1)
                                {
                                    FCU_HDG = 360;
                                }
                                else
                                {
                                    FCU_HDG = FCU_HDG - 1;
                                }
                                break;
                            case 3:
                                if (FCU_ALT >= 100)
                                {
                                    FCU_ALT = FCU_ALT - 100;
                                }
                                break;
                            case 4:
                                FCU_VS = FCU_VS - 10;
                                if (FCU_VS < -2000)
                                {
                                    FCU_VS = -2000;
                                }
                                break;
                        }
                    }
                }
                else
                {
                    KnobAngle[i] = KnobAngle[i] - 1.0;
                    if (KnobAngle[i] < 0.0)
                    {
                        KnobAngle[i] = KnobAngle[i] + 360.0;
                    }
                    if (ticks % ticklimit == 0)
                    {
                        switch (i)
                        {
                            case 0:
                                FCU_BaroPressure = FCU_BaroPressure + 1;
                                if (FCU_BaroSTD == 0)
                                {
                                    FCU_BaroSTD = 1;
                                }
                                if (FCU_BaroHg)
                                {
                                    if (FCU_BaroPressure > 3100)
                                    {
                                        FCU_BaroPressure = 3100;
                                    }
                                }
                                else
                                {
                                    if (FCU_BaroPressure > 1050)
                                    {
                                        FCU_BaroPressure = 1050;
                                    }
                                }
                                break;
                            case 1:
                                FCU_SPD = FCU_SPD + 1;
                                if (FCU_SPD > 400)
                                {
                                    FCU_SPD = 400;
                                }
                                break;
                            case 2:
                                FCU_HDG = FCU_HDG + 1;
                                if (FCU_HDG > 360)
                                {
                                    FCU_HDG = 1;
                                }
                                break;
                            case 3:
                                FCU_ALT = FCU_ALT + 100;
                                if (FCU_ALT > 40000)
                                {
                                    FCU_ALT = 40000;
                                }
                                break;
                            case 4:
                                FCU_VS = FCU_VS + 10;
                                if (FCU_VS > 2000)
                                {
                                    FCU_VS = 2000;
                                }
                                break;
                        }
                    }
                }
            }
        }
    }

    oldleftb   = leftb;
    oldmiddleb = middleb;
    oldrightb  = rightb;

    PanelLib_DisplayFCU(PushSwitchList[SwFD],
                        PushSwitchList[SwLS],
                        PushSwitchList[SwCSTR],
                        PushSwitchList[SwWPT],
                        PushSwitchList[SwVORD],
                        PushSwitchList[SwNDB],
                        PushSwitchList[SwARPT],
                        PushSwitchList[SwLOC],
                        PushSwitchList[SwAP1],
                        PushSwitchList[SwAP2],
                        PushSwitchList[SwATHR],
                        PushSwitchList[SwEXPED],
                        PushSwitchList[SwAPPR],
                        NavSwitch1 + 2,
                        NavSwitch2 + 2,
                        NavSelector,
                        RangeSelector,
                        KnobAngle[0],
                        KnobAngle[1],
                        KnobAngle[2],
                        KnobAngle[3],
                        KnobAngle[4],
                        FCU_BaroHg);

    glDisable(GL_TEXTURE_2D);

    DisplayBaroPanel(FCU_BaroPressure, FCU_BaroSTD);
    DisplayPanel(2, FCU_SPD);
    DisplayPanel(3, FCU_HDG);
    DisplayPanel(4, FCU_ALT);
    DisplayPanel(5, FCU_VS);

    Glib_Colour(Glib_GREEN);
    if (FCU_SPD_Hold)
    {
        LED(NFD_FCUX + 383, NFD_FCUY + 130);
    }
    if (FCU_HDG_Hold)
    {
        LED(NFD_FCUX + 483, NFD_FCUY + 130);
    }
    if (FCU_ALT_Hold)
    {
        LED(NFD_FCUX + 651, NFD_FCUY + 130);
    }
    if (FCU_VS_Hold)
    {
        LED(NFD_FCUX + 731, NFD_FCUY + 130);
    }

    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(1.0);
    Glib_SetFont(Glib_GFONT8, 7);

    if (FCU_SPD_MACH_Button)
    {
        Glib_Chars("SPD", NFD_FCUX + 346, NFD_FCUY + 155);
    }
    else
    {
        Glib_Chars("MACH", NFD_FCUX + 346, NFD_FCUY + 155);
    }
    if (FCU_HDG_TRK_Button)
    {
        Glib_Chars("TRK", NFD_FCUX + 466, NFD_FCUY + 150);
        Glib_Chars("TRK", NFD_FCUX + 506, NFD_FCUY + 130);
        Glib_Chars("FPA", NFD_FCUX + 555, NFD_FCUY + 130);
        Glib_Chars("FPA", NFD_FCUX + 710, NFD_FCUY + 150);
    }
    else
    {
        Glib_Chars("HDG", NFD_FCUX + 446, NFD_FCUY + 155);
        Glib_Chars("HDG", NFD_FCUX + 506, NFD_FCUY + 140);
        Glib_Chars("V/S", NFD_FCUX + 555, NFD_FCUY + 140);
        Glib_Chars("V/S", NFD_FCUX + 690, NFD_FCUY + 155);
    }

    Glib_LineWidth(2.0);

    SetFCU();
}

/* ------------------------------------------------------ */
static void GetFCU()
{
    PushSwitchList[SwFD]    = FCU_FD;
    PushSwitchList[SwLS]    = FCU_LS;
    PushSwitchList[SwLOC]   = FCU_LOC;
    PushSwitchList[SwAP1]   = FCU_AP1;
    PushSwitchList[SwAP2]   = FCU_AP2;
    PushSwitchList[SwATHR]  = FCU_ATHR;
    PushSwitchList[SwEXPED] = FCU_EXPED;
    PushSwitchList[SwAPPR]  = FCU_APPR;
    NavSwitch1              = FCU_NavSwitch1;
    NavSwitch2              = FCU_NavSwitch2;
    NavSelector             = FCU_ModeSelector;
}

/* ------------------------------------------------------ */
static void SetFCU()
{
    unsigned int RangeValues[6] = { 10, 20, 40, 80, 160, 320 };

    FCU_FD            = PushSwitchList[SwFD];
    FCU_LS            = PushSwitchList[SwLS];
    FCU_LOC           = PushSwitchList[SwLOC];
    FCU_AP1           = PushSwitchList[SwAP1];
    FCU_AP2           = PushSwitchList[SwAP2];
    FCU_ATHR          = PushSwitchList[SwATHR];
    FCU_EXPED         = PushSwitchList[SwEXPED];
    FCU_APPR          = PushSwitchList[SwAPPR];
    FCU_NavSwitch1    = NavSwitch1;
    FCU_NavSwitch2    = NavSwitch2;
    FCU_ModeSelector  = NavSelector;
    FCU_RangeSelector = RangeValues[RangeSelector];
}

/* ------------------------------------------------------ */
/*
   static void LCDChars(char Str[], int x, int y)
   {
   unsigned int p;
   char Ch;

   p = 0;
   while (1) {
    Ch = Str[p];
    if (Ch == 0) {
      break;
    } else {
      LCDChar(Ch, x, y);
      x = x + 12;
      p = p + 1;
    }
   }
   }
 */

/* ------------------------------------------------------ */
static void LCDChar(char Ch, int x, int y)
{
    unsigned int Segments[10] = { 0x3F, 0x30, 0x5B, 0x79, 0x74, 0x6D, 0x6F, 0x38, 0x7f, 0x7D };
    unsigned int bits[8]      = { 1, 2, 4, 8, 16, 32, 64, 128 };
    unsigned int p;
    unsigned int i;
    unsigned int s;

    if (Ch == '-')
    {
        Glib_Draw(x + 2, y + 7, x + 6, y + 7);
        Glib_Draw(x + 1, y + 8, x + 7, y + 8);
    }
    else if (Ch == '.')
    {
        Glib_Draw(x + 4, y, x + 5, y);
        Glib_Draw(x + 4, y + 1, x + 5, y + 1);
    }
    else
    {
        p = Ch - '0';
        s = Segments[p];
        for (i = 0; i <= 6; i += 1)
        {
            if (bits[i] & s)
            {
                switch (i)
                {
                    case 0:
                        Glib_Draw(x + 1, y, x + 7, y);
                        Glib_Draw(x + 2, y + 1, x + 6, y + 1);
                        break;
                    case 1:
                        Glib_Draw(x, y + 1, x, y + 7);
                        Glib_Draw(x + 1, y + 2, x + 1, y + 6);
                        break;
                    case 2:
                        Glib_Draw(x, y + 9, x, y + 15);
                        Glib_Draw(x + 1, y + 10, x + 1, y + 14);
                        break;
                    case 3:
                        Glib_Draw(x + 2, y + 15, x + 6, y + 15);
                        Glib_Draw(x + 1, y + 16, x + 7, y + 16);
                        break;
                    case 4:
                        Glib_Draw(x + 7, y + 10, x + 7, y + 14);
                        Glib_Draw(x + 8, y + 9, x + 8, y + 15);
                        break;
                    case 5:
                        Glib_Draw(x + 7, y + 2, x + 7, y + 6);
                        Glib_Draw(x + 8, y + 1, x + 8, y + 7);
                        break;
                    case 6:
                        Glib_Draw(x + 2, y + 7, x + 6, y + 7);
                        Glib_Draw(x + 1, y + 8, x + 7, y + 8);
                        break;
                }
            }
        }
    }
}

/* ------------------------------------------------------ */
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

/* ------------------------------------------------------ */
static void DisplayPanel(int n, int f)
{
    int          x;
    int          y;
    unsigned int ndigits = 0;
    bool         MinusSign;
    unsigned int v;

    x = NFD_FCUX;
    y = NFD_FCUY + 129;
    switch (n)
    {
        case 2:
            x       = x + 336 - 2;
            ndigits = 3;
            break;
        case 3:
            x       = x + 436 - 2;
            ndigits = 3;
            break;
        case 4:
            x       = x + 550 + 28;
            ndigits = 5;
            break;
        case 5:
            x       = x + 640 + 30;
            ndigits = 4;
            break;
    }
    MinusSign = f < 0;
    if (n == 5)
    {
        if (MinusSign)
        {
            f = -f;
        }
    }
    v = (unsigned int) f;
    Glib_Colour(Glib_WHITE);
    DisplayLCDPanel(x, y, n, v, ndigits, MinusSign);
}

/* ------------------------------------------------------ */
static void DisplayLCDPanel(int x, int y, unsigned int n, unsigned int f,
                            unsigned int ndigits, bool MinusSign)
{
    char         str[11];
    unsigned int i;

    if (MinusSign)
    {
        LCDChar('-', x, y);
    }
    x = x + 12;

    for (i = 1; i <= ndigits; i += 1)
    {
        str[ndigits - i] = f % 10 + '0';
        f                = f / 10;
    }

    if (n == 5 && FCU_HDG_TRK_Button)
    {
        str[ndigits - 1] = str[ndigits - 2];
        str[ndigits - 2] = '.';
    }

    for (i = 0; i <= ndigits - 1; i += 1)
    {
        LCDChar(str[i], x, y);
        x = x + 12;
    }
}

/* ------------------------------------------------------ */
void FCU_RestoreFCU(IosDefn_RestoreVectorRecord v)
{
    FCU_FD              = v.FCU_FD;
    FCU_LS              = v.FCU_LS;
    FCU_LOC             = v.FCU_LOC;
    FCU_AP1             = v.FCU_AP1;
    FCU_AP2             = v.FCU_AP2;
    FCU_ATHR            = v.FCU_ATHR;
    FCU_EXPED           = v.FCU_EXPED;
    FCU_APPR            = v.FCU_APPR;
    FCU_SPD_MACH_Button = v.FCU_SPD_MACH;
    FCU_HDG_TRK_Button  = v.FCU_HDG_TRK;
    FCU_BaroPressure    = (unsigned int) v.FCU_BaroPressure;

    /* FCU_FCURange        = (unsigned int) v.FCURange; */
    FCU_HDG             = (unsigned int) v.FCU_HDG;
    FCU_ALT             = (unsigned int) v.FCU_ALT;
    FCU_SPD             = (unsigned int) v.FCU_SPD;
    FCU_VS              = (int) v.FCU_VS;
    
	FCU_BaroHg          = v.FCU_BaroHg;
    FCU_BaroSTD        = v.FCU_BaroSTD;
    FCU_HDG_Hold        = v.FCU_HDG_Hold;
    FCU_ALT_Hold        = v.FCU_ALT_Hold;
    FCU_SPD_Hold        = v.FCU_SPD_Hold;
    FCU_VS_Hold         = v.FCU_VS_Hold;
    FCU_NavSwitch1      = v.NavAid1;
    FCU_NavSwitch2      = v.NavAid2;
    FCU_ModeSelector    = v.Mode;
    FCU_DataMode        = v.DataMode;
    GetFCU();
}

/* ------------------------------------------------------ */
void BEGIN_FCU()
{
    for (i = 0; i <= 12; i += 1)
    {
        PushSwitchList[i] = false;
    }
    for (i = 0; i <= 4; i += 1)
    {
        KnobAngle[i] = 0.0;
    }
    NavSwitch1          = NavDefn_NavOFF;
    NavSwitch2          = NavDefn_NavOFF;
    NavSelector         = NavDefn_ModeILS;
    RangeSelector       = 0;
    PanelMode           = 0;
    FCU_BaroPressure    = 1013;
    FCU_BaroHg          = false;
    OldBaroHg           = false;
    FCU_BaroSTD         = 0;
    MetricMode          = false;
    FCU_DataMode        = 1;
    FCU_RangeSelector   = 0;
    FCU_SPD             = 0;
    FCU_HDG             = 360;
    FCU_ALT             = 0;
    FCU_VS              = 0;
    CurrentKnob         = 0;
    CurrentSwitch       = 0;
    CurrentSelector     = 0;
    FCU_NavSwitch1      = NavDefn_NavOFF;
    FCU_NavSwitch2      = NavDefn_NavOFF;
    FCU_BaroSTD         = 0;
    FCU_SPD_Hold        = false;
    FCU_HDG_Hold        = false;
    FCU_ALT_Hold        = false;
    FCU_VS_Hold         = false;
    FCU_SPD_MACH_Button = true;  /* Kts */
    FCU_HDG_TRK_Button  = false;
    FCU_ModeSelector    = NavDefn_ModeILS;
    oldleftb            = false;
    oldmiddleb          = false;
    oldrightb           = false;
    ticks               = 0;
}
