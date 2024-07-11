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

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/maths.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/glib.h>

#include "navlink.h"
#include "panellib.h"
#include "nfd.h"
#include "fcu.h"

#define ONERAD (180.0 / Maths_PI)
#define DEG30  (30.0 / ONERAD)
#define InHg   0
#define hPa    1

#define BIT0     0x01
#define BIT1     0x02
#define BIT2     0x04
#define BIT3     0x08
#define BIT4     0x10
#define BIT5     0x20
#define BIT6     0x40
#define BIT7     0x80

bool             FCU_FD;
bool             FCU_LS;
bool             FCU_LOC;
bool             FCU_AP1;
bool             FCU_AP2;
bool             FCU_ATHR;
bool             FCU_EXPED;
bool             FCU_APPR;
bool             FCU_SPD_MACH_Button;
unsigned int     FCU_BaroPressure;
bool             FCU_BaroHg;
bool             FCU_BaroSTD;
bool             FCU_HDG_TRK_Button;
unsigned int     FCU_MACH;
unsigned int     FCU_HDG;
unsigned int     FCU_TRK;
unsigned int     FCU_ALT;
unsigned int     FCU_SPD;
int              FCU_VS;
int              FCU_FPA;

NavDefn_FCUKnob  FCU_BaroKnob;
NavDefn_FCUKnob  FCU_HDGKnob;
NavDefn_FCUKnob  FCU_ALTKnob;
NavDefn_FCUKnob  FCU_SPDKnob;
NavDefn_FCUKnob  FCU_VSKnob;

unsigned int     Panel_HDG;
unsigned int     Panel_ALT;
unsigned int     Panel_SPD;
int              Panel_VS;
 
unsigned int     FCU_ALT_Range;
bool             FCU_Metric_Button;

NavDefn_FCUNav   FCU_NavSwitch1;
NavDefn_FCUNav   FCU_NavSwitch2;
NavDefn_FCUMode  FCU_ModeSelector;
NavDefn_FCUData  FCU_DataMode;
unsigned int     FCU_RangeSelector;

bool             FCU_CSTR_Button;
bool             FCU_WPT_Button;
bool             FCU_VORD_Button;
bool             FCU_NDB_Button;
bool             FCU_ARPT_Button;

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

#define SwFD     0
#define SwLS     1
#define SwCSTR   2
#define SwWPT    3
#define SwVORD   4
#define SwNDB    5
#define SwARPT   6
#define SwLOC    7
#define SwAP1    8
#define SwAP2    9
#define SwATHR   10
#define SwEXPED  11
#define SwAPPR   12
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

#define Nav1X           NFD_FCUX + 162  /* NAV1 selector */
#define Nav1Y           NFD_FCUY + 44
#define Nav2X           NFD_FCUX + 253  /* NAV2 selector */
#define Nav2Y           NFD_FCUY + 44
#define NavSelectorX    NFD_FCUX + 162  /* MAV mode selector */
#define NavSelectorY    NFD_FCUY + 93
#define RangeSelectorX  NFD_FCUX + 253  /* range selector */
#define RangeSelectorY  NFD_FCUY + 93

static bool             PushSwitchList[13];
static float            KnobAngle[5];

static NavDefn_FCUNav   NavSwitch1;
static NavDefn_FCUNav   NavSwitch2;
static NavDefn_FCUMode  NavSelector;
static int              RangeSelector;

static unsigned int     CurrentKnob;
static unsigned int     CurrentSwitch;
static unsigned int     CurrentSelector;
static unsigned int     PanelMode;
static bool             MetricMode;

static unsigned int     i;
static int              oldleftb;
static int              oldmiddleb;
static int              oldrightb;
static unsigned int     ticks;
static unsigned int     ticklimit;
static bool             OldBaroHg;
static bool             Alt000;   /* Altitude count in 000 */

static void DisplayBaroPanel(unsigned int p, NavDefn_FCUKnob knob);
static bool InsideCircle(int x, int y, int x0, int y0, int r);
static bool InsideBox(int x, int y, int x0, int y0, int xs, int ys);
static void GetFCU();
static void SetFCU();

static void LED(int x, int y);
static void DisplayPanel(int n, int f);
static void DisplayLCDPanel(int x, int y, int f, unsigned int ndigits, bool sign);

IODefn_SwitchPosition GetAPSwitch();
IODefn_SwitchPosition GetATHRSwitch();

/* ------------------------------------------------------ */
static void DisplayBaroPanel(unsigned int p, NavDefn_FCUKnob knob)
{
    Glib_Colour(Glib_WHITE);
    if (knob == NavDefn_Pulled)
    {
        Glib_SetFont(Glib_EFONT16, 10);
        Glib_Chars("STD", 43, 142);
    }
    else
    {
        Glib_SetFont(Glib_EFONT8, 0);
        Glib_Chars("QNH", 60, 150);
        DisplayLCDPanel(28, 119, p, 4, false);
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
    
    if (GetAPSwitch() == IODefn_Off)
    {
        FCU_AP1 = false;
        FCU_AP2 = false;
        PushSwitchList[SwAP1] = false;
        PushSwitchList[SwAP2] = false;
    }
    
    if (GetATHRSwitch() == IODefn_Off)
    {
        FCU_ATHR = false;
        PushSwitchList[SwATHR] = false;
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
    
    if (leftb && !oldleftb)
    {
        if (InsideCircle(x, y, Nav1X, Nav1Y, 30))
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
        else if (InsideCircle(x, y, Nav2X, Nav2Y, 30))
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
        else if (InsideCircle(x, y, NavSelectorX, NavSelectorY, 40))
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
        else if (InsideCircle(x, y, RangeSelectorX, RangeSelectorY, 40))
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
            FCU_Metric_Button = !FCU_Metric_Button;
        }
        else if (InsideCircle(x, y, NFD_FCUX + 40, NFD_FCUY + 91, 10))
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
        else if (InsideCircle(x, y, NFD_FCUX + 623 - 13, NFD_FCUY + 83 + 22, 10))
        {
            Alt000 = false;
        }
        else if (InsideCircle(x, y, NFD_FCUX + 623 + 13, NFD_FCUY + 83 + 22, 10))
        {
            Alt000 = true;
        }
        else
        {
            for (i = 0; i <= 4; i += 1)
            {
                if ((x > KnobCoords[i].xc - 10) && (x < KnobCoords[i].xc + 10) &&  /* knob pushed in */
                    (y > KnobCoords[i].yc + 10) && (y < KnobCoords[i].yc + 35))
                {
                    switch (i)
                    {
                        case 0:
                            FCU_BaroKnob = NavDefn_Pushed;
                            break;
                        case 1:
						    if (FCU_SPDKnob == NavDefn_Pulled)
							{
							    FCU_SPDKnob = NavDefn_Middle;
							}
							else if (FCU_SPDKnob == NavDefn_Middle)
							{
							    FCU_SPDKnob = NavDefn_Pushed;
							}
                            break;
                        case 2:
						    if (FCU_HDGKnob == NavDefn_Pulled)
							{
							    FCU_HDGKnob = NavDefn_Middle;
							}
							else if (FCU_HDGKnob == NavDefn_Middle)
							{
							    FCU_HDGKnob = NavDefn_Pushed;
							}
                            break;
                        case 3:
						    if (FCU_ALTKnob == NavDefn_Pulled)
							{
							    FCU_ALTKnob = NavDefn_Middle;
							}
							else if (FCU_ALTKnob == NavDefn_Middle)
							{
							    FCU_ALTKnob = NavDefn_Pushed;
							}
                            break;
                        case 4:
						    if (FCU_VSKnob == NavDefn_Pulled)
							{
							    FCU_VSKnob = NavDefn_Middle;
							}
                            else if (FCU_VSKnob == NavDefn_Middle)
                            {
							    FCU_VSKnob = NavDefn_Pushed;
								/* engage level off *** */
                            }
                            break;
                    }
                }
            }
            for (i = 0; i <= 4; i += 1)
            {
                if ((x > KnobCoords[i].xc - 10) && (x < KnobCoords[i].xc + 10) &&  /* knob pulled out */
                    (y > KnobCoords[i].yc - 35) && (y < KnobCoords[i].yc - 10))
                {
                    switch (i)
                    {
                        case 0:
                            FCU_BaroKnob = NavDefn_Pulled;
                            if (FCU_BaroHg)
                            {
                                FCU_BaroPressure = 2992;
                            }
                            else
                            {
                                FCU_BaroPressure = 1013;
                            }
                            break;
                        case 1:
						    if (FCU_SPDKnob == NavDefn_Pushed)
							{
							    FCU_SPDKnob = NavDefn_Middle;
							}
							else if (FCU_SPDKnob == NavDefn_Middle)
							{
							    FCU_SPDKnob = NavDefn_Pulled;
								FCU_SPD = Panel_SPD;
							}
                            break;
                        case 2:
						    if (FCU_HDGKnob == NavDefn_Pushed)
							{
							    FCU_HDGKnob = NavDefn_Middle;
							}
							else if (FCU_HDGKnob == NavDefn_Middle)
							{
							    FCU_HDGKnob = NavDefn_Pulled;
								FCU_HDG = Panel_HDG;
							}
                            break;
                        case 3:
						    if (FCU_ALTKnob == NavDefn_Pushed)
							{
							    FCU_ALTKnob = NavDefn_Middle;
							}
							else if (FCU_ALTKnob == NavDefn_Middle)
							{
                                FCU_ALTKnob = NavDefn_Pulled;
								FCU_ALT = Panel_ALT;
                            }
                            break;
                        case 4:
                            if (FCU_VSKnob == NavDefn_Pushed)
							{
							    FCU_VSKnob = NavDefn_Middle;
							}
							else if (FCU_VSKnob == NavDefn_Middle)
							{
							    FCU_VSKnob = NavDefn_Pulled;
								FCU_VS = Panel_VS;
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
            if ((x > KnobCoords[i].xc - 35) && (x < KnobCoords[i].xc - 10) &&
                (y > KnobCoords[i].yc - 10) && (y <  KnobCoords[i].yc + 10))
            {
                KnobAngle[i] += 1.0;
                if (KnobAngle[i] > 360.0)
                {
                    KnobAngle[i] -= 360.0;
                }
                
                if (ticks % ticklimit == 0)
                {
                    switch (i)
                    {
                        case 0:
                            if (FCU_BaroKnob == NavDefn_Pushed)
                            {
                                FCU_BaroPressure -= 1;
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
                            }
                            break;
                        case 1:
                            if (Panel_SPD > 0)
                            {
                                Panel_SPD -= 1;
                            }
							if (FCU_SPDKnob == NavDefn_Pulled)
							{
							    FCU_SPD = Panel_SPD;
							}
                            break;
                        case 2:
                            if (Panel_HDG == 1)
                            {
                                Panel_HDG = 360;
                            }
                            else
                            {
                                Panel_HDG -= 1;
                            }
							if (FCU_HDGKnob == NavDefn_Pulled)
							{
							    FCU_HDG = Panel_HDG;
							}
                            break;
                        case 3:
						    if (Alt000)
							{
                                if (Panel_ALT >= 1000)
                                {
                                    Panel_ALT -= 1000;
                                }
							}
							else
							{
                                if (Panel_ALT >= 100)
                                {
                                    Panel_ALT -= 100;
                                }
							}
							if (FCU_ALTKnob == NavDefn_Pulled)
							{
							    FCU_ALT = Panel_ALT;
							}
                            break;
                        case 4:
                            Panel_VS -= 100;
                            if (Panel_VS < -2000)
                            {
                                Panel_VS = -2000;
                            }
							if (FCU_VSKnob == NavDefn_Pulled)
							{
							    FCU_VS = Panel_VS;
							}
                            break;
                    }
                }
            }
            else if ((x > KnobCoords[i].xc + 10) && (x < KnobCoords[i].xc + 35) &&
                     (y > KnobCoords[i].yc - 10) && (y <  KnobCoords[i].yc + 10))

            {
                KnobAngle[i] -= 1.0;
                if (KnobAngle[i] < 0.0)
                {
                    KnobAngle[i] += 360.0;
                }
                if (ticks % ticklimit == 0)
                {
                    switch (i)
                    {
                        case 0:
                            if (FCU_BaroKnob == NavDefn_Pushed)
                            {
                                FCU_BaroPressure += 1;
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
                            }
                            break;
                        case 1:
                            Panel_SPD += 1;
                            if (Panel_SPD > 400)
                            {
                                Panel_SPD = 400;
                            }
							if (FCU_SPDKnob == NavDefn_Pulled)
							{
							    FCU_SPD = Panel_SPD;
							}
                            break;
                        case 2:
                            Panel_HDG += 1;
                            if (Panel_HDG > 360)
                            {
                                Panel_HDG = 1;
                            }
							if (FCU_HDGKnob == NavDefn_Pulled)
							{
							    FCU_HDG = Panel_HDG;
							}
                            break;
                        case 3:
						    if (Alt000)
							{
                                Panel_ALT += 1000;
                                if (Panel_ALT > 40000)
                                {
                                    Panel_ALT = 40000;
                                }
							}
							else
							{
                                Panel_ALT += 100;
                                if (Panel_ALT > 40000)
                                {
                                    Panel_ALT = 40000;
                                }
							}
							if (FCU_ALTKnob == NavDefn_Pulled)
							{
							    FCU_ALT = Panel_ALT;
							}
                            break;
                        case 4:
                            Panel_VS += 100;
                            if (Panel_VS > 2000)
                            {
                                Panel_VS = 2000;
                            }
							if (FCU_VSKnob == NavDefn_Pulled)
							{
							    FCU_VS = Panel_VS;
							}
                            break;
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
                        FCU_BaroHg,
						Alt000);

    DisplayBaroPanel(FCU_BaroPressure, FCU_BaroKnob);
    DisplayPanel(2, Panel_SPD);
    DisplayPanel(3, Panel_HDG);
    DisplayPanel(4, Panel_ALT);
    DisplayPanel(5, FCU_VS);

    Glib_Colour(Glib_GREEN);
    if (FCU_SPDKnob == NavDefn_Pushed)
    {
        LED(393, 130);
    }
    if (FCU_HDGKnob == NavDefn_Pushed)
    {
        LED(492, 130);
    }
    if (FCU_ALTKnob == NavDefn_Pushed)
    {
        LED(655, 130);
    }

    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(1.0);
    Glib_SetFont(Glib_EFONT8, 7);

    Glib_Chars("ALT", 610, 155);
    Glib_Chars("LVL/CH", 650, 155);
    Glib_Draw(635, 155, 635, 160);
    Glib_Draw(635, 160, 645, 160);
    Glib_Draw(695, 160, 705, 160);
    Glib_Draw(705, 160, 705, 155);
    
    if (FCU_SPD_MACH_Button)
    {
        Glib_Chars("SPD", 344, 155);
    }
    else
    {
        Glib_Chars("MACH", 344, 155);
    }
    if (FCU_HDG_TRK_Button)
    {
        Glib_Chars("TRK", 446, 155);
        Glib_Chars("TRK", 506, 140);
        Glib_Chars("FPA", 555, 140);
        Glib_Chars("FPA", 710, 155);
    }
    else
    {
        Glib_Chars("HDG", 446, 155);
        Glib_Chars("HDG", 506, 140);
        Glib_Chars("V/S", 555, 140);
        Glib_Chars("V/S", 710, 155);
    }

    Glib_LineWidth(2.0);

    SetFCU();
}

/* ------------------------------------------------------ */
static void GetFCU()
{
    if (FCU_APPR)
    {
        FCU_HDG = false;
        FCU_ALT = false;
        FCU_VS = false;
    }
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
static void LED(int x, int y)
{
    float tx1 = PanelTextures[8].tex_umin;
    float ty1 = PanelTextures[8].tex_vmin;
    float tx2 = PanelTextures[8].tex_umax;
    float ty2 = PanelTextures[8].tex_vmax;

    Glib_DrawTextureRotated(x+4, y+5, 16, 16, tx1, ty1, tx2, ty2, 0.0, 1.0);
    return; 
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
    int x = 0;
    int y = 129;

    Glib_Colour(Glib_WHITE);

    switch (n)
    {
        case 2:  /* SPD */
            x += 346 - 2;
            if (FCU_SPDKnob == NavDefn_Pushed)
			{
			    Glib_Chars("---", x, y);
			}
			else
			{
                DisplayLCDPanel(x, y, f, 3, false);
            }
			break;
        case 3:  /* HDG */
            x += 445 - 2;
            if (FCU_HDGKnob == NavDefn_Pushed)
			{
			    Glib_Chars("---", x, y);
			}
			else
			{
                DisplayLCDPanel(x, y, f, 3, false);
            }
			break;
        case 4:  /* ALT */
            x += 550 + 28;
            DisplayLCDPanel(x, y, f, 5, false);
            break;
        case 5:  /* VS */
            x += 635 + 45;
            DisplayLCDPanel(x, y, f, 4, true);
            break;
    }
}

/* ------------------------------------------------------ */
static void DisplayLCDPanel(int x, int y, int f, unsigned int ndigits, bool sign)
{
    char str[20];
    char width[] ="%0xd";
    
    width[2] = (char) ndigits + '0';
    Glib_SetFont(Glib_LFONT20, 10);
    if (sign)
    {
        if (f >= 0)
        {
	        Glib_Draw(x - 9, y + 5, x - 9, y + 13);
        }
    	Glib_Draw(x - 13, y + 9, x - 5, y + 9);
	}
	sprintf(str, width, abs(f));
    Glib_Chars(str, x, y);
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
    Panel_HDG           = (unsigned int) v.FCU_HDG;
    Panel_ALT           = (unsigned int) v.FCU_ALT;
    Panel_SPD           = (unsigned int) v.FCU_SPD;
    Panel_VS            = (int) v.FCU_VS;
    
    FCU_BaroHg          = v.FCU_BaroHg;
    FCU_BaroKnob        = v.FCU_BaroKnob;
    FCU_HDGKnob         = v.FCU_HDGKnob;
    FCU_ALTKnob         = v.FCU_ALTKnob;
    FCU_SPDKnob         = v.FCU_SPDKnob;
    FCU_VSKnob          = v.FCU_VSKnob;
    FCU_HDG             = v.FCU_HDG;
    FCU_ALT             = v.FCU_ALT;
    FCU_SPD             = v.FCU_SPD;
    FCU_VS              = v.FCU_VS;
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
    FCU_BaroKnob        = NavDefn_Pulled;
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
    FCU_SPDKnob         = NavDefn_Middle;
    FCU_HDGKnob         = NavDefn_Middle;
    FCU_ALTKnob         = NavDefn_Middle;
    FCU_VSKnob          = NavDefn_Middle;
    Panel_SPD           = 0;
    Panel_HDG           = 360;
    Panel_ALT           = 0;
    FCU_SPD_MACH_Button = true;  /* Kts */
    FCU_HDG_TRK_Button  = false;
    FCU_ModeSelector    = NavDefn_ModeILS;
    oldleftb            = false;
    oldmiddleb          = false;
    oldrightb           = false;
    ticks               = 0;
	Alt000              = false;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition GetAPSwitch()
{
    if ((NavLink_IOPkt1.DigitalDataA & BIT5) == 0)
    {
        return IODefn_On;
    }
    else
    {
        return IODefn_Off;
    }
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition GetATHRSwitch()
{
    if ((NavLink_IOPkt1.DigitalDataB & BIT1) == 0)
    {
        return IODefn_On;
    }
    else
    {
        return IODefn_Off;
    }
}

