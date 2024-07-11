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

#define Nav1X           NFD_FCUX + 162  /* NAV1 selector */
#define Nav1Y           NFD_FCUY + 44
#define Nav2X           NFD_FCUX + 253  /* NAV2 selector */
#define Nav2Y           NFD_FCUY + 44
#define NavSelectorX    NFD_FCUX + 162  /* MAV mode selector */
#define NavSelectorY    NFD_FCUY + 93
#define RangeSelectorX  NFD_FCUX + 253  /* range selector */
#define RangeSelectorY  NFD_FCUY + 93

IODefn_SwitchPosition GetAPSwitch();
IODefn_SwitchPosition GetATHRSwitch();

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
}

/* ------------------------------------------------------ */
void FCU_RestoreFCU(IosDefn_RestoreVectorRecord v)
{
}

/* ------------------------------------------------------ */
void BEGIN_FCU()
{
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition GetAPSwitch()
{
    return IODefn_Off;
}

/* ---------------------------------------------------- */    
IODefn_SwitchPosition GetATHRSwitch()
{
    return IODefn_Off;
}

