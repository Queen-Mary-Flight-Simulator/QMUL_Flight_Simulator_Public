/* +------------------------------+---------------------------------+
   | Module      : compass.c      | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-02      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EFIS Compass Display              |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <SIM/glib.h>
#include <SIM/maths.h>
#include <SIM/navdefn.h>

#include "nfd.h"
#include "nav.h"
#include "fcu.h"
#include "nfd-compass.h"

#define ToFlag        0
#define FromFlag      1
#define OffFlag       2

#define ONERAD        (180.0f / M_PI)
#define DEG5          (5.0f / ONERAD)
#define DEG85         (85.0f / ONERAD)
#define DEG95         (95.0f / ONERAD)
#define TWOPI         (M_PI * 2.0f)

typedef unsigned char ToFromFlag;

static int   OldIntCdi;
static int   OldGlideslope;
static float OldRmi1;
static float OldRmi2;

static void PlanCircle(unsigned int radius);
static void WritePlanRange(unsigned int n, int x, int y);
static void CompassCard(float Hdg);
static void WrHdg(int hdg, int x, int y);
static void ExpandedModeBackground(NavDefn_FCUMode Mode);
static void CenteredModeBackground(NavDefn_FCUMode Mode);
static void Mark(int inc, int a1, int a2, int r1, int r2);
static void HeadingBug(float Bug);
static void ExpandedHeadingBug(float Bug);
static void WriteRange(unsigned int Range, int y);
static unsigned int Norm360(int a);
static void Circle5(int x, int y);

static int Circle5Tab[38] = 
{
     5,  0,  5,  2,  4,  3,  3,  4, 1,  5, -1,  5, -3,  4, -4,  3, -5,  2,
    -5,  0, -5, -2, -4, -3, -3, -4, -1, -5, 1, -5,  3, -4,  4, -3,  5, -2
};

/* ------------------------------------------------------------ */
void Compass_Rmi(int CompassX, int CompassY, float Rmi1, NavDefn_FCUNav n1, float Rmi2, NavDefn_FCUNav n2)
{
    int Rmi1FrontPtr[16] = 
    {
         -9,  173,    9,  173,    0,  173,    0,  215,    0,  215, -5,  210,
          0,  215,    5,  210
    };

    int Rmi1BackPtr[12] = 
    {
          0, -210,    0, -175,    0, -202,   -9, -206,    0, -202,  9, -206
    };

    int Rmi2FrontPtr[18] = 
    {
         -11,  173,  -11,  177,   -4,  177,   -4,  211,    0,   215,
           4,  211,    4,  177,   11,  177,   11,  173
    };

    int Rmi2BackPtr[28] = 
    {
          0,  -200,    0, -207,   -9, -211,   -9, -204,    0,  -200,
          4,  -202,    4, -177,    0, -173,   -4, -177,   -4,  -202,
          0,  -200,    9, -204,    9, -211,    0, -207
    };

    bool         Rmi1OK;
    bool         Rmi2OK;
    unsigned int Col1 = Glib_BLACK;
    unsigned int Col2 = Glib_BLACK;

    Glib_LoadIdentity();
    Glib_Translate((float) CompassX, (float) CompassY);
    
    while (OldRmi1 > Rmi1 + M_PI)
    {
        OldRmi1 = OldRmi1 - TWOPI;
    }
    while (OldRmi1 < Rmi1 - M_PI)
    {
        OldRmi1 = OldRmi1 + TWOPI;
    }
	
    if (n1 == NavDefn_NavVOR)
    {
        Col1   = Glib_GREEN;
        Rmi1OK = Nav_VOR1.BeaconStatus;
    }
    else if (n1 == NavDefn_NavADF)
    {
        Col1   = Glib_CYAN;
        Rmi1OK = Nav_ADF1.BeaconStatus;
    }
    else
    {
        Rmi1OK = false;
    }
	
    if (Rmi1OK && (n1 != NavDefn_NavOFF))
    {
        Glib_Colour(Col1);
        Glib_PushMatrix();
        Glib_Rotate(Maths_Degrees(-Rmi1));
        Glib_DrawLines(0, 0, 4, Rmi1FrontPtr);
        Glib_DrawLines(0, 0, 3, Rmi1BackPtr);
        Glib_PopMatrix();
    }
    while (OldRmi2 > Rmi2 + M_PI)
    {
        OldRmi2 = OldRmi2 - TWOPI;
    }
    while (OldRmi2 < Rmi2 - M_PI)
    {
        OldRmi2 = OldRmi2 + TWOPI;
    }
	
    if (n2 == NavDefn_NavVOR)
    {
        Col2   = Glib_GREEN;
        Rmi2OK = Nav_VOR2.BeaconStatus;
//printf("Nav_VOR2.BeaconStatus=%c\n", (Nav_VOR2.BeaconStatus) ? 'T' : 'F'); // ***
    }
    else if (n2 == NavDefn_NavADF)
    {
        Col2   = Glib_CYAN;
        Rmi2OK = Nav_ADF2.BeaconStatus;
//printf("Nav_ADF.BeaconStatus=%c\n", (Nav_ADF2.BeaconStatus) ? 'T' : 'F'); // ***
    }
    else
    {
        Rmi2OK = false;
    }
	
    if (Rmi2OK && (n2 != NavDefn_NavOFF))
    {
        Glib_Colour(Col2);
        Glib_PushMatrix();
        Glib_Rotate(Maths_Degrees(-Rmi2));
        Glib_Line_Loop(0, 0, 8, Rmi2FrontPtr);
        Glib_Line_Loop(0, 0, 13, Rmi2BackPtr);
        Glib_PopMatrix();
    }
//printf("Rmi1=%f n1=%d Rmi2=%f n2=%d Rmi1OK=%c Rmi2OK=%c\n", Maths_Degrees(Rmi1), (int) n1, Maths_Degrees(Rmi2), (int) n2, (Rmi1OK) ? 'T' : 'F', (Rmi2OK) ? 'T' : 'F'); // *** 
}

/* ------------------------------------------------------------ */
void Compass_ExpandedRmi(int CompassX, int CompassY, float Rmi1, NavDefn_FCUNav n1, float Rmi2,
                         NavDefn_FCUNav n2)
{
    int Rmi1FrontPtr[16] = 
    {
         -9,  360,    9,  360,    0,  360,    0,  416,    0,  416, -5,  411,
          0,  416,    5,  411
    };

    int Rmi1BackPtr[12] = 
    {
          0, -418,    0, -360,    0, -409,   -9, -418,    0, -409,  9, -418
    };

    int Rmi2FrontPtr[18] = 
    {
        -11,  358,  -11,  363,   -5,  363,   -5,  404,    0,  409,
          5,  404,    5,  363,   11,  363,   11,  358
    };

    int Rmi2BackPtr[28] = 
    {
          0, -400,    0, -410,  -10, -415,  -10, -405,    0, -400,
          5, -402,    5, -363,    0, -358,   -5, -363,   -5, -402,
          0, -400,   10, -405,   10, -415,    0, -410 
    };

    bool         Rmi1OK;
    bool         Rmi2OK;
    unsigned int Col1 = Glib_BLACK;
    unsigned int Col2 = Glib_BLACK;

    Glib_LoadIdentity();
    Glib_Translate((float) CompassX, (float) CompassY);
    
    while (OldRmi1 > Rmi1 + M_PI)
    {
        OldRmi1 = OldRmi1 - TWOPI;
    }
    while (OldRmi1 < Rmi1 - M_PI)
    {
        OldRmi1 = OldRmi1 + TWOPI;
    }
	
    if (n1 == NavDefn_NavVOR)
    {
        Col1   = Glib_GREEN;
        Rmi1OK = Nav_VOR1.BeaconStatus;
    }
    else if (n1 == NavDefn_NavADF)
    {
        Col1   = Glib_CYAN;
        Rmi1OK = Nav_ADF1.BeaconStatus;
    }
    else
    {
        Rmi1OK = false;
    }
	
	Glib_LineWidth(2.0); // ***
	
    if (Rmi1OK && n1 != NavDefn_NavOFF)
    {
        Glib_Colour(Col1);
        Glib_PushMatrix();
        Glib_Rotate(Maths_Degrees(-Rmi1));
        Glib_DrawLines(0, 0, 4, Rmi1FrontPtr);
        Glib_DrawLines(0, 0, 3, Rmi1BackPtr);
        Glib_PopMatrix();
    }

    while (OldRmi2 > Rmi2 + M_PI)
    {
        OldRmi2 = OldRmi2 - TWOPI;
    }
    while (OldRmi2 < Rmi2 - M_PI)
    {
        OldRmi2 = OldRmi2 + TWOPI;
    }
	
    if (n2 == NavDefn_NavVOR)
    {
        Col2   = Glib_GREEN;
        Rmi2OK = Nav_VOR2.BeaconStatus;
    }
    else if (n2 == NavDefn_NavADF)
    {
        Col2   = Glib_CYAN;
        Rmi2OK = Nav_ADF2.BeaconStatus;
    }
    else
    {
        Rmi2OK = false;
    }
	
    if (Rmi2OK && n2 != NavDefn_NavOFF)
    {
        Glib_Colour(Col2);
        Glib_PushMatrix();
        Glib_Rotate(Maths_Degrees(-Rmi2));
        Glib_Line_Loop(0, 0, 8, Rmi2FrontPtr);
        Glib_Line_Loop(0, 0, 13, Rmi2BackPtr);
        Glib_PopMatrix();
    }
}

/* ------------------------------------------------------------ */
void Compass_GlideSlope(int CompassX, int CompassY, float GlideSlopeError, bool ILSMode, bool Selected)
{
    int h;

    Glib_LoadIdentity();
    Glib_Translate((float) CompassX, (float) CompassY);
    
    h = intround(GlideSlopeError * 7039.2);
    
	if (h > OldGlideslope)
    {
        h = OldGlideslope + 1;
    }
    else if (h < OldGlideslope)
    {
        h = OldGlideslope - 1;
    }

    if (h < -108)
    {
        h = -108;
    }
    else if (h > 108)
    {
        h = 108;
    }

    if (Selected)
    {
        Glib_Colour(Glib_MAGENTA);
        Glib_Triangle(259-8, h, 259, h+8, 259, h-8);
        Glib_Triangle(259, h+8, 259+8, h, 259, h-8);
    }
    OldGlideslope = h;
}

/* ------------------------------------------------------------ */
static void ExpandedCompassCard(float Hdg)
{
    int          IntHdg;
    unsigned int h1;
    unsigned int h2;
    unsigned int MinHdg;
    unsigned int MaxHdg;
    float        Offset;
    float        a;
    float        t;

    if (Hdg < 0.0)
    {
        IntHdg = -intround(Maths_Degrees(-Hdg));
    }
    else
    {
        IntHdg = intround(Maths_Degrees(Hdg));
    }
	
    IntHdg = Norm360(IntHdg);
    MinHdg = IntHdg / 10 * 10;
	
    if (MinHdg >= 50)
    {
        MinHdg = MinHdg - 50;
    }
    else
    {
        MinHdg = MinHdg + 310;
    }
    
	MaxHdg = MinHdg + 110;
    Offset = Maths_Rads((float) (MinHdg)) - Hdg;
    Offset = Maths_Normalise(Offset);
    a  = Offset;
    h1 = MinHdg;
    h2 = MaxHdg;
    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    
	do
    {
        if (h1 <= h2)
        {
            t = a;
            Glib_PushMatrix();
            Glib_Rotate(Maths_Degrees(-t));
            Glib_Draw(-18, 400, 18, 400);
            if (h1 % 10 == 0)
            {
                Glib_Draw(0, 379, 0, 400);
                if (h1 % 30 == 0)
                {
                    Glib_SetFont(Glib_EFONT16, 12);
                    if (h1 > 90)
                    {
                        Glib_Char(h1 / 100 + '0', -10, 360);
                        Glib_Char(h1 / 10 % 10 + '0', 2, 360);
                    }
                    else
                    {
                        Glib_Char(h1 / 10 % 10 + '0', -5, 360);
                    }
                }
            }
            else
            {
                Glib_Draw(0, 391, 0, 400);
            }
            Glib_PopMatrix();
            
            a  = a + DEG5;
            h1 = h1 + 5;
        }
    } while (!(h1 > h2));
    WrHdg(IntHdg, -30, 425);
}

/* ------------------------------------------------------------ */
void Compass_ExpandedCompass(int CompassX, int CompassY, float Hdg, float Bearing, float Track, unsigned int CrsPointer,
                             unsigned int HdgBug, unsigned int Range, bool ILSMode,
                             NavDefn_FCUMode Mode)
{
    int CrsFrontPtr[10] = 
    {
        -4,   57,   -4,   75,    0,   85,    4,   75,    4,    57
    };

    int CrsBackPtr[8] = 
    {
        -4,  -57,   -4,  -74,    4,  -74,    4,  -57
    };

    float      Trk;
    int        i;
    float      Crs;
    float      Cdi;
    int        IntCdi;
    float      ScaleFactor;
    int        CdiBar[8];
    ToFromFlag HsiFlag;
    float      Bug;

    Glib_LoadIdentity();
    Glib_Translate((float) CompassX, (float) CompassY);
    
    ExpandedModeBackground(Mode);
    ExpandedCompassCard(Hdg);
    HsiFlag = OffFlag;

    if (Bearing >= -DEG85 && Bearing <= DEG85)
    {
        HsiFlag = ToFlag;
    }
    else if (Bearing < -DEG95)
    {
        Bearing = -M_PI - Bearing;
        HsiFlag = FromFlag;
    }
    else if (Bearing > DEG95)
    {
        Bearing = M_PI - Bearing;
        HsiFlag = FromFlag;
    }
    if (ILSMode)
    {
        ScaleFactor = 1970.975;
        HsiFlag     = OffFlag;
    }
    else
    {
        ScaleFactor = 492.743;
    }

    Cdi = Bearing * ScaleFactor;
    Cdi = Maths_Limit(Cdi, -108.0, 108.0);
    IntCdi = intround(Cdi);
    if (IntCdi > OldIntCdi)
    {
        IntCdi = OldIntCdi + 1;
    }
    else if (IntCdi < OldIntCdi)
    {
        IntCdi = OldIntCdi - 1;
    }
    OldIntCdi = IntCdi;
    
    CdiBar[0] = IntCdi - 4; CdiBar[1] = -55;
    CdiBar[2] = IntCdi - 4; CdiBar[3] = 55;
    CdiBar[4] = IntCdi + 4; CdiBar[5] = 55;
    CdiBar[6] = IntCdi + 4; CdiBar[7] = -55;

    Bug = Maths_Rads((float) HdgBug) - Hdg;
    Bug = Maths_Normalise(Bug);
    ExpandedHeadingBug(Bug);

    Crs = Maths_Rads((float) (CrsPointer)) - Hdg;
    Crs = Maths_Normalise(Crs);
    Glib_Colour(Glib_WHITE);
    
    Glib_PushMatrix();
    Glib_Rotate(Maths_Degrees(-Crs));
    Glib_Line_Loop(0, 0, 4, CrsFrontPtr);

    for (i = -86; i <= 86; i += 43)
    {
        if (i != 0)
        {
            Circle5(i, 0);
        }
    }
    Glib_Line_Loop(0, 0, 3, CrsBackPtr);
    
    Glib_Colour(Glib_MAGENTA);
    Glib_Line_Loop(0, 0, 3, CdiBar);

    Glib_Draw(0, 94, 0, 400);  /* dotted line */
    for (i = 85; i <= 360; i += 25)
    {
        Glib_Draw(0, -i, 0, -i - 16);
    }
    Glib_PopMatrix();

    Glib_Colour(Glib_WHITE);
    Trk = Track - Hdg;
    Trk = Maths_Normalise(Trk);

    Glib_PushMatrix();
    Glib_Rotate(Maths_Degrees(-Trk));

    if (FCU_ModeSelector == NavDefn_ModeILS)
    {
        Glib_Draw(0, 7, 0, 398);
    }
    else if (FCU_ModeSelector == NavDefn_ModeVOR)
    {
        Glib_Draw(0, 7, 0, 398);
        Glib_Draw(-4, 100, 4, 100);
        Glib_Draw(-4, 200, 4, 200);
        Glib_Draw(-4, 300, 4, 300);
    }
    else
    {
        Glib_SetFont(Glib_EFONT16, 12);
        Glib_Draw(0, 14, 0, 398);
        Glib_Draw(-4, 100, 4, 100);
        Glib_Draw(-4, 200, 4, 200);
        Glib_Draw(-4, 300, 4, 300);
        WriteRange(Range, 200);
    }
    Glib_PopMatrix();

    Glib_SetFont(Glib_EFONT16, 12);
    if (HsiFlag == ToFlag)
    {
        Glib_Chars("TO", 110, -39);
    }
    else if (HsiFlag == FromFlag)
    {
        Glib_Chars("FROM", 110, -39);
    }
}

/* ------------------------------------------------------------ */
static void PlanCircle(unsigned int radius)
{
    unsigned int angle;
    int          x1 = 0, y1 = (int) radius;
    int          x2, y2;
    float        r;
    float        a;

    r = (float) (radius);
    for (angle = 0; angle <= 360; angle += 5)
    {
        a  = Maths_Rads((float) (angle));
        x2 = intround(r * sin(a));
        y2 = intround(r * cos(a));
        if (angle > 0)
        {
            Glib_Draw(x1, y1, x2, y2);
        }
        x1 = x2;
        y1 = y2;
    }
}

/* ------------------------------------------------------------ */
void Compass_DisplayPlan(int CompassX, int CompassY, unsigned int Range)
{
    Glib_LoadIdentity();
    Glib_Translate((float) CompassX, (float) CompassY);
    
    Glib_SetFont(Glib_EFONT16, 12);
    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    
    PlanCircle(125);
    PlanCircle(250);
    Glib_Char('N', -4, 270 - 8);
    Glib_Char('S', -4, -270 - 8);
    Glib_Char('E', 270 - 4, -8);
    Glib_Char('W', -270 - 4, -8);

    Glib_Colour(Glib_CYAN); 
    WritePlanRange(Range, -173, -173);
    WritePlanRange(Range / 2, -86, -86);
 }

/* ------------------------------------------------------------ */
static void WritePlanRange(unsigned int n, int x, int y)
{
    char str[20];
	
	sprintf(str, "%3d", n);
	Glib_Chars(str, x, y);
}

/* ------------------------------------------------------------ */
void Compass_Compass(int CompassX, int CompassY, float Hdg, float Bearing, float Track, unsigned int CrsPointer,
                     unsigned int HdgBug, unsigned int Range, bool ILSMode,
                     NavDefn_FCUMode Mode)
{
    int CrsFrontPtr[26] = 
    {
        0,  185,    4,  181,    4,   95,   12,   95,   12,   85,
        4,   85,    4,   76,   -4,   76,   -4,   85,  -12,   85,
      -12,   95,   -4,   95,   -4,  181
    };

    int CrsBackPtr[10] = 
    {
        0, -185,    4, -181,    4,  -76,   -4,  -76,  -4,  -181
    };

    int ToFromFlagPtr[6] = 
    {
      -16,   49,    0,   65,   16,   49
    };

    float      Trk;
    int        i;
    float      Crs;
    float      Cdi;
    int        IntCdi;
    float      ScaleFactor;
    int        CdiBar[8];
    ToFromFlag HsiFlag;
    float      Bug;

    Glib_LoadIdentity();
    Glib_Translate((float) CompassX, (float) CompassY);
    
    CenteredModeBackground(Mode);
    CompassCard(Hdg);
    HsiFlag = OffFlag;
    
    if (Bearing >= -DEG85 && Bearing <= DEG85)
    {
        HsiFlag = ToFlag;
    }
    else if (Bearing < -DEG95)
    {
        Bearing = -M_PI - Bearing;
        HsiFlag = FromFlag;
    }
    else if (Bearing > DEG95)
    {
        Bearing = M_PI - Bearing;
        HsiFlag = FromFlag;
    }

    if (ILSMode)
    {
        ScaleFactor = 1970.975;
        HsiFlag     = OffFlag;
    }
    else
    {
        ScaleFactor = 492.743;
    }

    Cdi = Bearing * ScaleFactor;
    Cdi = Maths_Limit(Cdi, -108.0, 108.0);
    IntCdi = intround(Cdi);
    if (IntCdi > OldIntCdi)
    {
        IntCdi = OldIntCdi + 1;
    }
    else if (IntCdi < OldIntCdi)
    {
        IntCdi = OldIntCdi - 1;
    }
    OldIntCdi = IntCdi;

    CdiBar[0] = IntCdi - 4; CdiBar[1] = -68;
    CdiBar[2] = IntCdi - 4; CdiBar[3] = 68;
    CdiBar[4] = IntCdi + 4; CdiBar[5] = 68;
    CdiBar[6] = IntCdi + 4; CdiBar[7] = -68;
    
    Bug = Maths_Rads((float) HdgBug) - Hdg;
    Bug = Maths_Normalise(Bug);
    HeadingBug(Bug);

    if (Mode == NavDefn_ModeNAV)
    {
        Trk = Track - Hdg;
        Trk = Maths_Normalise(Trk);
        Glib_Colour(Glib_WHITE);
        Glib_PushMatrix();
        Glib_Rotate(Maths_Degrees(-Trk));
        Glib_Draw(0, 72, 0, 200);
        Glib_Draw(-4, 91, 4, 91);
        Glib_Draw(0, -64, 0, -200);
        Glib_Draw(-4, -91, 4, -91);
        WriteRange(Range, 91);
        WriteRange(Range, -91);
        Glib_PopMatrix();
        return;
    }
    
    Crs = Maths_Rads((float) CrsPointer) - Hdg;
    Crs = Maths_Normalise(Crs);
    Glib_Colour(Glib_WHITE);
    
    Glib_PushMatrix();
    Glib_Rotate(Maths_Degrees(-Crs));
    Glib_Line_Loop(0, 0, 12, CrsFrontPtr);

    for (i = -86; i <= 86; i += 43)
    {
        if (i != 0)
        {
            Circle5(i, 0);
        }
    }
    Glib_Line_Loop(0, 0, 4, CrsBackPtr);
    
    Glib_Colour(Glib_MAGENTA);
    Glib_Line_Loop(0, 0, 3, CdiBar);
    
    Glib_Colour(Glib_WHITE);
    if (HsiFlag == ToFlag)
    {
        Glib_Line_Loop(0, 0, 2, ToFromFlagPtr);
    }
    else if (HsiFlag == FromFlag)
    {
        Glib_Rotate(Maths_Degrees(M_PI));
        Glib_Line_Loop(0, 0, 2, ToFromFlagPtr);
    }
    Glib_PopMatrix();

    Glib_SetFont(Glib_EFONT16, 8);
    if (HsiFlag == ToFlag)
    {
        Glib_Chars("TO", 110, -190);
    }
    else if (HsiFlag == FromFlag)
    {
        Glib_Chars("FROM", 110, -190);
    }
}

/* ------------------------------------------------------------ */
static void CompassCard(float Hdg)
{
    int          IntHdg;
    unsigned int h;

    if (Hdg < 0.0)
    {
        IntHdg = -intround(Maths_Degrees(-Hdg));
    }
    else
    {
        IntHdg = intround(Maths_Degrees(Hdg));
    }
    IntHdg = Norm360(IntHdg);
    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);

    for (h = 0; h <= 355; h += 5)
    {
        Glib_PushMatrix();
        Glib_Rotate(-((float) h - Maths_Degrees(Hdg)));
        if (h % 10 == 0)
        {
            Glib_Draw(0, 176, 0, 200);
            if (h % 30 == 0)
            {
                Glib_SetFont(Glib_EFONT16, 12);
                if (h > 90)
                {
                    Glib_Char(h / 100 + '0', -10, 156);
                    Glib_Char(h / 10 % 10 + '0', 2, 156);
                }
                else
                {
                    Glib_Char(h / 10 % 10 + '0', -5, 156);
                }
            }
        }
        else
        {
            Glib_Draw(0, 189, 0, 200);
        }
        Glib_PopMatrix();
    }
	
    WrHdg(IntHdg, -30, 225);
}

/* ------------------------------------------------------------ */
static void WrHdg(int hdg, int x, int y)
{
    Glib_SetFont(Glib_EFONT24, 16);
    Glib_Colour(Glib_WHITE);
    Compass_Wrn(x, y, hdg, 3, true);
}

/* ------------------------------------------------------------ */
static void ExpandedModeBackground(NavDefn_FCUMode Mode)
{
    int i;

    Glib_Colour(Glib_WHITE);
    Glib_Draw(0, 0, -16, -42);
    Glib_Draw(-16, -42, 16, -42);
    Glib_Draw(16, -42, 0, 0);
    Glib_Draw(-37, 420, 37, 420);
    Glib_Draw(-37, 420, -37, 448);
    Glib_Draw(37, 420, 37, 448);
    Glib_Colour(Glib_GREEN);
    Glib_SetFont(Glib_EFONT16, 12);
    Glib_Chars("HDG", -82, 427);
    Glib_Chars("MAG", 42, 427);
    Glib_Colour(Glib_WHITE);
    Glib_Draw(0, 401, -8, 417);
    Glib_Draw(-8, 417, 8, 417);
    Glib_Draw(8, 417, 0, 401);

    if (FCU_ModeSelector == NavDefn_ModeILS)
    {
        Glib_Draw(244, 71, 273, 71);
        Glib_SetFont(Glib_EFONT16, 10);
        for (i = -2; i <= 2; i += 1)
        {
            if (i != 0)
            {
                Circle5(259, 71 + i * 43);
            }
        }
    }
    else if (FCU_ModeSelector == NavDefn_ModeVOR)
    {
        Glib_Draw(245, -23, 254, -23);
        Glib_Draw(254, -23, 254, 104);
        Glib_Draw(254, 104, 245, 104);
        Glib_Draw(238, 41, 254, 41);
    }
}

/* --------------------------------------------------- */
static void CenteredModeBackground(NavDefn_FCUMode Mode)
{
    int i;

    Glib_Colour(Glib_WHITE);
    if (Mode != NavDefn_ModeNAV)
    {
        Mark(450, 135, 405, 202, 227);
        Glib_Draw(-9, -35, -9, 35);
        Glib_Draw(9, -35, 9, 35);
        Glib_Draw(-21, -35, -9, -35);
        Glib_Draw(9, -35, 21, -35);
        Glib_Draw(-39, 0, -9, 0);
        Glib_Draw(9, 0, 39, 0);
    }
    else
    {
        Glib_Draw(0, 0, -16, -42);
        Glib_Draw(-16, -42, 16, -42);
        Glib_Draw(16, -42, 0, 0);
    }

    Glib_Draw(-37, 220, 37, 220);
    Glib_Draw(-37, 220, -37, 248);
    Glib_Draw(37, 220, 37, 248);
    Glib_Colour(Glib_GREEN);
    Glib_SetFont(Glib_EFONT16, 12);
    Glib_Chars("HDG", -82, 227);
    Glib_Chars("MAG", 42, 227);
    Glib_Colour(Glib_WHITE);
    Glib_Draw(0, 201, -8, 217);
    Glib_Draw(-8, 217, 8, 217);
    Glib_Draw(8, 217, 0, 201);

    if (Mode == NavDefn_ModeILS)
    {
        Glib_Draw(245, 0, 273, 0);
        Glib_SetFont(Glib_EFONT16, 10);
        for (i = -2; i <= 2; i += 1)
        {
            if (i != 0)
            {
                Circle5(259, i * 43);
            }
        }
    }
    else if (Mode == NavDefn_ModeNAV)
    {
        Glib_Draw(250, -65, 259, -65);
        Glib_Draw(259, -65, 259, 65);
        Glib_Draw(259, 65, 250, 65);
        Glib_Draw(243, 0, 259, 0);
    }
}

/* --------------------------------------------------- */
static void Mark(int inc, int a1, int a2, int r1, int r2)
{
    int   x1, y1, x2, y2;
    float xx;

    a1 = a1 * 10;
    a2 = a2 * 10;
	
    while (a1 <= a2)
    {
        xx = Maths_Rads((float) a1 * 0.1);
        xx = Maths_Normalise(xx);
        x1 = intround(cos(xx) * (float) r1);
        y1 = intround(sin(xx) * (float) r1);
        x2 = intround(cos(xx) * (float) r2);
        y2 = intround(sin(xx) * (float) r2);
        Glib_Draw(x1, y1, x2, y2);
        a1 = a1 + inc;
    }
}

/* --------------------------------------------------- */
static void HeadingBug(float Bug)
{
    unsigned int i;
    int          HdgBugPtr[14] = 
    {
        -10,  200,  -10,  210,   -7,  210,    0,  200,    7,  210,   10,  210,   10,  200
    };

    Glib_Colour(Glib_MAGENTA);
    Glib_PushMatrix();
    Glib_Rotate(Maths_Degrees(-Bug));
    Glib_Line_Loop(0, 0, 6, HdgBugPtr);
    for (i = 10; i <= 160; i += 30)
    {
        Glib_Draw(0, i, 0, i + 10);
    }
    Glib_PopMatrix();
}

/* --------------------------------------------------- */
static void ExpandedHeadingBug(float Bug)
{
    unsigned int i;
    int HdgBugPtr[14] = 
    {
        -15,  400,  -15,  410,   -8,  410,    0,  400,    8,  410,   15,  410,   15,  400
    };

    Glib_Colour(Glib_MAGENTA);
    Glib_PushMatrix();
    Glib_Rotate(Maths_Degrees(-Bug));
    Glib_Line_Loop(0, 0, 6, HdgBugPtr);
    for (i = 10; i <= 360; i += 35)
    {
        Glib_Draw(0, i, 0, i + 10);
    }
    Glib_PopMatrix();
}

/* --------------------------------------------------- */
static void WriteRange(unsigned int Range, int y)
{
    char str[10];
	
	Glib_SetFont(Glib_EFONT16, 12);
	sprintf(str, "%3d", Range);
	Glib_Chars(str, -5 - Glib_StringSize(str), y - 8);
}

/* --------------------------------------------------- */
static unsigned int Norm360(int a)
{
    while (a < 0)
    {
        a = a + 360;
    }
    while (a >= 360)
    {
        a = a - 360;
    }
    return (unsigned int) a;
}

/* --------------------------------------------------- */
void Compass_Wrn(int x, int y, unsigned int n, unsigned int w, bool LeadingZeros)
{
    int  p;
    char s[20];

    for (p = w - 1; p >= 0; p -= 1)
    {
        if (n > 0)
        {
            s[p] = (unsigned char) (n % 10) + '0';
        }
        else
        {
            if (LeadingZeros)
            {
                s[p] = '0';
            }
            else
            {
                s[p] = ' ';
            }
        }
        n = n / 10;
    }

    s[w] = 0;
    Glib_Chars(s, x, y);
}

/* ------------------------------------------------- */
void Circle5(int x, int y)
{
    Glib_Line_Loop(x, y, 17, Circle5Tab);
}

/* --------------------------------------------------- */
void BEGIN_Compass()
{
    OldIntCdi       = 0;
    OldGlideslope = 0;
    OldRmi1         = 0.0;
    OldRmi2         = 0.0;
}
