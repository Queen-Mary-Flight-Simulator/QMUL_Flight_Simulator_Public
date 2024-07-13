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

#include <GL/gl.h>
#include <math.h>
#include <stdbool.h>

#include <SIM/glib.h>
#include <SIM/maths.h>
#include <SIM/navdefn.h>

#include "nfd.h"
#include "nav.h"
#include "fcu.h"
#include "nfd-compass.h"

#define ToFlag      0
#define FromFlag    1
#define OffFlag     2

typedef unsigned char   ToFromFlag;

typedef struct
{
    int x;
    int y;
} Vertex;

#define Font16Spacing    12
#define Font16Height     16
#define DEG5             (5.0 / Maths_ONERAD)
#define DEG85            (85.0 / Maths_ONERAD)
#define DEG95            (95.0 / Maths_ONERAD)

static int   OldIntCdi;
static int   OldGlideslope_h;
static float OldRmi1;
static float OldRmi2;


static void PlanCircle(unsigned int radius);
static void WritePlanRange(unsigned int n, int x, int y);
static void Blanks(int n, int x, int y);
static void CompassCard(float Hdg);
static void WrHdg(int hdg, int x, int y);
static void ExpandedModeBackground(NavDefn_FCUMode Mode);
static void CenteredModeBackground(NavDefn_FCUMode Mode);
static void Mark(int inc, int a1, int a2, int r1, int r2);
static void HeadingBug(float Bug);
static void ExpandedHeadingBug(float Bug);
static void WriteRange(unsigned int Range, int y);
static unsigned int Norm360(int a);

void Compass_Rmi(float Rmi1, NavDefn_FCUNav n1, float Rmi2,
                 NavDefn_FCUNav n2)
{
    Vertex       Rmi1FrontPtr[7] = {
        { -9, 173 }, { 9, 173 }, { 0, 173 }, { 0, 215 }, { -5, 210 }, { 0, 215 }, { 5, 210 }
    };

    Vertex       Rmi1BackPtr[6] = {
        { -9, -206 }, { 0, -202 }, { 9, -206 }, { 0, -202 }, { 0, -210 }, { 0, -176 }
    };

    Vertex       Rmi2FrontPtr[10] = {
        { -11, 173 }, { -11, 177 }, { -4, 177 }, { -4, 211 }, {   0, 215 },
        {   4, 211 }, {   4, 177 }, { 11, 177 }, { 11, 173 }, { -11, 173 }
    };

    Vertex       Rmi2BackPtr[14] = {
        { 0, -200 }, { 0, -207 }, { -9, -211 }, { -9, -204 }, { 0, -200 }, { 4, -202 },
        { 4, -177 }, { 0, -173 }, { -4, -177 }, { -4, -202 }, { 0, -200 },
        { 9, -204 }, { 9, -211 }, {  0, -207 }
    };

    bool         Rmi1OK;
    bool         Rmi2OK;
    unsigned int Col1 = Glib_BLACK;
    unsigned int Col2 = Glib_BLACK;
    unsigned int i;

    while (OldRmi1 > Rmi1 + Maths_PI)
    {
        OldRmi1 = OldRmi1 - Maths_TWOPI;
    }
    while (OldRmi1 < Rmi1 - Maths_PI)
    {
        OldRmi1 = OldRmi1 + Maths_TWOPI;
    }
    if (n1 == NavDefn_NavVOR)
    {
        Col1   = Glib_GREEN;
        Rmi1OK = Nav_VOR1.BeaconStatus;
    }
    else if (n1 == NavDefn_NavADF)
    {
        Col1   = Glib_BLUE;
        Rmi1OK = Nav_ADF1.BeaconStatus;
    }
    else
    {
        Rmi1OK = false;
    }
    if (Rmi1OK && n1 != NavDefn_NavOFF)
    {
        Glib_Colour(Col1);
        glPushMatrix();
        glRotatef(Maths_Degrees(-Rmi1), 0.0, 0.0, 1.0);
        glBegin(GL_LINE_STRIP);
        for (i = 0; i <= 6; i += 1)
        {
            glVertex2i(Rmi1FrontPtr[i].x, Rmi1FrontPtr[i].y);
        }
        glEnd();
        glBegin(GL_LINE_STRIP);
        for (i = 0; i <= 5; i += 1)
        {
            glVertex2i(Rmi1BackPtr[i].x, Rmi1BackPtr[i].y);
        }
        glEnd();
        glPopMatrix();
    }
    while (OldRmi2 > Rmi2 + Maths_PI)
    {
        OldRmi2 = OldRmi2 - Maths_TWOPI;
    }
    while (OldRmi2 < Rmi2 - Maths_PI)
    {
        OldRmi2 = OldRmi2 + Maths_TWOPI;
    }
    if (n2 == NavDefn_NavVOR)
    {
        Col2   = Glib_GREEN;
        Rmi2OK = Nav_VOR2.BeaconStatus;
    }
    else if (n2 == NavDefn_NavADF)
    {
        Col2   = Glib_BLUE;
        Rmi2OK = Nav_ADF2.BeaconStatus;
    }
    else
    {
        Rmi2OK = false;
    }
    if (Rmi2OK && n2 != NavDefn_NavOFF)
    {
        Glib_Colour(Col2);
        glPushMatrix();
        glRotatef(Maths_Degrees(-Rmi2), 0.0, 0.0, 1.0);
        glBegin(GL_LINE_LOOP);
        for (i = 0; i <= 9; i += 1)
        {
            glVertex2i(Rmi2FrontPtr[i].x, Rmi2FrontPtr[i].y);
        }
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (i = 0; i <= 13; i += 1)
        {
            glVertex2i(Rmi2BackPtr[i].x, Rmi2BackPtr[i].y);
        }
        glEnd();
        glPopMatrix();
    }
}

void Compass_ExpandedRmi(float Rmi1, NavDefn_FCUNav n1, float Rmi2,
                         NavDefn_FCUNav n2)
{
    Vertex       Rmi1FrontPtr[7] = {
        { -9, 360 }, { 9, 360 }, { 0, 360 }, { 0, 416 }, { -5, 411 }, { 0, 416 }, { 5, 411 }
    };

    Vertex       Rmi1BackPtr[6] = {
        { -9, -418 }, { 0, -409 }, { 9, -418 }, { 0, -409 }, { 0, -418 }, { 0, -360 }
    };

    Vertex       Rmi2FrontPtr[10] = {
        { -11, 358 }, { -11, 363 }, { -5, 363 }, { -5, 404 }, {   0, 409 },
        {   5, 404 }, {   5, 363 }, { 11, 363 }, { 11, 358 }, { -11, 358 }
    };

    Vertex       Rmi2BackPtr[14] = {
        {  0, -400 }, {  0, -410 }, { -10, -415 }, { -10, -405 }, { 0, -400 }, { 5, -402 },
        {  5, -363 }, {  0, -358 }, {  -5, -363 }, {  -5, -402 }, { 0, -400 },
        { 10, -405 }, { 10, -415 }, {   0, -410 }
    };

    bool         Rmi1OK;
    bool         Rmi2OK;
    unsigned int Col1 = Glib_BLACK;
    unsigned int Col2 = Glib_BLACK;
    unsigned int i;

    while (OldRmi1 > Rmi1 + Maths_PI)
    {
        OldRmi1 = OldRmi1 - Maths_TWOPI;
    }
    while (OldRmi1 < Rmi1 - Maths_PI)
    {
        OldRmi1 = OldRmi1 + Maths_TWOPI;
    }
    if (n1 == NavDefn_NavVOR)
    {
        Col1   = Glib_GREEN;
        Rmi1OK = Nav_VOR1.BeaconStatus;
    }
    else if (n1 == NavDefn_NavADF)
    {
        Col1   = Glib_BLUE;
        Rmi1OK = Nav_ADF1.BeaconStatus;
    }
    else
    {
        Rmi1OK = false;
    }
    if (Rmi1OK && n1 != NavDefn_NavOFF)
    {
        Glib_Colour(Col1);
        glPushMatrix();
        glRotatef(Maths_Degrees(-Rmi1), 0.0, 0.0, 1.0);
        glBegin(GL_LINE_STRIP);
        for (i = 0; i <= 6; i += 1)
        {
            glVertex2i(Rmi1FrontPtr[i].x, Rmi1FrontPtr[i].y);
        }
        glEnd();
        glBegin(GL_LINE_STRIP);
        for (i = 0; i <= 5; i += 1)
        {
            glVertex2i(Rmi1BackPtr[i].x, Rmi1BackPtr[i].y);
        }
        glEnd();
        glPopMatrix();
    }
    while (OldRmi2 > Rmi2 + Maths_PI)
    {
        OldRmi2 = OldRmi2 - Maths_TWOPI;
    }
    while (OldRmi2 < Rmi2 - Maths_PI)
    {
        OldRmi2 = OldRmi2 + Maths_TWOPI;
    }
    if (n2 == NavDefn_NavVOR)
    {
        Col2   = Glib_GREEN;
        Rmi2OK = Nav_VOR2.BeaconStatus;
    }
    else if (n2 == NavDefn_NavADF)
    {
        Col2   = Glib_BLUE;
        Rmi2OK = Nav_ADF2.BeaconStatus;
    }
    else
    {
        Rmi2OK = false;
    }
    if (Rmi2OK && n2 != NavDefn_NavOFF)
    {
        Glib_Colour(Col2);
        glPushMatrix();
        glRotatef(Maths_Degrees(-Rmi2), 0.0, 0.0, 1.0);
        glBegin(GL_LINE_LOOP);
        for (i = 0; i <= 9; i += 1)
        {
            glVertex2i(Rmi2FrontPtr[i].x, Rmi2FrontPtr[i].y);
        }
        glEnd();
        glBegin(GL_LINE_LOOP);
        for (i = 0; i <= 13; i += 1)
        {
            glVertex2i(Rmi2BackPtr[i].x, Rmi2BackPtr[i].y);
        }
        glEnd();
        glPopMatrix();
    }
}

void Compass_GlideSlope(float GlideSlopeError, bool ILSMode, bool Selected)
{
    int Diamond[8] = { -8, 0, 0, 8, 8, 0, 0, -8 };

    int h;

    h = intround(GlideSlopeError * 7039.2);
    if (h > OldGlideslope_h)
    {
        h = OldGlideslope_h + 1;
    }
    else if (h < OldGlideslope_h)
    {
        h = OldGlideslope_h - 1;
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
        Glib_DrawPolygon(4, 259, h, Diamond);
    }
    OldGlideslope_h = h;
}

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
    Maths_Normalise(&Offset);
    a  = Offset;
    h1 = MinHdg;
    h2 = MaxHdg;
    Glib_Colour(Glib_WHITE);
    glLineWidth(2.0);
    do
    {
        if (h1 <= h2)
        {
            t = a;
            glPushMatrix();
            glRotatef(Maths_Degrees(-t), 0.0, 0.0, 1.0);
            Glib_Draw(-18, 400, 18, 400);
            if (h1 % 10 == 0)
            {
                Glib_Draw(0, 379, 0, 400);
                if (h1 % 30 == 0)
                {
                    Glib_SetFont(Glib_GFONT16, 12);
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
            glPopMatrix();
            a  = a + DEG5;
            h1 = h1 + 5;
        }
    } while (!(h1 > h2));
    WrHdg(IntHdg, -21, 425);
}

void Compass_ExpandedCompass(float Hdg, float Bearing, float Track, unsigned int CrsPointer,
                             unsigned int HdgBug, unsigned int Range, bool ILSMode,
                             NavDefn_FCUMode Mode)
{
    Vertex     CrsFrontPtr[5] = {
        { -4, 57 }, { -4, 75 }, { 0, 85 }, { 4, 75 }, { 4, 57 }
    };

    Vertex     CrsBackPtr[5] = {
        { -4, -57 }, { -4, -74 }, { 4, -74 }, { 4, -57 }, { -4, -57 }
    };

    ToFromFlag HsiFlag;
    float      ScaleFactor;
    float      Cdi;
    int        IntCdi;
    float      Bug;
    float      Crs;
    int        i;
    Vertex     CdiBar[5];
    float      Trk;

    ExpandedModeBackground(Mode);
    ExpandedCompassCard(Hdg);
    HsiFlag = OffFlag;
    if (Bearing >= -DEG85 && Bearing <= DEG85)
    {
        HsiFlag = ToFlag;
    }
    else if (Bearing < -DEG95)
    {
        Bearing = -Maths_PI - Bearing;
        HsiFlag = FromFlag;
    }
    else if (Bearing > DEG95)
    {
        Bearing = Maths_PI - Bearing;
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
    Maths_Limit(&Cdi, -108.0, 108.0);
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
    Crs       = Maths_Rads((float) (CrsPointer)) - Hdg;
    Maths_Normalise(&Crs);
    Glib_Colour(Glib_WHITE);
    glPushMatrix();
    glRotatef(Maths_Degrees(-Crs), 0.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 4; i += 1)
    {
        glVertex2i(CrsFrontPtr[i].x, CrsFrontPtr[i].y);
    }
    glEnd();
    Glib_SetFont(Glib_GFONT16, 8);
    for (i = -86; i <= 86; i += 43)
    {
        if (i != 0)
        {
            Glib_Char(15, i - 4, -4);
        }
    }
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 4; i += 1)
    {
        glVertex2i(CrsBackPtr[i].x, CrsBackPtr[i].y);
    }
    glEnd();
    glPopMatrix();
    CdiBar[0].x = IntCdi - 4;
    CdiBar[0].y = -55;
    CdiBar[1].x = IntCdi - 4;
    CdiBar[1].y = 55;
    CdiBar[2].x = IntCdi + 4;
    CdiBar[2].y = 55;
    CdiBar[3].x = IntCdi + 4;
    CdiBar[3].y = -55;
    CdiBar[4].x = IntCdi - 4;
    CdiBar[4].y = -55;
    Glib_Colour(Glib_MAGENTA);
    glPushMatrix();
    glRotatef(Maths_Degrees(-Crs), 0.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 4; i += 1)
    {
        glVertex2i(CdiBar[i].x, CdiBar[i].y);
    }
    glEnd();
    Glib_Draw(0, 94, 0, 400);
    for (i = 85; i <= 360; i += 25)
    {
        Glib_Draw(0, -i, 0, -i - 16);
    }
    glPopMatrix();
    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_GFONT16, 12);
    if (HsiFlag == ToFlag)
    {
        Glib_Chars("TO", 110, -39);
    }
    else if (HsiFlag == FromFlag)
    {
        Glib_Chars("FROM", 110, -39);
    }
    Bug = Maths_Rads((float) HdgBug) - Hdg;
    Maths_Normalise(&Bug);
    ExpandedHeadingBug(Bug);
    Trk = Track - Hdg;
    Maths_Normalise(&Trk);
    Glib_Colour(Glib_WHITE);
    glPushMatrix();
    glRotatef(Maths_Degrees(-Trk), 0.0, 0.0, 1.0);
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
        Glib_SetFont(Glib_GFONT16, 12);
        Glib_Draw(0, 14, 0, 398);
        Glib_Draw(-4, 100, 4, 100);
        Glib_Draw(-4, 200, 4, 200);
        Glib_Draw(-4, 300, 4, 300);
        WriteRange(Range, 200);
    }
    glPopMatrix();
}

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

void Compass_DisplayPlan(unsigned int Range)
{
    Glib_SetFont(Glib_GFONT16, 12);
    Glib_Colour(Glib_WHITE);
    PlanCircle(125);
    PlanCircle(250);
    Glib_Char('N', -4, 270 - 8);
    Glib_Char('S', -4, -270 - 8);
    Glib_Char('E', 270 - 4, -8);
    Glib_Char('W', -270 - 4, -8);
    WritePlanRange(Range, 0, 250);
    WritePlanRange(Range, 0, -250);
    WritePlanRange(Range / 2, 0, 125);
    WritePlanRange(Range / 2, 0, -125);
}

static void WritePlanRange(unsigned int n, int x, int y)
{
    if (n >= 100)
    {
        Blanks(3, x - Font16Spacing - Font16Spacing / 2, y - Font16Height / 2);
        Compass_Wrn(x - Font16Spacing - Font16Spacing / 2, y - Font16Height / 2, n, 3, false);
    }
    else if (n >= 10)
    {
        Blanks(2, x - Font16Spacing, y - Font16Height / 2);
        Compass_Wrn(x - Font16Spacing, y - Font16Height / 2, n, 2, false);
    }
    else
    {
        Blanks(1, x - Font16Spacing / 2, y - Font16Height / 2);
        Compass_Wrn(x - Font16Spacing / 2, y - Font16Height / 2, n, 1, false);
    }
}

static void Blanks(int n, int x, int y)
{
    int i;

    Glib_Colour(Glib_BLACK);
    for (i = -2; i <= 18; i += 1)
    {
        Glib_Draw(x - 5, y + i, x + n * 10 + 5, y + i);
    }
    Glib_Colour(Glib_WHITE);
}

void Compass_Compass(float Hdg, float Bearing, float Track, unsigned int CrsPointer,
                     unsigned int HdgBug, unsigned int Range, bool ILSMode,
                     NavDefn_FCUMode Mode)
{
    Vertex     CrsFrontPtr[14] = {
        {  0, 185 }, {  4, 181 }, {   4, 95 }, {  12, 95 }, { 12, 85 }, {  4,  85 }, { 4,  76 },
        { -4,  76 }, { -4,  85 }, { -12, 85 }, { -12, 95 }, { -4, 95 }, { -4, 181 }, { 0, 185 }
    };

    Vertex     CrsBackPtr[6] = {
        { 0, -185 }, { 4, -181 }, { 4, -76 }, { -4, -76 }, { -4, -181 }, { 0, -185 }
    };

    Vertex     ToFromFlagPtr[4] = {
        { -16, 49 }, { 0, 65 }, { 16, 49 }, { -16, 49 }
    };

    float      Trk;
    int        i;
    float      Crs;
    float      Cdi;
    int        IntCdi;
    float      ScaleFactor;
    Vertex     CdiBar[5];
    ToFromFlag HsiFlag;
    float      Bug;

    CenteredModeBackground(Mode);
    CompassCard(Hdg);
    HsiFlag = OffFlag;
    if (Bearing >= -DEG85 && Bearing <= DEG85)
    {
        HsiFlag = ToFlag;
    }
    else if (Bearing < -DEG95)
    {
        Bearing = -Maths_PI - Bearing;
        HsiFlag = FromFlag;
    }
    else if (Bearing > DEG95)
    {
        Bearing = Maths_PI - Bearing;
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
    Maths_Limit(&Cdi, -108.0, 108.0);
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
    Bug       = Maths_Rads((float) HdgBug) - Hdg;
    Maths_Normalise(&Bug);
    HeadingBug(Bug);
    if (Mode == NavDefn_ModeNAV)
    {
        Trk = Track - Hdg;
        Maths_Normalise(&Trk);
        Glib_Colour(Glib_WHITE);
        glPushMatrix();
        glRotatef(Maths_Degrees(-Trk), 0.0, 0.0, 1.0);
        Glib_Draw(0, 72, 0, 200);
        Glib_Draw(-4, 91, 4, 91);
        Glib_Draw(0, -64, 0, -200);
        Glib_Draw(-4, -91, 4, -91);
        WriteRange(Range, 91);
        WriteRange(Range, -91);
        glPopMatrix();
        return;
    }
    Crs = Maths_Rads((float) CrsPointer) - Hdg;
    Maths_Normalise(&Crs);
    Glib_Colour(Glib_WHITE);
    glPushMatrix();
    glRotatef(Maths_Degrees(-Crs), 0.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 13; i += 1)
    {
        glVertex2i(CrsFrontPtr[i].x, CrsFrontPtr[i].y);
    }
    glEnd();
    Glib_SetFont(Glib_GFONT16, 8);
    for (i = -86; i <= 86; i += 43)
    {
        if (i != 0)
        {
            Glib_Char(15, i - 4, -4);
        }
    }
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 5; i += 1)
    {
        glVertex2i(CrsBackPtr[i].x, CrsBackPtr[i].y);
    }
    glEnd();
    glPopMatrix();
    CdiBar[0].x = IntCdi - 4;
    CdiBar[0].y = -68;
    CdiBar[1].x = IntCdi - 4;
    CdiBar[1].y = 68;
    CdiBar[2].x = IntCdi + 4;
    CdiBar[2].y = 68;
    CdiBar[3].x = IntCdi + 4;
    CdiBar[3].y = -68;
    CdiBar[4].x = IntCdi - 4;
    CdiBar[4].y = -68;
    Glib_Colour(Glib_MAGENTA);
    glPushMatrix();
    glRotatef(Maths_Degrees(-Crs), 0.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 4; i += 1)
    {
        glVertex2i(CdiBar[i].x, CdiBar[i].y);
    }
    glEnd();
    glPopMatrix();
    Glib_Colour(Glib_WHITE);
    glPushMatrix();
    if (HsiFlag == ToFlag)
    {
        glRotatef(Maths_Degrees(-Crs), 0.0, 0.0, 1.0);
    }
    else if (HsiFlag == FromFlag)
    {
        Crs = Crs + Maths_PI;
        Maths_Normalise(&Crs);
        glRotatef(Maths_Degrees(-Crs), 0.0, 0.0, 1.0);
    }
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 2; i += 1)
    {
        glVertex2i(ToFromFlagPtr[i].x, ToFromFlagPtr[i].y);
    }
    glEnd();
    glPopMatrix();
}

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
    glLineWidth(2.0);
    for (h = 0; h <= 355; h += 5)
    {
        glPushMatrix();
        glRotatef(-((float) h - Maths_Degrees(Hdg)), 0.0, 0.0, 1.0);
        if (h % 10 == 0)
        {
            Glib_Draw(0, 176, 0, 200);
            if (h % 30 == 0)
            {
                Glib_SetFont(Glib_GFONT16, 12);
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
        glPopMatrix();
    }
    WrHdg(IntHdg, -21, 225);
}

static void WrHdg(int hdg, int x, int y)
{
    Glib_SetFont(Glib_GFONT24, 16);
    Glib_Colour(Glib_WHITE);
    Compass_Wrn(x, y, hdg, 3, true);
}

static void ExpandedModeBackground(NavDefn_FCUMode Mode)
{
    int i;

    Glib_Colour(Glib_WHITE);
    Glib_Draw(0, 0, -16, -42);
    Glib_Draw(-16, -42, 16, -42);
    Glib_Draw(16, -42, 0, 0);
    Glib_Draw(-32, 420, 32, 420);
    Glib_Draw(-32, 420, -32, 448);
    Glib_Draw(32, 420, 32, 448);
    Glib_Colour(Glib_GREEN);
    Glib_SetFont(Glib_GFONT16, 12);
    Glib_Chars("HDG", -70, 427);
    Glib_Chars("MAG", 40, 427);
    Glib_Colour(Glib_WHITE);
    Glib_Draw(0, 401, -8, 417);
    Glib_Draw(-8, 417, 8, 417);
    Glib_Draw(8, 417, 0, 401);
    if (FCU_ModeSelector == NavDefn_ModeILS)
    {
        Glib_Draw(244, 71, 273, 71);
        Glib_SetFont(Glib_GFONT16, 10);
        for (i = -2; i <= 2; i += 1)
        {
            if (i != 0)
            {
                Glib_Char(15, 259 - 4, 71 + i * 43 - 4);
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
    Glib_Draw(-32, 220, 32, 220);
    Glib_Draw(-32, 220, -32, 248);
    Glib_Draw(32, 220, 32, 248);
    Glib_Colour(Glib_GREEN);
    Glib_SetFont(Glib_GFONT16, 12);
    Glib_Chars("HDG", -70, 227);
    Glib_Chars("MAG", 40, 227);
    Glib_Colour(Glib_WHITE);
    Glib_Draw(0, 201, -8, 217);
    Glib_Draw(-8, 217, 8, 217);
    Glib_Draw(8, 217, 0, 201);
    if (Mode == NavDefn_ModeILS)
    {
        Glib_Draw(245, 0, 273, 0);
        Glib_SetFont(Glib_GFONT16, 10);
        for (i = -2; i <= 2; i += 1)
        {
            if (i != 0)
            {
                Glib_Char(15, 259 - 4, i * 43 - 4);
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

static void Mark(int inc, int a1, int a2, int r1, int r2)
{
    int   x1, y1, x2, y2;
    float xx;

    a1 = a1 * 10;
    a2 = a2 * 10;
    while (a1 <= a2)
    {
        xx = Maths_Rads((float) a1 * 0.1);
        Maths_Normalise(&xx);
        x1 = intround(cos(xx) * (float) r1);
        y1 = intround(sin(xx) * (float) r1);
        x2 = intround(cos(xx) * (float) r2);
        y2 = intround(sin(xx) * (float) r2);
        Glib_Draw(x1, y1, x2, y2);
        a1 = a1 + inc;
    }
}

static void HeadingBug(float Bug)
{
    Vertex       HdgBugPtr[7] = {
        { -10, 200 }, { -10, 210 }, { -7, 210 }, { 0, 200 }, { 7, 210 }, { 10, 210 }, { 10, 200 }
    };

    unsigned int i;

    Glib_Colour(Glib_MAGENTA);
    glPushMatrix();
    glRotatef(Maths_Degrees(-Bug), 0.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 6; i += 1)
    {
        glVertex2i(HdgBugPtr[i].x, HdgBugPtr[i].y);
    }
    glEnd();
    for (i = 10; i <= 160; i += 30)
    {
        Glib_Draw(0, i, 0, i + 10);
    }
    glPopMatrix();
}

static void ExpandedHeadingBug(float Bug)
{
    Vertex       HdgBugPtr[7] = {
        { -15, 400 }, { -15, 410 }, { -8, 410 }, { 0, 400 }, { 8, 410 }, { 15, 410 }, { 15, 400 }
    };

    unsigned int i;

    Glib_Colour(Glib_MAGENTA);
    glPushMatrix();
    glRotatef(Maths_Degrees(-Bug), 0.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 6; i += 1)
    {
        glVertex2i(HdgBugPtr[i].x, HdgBugPtr[i].y);
    }
    glEnd();
    for (i = 10; i <= 360; i += 35)
    {
        Glib_Draw(0, i, 0, i + 10);
    }
    glPopMatrix();
}

static void WriteRange(unsigned int Range, int y)
{
    Glib_SetFont(Glib_GFONT16, 12);
    if (Range >= 100)
    {
        Compass_Wrn(-Font16Spacing * 3 - 3, y - 8, Range, 3, false);
    }
    else if (Range >= 10)
    {
        Compass_Wrn(-Font16Spacing * 2 - 3, y - 8, Range, 2, false);
    }
    else
    {
        Compass_Wrn(-3, y - 8, Range, 3, false);
    }
}

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

void Compass_Wrn(int x, int y, unsigned int n, unsigned int w, bool LeadingZeros)
{
    int  p;
    char s[20];

    {
        for (p = w - 1; p >= 0; p += -1)
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
    }
    s[w] = 0;
    Glib_Chars(s, x, y);
}

void BEGIN_Compass()
{
    OldIntCdi       = 0;
    OldGlideslope_h = 0;
    OldRmi1         = 0.0;
    OldRmi2         = 0.0;
}
