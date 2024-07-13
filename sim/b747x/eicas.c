/* +------------------------------+---------------------------------+
   | Module      : eicas.c        | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-09      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EICAS Display                     |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <SIM/glib.h>
#include <SIM/maths.h>
#include <SIM/iodefn.h>
#include "aerolink.h"
#include "systems.h"
#include "eicas.h"

#define Up         0
#define Down       1
#define Transit    2
typedef unsigned char   GearState;

#define DEG5       (5.0 / Maths_ONERAD)
#define DEG198     (198.0 / Maths_ONERAD)
#define DEG200     (200.0 / Maths_ONERAD)
#define DEG216     (216.0 / Maths_ONERAD)

static void Box(int x, int y, int xs, int ys);

static void DrawArc(unsigned int a1, unsigned int a2,
                unsigned int r, unsigned int segments);

static void Mark(int inc, int a1, int a2, int r1, int r2);

static void SmallDigits(int d1, int d2, int dinc, int a,
                        int ainc, int r);

static void DrawRectangle(int x1, int y1, int xs, int ys);

/* ------------------------------------------------ */
void DrawRectangle(int x, int y, int xs, int ys)
{
    Glib_Draw(x, y, x + xs, y);
    Glib_Draw(x + xs, y, x + xs, y + ys);
    Glib_Draw(x + xs, y + ys, x, y + ys);
    Glib_Draw(x, y + ys, x, y);
}

/* ------------------------------------------------ */
void DrawArc(unsigned int a1, unsigned int a2, unsigned int r, unsigned int segments)
{
    float        a;
    float        astart;
    float        aend;
    float        rad;
    int          x, y;
    float        spacing;
    unsigned int i;

    rad     = (float) r;
    astart  = Maths_Rads((float) a1);
    aend    = Maths_Rads((float) a2);
    spacing = (aend - astart) / (float) segments;
    a       = astart;
    glBegin(GL_LINE_STRIP);
    {
        for (i = 0; i <= segments; i += 1)
        {
            a = astart + (float) i * spacing;
            x = intround(rad * cos(a));
            y = intround(rad * sin(a));
            glVertex2i(x, y);
        }
    }
    glEnd();
}

/* ------------------------------------------------ */
void Mark(int inc, int a1, int a2, int r1, int r2)
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

/* ------------------------------------------------ */
void SmallDigits(int d1, int d2, int dinc, int a, int ainc, int r)
{
    int   n, i;
    int   x, y;
    char  v[6];
    float aa, rr;

    Glib_SetFont(Glib_GFONT8, 6);
    n = abs(d1 - d2) / dinc;
    {
        for (i = 0; i <= n; i += 1)
        {
            aa = Maths_Rads((float) a);
            rr = (float) (r - 2);
            if (d1 >= 10)
            {
                v[0] = (unsigned char) d1 / 10 + '0';
                v[1] = (unsigned char) d1 % 10 + '0';
                v[2] = 0;
            }
            else
            {
                v[0] = (unsigned char) d1 + '0';
                v[1] = 0;
            }
            y = intround(sin(aa) * rr - 2.5);
            if (d1 < 10)
            {
                x = intround(cos(aa) * rr - 1.5);
            }
            else
            {
                x = intround(cos(aa) * rr - 3.5);
            }
            Glib_Chars(v, x, y);
            d1 = d1 + dinc;
            a  = a + ainc;
        }
    }
}

/* ------------------------------------------------ */
void EICAS_EprGauge(unsigned int EngineNumber)
{
    int   x;
    int   y;
    float e;
    float a;
    int   digits;
    char  d1;
    char  d2;
    char  d3;

    e = Maths_Rads(AeroLink_EngPkt.Engines[EngineNumber].Epr * 110.0);
    if (e < 0.0)
    {
        e = 0.0;
    }
    else if (e > DEG198)
    {
        e = DEG198;
    }
    Glib_AntiAliasing(true);

    a = 0.0;
    Glib_Colour(Glib_GREY);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2i(0, 0);
    do
    {
        glVertex2i(intround(70.0 * cos(a)), intround(-70.0 * sin(a)));
        a = a + DEG5;
    } while (!(fabs(a - e) <= DEG5));
    glVertex2i(intround(70.0 * cos(e)), intround(-70.0 * sin(e)));
    glEnd();

    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(2.0);
    x = intround(70.0 * cos(e));
    y = intround(-70.0 * sin(e));
    Glib_Draw(0, 0, x, y);

    glLineWidth(4.0);
    DrawArc(162, 360, 70, 20);
    Mark(10, 162, 162, 70, 80);

    glLineWidth(2.0);
    DrawRectangle(2, 2, 50, 26);

    glLineWidth(1.0);
    Mark(220, 162, 338, 64, 70);
    SmallDigits(2, 18, 2, -22, -22, 60);

    digits = intround(AeroLink_EngPkt.Engines[EngineNumber].Epr * 100.0);
    if (digits < 0)
    {
        digits = 0;
    }
    d1 = (unsigned char) (digits / 100) + '0';
    d2 = (unsigned char) (digits / 10 % 10) + '0';
    d3 = (unsigned char) (digits % 10) + '0';
    Glib_SetFont(Glib_GFONT16, 12);
    glLineWidth(2.0);
    Glib_AntiAliasing(true);
    Glib_Char(d1, 10, 7);
    Glib_Char('.', 16, 7);
    Glib_Char(d2, 26, 7);
    Glib_Char(d3, 38, 7);

    Glib_SetFont(Glib_GFONT12, 10);
    Glib_Colour(Glib_CYAN);
    Glib_Chars("EPR", 55, -65);
}

/* ------------------------------------------------ */
void EICAS_RpmGauge(unsigned int EngineNumber)
{
    int   x;
    int   y;
    float r;
    float a;

    int   digits;
    char  d1;
    char  d2;
    char  d3;
    char  d4;

    r = Maths_Rads(AeroLink_EngPkt.Engines[EngineNumber].Rpm * 1.8);
    if (r < 0.0)
    {
        r = 0.0;
    }
    else if (r > DEG216)
    {
        r = DEG216;
    }

    Glib_AntiAliasing(true);

    a = 0.0;
    Glib_Colour(Glib_GREY);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2i(0, 0);
    do
    {
        glVertex2i(intround(70.0 * cos(a)), intround(-70.0 * sin(a)));
        a = a + DEG5;
    } while (!(fabs(a - r) <= DEG5));
    glVertex2i(intround(70.0 * cos(r)), intround(-70.0 * sin(r)));
    glEnd();

    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(2.0);
    x = intround(70.0 * cos(r));
    y = intround(-70.0 * sin(r));
    Glib_Draw(0, 0, x, y);

    glLineWidth(3.0);
    DrawArc(144, 360, 70, 20);
    Mark(10, 144, 144, 70, 80);

    glLineWidth(2.0);
    DrawRectangle(2, 2, 60, 26);

    glLineWidth(1.0);
    Mark(360, 180, 324, 64, 70);
    SmallDigits(2, 10, 2, -36, -36, 60);

    digits = intround(AeroLink_EngPkt.Engines[EngineNumber].Rpm * 10.0);
    if (digits < 0)
    {
        digits = 0;
    }
    d1 = (unsigned char) (digits / 1000) + '0';
    d2 = (unsigned char) (digits / 100 % 10) + '0';
    d3 = (unsigned char) (digits / 10 % 10) + '0';
    d4 = (unsigned char) (digits % 10) + '0';
    Glib_SetFont(Glib_GFONT16, 12);
    glLineWidth(2.0);
    Glib_AntiAliasing(true);
    if (d1 != '0')
    {
        Glib_Char(d1, 10, 7);
    }
    Glib_Char(d2, 22, 7);
    Glib_Char(d3, 34, 7);
    Glib_Char('.', 40, 7);
    Glib_Char(d4, 50, 7);

    Glib_SetFont(Glib_GFONT12, 10);
    Glib_Colour(Glib_CYAN);
    Glib_Chars("N1", 55, -65);
}

/* ------------------------------------------------ */
void EICAS_EgtGauge(unsigned int EngineNumber)
{
    int   x;
    int   y;
    float t;
    float a;

    int   digits;
    char  d1;
    char  d2;
    char  d3;
    char  d4;

    t = Maths_Rads(AeroLink_EngPkt.Engines[EngineNumber].Egt * 0.2);
    if (t < 0.0)
    {
        t = 0.0;
    }
    else if (t > DEG200)
    {
        t = DEG200;
    }

    Glib_AntiAliasing(true);

    a = 0.0;
    Glib_Colour(Glib_GREY);
    glBegin(GL_TRIANGLE_FAN);
    glVertex2i(0, 0);
    do
    {
        glVertex2i(intround(70.0 * cos(a)), intround(-70.0 * sin(a)));
        a = a + DEG5;
    } while (!(fabs(a - t) <= DEG5));
    glVertex2i(intround(70.0 * cos(t)), intround(-70.0 * sin(t)));
    glEnd();

    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(2.0);
    x = intround(70.0 * cos(t));
    y = intround(-70.0 * sin(t));
    Glib_Draw(0, 0, x, y);

    glLineWidth(3.0);
    DrawArc(160, 360, 70, 20);
    Mark(10, 160, 160, 70, 80);

    glLineWidth(2.0);
    DrawRectangle(2, 2, 60, 26);

    glLineWidth(1.0);
    digits = intround(AeroLink_EngPkt.Engines[EngineNumber].Egt * 10.0);
    if (digits < 0)
    {
        digits = 0;
    }
    d1 = (unsigned char) (digits / 1000) + '0';
    d2 = (unsigned char) (digits / 100 % 10) + '0';
    d3 = (unsigned char) (digits / 10 % 10) + '0';
    d4 = (unsigned char) digits % 10 + '0';
    Glib_SetFont(Glib_GFONT16, 12);
    glLineWidth(2.0);
    Glib_AntiAliasing(true);
    if (d1 != '0')
    {
        Glib_Char(d1, 10, 7);
    }
    Glib_Char(d2, 22, 7);
    Glib_Char(d3, 34, 7);
    Glib_Char('.', 40, 7);
    Glib_Char(d4, 50, 7);

    Glib_SetFont(Glib_GFONT12, 10);
    Glib_Colour(Glib_CYAN);
    Glib_Chars("EGT", 55, -65);
}

/* ------------------------------------------------ */
void EICAS_ParkBrake(IODefn_SwitchPosition brake)
{
    if (brake == IODefn_On)
	{
        glLineWidth(2.0);
        Glib_SetFont(Glib_GFONT16, 10);
        Glib_Colour(Glib_GREEN);
        Glib_SetFont(Glib_GFONT16, 10);
		Box(0, 0, 57, 50);
		Glib_Chars("PARK", 10, 30);
		Glib_Chars("BRAKE", 5, 10);
	}
}

/* ------------------------------------------------ */
void EICAS_DisplayGear(float GearPosition)
{
    GearState Gear;

    glLineWidth(2.0);
    if (GearPosition <= 0.0)
    {
        Gear = Up;
    }
    else if (GearPosition >= 1.0)
    {
        Gear = Down;
    }
    else
    {
        Gear = Transit;
    }
    if (Gear != Up)
    {
        Glib_SetFont(Glib_GFONT16, 10);
        if (Gear == Down)
        {
            Glib_Colour(Glib_GREEN);
        }
        else
        {
            Glib_Colour(Glib_YELLOW);
        }
        Box(0, 0, 44, 24);
        Glib_Chars("DOWN", 4, 4);
        Glib_Colour(Glib_BLUE);
        Glib_SetFont(Glib_GFONT12, 10);
        Glib_Chars("GEAR", 5, -17);
    }
}

/* ------------------------------------------------ */
void EICAS_FlapsIndicator(float FlapPosition, unsigned int FlapSetting)
{
    int f;
    int i;
    int dx;

    glLineWidth(2.0);
    if (FlapSetting > 30)
    {
        FlapSetting = 30;
    }
    Glib_SetFont(Glib_GFONT12, 8);
    if (FlapPosition > 0.001 || FlapSetting != 0)
    {
        Glib_Colour(Glib_WHITE);
        Box(0, 0, 14, 112);
        Glib_Colour(Glib_BLUE);
        Glib_Char('F', -14, 90);
        Glib_Char('L', -14, 74);
        Glib_Char('A', -14, 58);
        Glib_Char('P', -14, 42);
        Glib_Char('S', -14, 26);
    }
    else
    {
        return;
    }
    Glib_Colour(Glib_WHITE);
    f = intround(FlapPosition * 112.0);
    if (f < 0)
    {
        f = 0;
    }

    for (i = 0; i <= f; i += 1)
    {
        Glib_Draw(1, 112 - i, 13, 112 - i);
    }
    f = intround(112.0 - (float) (FlapSetting) * 112.0 / 30.0);
    Glib_Colour(Glib_MAGENTA);
    Glib_Draw(-5, f, 19, f);
    dx = 25;
    if (FlapSetting > 5)
    {
        Glib_Char(FlapSetting / 10 + '0', dx, f - 6);
        dx = dx + 8;
    }
    Glib_Char(FlapSetting % 10 + '0', dx, f - 6);
}

/* ------------------------------------------------ */
static void Box(int x, int y, int xs, int ys)
{
    Glib_Draw(x, y, x + xs, y);
    Glib_Draw(x + xs, y, x + xs, y + ys);
    Glib_Draw(x + xs, y + ys, x, y + ys);
    Glib_Draw(x, y + ys, x, y);
}

/* ------------------------------------------------ */
void BEGIN_EICAS()
{
}
