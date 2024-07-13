/* +------------------------------+---------------------------------+
   | Module      : compass.c      | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-07      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EFIS Compass                      |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <GL/gl.h>
#include <math.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "compass.h"
#include "systems.h"

typedef struct
{
    int x;
    int y;
} Vertex;

#define minY            145
#define radius          201
#define ScaleFactor     1.574
#define DEG5            (5.0 / Maths_ONERAD)
#define DEG33           (33.0 / Maths_ONERAD)

#define AiHeight        324
#define AiWidth         302
#define AiHalfWidth     (AiWidth / 2)
#define AiHalfHeight    (AiHeight / 2)

static Vertex HdgBugPtr[7] = {
    { -10, radius      }, { -10, radius + 10 }, { -7, radius + 10 }, { 0, radius },
    {   7, radius + 10 }, {  10, radius + 10 }, { 10, radius      }
};

static void Wrn3(int d, int x, int y);

void Compass_PFD_Compass(int CompassX, int CompassY, float Hdg, int HdgBug, float Trk)
{
    int          x;
    float        t;
    float        offset;
    unsigned int MinHdg;
    unsigned int MaxHdg;
    int          IntHdg;
    unsigned int h1;
    unsigned int h2;
    float        Bug;
    unsigned int i;

    if (Systems_Failures[19])
	{
	    Glib_SetFont(Glib_GFONT12, 10);
	    Glib_HBox("HDG", 0, radius - 20);
		return;
	}
	
    Glib_ClipWindow(CompassX - AiHalfWidth, CompassY + radius - 80, AiWidth, 100);
    Glib_Colour(Glib_GREY);
    glLineWidth(2.0);

    glBegin(GL_POLYGON);
    for (x = -35; x <= 35; x += 2)
    {
        glVertex2f(-((float) radius * sin(Maths_Rads((float) x * ScaleFactor))),
                   ((float) radius * cos(Maths_Rads((float) x * ScaleFactor))));
    }
    glEnd();
    Glib_Colour(Glib_WHITE);
    Glib_Draw(0, radius + 1, -9, radius + 1 + 16);
    Glib_Draw(-9, radius + 1 + 16, 9, radius + 1 + 16);
    Glib_Draw(9, radius + 1 + 16, 0, radius + 1);

    if (Hdg < 0.0)
    {
        IntHdg = 360 - intround(Maths_Degrees(-Hdg));
    }
    else
    {
        IntHdg = intround(Maths_Degrees(Hdg));
    }

    MinHdg = IntHdg / 10 * 10;
    if (MinHdg >= 40)
    {
        MinHdg = MinHdg - 35;
    }
    else
    {
        MinHdg = MinHdg + 325;
    }
    MaxHdg = MinHdg + 80;
    if (MaxHdg > 360)
    {
        MaxHdg = MaxHdg - 360;
    }

    offset = Maths_Rads((float) MinHdg) - Hdg;
    Maths_Normalise(&offset);
    h1 = MinHdg;
    h2 = MaxHdg;

    do
    {
        glPushMatrix();
        t = offset * ScaleFactor;
        glRotatef(Maths_Degrees(-t), 0.0, 0.0, 1.0);
        if (h1 % 10 == 0)
        {
            Glib_Draw(0, radius - 10, 0, radius);
            if (h1 % 30 == 0)
            {
                Glib_SetFont(Glib_GFONT16, 12);
                if (h1 > 90)
                {
                    Glib_Char(h1 / 100 + '0', -10, radius - 10 - 20);
                    Glib_Char(h1 / 10 % 10 + '0', 2, radius - 10 - 20);
                }
                else
                {
                    Glib_Char(h1 / 10 % 10 + '0', -5, radius - 10 - 20);
                }
            }
            else
            {
                Glib_SetFont(Glib_GFONT12, 8);
                if (h1 > 90)
                {
                    Glib_Char(h1 / 100 + '0', -8, radius - 10 - 16);
                    Glib_Char(h1 / 10 % 10 + '0', 2, radius - 10 - 16);
                }
                else
                {
                    Glib_Char(h1 / 10 % 10 + '0', -4, radius - 10 - 16);
                }
            }
        }
        else
        {
            Glib_Draw(0, radius - 5, 0, radius);
        }
        offset = offset + DEG5;
        if (h1 >= 360)
        {
            h1 = 5;
        }
        else
        {
            h1 = h1 + 5;
        }
        glRotatef(t, 0.0, 0.0, 1.0);
        glPopMatrix();
    } while (!(h1 == h2));

    t = Trk - Hdg;
    Maths_Normalise(&t);
    Glib_Colour(Glib_WHITE);

    glPushMatrix();
    glRotatef(Maths_Degrees(-t * ScaleFactor), 0.0, 0.0, 1.0);
    Glib_Draw(0, 0, 0, radius - 1);
    Glib_Draw(-4, radius - 57, 4, radius - 57);
    glRotatef(Maths_Degrees(t * ScaleFactor), 0.0, 0.0, 1.0);
    glPopMatrix();

    Bug = Hdg - Maths_Rads((float) HdgBug);
    Maths_Normalise(&Bug);
    if (Bug < -DEG33)
    {
        Bug = -DEG33;
    }
    else if (Bug > DEG33)
    {
        Bug = DEG33;
    }
    Glib_ClipWindow(CompassX - AiHalfWidth - 20, CompassY + radius - 80, AiWidth + 40, 100);
    Glib_Colour(Glib_MAGENTA);

    glPushMatrix();
    glRotatef(Maths_Degrees(Bug * ScaleFactor), 0.0, 0.0, 1.0);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i <= 6; i += 1)
    {
        glVertex2i(HdgBugPtr[i].x, HdgBugPtr[i].y);
    }
    glEnd();
    glRotatef(Maths_Degrees(Bug * ScaleFactor), 0.0, 0.0, 1.0);
    glPopMatrix();

    glDisable(GL_SCISSOR_TEST);
    Glib_Colour(Glib_MAGENTA);
    Glib_SetFont(Glib_GFONT12, 10);
    Wrn3(HdgBug, -68, radius - 80 + 5);
    Glib_Colour(Glib_GREEN);
    Glib_Chars("MAG", 45, radius - 80 + 5);
}

static void Wrn3(int d, int x, int y)
{
    char Str[4];

    Str[0] = (unsigned char) (d / 100) + '0';
    d      = d % 100;
    Str[1] = (unsigned char) (d / 10) + '0';
    Str[2] = (unsigned char) (d % 10) + '0';
    Str[3] = 0;
    Glib_Chars(Str, x, y);
}

void BEGIN_PFD_Compass()
{
}
