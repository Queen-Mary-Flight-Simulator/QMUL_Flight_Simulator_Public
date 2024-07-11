#include <GL/gl.h>
#include <GL/glu.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "nfd.h"
#include "vor.h"

#define NoFlag      0
#define OffFlag     1
#define ToFlag      2
#define FromFlag    3

#define ONERAD      (180.0f / M_PI)
#define DEG85       (85.0 / ONERAD)
#define DEG95       (95.0 / ONERAD)

static int OldVptr = 0;
static int OldHptr = 0;

void Vor_Vor(int x0, int y0, float Localiser, float GlideSlope, int Crs, bool ILSMode, bool Status,
             GLuint texobj1, GLuint texobj2, GLuint texobs)
/*  Localiser   bearing to station (rads)
    GlideSlope  G/S error (rads)
    ILSMode     TO/FROM flag (omitted in ILS mode)
    Status      freq select, in range etc
 */

{
    bool NavFlag;
    bool GSFlag;
    int  VorFlag;
    int  Hptr;
    int  Vptr;
    int  Radial;

	Radial = Crs / 10;

    NavFlag = Status;
    GSFlag  = Status && ILSMode;
    VorFlag = OffFlag;
    Vptr    = 0;
    Hptr    = 0;
	
    if (ILSMode)
    {
        if (NavFlag || GSFlag)
        {
            VorFlag = NoFlag;
        }
        if (GSFlag)
        {
            Hptr = (int) (GlideSlope * 1227.765);
        }
        if (NavFlag)
        {
            Vptr = (int) (Localiser * 1375.099); /* 60 pix = 2.5 deg ILS */
        }
	}
    else if (NavFlag)
    {
        if ((Localiser >= -DEG85) && (Localiser <= DEG85))
        {
                VorFlag = ToFlag;
        }
        else if (Localiser < -DEG95)
        {
            Localiser = -M_PI - Localiser;
			VorFlag = FromFlag;
        }
        else if (Localiser > DEG95)
        {
            Localiser = M_PI - Localiser;
			VorFlag = FromFlag;
        }
        Vptr = (int) (Localiser * 343.775); /* 60 pix = 10 deg VOR */
		GSFlag = false;
    }

    if (Vptr > OldVptr)
    {
        Vptr = OldVptr + 1;
    }
    else if (Vptr < OldVptr)
    {
        Vptr = OldVptr - 1;
    }

    if (Hptr > OldHptr)
    {
        Hptr = OldHptr + 1;
    }
    else if (Hptr < OldHptr)
    {
        Hptr = OldHptr - 1;
    }

    if (Vptr < -60)
    {
        Vptr = -60;
    }
    else if (Vptr > 60)
    {
        Vptr = 60;
    }

    if (Hptr < -60)
    {
        Hptr = -60;
    }
    else if (Hptr > 60)
    {
        Hptr = 60;
    }

    OldVptr = Vptr;
    OldHptr = Hptr;

    Glib_SetTexture(texobj1);
    Glib_DrawTexture(x0 - 128, y0 - 128, 256, 256, 0.0, 0.0, 1.0, 1.0, 1.0);
	Glib_SetTexture(texobj2);
    Glib_DrawTextureRotated(x0, y0, 256, 256, 0.0, 0.0, 1.0, 1.0, (float) Radial, 1.0);
	Glib_SetTexture(texobs);
    Glib_DrawTextureRotated(x0 + 110, y0 - 110, 64, 64, 0.0, 0.0, 1.0, 1.0, (float) -Crs / 5.0, 1.0);

    Glib_PushMatrix();
	Glib_Translate(x0, y0);
    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(3.0);
    Glib_Draw(Vptr, -65, Vptr, 65);
    Glib_Draw(-65, Hptr, 65, Hptr);

    Glib_LineWidth(1.0);
    if (VorFlag == ToFlag)
    {
        Glib_Triangle(47-8, 25+0, 47+0, 25+8, 47+8, 25+0);
    }
    else if (VorFlag == FromFlag)
    {
        Glib_Triangle(47-8, -25+0, 47+0, -25-8, 47+8, -25+0);
    }

    if (!GSFlag)
    {
        Glib_Colour(Glib_RED);
        Glib_Rectangle(-60, -10, 25, 20);
        Glib_Colour(Glib_BLACK);
        Glib_SetFont(1, 10);
        Glib_Chars("LS", -55, -5);
    }

    if (!NavFlag)
    {
        Glib_Colour(Glib_RED);
        Glib_Rectangle(15, 15, 15, 50);
        Glib_Colour(Glib_BLACK);
        Glib_SetFont(1, 10);
        Glib_Char('N', 18, 50);
        Glib_Char('A', 18, 35);
        Glib_Char('V', 18, 20);
    }
    Glib_PopMatrix();
}

void BEGIN_Vor()
{
    OldHptr = 0;
    OldVptr = 0;
}
