/* +------------------------------+---------------------------------+
   | Module      : vsi.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-15      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EFIS Vertical Speed Indicator     |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <GL/gl.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "vsi.h"
#include "systems.h"

static int VsiBox1[12] = {
    0, 38, -15, 53, -15, 155, 4, 155, 34, 72, 34, 38
};

static int VsiBox2[12] = {
    0, -38, -15, -53, -15, -155, 4, -155, 34, -72, 34, -38
};

static int VsiBox3[8] = {
    0, 38, 34, 38, 34, -38, 0, -38
};

#define FeetPerMin    196.8504

static void Wrn3(int d, int x, int y);

void Vsi_Vsi(int VsiX, int VsiY, float VerticalSpeed)
{
    int   y;
    float fpm;
    int   intfpm;

    if (Systems_Failures[16])
	{
	    Glib_SetFont(Glib_GFONT12, 10);
	    Glib_VBox("VERT", 40, 0);
		return;
	}
	
    Glib_Colour(Glib_GREY);
    glLineWidth(2.0);
    Glib_DrawPolygon(6, 0, 0, VsiBox1);
    Glib_DrawPolygon(6, 0, 0, VsiBox2);
    Glib_DrawPolygon(4, 0, 0, VsiBox3);

    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_GFONT12, 10);

    Glib_Draw(2, 0, 12, 0);
    Glib_Draw(0, -32, 5, -32);
    Glib_Draw(0, -64, 5, -64);
    Glib_Draw(0, -87, 5, -87);
    Glib_Draw(0, -110, 5, -110);
    Glib_Draw(0, -127, 5, -127);
    Glib_Draw(0, -144, 5, -144);

    Glib_Draw(0, 32, 5, 32);
    Glib_Draw(0, 64, 5, 64);
    Glib_Draw(0, 87, 5, 87);
    Glib_Draw(0, 110, 5, 110);
    Glib_Draw(0, 127, 5, 127);
    Glib_Draw(0, 144, 5, 144);

    Glib_Char('1', -8, 58);
    Glib_Char('1', -8, -70);
    Glib_Char('2', -8, 104);
    Glib_Char('2', -8, -116);
    Glib_Char('6', -8, 138);
    Glib_Char('6', -8, -150);

    fpm = -VerticalSpeed * FeetPerMin;

    if (fpm < -6000.0)
    {
        fpm = -6000.0;
    }
    else if (fpm > 6000.0)
    {
        fpm = 6000.0;
    }

    intfpm = intround(fpm);

    if (fpm >= -1000.0 && fpm <= 1000.0)
    {
        y = intround(fpm * 0.064);
    }
    else if (fpm >= 1000.0 && fpm < 2000.0)
    {
        y = intround(64.0 + (fpm - 1000.0) * 0.046);
    }
    else if (fpm >= 2000.0)
    {
        y = intround(110.0 + (fpm - 2000.0) * 0.0085);
    }
    else if (fpm <= -1000.0 && fpm >= -2000.0)
    {
        y = intround(-64.0 + (fpm + 1000.0) * 0.046);
    }
    else
    {
        y = intround(-110.0 + (fpm + 2000.0) * 0.0085);
    }

    Glib_ClipWindow(VsiX - 15, VsiY - 155, 50, 310);
    Glib_Draw(33, 0, 5, y);
    glDisable(GL_SCISSOR_TEST);

    if (intfpm > 400)
    {
        if (intfpm >= 1000)
        {
            Glib_Char((unsigned char) (intfpm / 1000 + '0'), -15, 160);
        }
        intfpm = intfpm % 1000;
        Wrn3((intfpm / 100) * 100, -5, 160);
    }
    else if (intfpm < -400)
    {
        intfpm = -intfpm;
        if (intfpm >= 1000)
        {
            Glib_Char((unsigned char) (intfpm / 1000 + '0'), -15, -160 - 12);
        }
        intfpm = intfpm % 1000;
        Wrn3((intfpm / 100) * 100, -5, -160 - 12);
    }
}

static void Wrn3(int d, int x, int y)
{
    char Str[4];

    Str[0] = (unsigned char) (d / 100) + '0';
    d      = d % 100;
    Str[1] = (unsigned char) (d / 10) + '0';
    Str[2] = (unsigned int) (d % 10) + '0';
    Str[3] = 0;
    Glib_Chars(Str, x, y);
}

void BEGIN_Vsi()
{
}
