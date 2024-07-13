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

#include <GLFW/glfw3.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "vsi.h"
#include "ai.h"
#include "systems.h"

#define FeetPerMin 196.8504

static void Wrn3(int d, int x, int y);

/* ------------------------------------------ */
void Vsi_Vsi(int VsiX, int VsiY, float VerticalSpeed)
{
    int   y;
    float fpm;
    int   intfpm;

    Glib_LoadIdentity();
    Glib_Translate((float) VsiX, (float) VsiY);
	
    if (Systems_Failures[16])
	{
	    Glib_SetFont(Glib_EFONT12, 10);
	    Ai_VBox("VERT", 40, 0);
		return;
	}
	
    Glib_DrawTexture(-15, -155, 49, 310, 0.5-0.015, 0.2-0.155, 0.5+0.034, 0.2+0.155, 1.0); 

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
    Glib_Colour(Glib_WHITE);
	Glib_Draw(33, 0, 5, y);
    Glib_RemoveClipWindow();

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

/* ------------------------------------------ */
static void Wrn3(int d, int x, int y)
{
    char Str[4];

    Str[0] = (unsigned char) (d / 100) + '0';
    d      = d % 100;
    Str[1] = (unsigned char) (d / 10) + '0';
    Str[2] = (unsigned int) (d % 10) + '0';
    Str[3] = '\0';
    Glib_Chars(Str, x, y);
}

/* ------------------------------------------ */
void BEGIN_Vsi()
{
}
