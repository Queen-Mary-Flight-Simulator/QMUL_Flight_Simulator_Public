/* +------------------------------+---------------------------------+
   | Module      : alt.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-01      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/c172        |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Light aircraft altimeter                         |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdbool.h>
#include <GL/gl.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "nfd.h"
#include "alt.h"
#include "nav.h"

void Alt_Altimeter(int x0, int y0, float z, unsigned int Baro, bool BaroHg, unsigned int BaroMode,
                   GLuint texobj_dial, GLuint texobj_dial_press, GLuint texobj_needle,
                   GLuint texobj_needle_hour, GLuint texobj_baro)
{
    float Altitude;
	float dAltitude;  /* QNH offset */
	
    float n100_start  = 0.0f;
    float n1000_start = 0.0f;
    float n100_rot    = 0.0f;
    float n1000_rot   = 0.0f;
    float hund        = 0.0f;
    float thous       = 0.0f;
    float baro_rot    = 0.0f;
    float knob_rot    = 0.0f;

    dAltitude = (float) ((int) Baro - (int) Nav_RegionalQNH * 10) * 3.0;   /* 30 ft per mb */	
    if (z <= 0.0)
    {
        Altitude = 0.0;
    }
    else
    {
        Altitude = z * 3.280840 + dAltitude;
    }

    hund  = Altitude / 100.0f;
    thous = Altitude / 1000.0f;

    // Dial shows : marks begin at 0.0 degrees
    // Major marks = 36 degrees
    n100_rot  = n100_start + -(hund * 36);
    n1000_rot = n1000_start + -(thous * 36);

    baro_rot = 3.0 * ((int) Baro / 10 - 1000);
    knob_rot = ((float) Baro - 9050.0) * 360.0 / 1000;
	
    Glib_SetTexture(texobj_dial_press);
    Glib_DrawTextureRotated(x0, y0, 320, 320, 0.0, 0.0, 1.0, 1.0, baro_rot, 1.0);
	Glib_SetTexture(texobj_dial);
    Glib_DrawTexture(x0 - 160, y0 - 160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);
	Glib_SetTexture(texobj_needle);
    Glib_DrawTextureRotated(x0, y0, 235, 235, 0.0, 0.0, 1.0, 1.0, n100_rot, 1.0);
	Glib_SetTexture(texobj_needle_hour);
    Glib_DrawTextureRotated(x0, y0, 180, 180, 0.0, 0.0, 1.0, 1.0, n1000_rot, 1.0);
	Glib_SetTexture(texobj_baro);
    Glib_DrawTextureRotated(x0 + 130, y0 - 130, 64, 64, 0.0, 0.0, 1.0, 1.0, -knob_rot, 1.0);
}

void BEGIN_Alt()
{
}
