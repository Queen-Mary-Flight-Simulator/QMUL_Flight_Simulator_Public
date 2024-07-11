/* +------------------------------+---------------------------------+
   | Module      : egt.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-04      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/c172        |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Light aircraft EGT                               |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>
#include <stdio.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "egt.h"

void Egt_Egt(int x0, int y0, float Egt, GLuint dial, GLuint needle)
{
    float e;

    Egt = Maths_Limit(Egt, 700.0, 1700.0);
    e = 50.0 - (Egt - 700.0) * 0.1;

    Glib_SetTexture(dial);
    Glib_DrawTexture(x0 - 128, y0 - 128, 256.0, 256.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(needle);
	Glib_DrawTextureRotated(x0, y0, 150.0, 150.0, 0.0, 0.0, 1.0, 1.0, e, 1.0);
}


void BEGIN_Egt()
{
}
