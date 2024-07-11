/* +------------------------------+---------------------------------+
   | Module      : asi.c          | Version         : 1.1           | 
   | Last Edit   : 31-10-2020     | Reference Number: 02-01-05      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/homesim/pfd/c172                            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows10                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : C172 Airspeed Indicator                          |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <SIM/glib.h>

#include "asi.h"

/* ---------------------------------------------------- */
void Asi_Asi(int x0, int y0, float IAS, int dial, int needle)
{
    float Kts;
    int   MinSpeed = 30.0f;
    int   MaxSpeed = 180.0f;
    float needle_start = 20.0f; /* dial marking starting angle */
    float needle_rot = 0.0f;
  
    Kts = IAS * 1.944;
    if (Kts < 30.0) 
    {
        Kts = 30.0;
    }
    if (Kts < MinSpeed)
    {
        Kts = MinSpeed;
    }
    else if(Kts > MaxSpeed) 
    {
        Kts = MaxSpeed;
    }
  
    // Dial shows 1 kt = 0.5 degrees. 90 kts = 180 degrees.
    needle_rot = -(needle_start) - (Kts - MinSpeed) * 2.0f; 

    Glib_SetTexture(dial);
    Glib_DrawTexture(x0 - 160, y0 - 160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(needle);
    Glib_DrawTextureRotated(x0, y0, 235, 235, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
}

/* ---------------------------------------------------- */
void BEGIN_Asi()
{
}
