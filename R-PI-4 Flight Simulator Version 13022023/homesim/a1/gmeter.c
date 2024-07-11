#include <stdio.h>

#include <SIM/glib.h>

#include "gmeter.h"

void GMeter_GMeter(int x0, int y0, float g, float gmin, float gmax, int dial, int needle)
{
    float needle_start = 230.0f; // dial marking starting angle
    float needle_rot   = 0.0f;

    if (g > 5.0) 
    {
        g = 5.0;
    } 
    else if (g < -3.0) 
    {
        g = -3.0;
    }
    // Dial shows : marks begin at 230.0 degrees
    // Units : Major units = 40 degrees. 1 G = 40 degrees
    needle_rot = -(needle_start) - (g * 40.0);

    Glib_SetTexture(dial);
    Glib_DrawTexture(x0 - 128, y0 - 128, 256, 256, 0.0, 0.0, 1.0, 1.0, 1.0);
	Glib_SetTexture(needle);
    Glib_DrawTextureRotated(x0, y0, 192, 192, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
    needle_rot = -(needle_start) + -(gmin * 40.0);
    Glib_DrawTextureRotated(x0, y0, 192, 192, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
    needle_rot = -(needle_start) + -(gmax * 40.0);
    Glib_DrawTextureRotated(x0, y0, 192, 192, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
}

void BEGIN_GMeter()
{
}
