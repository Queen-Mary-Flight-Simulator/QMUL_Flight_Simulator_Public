#include <math.h>
#include <stdio.h>

#include <SIM/maths.h>
#include <SIM/glib.h>

#include "rpm.h"

void Rpm_Rpm(int x0, int y0, float Rpm, int dial, int needle)
{
    float needle_start = -150.0f; // dial marking starting angle
    float needle_rot   = 0.0f;
 
    // Rpm Units??? Value * 100 ? * 10?
    //printf("Rpm : %f\n", Rpm);
 
    Rpm = Maths_Limit(Rpm, 0.0, 3000.0);
 
    // Dial shows : marks begin at -150.0 degrees
    // Units : Major units = 50 degrees. 500 rpm = 50 deg. 10 rpm = 1 deg
    needle_rot = -needle_start - Rpm / 10.0;

    Glib_SetTexture(dial);
	Glib_DrawTexture(x0 - 128, y0 - 128, 256, 256, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(needle);
    Glib_DrawTextureRotated(x0, y0, 192, 192, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
}

void BEGIN_Rpm()
{
}
