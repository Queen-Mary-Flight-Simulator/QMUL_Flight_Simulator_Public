#include <stdio.h>

#include <SIM/clocks.h>
#include <SIM/glib.h>

#include "pfd.h"
#include "clock.h"

void Clock_Clock(int x0, int y0, int dial, int needle, int seconds)
{
    float nhour_start = 0.0f; // dial marking starting angle
    float nmin_start  = 0.0f;
    float nhour_rot   = 0.0f;
    float nmin_rot    = 0.0f;
    float seconds_rot = 0.0f;

    //printf("Clocks_ClockHours : %d Clocks_ClockMins : %d\n", Clocks_ClockHours, Clocks_ClockMins);
    
    // Dial shows : marks begin at 0.0 degrees
    // Units : Major units = 30 degrees. 1 min = 6 deg for min hand. 1 hour = 30 degress for hr hand

    nhour_rot = nhour_start + -(float)( Clocks_ClockHours*30 + Clocks_ClockMins/2);
    nmin_rot  = nmin_start  + -(float)(Clocks_ClockMins*6);
    seconds_rot = -(float) (Clocks_TimerSecs * 6.0);
  
    Glib_SetTexture(dial);
	Glib_DrawTexture(x0 - 96, y0 - 96, 192, 192, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(needle);
	Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, nmin_rot, 1.0);
	Glib_DrawTextureRotated(x0, y0, 96, 96, 0.0, 0.0, 1.0, 1.0, nhour_rot, 1.0);
    Glib_SetTexture(needle);
	Glib_DrawTextureRotated(x0, y0, 150, 150, 0.0, 0.0, 1.0, 1.0, seconds_rot, 1.0);
}

void BEGIN_Clock()
{
}
