#include <stdio.h>
#include <math.h>

#include <SIM/maths.h>
#include <SIM/iodefn.h>
#include <SIM/glib.h>

#include "pfd.h"
#include "iolib.h"
#include "enginegauges_left.h"

/* ---------------------------------------- */
extern void EngineGauges_Ammeter(int x0, int y0, float rpm, int gauge, int needle)
{
    float needle_rot = 0.0f;
    float a = 0.0;
  
    if(rpm < 500.0)
    {
        a = 0.0f;
    }
    else if (rpm > 1500.0)
    {
        a = -20.0f;
    }
    else 
    {
        a = 0.0 - (rpm - 500.0) * 0.02;
    }

    if (IOLib_GetMasterSwitch() == IODefn_On)
    {
        a = a + 5.0;
    }

    needle_rot = a;

    Glib_SetTexture(gauge);
	Glib_DrawTexture(x0-64, y0-64, 128, 128, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_ClipWindow(PFD_AmmeterX-64, PFD_AmmeterY+24, 128, 32);
    Glib_SetTexture(needle);
    Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
    Glib_RemoveClipWindow();  
}

/* ---------------------------------------- */
extern void EngineGauges_Suction(int x0, int y0, float rpm, int gauge, int needle)
{
    float needle_rot = 0.0f;
    float g=0.0;
  
    if (rpm < 500.0)
    {
        g = 30.0f;
    }
    else if (rpm > 1500.0)
    {
        g = -20.0f;
    }
    else 
    {
        g = 30.0 - (rpm - 500.0) * 0.05;
    }

    needle_rot = g;

    Glib_SetTexture(gauge);
	Glib_DrawTexture(x0-64, y0-64, 128, 128, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_ClipWindow(PFD_SuctionX-64, PFD_SuctionY+24, 128, 32);
    Glib_SetTexture(needle);
    Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
    Glib_RemoveClipWindow();  
}

/* ---------------------------------------- */
void BEGIN_EngineGauges_left()
{
}
