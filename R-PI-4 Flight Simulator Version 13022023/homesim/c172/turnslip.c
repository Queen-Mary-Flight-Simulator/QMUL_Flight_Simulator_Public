#include <GL/gl.h>
#include <math.h>

#include <SIM/maths.h>
#include <SIM/weather.h>
#include <SIM/glib.h>

#include "aero.h"
#include "model.h"
#include "iolib.h"
#include "turnslip.h"

#define ONERAD (180.0f / M_PI)
#define DEG60  (60.0/ONERAD)

void TurnSlip_TurnSlip(int x0, int y0, float turnrate, int dial, int needle, int slipball, int overlay)
{
    float t = 6.66666667 * turnrate; /* 3 deg/sec -> 20 deg */
    float needle_start = 0.0f; // dial marking starting angle
    float needle_rot = 0.0f;
    float f = 0.0f;
    static float xsdot = 0.0f; 
    static float xs = 0.0f;
    float x;

    if (Model_OnTheGround) 
	{
        f = Model_R * Model_U;
    } 
	else 
	{
        f = Model_SideForce / Aero_Mass;
    }

    xsdot = Maths_Integrate(xsdot, -65.4 * xs - f );
    xs = Maths_Integrate(xs, xsdot - 16.2 * xs );
  
    if (xs < -0.01848) 
	{
        xs = -0.01848;
        xsdot = 16.2 * xs;
    } 
	else if (xs > 0.01848) 
	{
        xs = 0.01848;
        xsdot = 16.2 * xs;
    }
    x = 3600.0 * xs;

    t = Maths_Limit(t, -DEG60, DEG60);
    needle_rot = needle_start - Maths_Degrees(t); 

    Glib_SetTexture(dial);
	Glib_DrawTexture(x0-160, y0-160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(slipball);
	Glib_DrawTexture(x0 -18 + (int) x, y0 -18 - 57 + (int) (x * x * (6.0/(66.5*66.5))), 35, 35, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(overlay);
	Glib_DrawTexture(x0-160, y0-160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);
	Glib_SetTexture(needle);
	Glib_DrawTextureRotated(x0, y0, 192, 192, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);

    if (IOLib_GetMasterSwitch() == IODefn_On) 
    {
        Glib_Colour(Glib_GREEN);
    }
    else 
	{
        Glib_Colour(Glib_RED);
    }
    Glib_Rectangle(x0 + 64, y0 + 32, 13, 18);  
}

void BEGIN_TurnSlip()
{
}
