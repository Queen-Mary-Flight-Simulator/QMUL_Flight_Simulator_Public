#include <GL/gl.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "nfd.h"
#include "vsi.h"

#define FeetPerMin    196.8504

void Vsi_Vsi(int x0, int y0, float VerticalSpeed, GLuint texobj_dial, GLuint texobj_needle)
{
    float fpm          = 0.0f;
    float needle_start = 270.0f; // dial marking starting angle
    float needle_rot   = 0.0f;

    fpm = -VerticalSpeed * FeetPerMin;
    if (fpm < -2000.0) // was 6000, but this is for Cranfield A1
    {
        fpm = -2000.0;
    }
    else if (fpm > 2000.0)
    {
        fpm = 2000.0;
    }

    // Dial shows : marks begin at 270.0 degrees
    // Major marks = 80 degrees
    // Units is x 1000 ft/min
    needle_rot = -(needle_start) + -((fpm / 1000.0) * 80.0f);

    Glib_SetTexture(texobj_dial);
    Glib_DrawTexture(x0 - 160, y0 - 160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);
	Glib_SetTexture(texobj_needle);
    Glib_DrawTextureRotated(x0, y0, 235, 235, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
}

void BEGIN_Vsi()
{
}
