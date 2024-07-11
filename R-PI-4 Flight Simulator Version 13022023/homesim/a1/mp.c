#include <stdio.h>

#include <SIM/maths.h>
#include <SIM/glib.h>

#include "mp.h"

void Mp_Mp(int x0, int y0, float mp, int dial, int needle)
{
    float needle_start = -150.0f; // dial marking starting angle
    float needle_rot   = 0.0f;

    mp = Maths_Limit(mp, 0.0, 50.0);

    // Dial shows : marks begin at -150.0 degrees
    // Units : Major units = 30" = 300 degrees
    needle_rot = -needle_start - mp * 10.0;

    Glib_SetTexture(dial);
    Glib_DrawTexture(x0 - 128, y0 - 128, 256, 256, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(needle);
    Glib_DrawTextureRotated(x0, y0, 192, 192, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
}
void BEGIN_Mp()
{
}
