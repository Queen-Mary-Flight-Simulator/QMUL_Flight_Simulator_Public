#include <GL/gl.h>

#include <SIM/maths.h>
#include <SIM/glib.h>

#include "pfd.h"
#include "compass.h"

float Compass_Adjust;

void Compass_Compass(int x0, int y0, float Hdg, int HdgBug, int texobj1, int texobj2, int texobj3, int texobj4)
{
    Glib_SetTexture(texobj1);
	Glib_DrawTexture(x0-160, y0-160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_SetTexture(texobj2);
	Glib_DrawTextureRotated(x0, y0, 320, 320, 0.0, 0.0, 1.0, 1.0, Maths_Degrees(Hdg + Compass_Adjust / 10.0), 1.0);
    Glib_SetTexture(texobj3);
	Glib_DrawTexture(x0-160, y0-160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);
	Glib_SetTexture(texobj4);
    Glib_DrawTextureRotated(x0 + 130, y0 - 130, 64, 64, 0.0, 0.0, 1.0, 1.0, -Compass_Adjust, 1.0);
}

void BEGIN_Compass()
{
    Compass_Adjust = 0.0;
}
