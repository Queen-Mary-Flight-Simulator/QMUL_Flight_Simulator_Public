#include <math.h>
#include <stdio.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "nfd.h"
#include "adf.h"

static float oldrb = 0.0;

/* ------------------------------------------ */
void Adf_Adf(int x0, int y0, float Hdg, float rb1, GLuint texobj1, GLuint texobj2)
{
    float rb;

    rb1 = Maths_Normalise(rb1);
    rb = -Maths_Degrees(rb1);  /* negated DJA 5/5/16 */

	if (rb > (oldrb + 0.5))
	{
	    rb = oldrb + 0.5;
	}
	if (rb < (oldrb - 0.5))
	{
	    rb = oldrb - 0.5;
	}
	
	oldrb = rb;
	
    Glib_SetTexture(texobj1);
    Glib_DrawTexture(x0 - 128, y0 - 128, 256, 256, 0.0, 0.0, 1.0, 1.0, 1.0);
	Glib_SetTexture(texobj2);
    Glib_DrawTextureRotated(x0, y0, 256, 256, 0.0, 0.0, 1.0, 1.0, Maths_Degrees(Hdg), 1.0);

	Glib_Colour(Glib_YELLOW);
	Glib_LineWidth(4.0);
    Glib_PushMatrix();
	Glib_Translate((float) x0, (float) y0);
    Glib_Rotate(rb);
    Glib_Draw(0, -81, 0, 78);
	Glib_Triangle(0, 83, -10, 83-20, 10, 83-20);
    Glib_PopMatrix();
	
    Glib_PushMatrix();
	Glib_Translate((float) x0, (float) y0);
	Glib_Colour(Glib_ORANGE);
	Glib_LineWidth(3.0);
	Glib_Draw(0, 10, 0, -30);
	Glib_Draw(-15, 0, 15, 0);
	Glib_Draw(-5, -25, 5, -25);
	
	Glib_Draw(0, 83, 0, 105);
	Glib_Draw(0, -95, 0, -105);
	Glib_Draw(95, 0, 105, 0);
	Glib_Draw(-95, 0, -105, 0);
	Glib_Draw(67, 67, 74, 74);
	Glib_Draw(-67, -67, -74, -74);
	Glib_Draw(67, -67, 74, -74);
	Glib_Draw(-67, 67, -74, 74);
    Glib_PopMatrix();
	
	Glib_LineWidth(1.0);
}

/* ------------------------------------------ */
void BEGIN_Adf()
{
    oldrb = 0.0;
}
