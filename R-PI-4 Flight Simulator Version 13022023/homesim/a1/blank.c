#include <stdio.h>

#include <SIM/glib.h>

#include "blank.h"

void Blank_Dial(int x0, int y0, int dial)
{
    Glib_SetTexture(dial);
	Glib_DrawTexture(x0 - 128, y0 - 128, 256, 256, 0.0, 0.0, 1.0, 1.0, 1.0);
}

void BEGIN_Blank()
{
}
