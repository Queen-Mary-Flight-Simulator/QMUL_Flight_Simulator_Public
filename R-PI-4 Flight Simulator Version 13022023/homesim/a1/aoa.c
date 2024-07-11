#include <SIM/glib.h>
#include <SIM/maths.h>

#include "aoa.h"

void Aoa_Aoa(int x0, int y0, float aoa, int dial, int needle)
{
  float needle_start = 270.0f; // dial marking starting angle
  float needle_rot   = 0.0f;
 
  // Dial shows : marks begin at 270.0 degrees
  // Units : 1 degree input = 10 degs
  needle_rot = -needle_start - (Maths_Degrees(aoa) * 10.0);
  
  Glib_SetTexture(dial);
  Glib_DrawTexture(x0 - 128, y0 - 128, 256.0, 256.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  Glib_SetTexture(needle);
  Glib_DrawTextureRotated(x0, y0, 192, 192, 0.0, 0.0, 1.0, 1.0, needle_rot, 1.0);
}

void BEGIN_Aoa()
{
}
