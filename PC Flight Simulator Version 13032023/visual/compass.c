#include <GL/gl.h>
#include <GL/glu.h>

#include "glib.h"
#include "hud.h"
#include "maths.h"
#include "compass.h"

/* ------------------------------------------------------- */
void Compass_Compass(float MagHdg)
{
  int x;
  float offset;
  float dh;
  float hdg;
  int IntHdg;
  unsigned int h1;
  unsigned int h2;

  HUD_SetOrigin(HUD_CompassX, HUD_CompassY);
  Glib_ClipWindow(HUD_CompassX - 150, HUD_CompassY - 30, 300, 50);
  glLineWidth(2.0);

  Glib_Draw(0, 1, -9, 1 + 16);
  Glib_Draw(-9, 1 + 16, 9, 1 + 16);
  Glib_Draw(9, 1 + 16, 0, 1);

  hdg = Maths_Degrees(MagHdg);
  if (hdg < 0.0) {
    IntHdg = 360 - intround(-hdg);
  }
  else {
    IntHdg = intround(hdg);
  }

  h1 = IntHdg / 10 * 10;
  if (h1 >= 40) {
    h1 = h1 - 35;
  } else {
    h1 = h1 + 325;
  }
  h2 = h1 + 80;
  if (h2 > 360) {
    h2 = h2 - 360;
  }

  dh = (float) h1 - hdg;
  if (dh > 0.0) {
    dh = dh - 360.0;
  }
  offset = dh * 6.0;  /* 40 deg = 240 pix */
  glPushMatrix();
  glTranslatef(offset, 0.0, 0.0);
  x = 0;
  
  do {
    if (h1 % 10 == 0) {
      Glib_Draw(x, 0, x, -10);
      Glib_SetFont(Glib_gFont12, 10);
      if (h1 > 90) {
        Glib_Char(h1 / 100 + '0', x - 8, -27);
        Glib_Char(h1 / 10 % 10 + '0', x + 2, -27);
      } else {
        Glib_Char(h1 / 10 % 10 + '0', x - 4, -27);
      }
    } 
    else {
      Glib_Draw(x, 0, x, -5);
    }
    x = x + 30;
    if (h1 >= 360) {
      h1 = 5;
    } else {
      h1 = h1 + 5;
    }
  } while (!(h1 == h2));
  
  glTranslatef(offset, 0.0, 0.0);
  glPopMatrix();
  glDisable(GL_SCISSOR_TEST);
}

/* ------------------------------------------------------- */
void BEGIN_Compass()
{
}
