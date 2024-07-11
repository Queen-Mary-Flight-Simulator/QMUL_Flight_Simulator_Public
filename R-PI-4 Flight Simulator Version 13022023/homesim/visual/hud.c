#include <math.h>
#include <stdbool.h>

#include <GL/gl.h>

#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "asi.h"
#include "compass.h"
#include "hud.h"
#include "glib.h"
#include "maths.h"

static int xOrigin = 0;
static int yOrigin = 0;

bool HUD_Mode = false;

void Frame();

void HUD_SetOrigin(int x, int y)
{
  glTranslatef((float) (x - xOrigin), (float) (y - yOrigin), 0.0);
  xOrigin = x;
  yOrigin = y;
}

/* ------------------------------------------------------------------ */
void HUD_DrawHUD(float pitch, float roll, float yaw, float ias, float altitude,
                 float fpa_vertical, float fpa_lateral, float gammadot, float betadot,
                 float UDot, unsigned int baro)
{
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
//  glClear(GL_COLOR_BUFFER_BIT);

  Ai_AttitudeIndicator(pitch, roll, yaw);
  Compass_Compass(yaw);
  Alt_Altimeter(altitude, baro);
  Asi_Asi(ias, UDot);
  Ai_FlightPathVector(fpa_vertical, fpa_lateral, gammadot, betadot);
  Frame();
  HUD_SetOrigin(0, 0);
}

/* ------------------------------------------------------------------ */
void Frame()
{ int theta;
  int s[57], c[57];

  for (theta=0; theta <= 56; theta +=4) {
    c[theta] = (int) (cos(Maths_Rads((float) theta)) * 290.0);
    s[theta] = (int) (sin(Maths_Rads((float) theta)) * 290.0);
  }
  
  HUD_SetOrigin(HUD_HUDX, HUD_HUDY - 57);
  Glib_ClipWindow(0, 0, 1023, 767);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(0.0, 1.0, 0.0, 0.1);

  glBegin(GL_POLYGON);
    for (theta=0; theta <= 56; theta +=4){
      glVertex2i(c[theta], s[theta]);
    }
    for (theta=56; theta >= 0; theta -=4) {
      glVertex2i(-c[theta], s[theta]);
    }
    for (theta=0; theta <= 56; theta +=4) {
      glVertex2i(-c[theta], -s[theta]);
    }
    for (theta=56; theta >= 0; theta -=4) {
      glVertex2i(c[theta], -s[theta]);
    }
  glEnd();
  
//  glDisable(GL_BLEND);
}

/* ------------------------------------------------------------------ */
void BEGIN_HUD()
{
    xOrigin = 0;
    yOrigin = 0;
}
