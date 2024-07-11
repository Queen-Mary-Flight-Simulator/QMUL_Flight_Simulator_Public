#include <GL/gl.h>
#include <GL/glu.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "hud.h"
#include "glib.h"
#include "maths.h"
#include "ai.h"
#include "cbool.h"

#define HUDHeight   346  /* 18 deg */
#define HUDWidth    576  /* 30 deg */
#define HUDHalfWidth	(HUDWidth / 2)
#define HUDHalfHeight	(HUDHeight / 2)
#define PixPerDegree	19.0
#define BankAngleRadius	250

static int boresight[10] = {
    -26,  0, -9,   0,  0,  -9,  9, 0, 26, 0 };
    
static int circ8[38] = {
     0,  8,  2,  7,  5,  6,  6,  3,  7,  1,
     7, -1,  6, -4,  5, -6,  2, -7,  0, -8,
    -2, -7, -5, -6, -6, -3, -7, -1, -7,  1,
    -6,  3, -5,  6, -2,  7,  0, 8 };
    
void HeadingMarks(float yaw);

/* --------------------------------------------------------- */
void Ai_AttitudeIndicator(float Pitch, float Roll, float yaw)
{
  float Pitch_Deg;
  float Roll_Deg;
  GLfloat Pitch_Pix;
  int p;
  int dy;
  char str[10];

  glColor3f(0.0, 1.0, 0.0);

  Pitch_Deg = -Pitch * Maths_ONERAD;
  Pitch_Pix = Pitch_Deg * PixPerDegree;
  Roll_Deg = Roll * Maths_ONERAD;

  HUD_SetOrigin(HUD_HUDX, HUD_HUDY);
  Glib_AntiAliasing(true);
  Glib_ClipWindow(HUD_HUDX - 160, HUD_HUDY - 175, 320, 270);

  glPushMatrix();
  glRotatef(Roll_Deg, 0.0, 0.0, 1.0);
  glTranslatef(0.0, Pitch_Pix, 0.0);
  glColor3f(0.0, 1.0, 0.0);
  //  Glib_Colour(Glib_GREEN);
  glLineWidth(2.0);
    
  Glib_SetFont(Glib_gFont12, 10);
  for (p = -30; p <= 30; p += 5) {   /* pitch lines */
    dy = (int) ((float) p * 95.0 / 5.0);   /* 5 deg = 95 pix */
    if (p < 0) {
      glLineStipple(2, 0x0F0F);
      glEnable(GL_LINE_STIPPLE);
    } 
    else {
      glDisable(GL_LINE_STIPPLE);
    }
    if (p == 0) {
      Glib_Draw(-250, 0, 250, 0);    /* zero degree pitch line */
      HeadingMarks(yaw);
    }
    else {
      Glib_Draw(-100, dy, -30, dy);
      Glib_Draw(100, dy, 30, dy);
      glDisable(GL_LINE_STIPPLE);
      if (p > 0) {
        Glib_Draw(-100, dy, -100, dy - 8);
        Glib_Draw(100, dy, 100, dy - 8);
      }
      else {
        Glib_Draw(-100, dy, -100, dy + 8);
        Glib_Draw(100, dy, 100, dy + 8);
      }
      sprintf(str, "%d", abs(p));  /* waiting for minus symbol in font */
      Glib_Chars(str, -100 - strlen(str) * 11, dy - 6);
      Glib_Chars(str, 100 + 4, dy - 6);
    }
  }
  
  Glib_ClipWindow(HUD_HUDX - 180, HUD_HUDY - BankAngleRadius - 20, 360, 200);
  glTranslatef(0.0, -Pitch_Pix, 0.0);   /* moving bank triangle */
  Glib_Draw(0, -BankAngleRadius, -9, -BankAngleRadius + 12);
  Glib_Draw(-9, -BankAngleRadius + 12, 9, -BankAngleRadius + 12);
  Glib_Draw(9, -BankAngleRadius + 12, 0, -BankAngleRadius);
  
  glDisable(GL_SCISSOR_TEST);
  glPopMatrix();

  glLineWidth(3.0);
  Glib_DrawLines(5, 0, 0, boresight);
  glLineWidth(2.0);

  glPushMatrix();
  Glib_Draw(0, -BankAngleRadius, -9, -BankAngleRadius - 12);
  Glib_Draw(-9, -BankAngleRadius - 12, 9, -BankAngleRadius - 12);
  Glib_Draw(9, -BankAngleRadius - 12, 0, -BankAngleRadius);
  glRotatef(10.0, 0.0, 0.0, 1.0);  /* bank angle marks */
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 12);
  glRotatef(10.0, 0.0, 0.0, 1.0);
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 12);
  glRotatef(10.0, 0.0, 0.0, 1.0);
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 28);
  glRotatef(15.0, 0.0, 0.0, 1.0);
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 12);
  glRotatef(-55.0, 0.0, 0.0, 1.0);
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 12);
  glRotatef(-10.0, 0.0, 0.0, 1.0);
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 12);
  glRotatef(-10.0, 0.0, 0.0, 1.0);
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 28);
  glRotatef(-15.0, 0.0, 0.0, 1.0);
  Glib_Draw(0, -BankAngleRadius, 0, -BankAngleRadius - 12);
  glRotatef(45.0, 0.0, 0.0, 1.0);
  glPopMatrix();
}

/* --------------------------------------------------------- */
void HeadingMarks(float yaw)
{
  int x;
  float dh;
  float hdg;
  int IntHdg;
  unsigned int h1;
  unsigned int h2;
    
  Glib_Draw(0, 1, -7, 1 + 12);
  Glib_Draw(-7, 1 + 12, 7, 1 + 12);
  Glib_Draw(7, 1 + 12, 0, 1);
      
  hdg = Maths_Degrees(yaw);
  if (hdg < 0.0) {
    IntHdg = 360 - intround(-hdg);
  }
  else {
    IntHdg = intround(hdg);
  }

  h1 = IntHdg / 10 * 10;
  if (h1 >= 40) {
    h1 = h1 - 30;
  } else {
    h1 = h1 + 330;
  }
  h2 = h1 + 60;
  if (h2 > 360) {
    h2 = h2 - 360;
  }

  dh = (float) h1 - hdg;
  if (dh > 0.0) {
    dh = dh - 360.0;
  }
  x = (int) (dh * 19.0);  /* 5 deg = 95 pix */

  do {
    Glib_Draw(x, 0, x, 10);
    Glib_SetFont(Glib_gFont12, 10);
    Glib_Char(h1 / 100 + '0', x - 8, 15);
    Glib_Char(h1 / 10 % 10 + '0', x + 2, 15);
    x = x + 190;
    if (h1 >= 360) {
      h1 = 10;
    } else {
      h1 = h1 + 10;
    }
  } while (!(h1 == h2));
}

/* --------------------------------------------------------- */
void Ai_FlightPathVector(float gamma, float beta, float gammadot, float betadot)
{
  float Gamma_Pix;
  float Beta_Pix;
  float Gammadot_pix;
  float Betadot_pix;

  Gamma_Pix = Maths_Degrees(gamma) * PixPerDegree;
  Beta_Pix = Maths_Degrees(beta) * PixPerDegree;

  Gammadot_pix = Maths_Degrees(gammadot) * PixPerDegree * 5.0;
  Betadot_pix = Maths_Degrees(betadot) * PixPerDegree * 5.0;

  Glib_ClipWindow(HUD_HUDX - 160, HUD_HUDY - 172, 320, 252);
  HUD_SetOrigin(HUD_HUDX, HUD_HUDY);
  glLineWidth(3.0);
  
  glPushMatrix();
  glTranslatef(Beta_Pix, Gamma_Pix, 0.0);
  Glib_DrawLines(19, 0, 0, circ8);
  glLineWidth(2.0);
  Glib_Draw(0, 8, 0, 22);
  Glib_Draw(-8, 0, -30, 0);
  Glib_Draw(8, 0, 30, 0);
  if ((Gammadot_pix * Gammadot_pix + Betadot_pix * Betadot_pix) > 64.0) 
    Glib_Draw(0, 0, (int) Betadot_pix, (int) Gammadot_pix);
  glTranslatef(-Beta_Pix, -Gamma_Pix, 0.0);
  glPopMatrix();
  glDisable(GL_SCISSOR_TEST);
}

/* --------------------------------------------------------- */
void BEGIN_Ai()
{
}
