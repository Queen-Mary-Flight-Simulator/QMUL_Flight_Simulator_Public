#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include <GL/gl.h>

#include "hud.h"
#include "glib.h"
#include "asi.h"
#include "maths.h"

static int DigitsBox[24] = {
  -10, 0, -15, -10, -15, -23, -28, -23, -28, -10, -53, -10, -53, 10, -28, 10,
  -28, 23, -15, 23, -15, 10, -10, 0 };

static int UpArrow[6] = {
  -6, 0, 0, 6, 6, 0 };      

static int DownArrow[6] = {
  -6, 0, 0, -6, 6, 0 };      

#define TapeWidth 45
#define TapeHeight 186
#define TapeHalfHeight (TapeHeight / 2)

void WrDigit(unsigned int n, int x, int y, bool Flag);

/* --------------------------------------------------------------- */
void Asi_Asi(float IAS, float UDot)
{
  float Kts;
  float Offset;
  int Dig1Offset;
  int Dig2Offset;
  int Dig3Offset;
  unsigned int KtsDigits;
  int MinSpeed;
  int MaxSpeed;
  int y;
  int s1;
  int s2;
  int Trend;
  unsigned int n;
  char str [20];
  
  HUD_SetOrigin(HUD_AsiX, HUD_AsiY);
  Glib_AntiAliasing(true);
  
  Kts = IAS * 1.944;
  if (Kts < 30.0) {
    Kts = 30.0;
  }
  KtsDigits = (unsigned int) Kts;
  MinSpeed = intround(Kts - 0.5) / 10 * 10 - 60;
  MaxSpeed = MinSpeed + 140;
  Offset = (Kts - (float) (KtsDigits / 10 * 10)) * 1.6;
  s1 = MinSpeed;
  s2 = MaxSpeed;

  Glib_Draw(0, -TapeHalfHeight, -TapeWidth, -TapeHalfHeight);   
  Glib_Draw(0,  TapeHalfHeight, -TapeWidth,  TapeHalfHeight);
  Glib_Draw(0, -TapeHalfHeight, 0, TapeHalfHeight);
  Glib_DrawLines(12, 0, 0, DigitsBox);
  
  glPushMatrix();
  glTranslatef(0.0, -Offset, 0.0);
  y = -96;
  Glib_SetFont(Glib_gFont8, 6);
  
  do {
    if (s1 >= 30) {
      Glib_ClipWindow(HUD_AsiX - TapeWidth, HUD_AsiY - TapeHalfHeight, TapeWidth, TapeHeight);
      Glib_Draw(-7, y, 0, y);
      if (s1 % 20 == 0) {
	sprintf(str, "%d", s1);
	if (y > 0) {
	  Glib_ClipWindow(HUD_AsiX - 50, HUD_AsiY + 25, 50, TapeHalfHeight - 25);
        }
        else {
	  Glib_ClipWindow(HUD_AsiX - 50, HUD_AsiY - TapeHalfHeight, 50, TapeHalfHeight - 25);
        }        
        Glib_Chars(str, -strlen(str) * 6 - 8, y - 4);
      }
    }
    y = y + 16;
    s1 = s1 + 10;
  } while (!(s1 >= s2));

  glTranslatef(0.0, Offset, 0.0);
  glPopMatrix();
  
  Glib_SetFont(Glib_gFont12, 8);
  Glib_ClipWindow(HUD_AsiX - 50, HUD_AsiY - 23, 100, 46);
  
  Dig3Offset = (int) (Kts * 18.0) % 18;
  WrDigit((KtsDigits + 2) % 10, -25,  30 - Dig3Offset, false);
  WrDigit((KtsDigits + 1) % 10, -25,  12 - Dig3Offset, false);
  WrDigit(KtsDigits       % 10, -25,  -6 - Dig3Offset, false);
  WrDigit((KtsDigits + 9) % 10, -25, -24 - Dig3Offset, false);

  Glib_ClipWindow(HUD_AsiX - 50, HUD_AsiY - 10, 100, 20);
  
  n = KtsDigits % 10;
  if (n >= 9) {
    Dig2Offset = (int) ((Kts - (float) (KtsDigits / 10 * 10 + 9)) * 18.0);
  } else {
    Dig2Offset = 0;
  }
  WrDigit((KtsDigits / 10 + 1) % 10, -35,  12 - Dig2Offset, KtsDigits < 9);
  WrDigit(KtsDigits / 10 % 10,       -35,  -6 - Dig2Offset, KtsDigits < 10);
  WrDigit((KtsDigits / 10 + 9) % 10, -35, -24 - Dig2Offset, KtsDigits < 11);

  n = KtsDigits % 100;
  if (n >= 99) {
    Dig1Offset = (int) ((Kts - (float) (KtsDigits / 100 * 100 + 99)) * 18.0);
  } else {
    Dig1Offset = 0;
  }
  WrDigit((KtsDigits / 100 + 1) % 10, -45,  12 - Dig1Offset, KtsDigits < 100);
  WrDigit((KtsDigits / 100) % 10,     -45,  -6 - Dig1Offset, KtsDigits < 99);
  WrDigit((KtsDigits / 100 + 9) % 10, -45, -24 - Dig1Offset, KtsDigits < 101);

  glDisable(GL_SCISSOR_TEST);
  
  Trend = intround(UDot * 19.44 * 1.6);
  if (IAS < 15.432) {
    Trend = 0;
  }
  if (Trend > -16 && Trend < 16) {
    Trend = 0;
  } else if (Trend > TapeHalfHeight) {
    Trend = TapeHalfHeight;
  } else if (Trend < -TapeHalfHeight) {
    Trend = -TapeHalfHeight;
  }
  if (Trend != 0) {
    Glib_Draw(-7, 0, -7, Trend);
    if (Trend > 0) {
      Glib_DrawPolygon(3, -7, Trend, UpArrow);
    } else {
      Glib_DrawPolygon(3, -7, Trend, DownArrow);
    }
  }
}

/* --------------------------------------------------------------- */
void WrDigit(unsigned int n, int x, int y, bool Flag)
{
  if (Flag) {
    Glib_Draw(x, y, x, y + 12);
    Glib_Draw(x, y+12, x+6, y + 12);
    Glib_Draw(x+6, y+12, x+6, y);
    Glib_Draw(x+6, y, x, y);
    Glib_Draw(x, y, x+6, y+12);
    Glib_Draw(x, y+12, x+6, y);
  } 
  else {
    Glib_Char((unsigned char) n + '0', x, y);
  }
}

/* --------------------------------------------------------------- */
void BEGIN_Asi()
{
}
