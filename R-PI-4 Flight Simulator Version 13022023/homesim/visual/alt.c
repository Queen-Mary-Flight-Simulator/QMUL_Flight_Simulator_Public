#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include <GL/gl.h>

#include "hud.h"
#include "glib.h"
#include "maths.h"
#include "alt.h"

#define TapeWidth 60
#define TapeHeight 186
#define TapeHalfHeight (TapeHeight / 2)

static int DigitsBox[20] = {
    7, 0, 12, 10, 45, 10, 45, 23, 68, 23, 68, -23, 45, -23, 45, -10, 12, -10, 7, 0 };
    
static void WrDigit(unsigned int n, int x, int y, bool GreenFlag);
static void WriteLsdigit(unsigned int d, int x, int y);

/* --------------------------------------------------------------- */
void Alt_Altimeter(float z, unsigned int Baro)
{
  float Altitude;
  unsigned int AltitudeDigits;
  int Dig1Offset;
  int Dig2Offset;
  int Dig3Offset;
  float LSDigOffset;
  int MinAltitude;
  int MaxAltitude;
  int y;
  int a1;
  int a2;
  unsigned int LsDigit;
  float Offset;
  unsigned int n;
  char str[10];
  
  HUD_SetOrigin(HUD_AltX, HUD_AltY);
  Glib_AntiAliasing(true);
  Glib_ClipWindow(HUD_HUDX - 287, HUD_HUDY - 300, 574, 483);

  if (z <= 0.0) {
    Altitude = 0.0;
  } else {
    Altitude = z * 3.280840;
  }

  Altitude = Altitude + (float) Baro * 29.0;
  if (Altitude < 0.0) {
    Altitude = 0.0;
  }

  AltitudeDigits = (unsigned int) (Altitude);
  Glib_Draw(0, -TapeHalfHeight, TapeWidth, -TapeHalfHeight);
  Glib_Draw(0,  TapeHalfHeight, TapeWidth,  TapeHalfHeight);
  Glib_Draw(0, -TapeHalfHeight, 0, TapeHalfHeight);
  Glib_DrawLines(10, 0, 0, DigitsBox);

  MinAltitude = intround(Altitude - 0.5) / 100 - 4;
  MaxAltitude = MinAltitude + 10;
  Offset = (Altitude - (float) (AltitudeDigits / 100 * 100)) * 0.21;
  a1 = MinAltitude;
  a2 = MaxAltitude;
  Glib_ClipWindow(HUD_AltX - 10, HUD_AltY - TapeHalfHeight, TapeWidth + 10, TapeHeight);
  
  glPushMatrix();
  glTranslatef(0.0, -Offset, 0.0);
  y = -84;
  Glib_SetFont(Glib_gFont8, 6);

  do {
    if (a1 >= 0) {
      Glib_ClipWindow(HUD_AltX, HUD_AltY - TapeHalfHeight, 45, TapeHeight);
      Glib_Draw(0, y, 7, y);
      if (a1 % 2 == 0) {
	sprintf(str, "%d00", a1);
	if (y > 0) {
          Glib_ClipWindow(HUD_AltX, HUD_AltY + 12, 45, TapeHalfHeight - 12);
        }
        else {
          Glib_ClipWindow(HUD_AltX, HUD_AltY - TapeHalfHeight, 45, TapeHalfHeight - 12);
        }
        Glib_Chars(str, 10, y - 4);
      }
    }
    y = y + 21;
    a1 = a1 + 1;
  } while (!(a1 >= a2));
  
  glTranslatef(0.0, Offset, 0.0);
  glPopMatrix();
  
  Glib_SetFont(Glib_gFont12, 8);
  Glib_ClipWindow(HUD_AltX, HUD_AltY - 10, 45, 20);
  
  LsDigit = AltitudeDigits % 100;
  n = AltitudeDigits % 100;
  if (n > 80) {
    Dig3Offset = (int) ((float) (n - 80) / 20.0 * 18.0);
  } else {
    Dig3Offset = 0;
  }
  WrDigit((AltitudeDigits / 100 + 1) % 10, 37,  12 - Dig3Offset, false); /* 3rd digit */
  WrDigit(AltitudeDigits / 100 % 10,       37,  -6 - Dig3Offset, false);
  WrDigit((AltitudeDigits / 100 + 9) % 10, 37, -24 - Dig3Offset, false);

  n = AltitudeDigits % 1000;
  if (n >= 980) {
    Dig2Offset = (int) ((float) (n - 980) / 20.0 * 18.0);
  } else {
    Dig2Offset = 0;
  }
  WrDigit((AltitudeDigits / 1000 + 1) % 10, 27,  12 - Dig2Offset, AltitudeDigits < 900);  /* 2nd digit */
  WrDigit(AltitudeDigits / 1000 % 10,       27,  -6 - Dig2Offset, AltitudeDigits < 1000);
  WrDigit((AltitudeDigits / 1000 + 9) % 10, 27, -24 - Dig2Offset, AltitudeDigits < 1100);
  n = AltitudeDigits % 10000;
  if (n >= 9980) {
    Dig1Offset = (int) ((float) (n - 9980) / 20.0 * 18.0);
  } else {
    Dig1Offset = 0;
  }
  WrDigit((AltitudeDigits / 10000 + 1) % 10, 17,  12 - Dig1Offset, AltitudeDigits < 9000); /* 1st digit */
  WrDigit(AltitudeDigits / 10000 % 10,       17,  -6 - Dig1Offset, AltitudeDigits < 10000);
  WrDigit((AltitudeDigits / 10000 + 9) % 10, 17, -24 - Dig1Offset, AltitudeDigits < 11000);

  Glib_ClipWindow(HUD_AltX + 45, HUD_AltY - 23, 23, 46);
  
  LSDigOffset = (Altitude - (float) (AltitudeDigits / 20 * 20)) / 20.0 * 18.0;
  LsDigit = LsDigit / 20 * 20;
  glPushMatrix();
  glTranslatef(0.0, (float) -LSDigOffset, 0.0);
  WriteLsdigit(LsDigit,              49, -6);
  WriteLsdigit((LsDigit + 20) % 100, 49, 12);
  WriteLsdigit((LsDigit + 40) % 100, 49, 30);
  WriteLsdigit((LsDigit + 80) % 100, 49, -24);
  glTranslatef(0.0, (float) LSDigOffset, 0.0);
  glPopMatrix();
  glDisable(GL_SCISSOR_TEST);
}

/* --------------------------------------------------------------- */
static void WrDigit(unsigned int n, int x, int y, bool Flag)
{
  if (Flag) {
    Glib_Draw(x, y, x, y + 12);
    Glib_Draw(x, y+12, x+6, y + 12);
    Glib_Draw(x+6, y+12, x+6, y);
    Glib_Draw(x+6, y, x, y);
    Glib_Draw(x, y, x+6, y+12);
    Glib_Draw(x, y+12, x+6, y);
  } else {
    Glib_Char((unsigned char) n + '0', x, y);
  }
}

/* --------------------------------------------------------------- */
static void WriteLsdigit(unsigned int d, int x, int y)
{
  Glib_Char((unsigned char) (d / 10) + '0', x, y);
  Glib_Char((unsigned char) (d % 10) + '0', x + 8, y);
}

/* --------------------------------------------------------------- */
void BEGIN_Alt()
{
}
