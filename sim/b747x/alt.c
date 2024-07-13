/* +------------------------------+---------------------------------+
   | Module      : alt.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-04      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EFIS Altimeter                    |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <GL/gl.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/glib.h>
#include <SIM/weather.h>

#include "alt.h"
#include "systems.h"

#define Above2500         0
#define Above1000         1
#define Below1000         2
typedef unsigned char   RadAltMode;

#define TapeWidth         78
#define TapeHeight        448
#define TapeHalfHeight    (TapeHeight / 2)
#define TapeHalfWidth     (TapeWidth / 2)
#define TickSpacing       37

static int DigitsBox[18] = {
    25, -28, 25, -10, 13, 0, 25, 10, 25, 28, 107, 28, 107, -28, 25, -28
};

static int DigitsBox2[8] = {
    25, -28, 25, 28, 107, 28, 107, -28
};

static int DigitsBox3[6] = {
    25, -10, 13, 0, 25, 10
};

static int DigitsBox4[8] = {
    25, 28, 25, 48, 107, 48, 107, 28
};

static int AltBug[16] = {
    0, 0, -8, 8, -8, 28, 25, 28, 25, -28, -8, -28, -8, -8, 0, 0
};

static void WrDigit(unsigned int n, int x, int y, bool GreenFlag);
static void Wrn3(int d, int x, int y);
static void Wrn5(int d, int x, int y);
static void WriteLsdigit(unsigned int d, int x, int y);
static void Splodge(int h, int w, int x, int y);
#define BaroX    0
#define BaroY    (-TapeHalfHeight - 27)
static void WriteBaro(unsigned int b, bool Hg, int x, int y);

static int rtab25[22] = {
    0,   25,  15, 20, 24, 8, 24, -8, 15, -20, 0, -25, -15, -20, -24, -8,
    -24,  8, -15, 20,  0, 25
};

static int rtab20[22] = {
    0,   20,  12, 16, 19, 6, 19, -6, 12, -16, 0, -20, -12, -16, -19, -6,
    -19,  6, -12, 16,  0, 20
};

void Alt_Altimeter(int AltX, int AltY, float z,
                   unsigned int Baro, bool BaroHg, unsigned int BaroSTD,
                   unsigned short int FCUAlt, bool Metric)
{
    float        Altitude;
    unsigned int AltitudeDigits;
    int          Dig1Offset;
    int          Dig2Offset;
    int          Dig3Offset;
    float        LSDigOffset;
    int          MinAltitude;
    int          MaxAltitude;
    int          y;
    int          a1;
    int          a2;
    unsigned int LsDigit;
    float        Offset;
    unsigned int n;
    float        Metric_Altitude;
	
    if (Systems_Failures[17])
	{
	    Glib_SetFont(Glib_GFONT12, 10);
	    Glib_VBox("ALT", TapeHalfWidth, 0);
		return;
	}
	
    Glib_AntiAliasing(true);

    if (z <= 0.0)
    {
        Altitude = 0.0;
    }
    else
    {
        Altitude = z * 3.280840;
    }
    if (BaroSTD)
    {
        Baro = 1013;
    }
    else if (BaroHg)
    {
        Baro = (unsigned int) ((float) Baro * 1013.0 / 2992.0);
    }
    Altitude = Altitude + ((float) Baro - (float) Weather_RegionalQNH) * 29.0;
    if (Altitude < 0.0)
    {
        Altitude = 0.0;
    }

    Metric_Altitude = Altitude / 3.280840; 
    AltitudeDigits = (unsigned int) (Altitude);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(0, -TapeHalfHeight, TapeWidth, TapeHeight);
    MinAltitude = intround(Altitude - 0.5) / 100 - 4;
    MaxAltitude = MinAltitude + 10;
    Offset      = (Altitude - (float) (AltitudeDigits / 100 * 100)) * 0.56;
    a1          = MinAltitude;
    a2          = MaxAltitude;
    glLineWidth(2.0);
    Glib_Colour(Glib_WHITE);
    Glib_ClipWindow(AltX - 10, AltY - TapeHalfHeight, TapeWidth + 10, TapeHeight);

    glPushMatrix();
    glTranslatef(0.0, (float) -Offset, 0.0);
    y = -TapeHalfHeight;
    do
    {
        if (a1 >= 0)
        {
            Glib_Draw(0, y, 15, y);
            if (a1 % 2 == 0)
            {
                Glib_SetFont(Glib_GFONT16, 12);
                if (a1 >= 100)
                {
                    Glib_Char((unsigned int) a1 / 100 + '0', 19, y - 8);
                }
                if (a1 >= 10)
                {
                    Glib_Char((unsigned int) a1 % 100 / 10 + '0', 29, y - 8);
                }
                Glib_SetFont(Glib_GFONT12, 10);
                Glib_Char((unsigned int) a1 % 10 + '0', 39, y - 6);
                Glib_Char('0', 48, y - 6);
                Glib_Char('0', 57, y - 6);
                if (a1 % 10 == 0)
                {
                    Glib_Draw(19, y + 11, 67, y + 11);
                    Glib_Draw(19, y - 12, 67, y - 12);
                }
            }
        }
        y  = y + 56;
        a1 = a1 + 1;
    } while (!(a1 >= a2));
    glTranslatef(0.0, (float) Offset, 0.0);
    glPopMatrix();
	
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(16, -35, TapeWidth - 16, 70);
    Glib_Colour(Glib_MAGENTA);
    glLineWidth(2.0);
    Offset = (Altitude - (float) FCUAlt) * 0.56;
    if (Offset < (float) (-TapeHalfHeight))
    {
        Offset = (float) (-TapeHalfHeight);
    }
    else if (Offset > (float) TapeHalfHeight)
    {
        Offset = (float) TapeHalfHeight;
    }
    Glib_ClipWindow(AltX - 10, AltY - TapeHalfHeight, TapeWidth, TapeHeight);

    glPushMatrix();
    glTranslatef(0.0, (float) -Offset, 0.0);
    Glib_DrawLines(8, 0, 0, AltBug);
    glTranslatef(0.0, (float) Offset, 0.0);
    glPopMatrix();

    glDisable(GL_SCISSOR_TEST);
    Glib_Colour(Glib_BLACK);
    Glib_DrawPolygon(4, 0, 0, DigitsBox2);
    Glib_Colour(Glib_BLACK);
    Glib_DrawPolygon(3, 0, 0, DigitsBox3);
    Glib_Colour(Glib_WHITE);
    glColor3f(1.0, 1.0, 1.0);
    Glib_DrawLines(8, 0, 0, DigitsBox);
	if (Metric)
	{
	    Glib_DrawLines(4, 0, 0, DigitsBox4);
	}
    glLineWidth(2.0);
    Glib_ClipWindow(AltX + 25, AltY - 23, 82, 46);
    LsDigit = AltitudeDigits % 100;
    Glib_SetFont(Glib_GFONT16, 10);
    n = AltitudeDigits % 100;
    if (n > 80)
    {
        Dig3Offset = (int) ((float) (n - 80) / 20.0 * 38.0);
    }
    else
    {
        Dig3Offset = 0;
    }
    WrDigit((AltitudeDigits / 100 + 1) % 10, 35 + 30, 26 + 4 - Dig3Offset, false);
    WrDigit(AltitudeDigits / 100 % 10, 35 + 30, -12 + 4 - Dig3Offset, false);
    WrDigit((AltitudeDigits / 100 + 9) % 10, 35 + 30, -50 + 4 - Dig3Offset, false);
    Glib_SetFont(Glib_GFONT24, 12);
    n = AltitudeDigits % 1000;
    if (n >= 980)
    {
        Dig2Offset = (int) ((float) (n - 980) / 20.0 * 38.0);
    }
    else
    {
        Dig2Offset = 0;
    }
    WrDigit((AltitudeDigits / 1000 + 1) % 10, 35 + 15, 26 - Dig2Offset, false);
    WrDigit(AltitudeDigits / 1000 % 10, 35 + 15, -12 - Dig2Offset, false);
    WrDigit((AltitudeDigits / 1000 + 9) % 10, 35 + 15, -50 - Dig2Offset, false);
    n = AltitudeDigits % 10000;
    if (n >= 9980)
    {
        Dig1Offset = (int) ((float) (n - 9980) / 20.0 * 38.0);
    }
    else
    {
        Dig1Offset = 0;
    }
    WrDigit((AltitudeDigits / 10000 + 1) % 10, 35, 26 - Dig1Offset, AltitudeDigits < 9000);
    WrDigit(AltitudeDigits / 10000 % 10, 35, -12 - Dig1Offset, AltitudeDigits < 10000);
    WrDigit((AltitudeDigits / 10000 + 9) % 10, 35, -50 - Dig1Offset, AltitudeDigits < 11000);
    LSDigOffset = (Altitude - (float) (AltitudeDigits / 20 * 20)) / 20.0 * 22.0;
    LsDigit     = LsDigit / 20 * 20;
    glPushMatrix();
    glTranslatef(0.0, (float) -LSDigOffset, 0.0);
    WriteLsdigit(LsDigit, 77, -8);
    WriteLsdigit((LsDigit + 20) % 100, 77, 14);
    WriteLsdigit((LsDigit + 40) % 100, 77, 36);
    WriteLsdigit((LsDigit + 80) % 100, 77, -30);
    glTranslatef(0.0, (float) LSDigOffset, 0.0);
    glPopMatrix();

    glDisable(GL_SCISSOR_TEST);

    if (Metric)
	{
    	Glib_Colour(Glib_BLACK);
	    Glib_Rectangle(26, 29, 80, 18);
        Glib_Colour(Glib_WHITE);
        Glib_SetFont(Glib_GFONT12, 10);
	    Wrn5(Metric_Altitude, 40, 32);
	}
	
    Glib_Colour(Glib_MAGENTA);
    Glib_SetFont(Glib_GFONT24, 12);
    n = FCUAlt;
    if (n >= 10000)
    {
        Glib_Char(n / 10000 + '0', TapeWidth / 2 - 28, TapeHalfHeight + 10);
    }
    Glib_Char(n % 10000 / 1000 + '0', TapeWidth / 2 - 14, TapeHalfHeight + 10);
    Glib_SetFont(Glib_GFONT16, 12);
    Wrn3(n % 1000, TapeWidth / 2, TapeHalfHeight + 14);
	if (Metric)
	{
	    Glib_Char('M', TapeWidth / 2 + 35, TapeHalfHeight + 14);
	}
}

static void WrDigit(unsigned int n, int x, int y, bool GreenFlag)
{
    if (GreenFlag)
    {
        Glib_Colour(Glib_GREEN);
        Splodge(24, 10, x, y);
        Glib_Colour(Glib_WHITE);
    }
    else
    {
        Glib_Char((unsigned char) n + '0', x, y);
    }
}

static void Wrn3(int d, int x, int y)
{
    char Str[4];

    Str[0] = (unsigned char) (d / 100) + '0';
    d      = d % 100;
    Str[1] = (unsigned char) (d / 10) + '0';
    Str[2] = (unsigned char) (d % 10) + '0';
    Str[3] = 0;
    Glib_Chars(Str, x, y);
}

static void Wrn5(int d, int x, int y)
{
    char Str[7];
		
    Str[0] = (d >= 10000) ? (unsigned char) (d / 10000) + '0' : ' ';
    d      = d % 10000;
    Str[1] = (d >= 1000) ? (unsigned char) (d / 1000) + '0' : ' ';
	d      = d % 1000;
    Str[2] = (unsigned char) (d / 100) + '0';
    d      = d % 100;
    Str[3] = (unsigned char) (d / 10) + '0';
    Str[4] = (unsigned char) (d % 10) + '0';
    Str[5] = 'M';
    Str[6] = 0;
    Glib_Chars(Str, x, y);
}

static void WriteLsdigit(unsigned int d, int x, int y)
{
    Glib_SetFont(Glib_GFONT16, 8);
    Glib_Char((unsigned char) (d / 10) + '0', x, y);
    Glib_Char((unsigned char) (d % 10) + '0', x + 12, y);
}

static void Splodge(int h, int w, int x, int y)
{
    int i;

    {
        for (i = 0; i <= w - 1; i += 1)
        {
            Glib_Draw(x + i, y, x + i, y + h);
        }
    }
}

void Alt_Baro(unsigned int Baro, bool BaroHg, unsigned int BaroSTD)
{
    int y = BaroY;

    Glib_Colour(Glib_GREEN);
    if (BaroSTD)
    {
        Glib_SetFont(Glib_GFONT24, 16);
        Glib_Chars("STD", BaroX + 20, BaroY);
    }
    else
    {
        y = BaroY - 20;
        Glib_SetFont(Glib_GFONT16, 10);
        WriteBaro(Baro, BaroHg, BaroX, y);
        if (BaroHg)
        {
            Glib_Chars("IN", BaroX + 60, y);
        }
        else
        {
            Glib_Chars("HPA", BaroX + 50, y);
        }
    }
}

static void WriteBaro(unsigned int b, bool Hg, int x, int y)
{
    char Str[6];

    Str[0] = b / 1000 + '0';
    b      = b % 1000;
    Str[1] = b / 100 + '0';
    b      = b % 100;
    if (Hg)
    {
        Str[2] = '.';
        Str[3] = b / 10 + '0';
        Str[4] = b % 10 + '0';
        Str[5] = 0;
    }
    else
    {
        Str[2] = b / 10 + '0';
        Str[3] = b % 10 + '0';
        Str[4] = 0;
    }
    Glib_Chars(Str, x, y);
}

void Alt_RadioAltimeter(float h)
{
    unsigned int d;
    RadAltMode   m;
    unsigned int i;

    Glib_SetFont(Glib_GFONT16, 10);
    h = h * 3.280840;
    if (h < 0.0)
    {
        h = 0.0;
    }
    else if (h > 3000.0)
    {
        h = 3000.0;
    }
    d = (unsigned int) (h);
    if (d >= 2500)
    {
        m = Above2500;
    }
    else if (d >= 1000)
    {
        m = Above1000;
    }
    else
    {
        m = Below1000;
    }

    if (d >= 500)
    {
        d = d / 20 * 20;
    }
    else if (d >= 100)
    {
        d = d / 10 * 10;
    }
    else
    {
        d = d / 2 * 2;
    }

    if (m == Below1000)
    {
        Glib_Colour(Glib_WHITE);
        for (i = 0; i <= 18; i += 2)
        {
            Glib_Draw(rtab20[i], rtab20[i + 1], rtab25[i], rtab25[i + 1]);
        }
        for (i = 0; i <= (d / 100); i += 1)
        {
            Glib_Draw(rtab25[i * 2], rtab25[i * 2 + 1], rtab25[i * 2 + 2], rtab25[i * 2 + 3]);
        }
    }

    if (m != Above2500)
    {
        Glib_Colour(Glib_WHITE);
        if (d >= 1000)
        {
            Wrn3(d / 10, -20, -8);
            Glib_Char('0', 10, -8);
        }
        else
        {
            Wrn3(d, -15, -8);
        }
    }
}

void BEGIN_Alt()
{
}
