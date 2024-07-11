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

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <GLFW/glfw3.h>

#include <SIM/maths.h>
#include <SIM/glib.h>
#include <SIM/weather.h>
#include <SIM/navdefn.h>

#include "alt.h"
#include "ai.h"
#include "systems.h"

#define Above2500      0
#define Above1000      1
#define Below1000      2

#define TapeWidth      78
#define TapeHeight     448
#define TapeHalfHeight (TapeHeight / 2)
#define TapeHalfWidth  (TapeWidth / 2)
#define TickSpacing    37

#define BaroX          0
#define BaroY          (-TapeHalfHeight - 27)

typedef unsigned char  RadAltMode;

static int DigitsBox[18] = 
{
    25, -28, 25, -10, 13, 0, 25, 10, 25, 28, 107, 28, 107, -28
};

static int DigitsBox4[8] = 
{
    25, 28, 25, 48, 107, 48, 107, 28
};

static int AltBug[14] = 
{
    0, 0, -8, 8, -8, 28, 25, 28, 25, -28, -8, -28, -8, -8
};

static int rtab25[22] = 
{
    0,   25,  15, 20, 24, 8, 24, -8, 15, -20, 0, -25, -15, -20, -24, -8,
    -24,  8, -15, 20,  0, 25
};

static int rtab20[22] = 
{
    0,   20,  12, 16, 19, 6, 19, -6, 12, -16, 0, -20, -12, -16, -19, -6,
    -19,  6, -12, 16,  0, 20
};

static void WrDigit(unsigned int n, int x, int y, bool GreenFlag);
static void WriteLsdigit(unsigned int d, int x, int y);
static void Splodge(int h, int w, int x, int y);
static void WriteBaro(unsigned int b, bool Hg, int x, int y);

/* --------------------------------------------------- */
void Alt_Altimeter(int AltX, int AltY, float z,
                   unsigned int Baro, bool BaroHg, NavDefn_FCUKnob BaroKnob,
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
	char         str[20];
	
	Glib_LoadIdentity();
	Glib_Translate((float) AltX, (float) AltY);
	
    if (Systems_Failures[17])
	{
	    Glib_SetFont(Glib_EFONT12, 10);
	    Ai_VBox("ALT", TapeHalfWidth, 0);
		return;
	}
	
    if (z <= 0.0)
    {
        Altitude = 0.0;
    }
    else
    {
        Altitude = z * 3.280840;
    }
    if (BaroKnob == NavDefn_Pushed)
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
    Glib_LineWidth(2.0);
    Glib_Colour(Glib_WHITE);
    Glib_ClipWindow(AltX - 10, AltY - TapeHalfHeight, TapeWidth + 10, TapeHeight);

    Glib_PushMatrix();
    Glib_Translate(0.0, (float) -Offset);
    y = -TapeHalfHeight;
    do
    {
        if (a1 >= 0)
        {
            Glib_Draw(0, y, 15, y);
            if (a1 % 2 == 0)
            {
                Glib_SetFont(Glib_EFONT16, 12);
                if (a1 >= 100)
                {
                    Glib_Char((unsigned int) a1 / 100 + '0', 17, y - 8);
                }
                if (a1 >= 10)
                {
                    Glib_Char((unsigned int) a1 % 100 / 10 + '0', 27, y - 8);
                }
                Glib_SetFont(Glib_EFONT12, 10);
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

    Glib_PopMatrix();
	
    Glib_Colour(Glib_MAGENTA);
    Glib_LineWidth(2.0);

    Offset = (Altitude - (float) FCUAlt) * 0.56;
    if (Offset < (float) (-TapeHalfHeight))
    {
        Offset = (float) (-TapeHalfHeight);
    }
    else if (Offset > (float) TapeHalfHeight)
    {
        Offset = (float) TapeHalfHeight;
    }

    Glib_PushMatrix();
    Glib_Translate(0.0, (float) -Offset);
    Glib_Line_Loop(0, 0, 6, AltBug);
	Glib_PopMatrix();
	
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(16, -35, TapeWidth - 16, 70);
    Glib_ClipWindow(AltX - 10, AltY - TapeHalfHeight, TapeWidth, TapeHeight);

    Glib_RemoveClipWindow();
    Glib_Colour(Glib_BLACK);
	Glib_Rectangle(25, -28, 82, 56);
	Glib_Triangle(25, -10, 13, 0, 25, 10);
    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    Glib_Line_Loop(0, 0, 6, DigitsBox);
	if (Metric)
	{
	    Glib_Line_Strip(0, 0, 3, DigitsBox4);
	}
    Glib_ClipWindow(AltX + 25, AltY - 23, 82, 46);
    LsDigit = AltitudeDigits % 100;
    Glib_SetFont(Glib_EFONT16, 10);
    n = AltitudeDigits % 100;
    if (n > 80)
    {
        Dig3Offset = (int) ((float) (n - 80) / 20.0 * 38.0);
    }
    else
    {
        Dig3Offset = 0;
    }
    WrDigit((AltitudeDigits / 100 + 1) % 10, 35 + 30,  26 + 4 - Dig3Offset, false);
    WrDigit(AltitudeDigits / 100 % 10,       35 + 30, -12 + 4 - Dig3Offset, false);
    WrDigit((AltitudeDigits / 100 + 9) % 10, 35 + 30, -50 + 4 - Dig3Offset, false);
    Glib_SetFont(Glib_EFONT24, 12);
    n = AltitudeDigits % 1000;
    if (n >= 980)
    {
        Dig2Offset = (int) ((float) (n - 980) / 20.0 * 38.0);
    }
    else
    {
        Dig2Offset = 0;
    }
    WrDigit((AltitudeDigits / 1000 + 1) % 10, 30 + 15,  26 - Dig2Offset, false);
    WrDigit(AltitudeDigits / 1000 % 10,       30 + 15, -12 - Dig2Offset, false);
    WrDigit((AltitudeDigits / 1000 + 9) % 10, 30 + 15, -50 - Dig2Offset, false);
    n = AltitudeDigits % 10000;
    if (n >= 9980)
    {
        Dig1Offset = (int) ((float) (n - 9980) / 20.0 * 38.0);
    }
    else
    {
        Dig1Offset = 0;
    }
    WrDigit((AltitudeDigits / 10000 + 1) % 10, 27,  26 - Dig1Offset, AltitudeDigits < 9000);
    WrDigit(AltitudeDigits / 10000 % 10,       27, -12 - Dig1Offset, AltitudeDigits < 10000);
    WrDigit((AltitudeDigits / 10000 + 9) % 10, 27, -50 - Dig1Offset, AltitudeDigits < 11000);
    LSDigOffset = (Altitude - (float) (AltitudeDigits / 20 * 20)) / 20.0 * 22.0;
    LsDigit     = LsDigit / 20 * 20;

    Glib_PushMatrix();
    Glib_Translate((float) 0.0, (float) -LSDigOffset);
    WriteLsdigit(LsDigit, 77, -8);
    WriteLsdigit((LsDigit + 20) % 100, 77, 14);
    WriteLsdigit((LsDigit + 40) % 100, 77, 36);
    WriteLsdigit((LsDigit + 80) % 100, 77, -30);
    Glib_PopMatrix();

    Glib_RemoveClipWindow();

    if (Metric)
	{
    	Glib_Colour(Glib_BLACK);
	    Glib_Rectangle(26, 29, 80, 18);
        Glib_Colour(Glib_WHITE);
        Glib_SetFont(Glib_EFONT12, 10);
		sprintf(str, "%5d", (int) Metric_Altitude);
		Glib_Chars(str, 40, 32);
	}
	
    Glib_Colour(Glib_MAGENTA);
    Glib_SetFont(Glib_EFONT24, 12);
    n = FCUAlt;

    if (n >= 10000)
    {
        Glib_Char(n / 10000 + '0', TapeWidth / 2 - 35, TapeHalfHeight + 10);
    }
    Glib_Char(n % 10000 / 1000 + '0', TapeWidth / 2 - 18, TapeHalfHeight + 10);
    Glib_SetFont(Glib_EFONT16, 12);
	sprintf(str, "%03d", n%1000);
	Glib_Chars(str, TapeWidth / 2, TapeHalfHeight + 14);

	if (Metric)
	{
	    Glib_Char('M', TapeWidth / 2 + 39, TapeHalfHeight + 14);
	}
}

/* --------------------------------------------------- */
static void WrDigit(unsigned int n, int x, int y, bool GreenFlag)
{
    if (GreenFlag)
    {
        Glib_Colour(Glib_GREEN);
        Splodge(24, 10, x + 7, y);
        Glib_Colour(Glib_WHITE);
    }
    else
    {
        Glib_Char((unsigned char) n + '0', x, y);
    }
}

/* --------------------------------------------------- */
static void WriteLsdigit(unsigned int d, int x, int y)
{
    Glib_SetFont(Glib_EFONT16, 8);
    Glib_Char((unsigned char) (d / 10) + '0', x, y);
    Glib_Char((unsigned char) (d % 10) + '0', x + 12, y);
}

/* --------------------------------------------------- */
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

/* --------------------------------------------------- */
void Alt_Baro(unsigned int Baro, bool BaroHg, NavDefn_FCUKnob BaroKnob)
{
    int y = BaroY;

    Glib_Colour(Glib_GREEN);
    if (BaroKnob == NavDefn_Pulled)
    {
        Glib_SetFont(Glib_EFONT24, 16);
        Glib_Chars("STD", BaroX + 10, BaroY - 5);
    }
    else
    {
        y = BaroY - 10;
        Glib_SetFont(Glib_EFONT16, 10);
        WriteBaro(Baro, BaroHg, BaroX, y);
        if (BaroHg)
        {
            Glib_Chars("IN", BaroX + 60, y);
        }
        else
        {
            Glib_Chars("HPA", BaroX + 55, y);
        }
    }
}

/* --------------------------------------------------- */
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

/* --------------------------------------------------- */
void Alt_RadioAltimeter(int RadAltX, int RadAltY, float h)
{
    unsigned int d;
    RadAltMode   m;
    unsigned int i;

    Glib_LoadIdentity();
    Glib_Translate((float) RadAltX, (float) RadAltY);
	
    Glib_SetFont(Glib_EFONT16, 10);
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
	    char str[20];
		
        Glib_Colour(Glib_WHITE);
        if (d >= 1000)
        {
			sprintf(str, "%3d0", d / 10);
            Glib_Chars(str, -20, -8);
        }
        else
        {
		    sprintf(str, "%3d", d);
            Glib_Chars(str, -18, -8);
        }
    }
}

/* --------------------------------------------------- */
void BEGIN_Alt()
{
}
