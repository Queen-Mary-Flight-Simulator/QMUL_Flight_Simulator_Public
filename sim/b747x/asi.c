/* +------------------------------+---------------------------------+
   | Module      : asi.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-05      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EFIS Airspeed Indicator           |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <GL/gl.h>
#include <math.h>

#include <stdbool.h>

#include <SIM/glib.h>
#include <SIM/maths.h>
#include <SIM/weather.h>

#include "asi.h"
#include "systems.h"
#include "model.h"

static int DigitsBox[16] = {
    0, -31, 0, 31, 55, 31, 55, 10, 67, 0, 55, -10, 55, -31, 0, -31
};

static int UpArrow[6] = {
    -6, 0, 0, 6, 6, 0
};

static int DownArrow[6] = {
    -6, 0, 0, -6, 6, 0
};

static int SpeedBug[12] = {
    0, 0, 13, 9, 36, 9, 36, -9, 13, -9, 0, 0
};

#define TapeWidth         83
#define TapeHeight        444
#define TapeHalfHeight    (TapeHeight / 2)
#define TapeHalfWidth     (TapeWidth / 2)
#define TickSpacing       37

static void Wrn3(int d, int x, int y);
static void Dot(int x, int y);

/* ---------------------------------------------------- */
void Asi_Asi(int AsiX, int AsiY, float IAS, unsigned int IAS_Ref,
             float UDot, float GroundSpeed, float MachNumber, unsigned int FCUSpeed,
             bool SPD_Mode)
{
    float        Kts;
    float        Offset;
    int          Dig1Offset;
    int          Dig2Offset;
    int          Dig3Offset;
    unsigned int KtsDigits;
    unsigned int MachDigits;
    int          MinSpeed;
    int          MaxSpeed;
    int          y;
    int          s1;
    int          s2;
    int          Trend;
    unsigned int n;
    float        BugSpeed;
	
    if (SPD_Mode)
	{
    	BugSpeed = (float) FCUSpeed;
	}
	else
    {
        BugSpeed = Weather_Mach_to_Kts(-Model_Pz, (float) FCUSpeed / 100.0);
    }
	
    if (Systems_Failures[14])
	{
	    Glib_SetFont(Glib_GFONT12, 10);
	    Glib_VBox("SPD", TapeHalfWidth, 0);
		return;
	}
	
    Glib_AntiAliasing(true);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(0, -TapeHalfHeight, TapeWidth, TapeHeight);
    Kts = IAS * 1.944;
    if (Kts < 30.0)
    {
        Kts = 30.0;
    }
    KtsDigits = (unsigned int) Kts;
    MinSpeed  = intround(Kts - 0.5) / 10 * 10 - 60;
    MaxSpeed  = MinSpeed + 140;
    Offset = (Kts - (float) (KtsDigits / 10 * 10)) * 3.7;
    s1     = MinSpeed;
    s2     = MaxSpeed;
    Glib_SetFont(Glib_GFONT16, 12);
    glLineWidth(2.0);
    Glib_Colour(Glib_WHITE);
    Glib_ClipWindow(AsiX, AsiY - TapeHalfHeight, TapeWidth + 40, TapeHeight);
    glPushMatrix();
    glTranslatef(0.0, (float) -Offset, 0.0);
    y = -TapeHalfHeight;
    do
    {
        if (s1 >= 30)
        {
            Glib_Draw(TapeWidth - 15, y, TapeWidth - 1, y);
            if (s1 % 20 == 0)
            {
                Wrn3(s1, 24, y - 6);
            }
        }
        y  = y + TickSpacing;
        s1 = s1 + 10;
    } while (!(s1 >= s2));
    glTranslatef(0.0, (float) Offset, 0.0);
    glPopMatrix();
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(0, -36, 60, 71);
    Glib_SetFont(Glib_GFONT24, 12);
    Glib_Colour(Glib_MAGENTA);
    glLineWidth(2.0);
    Offset = (Kts - BugSpeed) * 3.7;
    if (Offset < (float) (-TapeHalfHeight))
    {
        Offset = (float) (-TapeHalfHeight);
    }
    else if (Offset > (float) TapeHalfHeight)
    {
        Offset = (float) TapeHalfHeight;
    }
    glPushMatrix();
    glTranslatef(0.0, (float) -Offset, 0.0);
    Glib_DrawLines(6, 68, 0, SpeedBug);
    glTranslatef(0.0, (float) Offset, 0.0);
    glPopMatrix();
    glDisable(GL_SCISSOR_TEST);
    Glib_Colour(Glib_BLACK);
    Glib_DrawPolygon(8, 0, 0, DigitsBox);
    Glib_Colour(Glib_WHITE);
    Glib_DrawLines(8, 0, 0, DigitsBox);
    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_GFONT24, 16);
    glLineWidth(2.0);
    Glib_ClipWindow(AsiX, AsiY - 26, 55, 52);
    Dig3Offset = (int) (Kts * 28.0) % 28;
    Glib_Char(KtsDigits % 10 + '0', 40, -12 - Dig3Offset);
    Glib_Char((KtsDigits + 1) % 10 + '0', 40, 16 - Dig3Offset);
    Glib_Char((KtsDigits + 2) % 10 + '0', 40, 44 - Dig3Offset);
    Glib_Char((KtsDigits + 9) % 10 + '0', 40, -40 - Dig3Offset);
    n = KtsDigits % 10;
    if (n >= 9)
    {
        Dig2Offset = (int) ((Kts - (float) (KtsDigits / 10 * 10 + 9)) * 40.0);
    }
    else
    {
        Dig2Offset = 0;
    }
    if (KtsDigits >= 10)
    {
        Glib_Char(KtsDigits % 100 / 10 + '0', 25, -12 - Dig2Offset);
        Glib_Char((KtsDigits + 1) % 100 / 10 + '0', 25, 28 - Dig2Offset);
        Glib_Char((KtsDigits + 9) % 100 / 10 + '0', 25, -52 - Dig2Offset);
    }
    n = KtsDigits % 100;
    if (n >= 99)
    {
        Dig1Offset = (int) ((Kts - (float) (KtsDigits / 10 * 10 + 9)) * 40.0);
    }
    else
    {
        Dig1Offset = 0;
    }
    if (KtsDigits >= 99)
    {
        Glib_Char(KtsDigits / 100 + '0', 10, -12 - Dig1Offset);
        Glib_Char((KtsDigits + 1) / 100 + '0', 10, 28 - Dig1Offset);
        Glib_Char((KtsDigits + 1) / 10 + '0', 10, -52 - Dig1Offset);
    }
    glDisable(GL_SCISSOR_TEST);
    Trend = intround(UDot * 19.44 * 3.6);
    if (IAS < 15.432)
    {
        Trend = 0;
    }
    if (Trend > -16 && Trend < 16)
    {
        Trend = 0;
    }
    else if (Trend > 218)
    {
        Trend = 218;
    }
    else if (Trend < -218)
    {
        Trend = -218;
    }
    if (Trend != 0)
    {
        Glib_Colour(Glib_WHITE);
        glLineWidth(2.0);
        Glib_Draw(67, 0, 67, Trend);
        if (Trend > 0)
        {
            Glib_DrawPolygon(3, 67, Trend, UpArrow);
        }
        else
        {
            Glib_DrawPolygon(3, 67, Trend, DownArrow);
        }
    }
	
    MachDigits  = intround(MachNumber * 1000.0);
    Glib_Colour(Glib_WHITE);
    Dot(20, -260);
    Wrn3(MachDigits, 31, -260);

    Glib_Colour(Glib_MAGENTA);
    if (SPD_Mode)
    {
        Wrn3(FCUSpeed, 20, TapeHalfHeight + 10);
    }
    else
    {
        Dot(20, TapeHalfHeight + 10);
        Glib_Char((FCUSpeed % 100) / 10 + '0', 35,  TapeHalfHeight + 10);
	    Glib_Char(FCUSpeed % 10 + '0', 50,  TapeHalfHeight + 10);
    }
}

/* ---------------------------------------------------- */
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

/* ---------------------------------------------------- */
static void Dot(int x, int y)
{
    Glib_Draw(x, y, x+3, y);
    Glib_Draw(x, y+1, x+3, y+1);
}

/* ---------------------------------------------------- */
void Asi_VSpeeds(float IAS, float Mass, unsigned int FlightMode)
{
}

/* ---------------------------------------------------- */
void BEGIN_Asi()
{
}
