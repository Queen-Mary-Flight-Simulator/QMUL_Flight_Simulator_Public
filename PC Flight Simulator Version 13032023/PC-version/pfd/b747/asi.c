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
#include <math.h>
#include <stdbool.h>

#include <SIM/glib.h>
#include <SIM/maths.h>
#include <SIM/weather.h>

#include "asi.h"
#include "ai.h"
#include "systems.h"
#include "model.h"

static int DigitsBox[16] = 
{
    0, -31, 0, 31, 55, 31, 55, 10, 67, 0, 55, -10, 55, -31
};

static int SpeedBug[10] = 
{
    0, 0, 13, 9, 36, 9, 36, -9, 13, -9
};

#define TapeWidth         83
#define TapeHeight        444
#define TapeHalfHeight    (TapeHeight / 2)
#define TapeHalfWidth     (TapeWidth / 2)
#define TickSpacing       37

static void Dot(int x, int y);

/* ---------------------------------------------------- */
void Asi_Asi(int AsiX, int AsiY, float IAS, unsigned int IAS_Ref,
             float UDot, float MachNumber, unsigned int FCUSpeed,
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
    char         str[20];
    
    Glib_LoadIdentity();
    Glib_Translate((float) AsiX, (float) AsiY);
    Glib_PushMatrix();
    
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
        Glib_Colour(Glib_WHITE);
        Glib_SetFont(Glib_EFONT12, 10);
        Ai_VBox("SPD", TapeHalfWidth, 0);
        return;
    }
    
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
    Glib_SetFont(Glib_EFONT16, 12);

    Glib_LineWidth(2.0);
    Glib_Colour(Glib_WHITE);
    Glib_ClipWindow(AsiX, AsiY - TapeHalfHeight, TapeWidth + 40, TapeHeight);
    Glib_Translate(0.0, -(float) Offset);
    y = -TapeHalfHeight;

    do
    {
        if (s1 >= 30)
        {
            Glib_Draw(TapeWidth - 15, y, TapeWidth - 1, y);
            if (s1 % 20 == 0)
            {
                sprintf(str, "%03d", s1);
                Glib_Chars(str, 23, y - 6);
            }
        }
        y  = y + TickSpacing;
        s1 = s1 + 10;
    } while (!(s1 >= s2));

    Glib_PopMatrix();
    Glib_PushMatrix();
    
    Glib_Colour(Glib_MAGENTA);
    Glib_LineWidth(2.0);
    Offset = (Kts - BugSpeed) * 3.7;
    if (Offset < (float) (-TapeHalfHeight))
    {
        Offset = (float) (-TapeHalfHeight);
    }
    else if (Offset > (float) TapeHalfHeight)
    {
        Offset = (float) TapeHalfHeight;
    }

    Glib_Translate(0.0, -(float) Offset);
    Glib_Line_Loop(68, 0, 4, SpeedBug);
    Glib_PopMatrix();
    
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(0, -36, 60, 71);

    Glib_RemoveClipWindow();
    Glib_Colour(Glib_BLACK);
    Glib_Rectangle(0, -31, 55, 62);
    Glib_Triangle(55, 10, 67, 0, 55, -10);
    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    Glib_Line_Loop(0, 0, 6, DigitsBox);

    Glib_ClipWindow(AsiX, AsiY - 28, 55, 56);

    Dig3Offset = (int) (Kts * 28.0) % 28;
    Glib_SetFont(Glib_EFONT24, 16);
    Glib_Char(KtsDigits % 10 + '0',       34, -12 - Dig3Offset);
    Glib_Char((KtsDigits + 1) % 10 + '0', 34,  16 - Dig3Offset);
    Glib_Char((KtsDigits + 2) % 10 + '0', 34,  44 - Dig3Offset);
    Glib_Char((KtsDigits + 9) % 10 + '0', 34, -40 - Dig3Offset);

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
        Glib_Char(KtsDigits % 100 / 10 + '0',       17, -12 - Dig2Offset);
        Glib_Char((KtsDigits + 1) % 100 / 10 + '0', 17,  28 - Dig2Offset);
        Glib_Char((KtsDigits + 9) % 100 / 10 + '0', 17, -52 - Dig2Offset);
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
        Glib_Char(KtsDigits / 100 + '0',       0, -12 - Dig1Offset);
        Glib_Char((KtsDigits + 1) / 100 + '0', 0,  28 - Dig1Offset);
        Glib_Char((KtsDigits + 1) / 10 + '0',  0, -52 - Dig1Offset);
    }

    Glib_RemoveClipWindow();

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
        Glib_LineWidth(2.0);
        Glib_Draw(67, 0, 67, Trend);
        if (Trend > 0)
        {
            Glib_Triangle(67-6, Trend, 67, Trend+6, 67+6, Trend);
        }
        else
        {
            Glib_Triangle(67-6, Trend, 67+6, Trend, 67, Trend-6);
        }
    }
    
    MachDigits  = intround(MachNumber * 1000.0);
    Glib_Colour(Glib_WHITE);
    Dot(15, -260);
    sprintf(str, "%03d", MachDigits);
    Glib_Chars(str, 20, -260);

    Glib_Colour(Glib_MAGENTA);
    if (SPD_Mode)
    {
        sprintf(str, "%03d", FCUSpeed);
        Glib_Chars( str, 10, TapeHalfHeight + 10);
    }
    else
    {
        Dot(20, TapeHalfHeight + 10);
        sprintf(str, "%02d", FCUSpeed);
        Glib_Chars(str, 35,  TapeHalfHeight + 10);
    }
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
