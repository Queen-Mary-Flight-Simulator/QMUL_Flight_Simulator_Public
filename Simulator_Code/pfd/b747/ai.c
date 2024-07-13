/* +------------------------------+---------------------------------+
   | Module      : ai.c           | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-03      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 Attitude Indicator                |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "ai.h"
#include "systems.h"

#define AiHeight           324
#define AiWidth            302

#define FDLimit            112
#define PI                 3.141592654
#define ONERAD             (180.0 / PI)
#define AiHalfWidth        (AiWidth / 2)
#define AiHalfHeight       (AiHeight / 2)
#define PixPerDegree       6.4
#define DEG95              (95.0 / ONERAD)
#define BankAngleRadius    153

static int OldGlideslope_h;
static int OldLocaliser_h;

const int PxTab[18] =  /* X width in pixels */
{   0,
   53,  13, 26, 13, 53, 13, 26, 13,
  250,
   13,  26, 13, 53, 13, 26, 13, 53
};

const int PyTab[18] =  /* Y displacement in pixels */
{   0,
 -128, -112, -96, -80, -64, -48, -32, -16,
    0,
   16,     32,  48,  64,  80,  96, 112, 128
};

static int Circle16Tab[38] = {
    16,   0,  15,   5, 12,  10,  8,  14, 3,  16, -3,  16, -8,  14, -12, 10, -15, 5, -16, 0,
    -15, -5, -12, -10, -8, -14, -3, -16, 3, -16,  8, -14, 12, -10,  15, -5,  16, 0
};

static int Circle5Tab[38] = {
     5,  0,  5,  2,  4,  3,  3,  4, 1,  5, -1,  5, -3,  4, -4,  3, -5, 2, -5, 0, 
    -5, -2, -4, -3, -3, -4, -1, -5, 1, -5,  3, -4,  4, -3,  5, -2,  5, 0
};

void MarkerLamp(char Str[], bool Mode, unsigned int Col);
static void Circle16(int x, int y);
static void Circle5(int x, int y);

/* ------------------------------------------------- */
void Ai_AttitudeIndicator(int AiX, int AiY, float Pitch, float Roll, float Slip)
{
    float        Pitch_Deg;
    float        Roll_Deg;
    GLfloat      Pitch_Pix;
    float        PitchLimit;
    int          yPitchLimit;
    int          SideSlip;
    unsigned int i;
	
    if (Systems_Failures[15])
    {
        Glib_SetFont(Glib_EFONT12, 10);
        Ai_HBox("ATT", 0, 30);
        return;
    }
    
    Pitch_Deg = -Pitch * ONERAD;
    Pitch_Pix = Pitch_Deg * PixPerDegree;
    Roll_Deg  = Roll * ONERAD;

    Glib_ClipWindow(AiX - AiHalfWidth, AiY - AiHalfHeight, AiWidth, AiHeight);

    Glib_LoadIdentity();
    Glib_Translate((float) AiX, (float) AiY);
    Glib_PushMatrix();

    Glib_Translate(0.0, Pitch_Pix);
    Glib_Rotate(Roll_Deg);

    Glib_Colour(Glib_BROWN);
    Glib_Rectangle(-5 * AiWidth, -5 * AiWidth, 10 * AiWidth, 5 * AiWidth);
    Glib_Colour(Glib_BLUE);
    Glib_Rectangle(-5 * AiWidth, 0, 10 * AiWidth, 5 * AiWidth);

    /* vector version of pitch lines */
    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(3.0);
    for (i = 1; i <= 17; i += 1)
    {
        Glib_Draw(-PxTab[i], PyTab[i], PxTab[i], PyTab[i]);
    }

    Glib_SetFont(Glib_EFONT12, 10);
    Glib_Chars("10", -PxTab[5] - 27, PyTab[5] - 6);
    Glib_Chars("10", PxTab[5] + 7, PyTab[5] - 6);
    Glib_Chars("10", -PxTab[5] - 27, -PyTab[5] - 6);
    Glib_Chars("10", PxTab[5] + 7, -PyTab[5] - 6);
    Glib_Chars("20", -PxTab[17] - 27, PyTab[17] - 6);
    Glib_Chars("20", PxTab[17] + 7, PyTab[17] - 6);
    Glib_Chars("20", -PxTab[17] - 27, -PyTab[17] - 6);
    Glib_Chars("20", PxTab[17] + 7, -PyTab[17] - 6);
	
	/* alternative method: draw pitch lines as a texture */
	//Glib_DrawTexture(-260, -140, 520, 280, 0.7-0.26, 0.6-0.14, 0.7+0.26, 0.6+0.14, 1.0);

    PitchLimit = 15.0;
    Glib_Colour(Glib_YELLOW);
    Glib_LineWidth(3.0);
    yPitchLimit = (int) (PitchLimit * PixPerDegree);
    Glib_Draw(-74, yPitchLimit, -49, yPitchLimit);
    Glib_Draw(-49, yPitchLimit + 1, -49, yPitchLimit - 6);
    Glib_Draw(-69, yPitchLimit, -74, yPitchLimit + 12);
    Glib_Draw(-63, yPitchLimit, -68, yPitchLimit + 12);
    Glib_Draw(-57, yPitchLimit, -62, yPitchLimit + 12);
    Glib_Draw(74, yPitchLimit, 49, yPitchLimit);
    Glib_Draw(49, yPitchLimit + 1, 49, yPitchLimit - 6);
    Glib_Draw(69, yPitchLimit, 74, yPitchLimit + 12);
    Glib_Draw(63, yPitchLimit, 68, yPitchLimit + 12);
    Glib_Draw(57, yPitchLimit, 62, yPitchLimit + 12);

    Glib_PopMatrix();

	Glib_PushMatrix();
    Glib_Rotate(Roll_Deg);

    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    Glib_Draw(0, BankAngleRadius, -9, BankAngleRadius - 12);
    Glib_Draw(-9, BankAngleRadius - 12, 9, BankAngleRadius - 12);
    Glib_Draw(9, BankAngleRadius - 12, 0, BankAngleRadius);
    SideSlip = (int) (-Slip * 180.0);
    Glib_Draw(-9 + SideSlip, BankAngleRadius - 18, -9 + SideSlip, BankAngleRadius - 12);
    Glib_Draw(-9 + SideSlip, BankAngleRadius - 12, 9 + SideSlip, BankAngleRadius - 12);
    Glib_Draw(9 + SideSlip, BankAngleRadius - 12, 9 + SideSlip, BankAngleRadius - 18);
    Glib_Draw(9 + SideSlip, BankAngleRadius - 18, -9 + SideSlip, BankAngleRadius - 18);
    
    Glib_PopMatrix();

    Glib_DrawTexture(-AiHalfWidth, -AiHalfHeight, AiWidth, AiHeight, 0.2-0.151, 0.6-0.162, 0.2+0.151, 0.6+0.162, 1.0);
	
//    Glib_Colour(Glib_BLACK);
//    Glib_LineWidth(1.0);
//
//    for (y = 0; y <= 24; y += 1)
//    {
//        dx = Fillets[y];
//        Glib_Draw(AiHalfWidth - dx, AiHalfHeight - 24 + y,
//                  AiHalfWidth + 1, AiHalfHeight - 24 + y);
//        Glib_Draw(-AiHalfWidth - 1, AiHalfHeight - 24 + y,
//                  -AiHalfWidth + dx, AiHalfHeight - 24 + y);
//        Glib_Draw(AiHalfWidth - dx, -AiHalfHeight + 24 - y,
//                  AiHalfWidth + 1, -AiHalfHeight + 24 - y);
//        Glib_Draw(-AiHalfWidth - 1, -AiHalfHeight + 24 - y,
//                  -AiHalfWidth + dx, -AiHalfHeight + 24 - y);
//    }
    Glib_RemoveClipWindow();
}

/* ------------------------------------------------- */
void Ai_FlightDirector(int x, int y, bool Enabled)
{
    if (Enabled)
    {
        if (Systems_Failures[20])
        {
            Glib_SetFont(Glib_EFONT12, 10);
            Ai_HBox("FD", 100, 100);
            return;
        }
        
        Glib_Colour(Glib_MAGENTA);
        Glib_LineWidth(4.0);
        if (x > FDLimit)
        {
            x = FDLimit;
        }
        else if (x < -FDLimit)
        {
            x = -FDLimit;
        }
        if (y > FDLimit)
        {
            y = FDLimit;
        }
        else if (y < -FDLimit)
        {
            y = -FDLimit;
        }
        Glib_Draw(x, -80, x, 80);
        Glib_Draw(-80, y, 80, y);
    }
}

/* ------------------------------------------------- */
void Ai_GlideSlope(float GlideSlopeError, bool ILSMode, bool Selected)
{
    int h;
    int p;

    if (Systems_Failures[4])
    {
        Glib_SetFont(Glib_EFONT12, 10);
        Ai_VBox("G/S", AiHalfWidth - 14, 0);
        return;
    }
    h = intround(GlideSlopeError * 7039.2);
    if (h > OldGlideslope_h)
    {
        h = OldGlideslope_h + 1;
    }
    else if (h < OldGlideslope_h)
    {
        h = OldGlideslope_h - 1;
    }
    if (h < -108)
    {
        h = -108;
    }
    else if (h > 108)
    {
        h = 108;
    }
    if (Selected)
    {
        Glib_Colour(Glib_WHITE);
        Glib_LineWidth(2.0);
        Glib_Draw(AiHalfWidth - 22, 0, AiHalfWidth, 0);
        Glib_Triangle(AiHalfWidth - 19, h, AiHalfWidth - 11, h + 8, AiHalfWidth - 11, h - 8);
        Glib_Triangle(AiHalfWidth - 11, h + 8, AiHalfWidth - 3, h, AiHalfWidth - 11, h - 8);
        for (p = -86; p <= 86; p += 43)
        {
            if (p != 0)
            {
                Circle5(AiHalfWidth - 10, p);
            }
        }
   }
    OldGlideslope_h = h;
}

/* ------------------------------------------------- */
void Ai_Localiser(float LocaliserError, bool ILSMode, bool Selected)
{
    int h, p;

    if (Systems_Failures[3])
    {
        Glib_SetFont(Glib_EFONT12, 10);
        Ai_HBox("LOC", 0, -AiHalfHeight + 11);
        return;
    }

    if (ILSMode)
    {
        h = intround(LocaliserError * 2463.72);
    }
    else
    {
        if (LocaliserError < -DEG95)
        {
            LocaliserError = -PI - LocaliserError;
        }
        else if (LocaliserError > DEG95)
        {
            LocaliserError = PI - LocaliserError;
        }
        h = intround(LocaliserError * 492.74);
    }
    if (h > OldLocaliser_h)
    {
        h = OldLocaliser_h + 1;
    }
    else if (h < OldLocaliser_h)
    {
        h = OldLocaliser_h - 1;
    }
    if (h < -108)
    {
        h = -108;
    }
    else if (h > 108)
    {
        h = 108;
    }
    if (Selected)
    {
        Glib_Colour(Glib_WHITE);
        Glib_LineWidth(2.0);
        Glib_Draw(0, -AiHalfHeight + 22, 0, -AiHalfHeight);
        Glib_Triangle(h - 8, -AiHalfHeight + 11, h + 0, -AiHalfHeight + 19, h + 0, -AiHalfHeight + 3);
        Glib_Triangle(h + 0, -AiHalfHeight + 19, h + 8, -AiHalfHeight + 11, h + 0, -AiHalfHeight + 3);
        for (p = -86; p <= 86; p += 43)
        {
            if (p != 0)
            {
                Circle5(p, -AiHalfHeight + 12);
            }
        }
    }
    OldLocaliser_h = h;
}

/* ------------------------------------------------- */
void Ai_Markers(bool OMLamp, bool MMLamp, bool IMLamp, bool LampTest)
{
    if (OMLamp)
    {
        MarkerLamp("OM", OMLamp, Glib_CYAN);
    }
    if (MMLamp)
    {
        MarkerLamp("MM", MMLamp, Glib_YELLOW);
    }
    if (LampTest)
    {
        MarkerLamp("FT", LampTest, Glib_WHITE);
    }
}

/* ------------------------------------------------- */
void MarkerLamp(char Str[], bool Mode, unsigned int Col)
{
    if (Mode)
    {
        Glib_LineWidth(2.0);
        Glib_SetFont(Glib_EFONT12, 12);
        Glib_Colour(Glib_WHITE);
        Circle16(124, 136);
        Glib_Colour(Col);
        Glib_Char(Str[0], 111, 130);
        Glib_Char(Str[1], 123, 130);
    }
}

/* ------------------------------------------------- */
static void Circle16(int x, int y)
{
    Glib_Line_Strip(x, y, 18, Circle16Tab);
}

/* ------------------------------------------------- */
static void Circle5(int x, int y)
{
    Glib_Line_Strip(x, y, 18, Circle5Tab);
}

/* ------------------------------------------------- */
void Ai_HBox(char str[], int x, int y)
{
    int x1, y1;
    int xs, ys;
    
    xs = Glib_StringSize(str) + 10;
    ys = 18;
    x1 = x - xs / 2;
    y1 = y - 10;
    Glib_Colour(Glib_YELLOW);
    Glib_Rectangle(x1, y1, xs, ys);
    Glib_Colour(Glib_BLACK);
    Glib_Chars(str, x1 + 7, y1 + 3);
}

/* ------------------------------------------------- */
void Ai_VBox(char str[], int x, int y)
{
    int s = strlen(str);
    int x1, y1;
    int xs, ys;
    int i;
    int p;
    
    xs = 20;
    ys = s * 16 + 4;
    x1 = x - 10;
    y1 = y - ys / 2;	
    Glib_Colour(Glib_YELLOW);
    Glib_Rectangle(x1, y1, xs, ys);
    Glib_Colour(Glib_BLACK);
    p = ys / 2 - 16;
    for (i=0; i<s; i+=1)
    {
        Glib_Char(str[i], x1 + 6, p);
        p -= 16;
    }
}

/* ------------------------------------------------- */
void BEGIN_Ai()
{
    OldGlideslope_h = 0;
    OldLocaliser_h  = 0;
}
