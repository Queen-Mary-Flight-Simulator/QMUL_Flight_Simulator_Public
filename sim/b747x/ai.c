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

#include <GL/gl.h>
#include <GL/glu.h>

#include <stdbool.h>

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

static int PxTab[18] = { /* X width in pixels */
    0,
    53,  13, 26, 13, 53, 13, 26, 13,
    250,
    13,  26, 13, 53, 13, 26, 13, 53
};

static int PyTab[18] = { /* Y displacement in pixels */
    0,
    -128, -112, -96, -80, -64, -48, -32, -16,
    0,
    16,     32,  48,  64,  80,  96, 112, 128
};

static int Fillets[25] = {
    0,   0,  0,  0, 0, 1, 1, 1, 1, 2,
    2,   3,  3,  4, 5, 5, 6, 7, 8, 9,
    11, 12, 14, 17, 24
};

static int Diamond[8] = {
    -8, 0, 0, 8, 8, 0, 0, -8
};

static int Circle16Tab[38] = {
    16,   0,  15,   5, 12,  10,  8,  14, 3,  16, -3,  16, -8,  14, -12, 10, -15, 5, -16, 0,
    -15, -5, -12, -10, -8, -14, -3, -16, 3, -16,  8, -14, 12, -10,  15, -5,  16, 0
};

void MarkerLamp(char Str[], bool Mode, unsigned int Col);
void Circle16(int x, int y);

void Ai_AttitudeIndicator(int AiX, int AiY, float Pitch, float Roll, float Slip)
{
    float        Pitch_Deg;
    float        Roll_Deg;
    GLfloat      Pitch_Pix;
    float        PitchLimit;
    int          y;
    int          yPitchLimit;
    int          SideSlip;
    unsigned int i;
    int          dx;

    if (Systems_Failures[15])
	{
	    Glib_SetFont(Glib_GFONT12, 10);
	    Glib_HBox("ATT", 0, 30);
		return;
	}
	
    Pitch_Deg = -Pitch * ONERAD;
    Pitch_Pix = Pitch_Deg * PixPerDegree;
    Roll_Deg  = Roll * ONERAD;
    Glib_AntiAliasing(true);
    glPushMatrix();
    Glib_ClipWindow(AiX - AiHalfWidth, AiY - AiHalfHeight, AiWidth, AiHeight);
    glRotatef(Roll_Deg, 0.0, 0.0, 1.0);
    glTranslatef(0.0, Pitch_Pix, 0.0);
    Glib_Colour(Glib_BROWN);
    glRectf((float) (-5 * AiWidth), 0.0, (float) (5 * AiWidth), (float) (-5 * AiWidth));
    Glib_Colour(Glib_BLUE);
    glRectf((float) (-5 * AiWidth), (float) (5 * AiWidth), (float) (5 * AiWidth), 0.0);
    glColor3f(1.0, 1.0, 1.0);
    glLineWidth(1.5);
    for (i = 1; i <= 17; i += 1)
    {
        Glib_Draw(-PxTab[i], PyTab[i], PxTab[i], PyTab[i]);
    }
    glLineWidth(1.5);
    Glib_SetFont(Glib_GFONT12, 10);
    Glib_Chars("10", -PxTab[5] - 27, PyTab[5] - 6);
    Glib_Chars("10", PxTab[5] + 7, PyTab[5] - 6);
    Glib_Chars("10", -PxTab[5] - 27, -PyTab[5] - 6);
    Glib_Chars("10", PxTab[5] + 7, -PyTab[5] - 6);
    Glib_Chars("20", -PxTab[17] - 27, PyTab[17] - 6);
    Glib_Chars("20", PxTab[17] + 7, PyTab[17] - 6);
    Glib_Chars("20", -PxTab[17] - 27, -PyTab[17] - 6);
    Glib_Chars("20", PxTab[17] + 7, -PyTab[17] - 6);

    PitchLimit = 15.0;
    Glib_Colour(Glib_YELLOW);
    glLineWidth(3.0);
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
    glColor3f(1.0, 1.0, 1.0);
    glTranslatef(0.0, -Pitch_Pix, 0.0);
    glLineWidth(2.0);
    Glib_Draw(0, BankAngleRadius, -9, BankAngleRadius - 12);
    Glib_Draw(-9, BankAngleRadius - 12, 9, BankAngleRadius - 12);
    Glib_Draw(9, BankAngleRadius - 12, 0, BankAngleRadius);
    SideSlip = (int) (-Slip * 180.0);
    Glib_Draw(-9 + SideSlip, BankAngleRadius - 18, -9 + SideSlip, BankAngleRadius - 12);
    Glib_Draw(-9 + SideSlip, BankAngleRadius - 12, 9 + SideSlip, BankAngleRadius - 12);
    Glib_Draw(9 + SideSlip, BankAngleRadius - 12, 9 + SideSlip, BankAngleRadius - 18);
    Glib_Draw(9 + SideSlip, BankAngleRadius - 18, -9 + SideSlip, BankAngleRadius - 18);
    glDisable(GL_SCISSOR_TEST);
    glPopMatrix();
    glPushMatrix();
    Glib_Colour(Glib_WHITE);
    Glib_Rectangle(-115, -4, 64, 8);
    Glib_Rectangle(-59, -26, 8, 30);
    Glib_Rectangle(51, -4, 68, 8);
    Glib_Rectangle(51, -26, 8, 30);
    Glib_Colour(Glib_BLACK);
    Glib_Rectangle(-113, -2, 60, 4);
    Glib_Rectangle(-57, -24, 4, 26);
    Glib_Rectangle(53, -2, 64, 4);
    Glib_Rectangle(53, -24, 4, 26);
    Glib_Rectangle(-3, -3, 6, 6);
    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_GFONT16, 8);
    Glib_Char(15, -4, -4);
    glPopMatrix();
    glPushMatrix();
    Glib_Colour(Glib_WHITE);
    for (dx = 0; dx <= 7; dx += 1)
    {
        Glib_Draw(-dx, BankAngleRadius + dx, dx + 1, BankAngleRadius + dx);
    }
    glRotatef(10.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 11);
    glRotatef(10.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 11);
    glRotatef(10.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 22);
    glRotatef(15.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 11);
    glRotatef(-55.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 11);
    glRotatef(-10.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 11);
    glRotatef(-10.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 22);
    glRotatef(-15.0, 0.0, 0.0, 1.0);
    Glib_Draw(0, BankAngleRadius, 0, BankAngleRadius + 11);
    glRotatef(45.0, 0.0, 0.0, 1.0);
    Glib_Colour(Glib_BLACK);
    glLineWidth(1.0);
    for (y = 0; y <= 24; y += 1)
    {
        dx = Fillets[y];
        Glib_Draw(AiHalfWidth - dx, AiHalfHeight - 24 + y,
                  AiHalfWidth + 1, AiHalfHeight - 24 + y);
        Glib_Draw(-AiHalfWidth - 1, AiHalfHeight - 24 + y,
                  -AiHalfWidth + dx, AiHalfHeight - 24 + y);
        Glib_Draw(AiHalfWidth - dx, -AiHalfHeight + 24 - y,
                  AiHalfWidth + 1, -AiHalfHeight + 24 - y);
        Glib_Draw(-AiHalfWidth - 1, -AiHalfHeight + 24 - y,
                  -AiHalfWidth + dx, -AiHalfHeight + 24 - y);
    }
    glPopMatrix();
}

void Ai_FlightDirector(int x, int y, bool Enabled)
{
    if (Enabled)
    {
        if (Systems_Failures[20])
	    {
	        Glib_SetFont(Glib_GFONT12, 10);
	        Glib_HBox("FD", 100, 100);
		    return;
	    }
		
        Glib_Colour(Glib_MAGENTA);
        glLineWidth(4.0);
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

void Ai_GlideSlope(float GlideSlopeError, bool ILSMode, bool Selected)
{
    int h;
    int p;

    if (Systems_Failures[4])
	{
	    Glib_SetFont(Glib_GFONT12, 10);
	    Glib_VBox("G/S", AiHalfWidth - 14, 0);
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
        glLineWidth(2.0);
        Glib_AntiAliasing(true);
        Glib_Draw(AiHalfWidth - 22, 0, AiHalfWidth, 0);
        Glib_DrawPolygon(4, AiHalfWidth - 11, h, Diamond);
        glLineWidth(2.0);
        Glib_SetFont(Glib_GFONT16, 8);
        for (p = -86; p <= 86; p += 43)
        {
            if (p != 0)
            {
                Glib_Char(15, AiHalfWidth - 15, p - 4);
            }
        }
    }
    OldGlideslope_h = h;
}

void Ai_Localiser(float LocaliserError, bool ILSMode, bool Selected)
{
    int h, p;

    if (Systems_Failures[3])
	{
	    Glib_SetFont(Glib_GFONT12, 10);
	    Glib_HBox("LOC", 0, -AiHalfHeight + 11);
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
        glLineWidth(2.0);
        Glib_AntiAliasing(true);
        Glib_Draw(0, -AiHalfHeight + 22, 0, -AiHalfHeight);
        Glib_DrawPolygon(4, h - 1, -AiHalfHeight + 11, Diamond);
        glLineWidth(2.0);
        Glib_SetFont(Glib_GFONT16, 8);
        for (p = -86; p <= 86; p += 43)
        {
            if (p != 0)
            {
                Glib_Char(15, p - 4, -AiHalfHeight + 7);
            }
        }
    }
    OldLocaliser_h = h;
}

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

void MarkerLamp(char Str[], bool Mode, unsigned int Col)
{
    if (Mode)
    {
        Glib_SetFont(Glib_GFONT16, 12);
        glLineWidth(2.0);
        Glib_Colour(Glib_WHITE);
        Circle16(124, 136);
        Glib_Colour(Col);
        Glib_Chars(Str, 114, 128);
    }
}

void Circle16(int x, int y)
{
    Glib_DrawLines(19, x, y, Circle16Tab);
}

void BEGIN_Ai()
{
    OldGlideslope_h = 0;
    OldLocaliser_h  = 0;
}
