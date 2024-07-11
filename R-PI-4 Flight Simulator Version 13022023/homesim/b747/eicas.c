/* +------------------------------+---------------------------------+
   | Module      : eicas.c        | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-09      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EICAS Display                     |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "aerolink.h"
#include "systems.h"
#include "eicas.h"

#define Up         0
#define Down       1
#define Transit    2
typedef unsigned char   GearState;

#define ONERAD     (180.0L / M_PI)
#define DEG5       (5.0 / ONERAD)
#define DEG180     (180.0 / ONERAD)
#define DEG198     (198.0 / ONERAD)
#define DEG200     (200.0 / ONERAD)
#define DEG216     (216.0 / ONERAD)

#define MAX_TEX_COORDS 10

struct texcoord
{
    float tex_umin;  /* left tex coord   */
    float tex_umax;  /* right tex coord  */
    float tex_vmin;  /* bottom tex coord */
    float tex_vmax;  /* top tex coord    */
} PFDTextures[MAX_TEX_COORDS];

static void DrawRectangle(int x1, int y1, int xs, int ys);
void InitialisePFDTextureCoords();

/* ------------------------------------------------ */
void DrawRectangle(int x1, int y1, int xs, int ys)
{
    int x2 = x1 + xs;
    int y2 = y1 + ys;
    
    Glib_Draw(x1, y1, x1, y2);
    Glib_Draw(x1, y2, x2, y2);
    Glib_Draw(x2, y2, x2, y1);
    Glib_Draw(x2, y1, x1, y1);
}

/* ------------------------------------------------ */
void EICAS_EprGauge(int EprX, int EprY, unsigned int EngineNumber)
{
    int   x, y;
    float e;
    char  str[20];

    Glib_LoadIdentity();
    Glib_Translate((float) EprX, (float) EprY);
    
    e = Maths_Rads(AeroLink_EngPkt.Engines[EngineNumber].Epr * 110.0);
    if (e < 0.0)
    {
        e = 0.0;
    }
    else if (e > DEG198)
    {
        e = DEG198;
    }

    if (e > DEG180)
	{
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, Maths_Degrees(-e), 1.0);
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, 180.0, 1.0);
	}
	else
	{
        Glib_ClipWindow(EprX - 70, EprY -70, 140, 69);
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, Maths_Degrees(-e), 1.0);
    	Glib_RemoveClipWindow();
    }
	/* draw arc */
    Glib_DrawTexture(-78, -73, 163.0, 105.0, PFDTextures[0].tex_umin, PFDTextures[0].tex_vmin, PFDTextures[0].tex_umax, PFDTextures[0].tex_vmax, 1.0);

    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    x = intround(70.0 * cos(e));
    y = intround(-70.0 * sin(e));
    Glib_Draw(0, 0, x, y);

    e = AeroLink_EngPkt.Engines[EngineNumber].Epr;
    if (e < 0.0)
    {
        e = 0.0;
    }
    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_EFONT16, 12);
    sprintf(str, "%4.2f", e);
    Glib_Chars(str, 7, 10);
}
/* ------------------------------------------------ */
void EICAS_RpmGauge(int RpmX, int RpmY, unsigned int EngineNumber)
{
    int   x, y;
    float r;
    char  str[20];

    Glib_LoadIdentity();
    Glib_Translate((float) RpmX, (float) RpmY);
    
    r = Maths_Rads(AeroLink_EngPkt.Engines[EngineNumber].Rpm * 1.8);
    if (r < 0.0)
    {
        r = 0.0;
    }
    else if (r > DEG216)
    {
        r = DEG216;
    }

    if (r > DEG180)
	{
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, Maths_Degrees(-r), 1.0);
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, 180.0, 1.0);
	}
	else
	{
        Glib_ClipWindow(RpmX - 70, RpmY -70, 140, 69);
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, Maths_Degrees(-r), 1.0);
    	Glib_RemoveClipWindow();
    }
    
	/* draw arc */
    Glib_DrawTexture(-75, -75, 153.0, 126.0, PFDTextures[2].tex_umin, PFDTextures[2].tex_vmin, PFDTextures[2].tex_umax, PFDTextures[2].tex_vmax, 1.0);

    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    x = intround(70.0 * cos(r));
    y = intround(-70.0 * sin(r));
    Glib_Draw(0, 0, x, y);

    r = AeroLink_EngPkt.Engines[EngineNumber].Rpm;
    if (r < 0.0)
    {
        r = 0.0;
    }
    Glib_SetFont(Glib_EFONT16, 12);
    sprintf(str, "%5.1f", r);
    Glib_Chars(str, 5, 10);
}

/* ------------------------------------------------ */
void EICAS_EgtGauge(int EgtX, int EgtY, unsigned int EngineNumber)
{
    int   x, y;
    float t;
    char  str[20];

    Glib_LoadIdentity();
    Glib_Translate((float) EgtX, (float) EgtY);

    t = Maths_Rads(AeroLink_EngPkt.Engines[EngineNumber].Egt * 0.2);
    if (t < 0.0)
    {
        t = 0.0;
    }
    else if (t > DEG200)
    {
        t = DEG200;
    }

    if (t > DEG180)
	{
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, Maths_Degrees(-t), 1.0);
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, 180.0, 1.0);
	}
	else
	{
        Glib_ClipWindow(EgtX - 70, EgtY -70, 140, 69);
        Glib_DrawTextureRotated(0, 0, 140, 140, 0.3-0.07, 0.1-0.07, 0.3+0.07, 0.1+0.07, Maths_Degrees(-t), 1.0);
    	Glib_RemoveClipWindow();
    }
	/* draw arc */
    Glib_DrawTexture(-78, -76, 163.0, 109.0, PFDTextures[1].tex_umin, PFDTextures[1].tex_vmin, PFDTextures[1].tex_umax, PFDTextures[1].tex_vmax, 1.0);

    Glib_Colour(Glib_WHITE);
    Glib_LineWidth(2.0);
    x = intround(70.0 * cos(t));
    y = intround(-70.0 * sin(t));
    Glib_Draw(0, 0, x, y);

    t = AeroLink_EngPkt.Engines[EngineNumber].Egt;
    if (t < 0.0)
    {
        t = 0.0;
    }
    Glib_SetFont(Glib_EFONT16, 12);
    sprintf(str, "%5.1f", t);
    Glib_Chars(str, 6, 10);
}

/* ------------------------------------------------ */
void EICAS_ParkBrake(int ParkBrakeX, int ParkBrakeY, IODefn_SwitchPosition brake)
{
    Glib_LoadIdentity();
    Glib_Translate((float) ParkBrakeX, (float) ParkBrakeY);
    
    if (brake == IODefn_On)
    {
        Glib_LineWidth(2.0);
        Glib_SetFont(Glib_EFONT16, 10);
        Glib_Colour(Glib_GREEN);
        DrawRectangle(0, 0, 70, 45);
        Glib_Chars("PARK", 8, 25);
        Glib_Chars("BRAKE", 3, 5);
    }
}

/* ------------------------------------------------ */
void EICAS_DisplayGear(int GearX, int GearY, float GearPosition)
{
    Glib_LoadIdentity();
    Glib_Translate((float) GearX, (float) GearY);
    
    GearState Gear;

    Glib_LineWidth(2.0);
    if (GearPosition <= 0.0)
    {
        Gear = Up;
    }
    else if (GearPosition >= 1.0)
    {
        Gear = Down;
    }
    else
    {
        Gear = Transit;
    }
    if (Gear != Up)
    {
        Glib_SetFont(Glib_EFONT16, 10);
        if (Gear == Down)
        {
            Glib_Colour(Glib_GREEN);
        }
        else
        {
            Glib_Colour(Glib_YELLOW);
        }
        DrawRectangle(-12, 0, 67, 24);
        Glib_Chars("DOWN", -10, 4);
        
        Glib_Colour(Glib_BLUE);
        Glib_SetFont(Glib_EFONT12, 10);
        Glib_Chars("GEAR", 0, -17);
    }
}

/* ------------------------------------------------ */
void EICAS_FlapsIndicator(int FlapsX, int FlapsY, float FlapPosition, unsigned int FlapSetting)
{
    int f;
    int i;
    int dx;

    Glib_LoadIdentity();
    Glib_Translate((float) FlapsX, (float) FlapsY);

    Glib_LineWidth(2.0);
    if (FlapSetting > 30)
    {
        FlapSetting = 30;
    }
    Glib_SetFont(Glib_EFONT12, 8);
    if (FlapPosition > 0.001 || FlapSetting != 0)
    {
        Glib_Colour(Glib_WHITE);
        DrawRectangle(0, 0, 14, 112);
        Glib_Colour(Glib_BLUE);
        Glib_Char('F', -14, 90);
        Glib_Char('L', -14, 74);
        Glib_Char('A', -14, 58);
        Glib_Char('P', -14, 42);
        Glib_Char('S', -14, 26);
    }
    else
    {
        return;
    }
    Glib_Colour(Glib_WHITE);
    f = intround(FlapPosition * 112.0);
    if (f < 0)
    {
        f = 0;
    }

    for (i = 0; i <= f; i += 1)
    {
        Glib_Draw(1, 112 - i, 13, 112 - i);
    }
    f = intround(112.0 - (float) (FlapSetting) * 112.0 / 30.0);
    Glib_Colour(Glib_MAGENTA);
    Glib_Draw(-5, f, 19, f);
    dx = 25;
    if (FlapSetting > 5)
    {
        Glib_Char(FlapSetting / 10 + '0', dx, f - 6);
        dx = dx + 8;
    }
    Glib_Char(FlapSetting % 10 + '0', dx, f - 6);
}

/* ------------------------------------------------ */
void InitialisePFDTextureCoords()
{
    /* EPR */
    PFDTextures[0].tex_vmin = (300.0 - 73.0) / 1000.0; /* (300, 300) arc radius 70 */
    PFDTextures[0].tex_vmax = (300.0 + 32.0) / 1000.0;
    PFDTextures[0].tex_umin = (300.0 - 78.0) / 1000.0;
    PFDTextures[0].tex_umax = (300.0 + 85.0) / 1000.0;

    /* N1 */
    PFDTextures[2].tex_vmin = (300.0 - 75.0) / 1000.0; /* (100, 300) arc radius 0.07 */
    PFDTextures[2].tex_vmax = (300.0 + 51.0) / 1000.0;
    PFDTextures[2].tex_umin = (100.0 - 75.0) / 1000.0;
    PFDTextures[2].tex_umax = (100.0 + 78.0) / 1000.0;

    /* EGT */
    PFDTextures[1].tex_vmin = (100.0 - 76.0) / 1000.0; /* (100, 100) arc radius 70 */
    PFDTextures[1].tex_vmax = (100.0 + 33.0) / 1000.0;
    PFDTextures[1].tex_umin = (100.0 - 78.0) / 1000.0;
    PFDTextures[1].tex_umax = (100.0 + 85.0) / 1000.0;
}

/* ------------------------------------------------ */
void BEGIN_EICAS()
{
    InitialisePFDTextureCoords();
}
