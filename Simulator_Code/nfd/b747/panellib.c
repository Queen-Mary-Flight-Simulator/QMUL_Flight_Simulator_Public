#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <SIM/glib.h>

#include "panellib.h"
#include "nfd.h"

#define       ONERAD (180.0f / M_PI)
#define DEG30 (30.0 / ONERAD)

typedef enum
{
    POS_LEFT = 2,
    POS_MID,
    POS_RIGHT,
} SWITCH_POS;

typedef enum
{
    POS_UP = 10,
    POS_DOWN,
} VSWITCH_POS;

texcoord PanelTextures[MAX_TEX_COORDS];

/* Prototypes */
void DrawSprite(int x0, int y0, int sx0, int sy0, float Alpha, int texnum);
void DrawSpriteRotated(int x0, int y0, int sx0, int sy0, float R, float Alpha, int texnum);
void InitialiseTextureCoords();

void PushButton(int x, int y, int sx, int sy);
void PushButtonWLight(int x, int y, int sx, int sy, bool state);
void PushButtonArrow(int x, int y, int sx, int sy, bool state);
void PushSwitch(int x, int y, int sx, int sy, bool state);
void SwitchGuard(int x, int y, int sx, int sy, bool state);
void SelectorKnob(int x, int y, int sx, int sy, int pos);
void ThreeWaySwitch(int x, int y, int sx, int sy, SWITCH_POS pos);
void TwoWaySwitch(int x, int y, int sx, int sy, VSWITCH_POS pos);
void Knob(int x, int y, int sx, int sy, float angle, int *state);
void KnobInner(int x, int y, int sx, int sy, float angle);
void KnobBlue(int x, int y, int sx, int sy, float angle, int *state);
void KnobGrey(int x, int y, int sx, int sy, float angle);
void BaroMark(int x, int y, int sx, int sy, float angle);
void BaroKnob(int x, int y, int sx, int sy, float angle, int *state);
void AltMark(int x, int y, int sx, int sy, float angle);

/*  Pushbutton :
    Draw a pushbutton at x,y with size sx and sy.
    Presently, pushbuttons are automatically rendered on the panel
    background, so there is nothing to display.  */
/*---------------------------------------------------------------------------*/
void PushButton(int x, int y, int sx, int sy)
{
    /* nothing to display */

    /* some txt in lcd display? */
}

/*---------------------------------------------------------------------------*/
void PushButtonWLight(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 1.0, 8);
    }
}

/*---------------------------------------------------------------------------*/
void PushButtonArrow(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 1.0, 10);
    }
}

/*
    PushSwitch :
    Draw a PushSwitch at x,y, with size sx and sy. state indicates whether
    the pushswitch is active or not.
 */
/*---------------------------------------------------------------------------*/
void PushSwitch(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 1.0, 1);
    }
}

/*
    SwitchGuard :
    Draw a switch guard at x,y, with size sx and sy. if statew is TRUE the
    guard in down.
 */
/*---------------------------------------------------------------------------*/
void SwitchGuard(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 0.4, 12);
    }
    else
    {
        DrawSprite(x, y + 60, sx + 9, sy - 64, 0.4, 13);
    }
}

/*
    SelectorKnob :
    Draw a SelectorKnob at x,y with size sx and sy. pos is the position
    of the knob and is converted to a rotation angle. Values of pos are
    0,1,2,3,4,5...

 */
/*---------------------------------------------------------------------------*/
void SelectorKnob(int x, int y, int sx, int sy, int pos)
{
    float rot = pos * 45.0f + 0.02;
    DrawSpriteRotated(x, y, sx, sy, -rot, 1.0, 5);
}

/*  ThreeWaySwitch :
    Draw a ThreeWaySwitch at x,y with size sx and sy. pos is the position
    of the switch and can be any of POS_LEFT,POS_MID, POS_RIGHT.  */
/*---------------------------------------------------------------------------*/
void ThreeWaySwitch(int x, int y, int sx, int sy, SWITCH_POS pos)
{
    DrawSprite(x, y, sx, sy, 255, pos);
}

/*  TwoWaySwitch :
    Draw a TwoWaySwitch at x,y with size sx and sy. pos is the position
    of the switch and can be any of POS_UP,POS_DOWN. */
/*---------------------------------------------------------------------------*/
void TwoWaySwitch(int x, int y, int sx, int sy, VSWITCH_POS pos)
{
    DrawSprite(x, y, sx, sy, 1.0, pos);
}

/*  DisplayKnob :
    Draw a Knob at x,y with size sx and sy. rot is the rotation angle of the
    knob in degrees.  */
/*---------------------------------------------------------------------------*/
void Knob(int x, int y, int sx, int sy, float angle, int *state)
{
    float rot = angle + 0.02;
	if (*state > 0)
	{
	    sx -= sx / 4;
		sy -= sy / 4;
		*state -= 1;
	}
	else if (*state < 0)
	{
	    sx += sx / 2;
		sy += sy / 2;
		*state += 1;
	}
    DrawSpriteRotated(x, y, sx, sy, rot, 1.0, 9);
}

/*---------------------------------------------------------------------------*/
void BaroKnob(int x, int y, int sx, int sy, float angle, int *state)
{
    float rot = angle + 0.02;
    DrawSpriteRotated(x, y, sx, sy, rot, 1.0, 22);
}

/*---------------------------------------------------------------------------*/
void KnobInner(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSpriteRotated(x, y, sx, sy, rot, 1.0, 18);
}

/*---------------------------------------------------------------------------*/
void KnobBlue(int x, int y, int sx, int sy, float angle, int *state)
{
    float rot = angle + 0.02;
	if (*state > 0)
	{
	    sx -= sx / 4;
		sy -= sy / 4;
		*state -= 1;
	}
	else if (*state < 0)
	{
	    sx += sx / 2;
		sy += sy / 2;
		*state += 1;
	}
    DrawSpriteRotated(x, y, sx, sy, rot, 1.0, 19);
}

/*---------------------------------------------------------------------------*/
void KnobGrey(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSpriteRotated(x, y, sx, sy, rot, 1.0, 21);
}

/*---------------------------------------------------------------------------*/
void BaroMark(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSpriteRotated(x, y, sx, sy, rot, 1.0, 20);
}

/*---------------------------------------------------------------------------*/
void AltMark(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSpriteRotated(x, y, sx, sy, rot, 1.0, 20);
}

/*  DisplayFCU :
    Procedure for drawing objects on the FCU panel  */
/*---------------------------------------------------------------------------*/
void PanelLib_DisplayFCU(unsigned char FD, unsigned char LS, unsigned char CSTR,
                         unsigned char WPT, unsigned char VORD, unsigned char NDB,
                         unsigned char ARPT, unsigned char LOC, unsigned char AP1,
                         unsigned char AP2, unsigned char ATHR, unsigned char EXPED,
                         unsigned char APPR, int NAV1, int NAV2, int NAVSELECT, int RANGE, 
						 float Baro, float Spd, float Hdg, float Alt, float VSpd,
						 int *Baro_State, int *Spd_State, int *Hdg_State, int *Alt_State, int *VSpd_State,
                         bool hPa, bool alt000)
{
    float dx1, dy1;
	float dx2, dy2;
	
    Glib_LoadIdentity();
    Glib_Translate((float) NFD_FCUX, (float) NFD_FCUY);
        
    /* FCU Panel */
    DrawSprite(0, 0, 768, 200, 1.0, 0);

    PushSwitch(28, -3, 32, 32, FD);
    PushSwitch(66, -3, 32, 32, LS);

    PushSwitch(114, 139, 32, 32, CSTR);
    PushSwitch(151, 139, 32, 32, WPT);
    PushSwitch(188, 139, 32, 32, VORD);
    PushSwitch(225, 139, 32, 32, NDB);
    PushSwitch(262, 139, 32, 32, ARPT);

    PushSwitch(438, 8, 32, 32, LOC);

    PushSwitch(502, 47, 32, 32, AP1);
    PushSwitch(545, 47, 32, 32, AP2);
    PushSwitch(523, 8, 32, 32, ATHR);

    PushSwitch(608, 8, 32, 32, EXPED);
    PushSwitch(698, 8, 32, 32, APPR);

    ThreeWaySwitch(129, 6, 64, 64, NAV1);
    ThreeWaySwitch(219, 6, 64, 64, NAV2);

    SelectorKnob(162, 85, 48, 48, NAVSELECT); /* rotated sprites need centre anchor */
    SelectorKnob(253, 85, 48, 48, RANGE);

    if (hPa)
    {
        BaroMark(62, 70, 64, 64, 45.0);  /* hPa or InHg? */
    }
    else
    {
        BaroMark(62, 70, 64, 64, -45.0);  /* hPa or InHg? */
    }

    BaroKnob(61, 67, 38, 38, Baro, Baro_State);

    Glib_Colour(Glib_WHITE);
	Glib_LineWidth(2.0);
	dx1 = (int) (22.0 * sin(DEG30));
	dy1 = (int) (22.0 * cos(DEG30));
	dx2 = (int) (30.0 * sin(DEG30));
	dy2 = (int) (30.0 * cos(DEG30));
	Glib_Draw(623 - dx1, 83 + dy1, 623 - dx2, 83 + dy2); 
	Glib_Draw(623 + dx1, 83 + dy1, 623 + dx2, 83 + dy2); 
	
    if (alt000)
    {
        AltMark(623, 83, 64, 64, -30.0);  /* 00s */
    }
    else
    {
        AltMark(623, 83, 64, 64, 30.0);  /* 000s */
    }

    Knob(383, 83, 38, 38, Spd, Spd_State);
    KnobBlue(453, 83, 38, 38, Hdg, Hdg_State);
    Knob(623, 83, 38, 38, Alt, Alt_State);
    Knob(714, 83, 38, 38, VSpd, VSpd_State);
}

/*  DisplayRadio :
    Procedure for drawing objects on the Radio panel  */
/*---------------------------------------------------------------------------*/
void PanelLib_DisplayRadio(unsigned char VHF1, unsigned char VHF2, unsigned char VHF3,
                           unsigned char HF1, unsigned char SEL, unsigned char HF2,
                           unsigned char AM, unsigned char NAV, unsigned char VOR,
                           unsigned char ILS, unsigned char MLS, unsigned char ADS,
                           unsigned char BFO, unsigned char NavCover,
                           int ToggleSwitch,
                           float InnerKnobPosition, float OuterKnobPosition)
{
    /* Radio Panel */
    Glib_LoadIdentity();
    Glib_Translate((float) NFD_RadioX, (float) NFD_RadioY); /* N.B. these offsets should match the offsets in radio.c */ 

    DrawSprite(0, 0, 601, 344, 1.0, 7);
    PushButtonWLight( 34, 176, 32, 32, VHF1); /* VHF1 */
    PushButtonWLight(108, 176, 32, 32, VHF2); /* VHF2 */
    PushButtonWLight(188, 176, 32, 32, VHF3); /* VHF3 */

    PushButtonWLight( 34, 121, 32, 32, HF1);  /* HF1 */
    PushButtonWLight(150, 121, 16, 16, SEL);  /* SEL */
    PushButtonWLight(188, 121, 32, 32, HF2);  /* HF1 */
    PushButtonWLight(267, 121, 32, 32, AM);   /* AM */

    PushButtonWLight( 34, 29, 32, 32, NAV);  /* NAV */
    PushButtonWLight(108, 29, 32, 32, VOR);  /* VOR */
    PushButtonWLight(188, 29, 32, 32, ILS);  /* ILS */
    PushButtonWLight(267, 29, 32, 32, MLS);  /* MLS */
    PushButtonWLight(346, 29, 32, 32, ADS);  /* ADS */
    PushButtonWLight(425, 29, 32, 32, BFO);  /* BFO */

    SwitchGuard(58, -32, 50, 128, NavCover); /* switch guard */

    /* on / off toggle */
    TwoWaySwitch(498, 16, 96, 96, ToggleSwitch);

    /* rotational knob */
    KnobGrey(451, 169, 64, 64, OuterKnobPosition);
    KnobGrey(451, 169, 48, 48, InnerKnobPosition);
}

/*  DisplayMode :
    Procedure for drawing objects on the Mode panel  */
/*---------------------------------------------------------------------------*/
void PanelLib_DisplayMode(int mode)
{
    /* Mode Panel */
    Glib_LoadIdentity();  /* initialise at start of texture rendering */
    Glib_Translate((float) NFD_OFFSET_X, (float) NFD_OFFSET_Y);

    DrawSprite(0, 0, 768, 40, 1.0, 6);

    /*  Draw an active mode button at x,y with size sx and sy.
        mode is a number from 0-3 representing the active mode
        from left to right (on-screen)  */
    
    switch (mode)
    {
        case 0:
            /* off */
            DrawSprite(17, 5, 102, 32, 1.0, 17);
            break;
        case 1:
            /* RMP1 */
            DrawSprite(137, 5, 102, 32, 1.0, 16);
            break;
        case 2:
            /* RMP2 */
            DrawSprite(257, 5, 102, 32, 1.0, 15);
            break;
        case 3:
            /* FCU */
            DrawSprite(377, 5, 102, 32, 1.0, 14);
            break;
        default:
            break;
    }
}

/*---------------------------------------------------------------------------*/
void DrawSprite(int x0, int y0, int sx0, int sy0, float Alpha, int texnum)
{
    float tx1 = PanelTextures[texnum].tex_umin;
    float ty1 = PanelTextures[texnum].tex_vmin;
    float tx2 = PanelTextures[texnum].tex_umax;
    float ty2 = PanelTextures[texnum].tex_vmax;

    Glib_DrawTexture(x0, y0, sx0, sy0, tx1, ty1, tx2, ty2, Alpha);
}

/*---------------------------------------------------------------------------*/
void DrawSpriteRotated(int x0, int y0, int sx0, int sy0, float R, float Alpha, int texnum)
{
    float tx1 = PanelTextures[texnum].tex_umin;
    float ty1 = PanelTextures[texnum].tex_vmin;
    float tx2 = PanelTextures[texnum].tex_umax;
    float ty2 = PanelTextures[texnum].tex_vmax;

    Glib_DrawTextureRotated(x0, y0, sx0, sy0, tx1, ty1, tx2, ty2, R, Alpha);
}

/*---------------------------------------------------------------------------*/
void InitialiseTextureCoords(void)
{
    /* background panel */
    PanelTextures[0].tex_vmax = 1.0 - 0.0;
    PanelTextures[0].tex_vmin = 1.0 - 0.194;
    PanelTextures[0].tex_umin = 0.0;
    PanelTextures[0].tex_umax = 0.75;

    /* green glow */
    PanelTextures[1].tex_vmax = 1.0 - 0.0;
    PanelTextures[1].tex_vmin = 1.0 - 0.0625;
    PanelTextures[1].tex_umin = 0.751;
    PanelTextures[1].tex_umax = 0.8124;

    /* three way left*/
    PanelTextures[2].tex_vmax = 1.0 - 0.219;
    PanelTextures[2].tex_vmin = 1.0 - 0.281;
    PanelTextures[2].tex_umin = 0.0234;
    PanelTextures[2].tex_umax = 0.102;

    /* three way mid*/
    PanelTextures[3].tex_vmax = 1.0 - 0.219;
    PanelTextures[3].tex_vmin = 1.0 - 0.281;
    PanelTextures[3].tex_umin = 0.148;
    PanelTextures[3].tex_umax = 0.227;

    /* three way right*/
    PanelTextures[4].tex_vmax = 1.0 - 0.219;
    PanelTextures[4].tex_vmin = 1.0 - 0.281;
    PanelTextures[4].tex_umin = 0.273;
    PanelTextures[4].tex_umax = 0.352;

    /* selector knob */
    PanelTextures[5].tex_vmax = 1.0 - 0.196;
    PanelTextures[5].tex_vmin = 1.0 - 0.304;
    PanelTextures[5].tex_umin = 0.384;
    PanelTextures[5].tex_umax = 0.491;

    /* mode buttons */
    PanelTextures[6].tex_vmax = 1.0 - 0.688;
    PanelTextures[6].tex_vmin = 1.0 - 0.73;
    PanelTextures[6].tex_umin = 0.0;
    PanelTextures[6].tex_umax = 0.75;

    /* radio panel */
    PanelTextures[7].tex_vmax = 1.0 - 0.3125;
    PanelTextures[7].tex_vmin = 1.0 - 0.648;
    PanelTextures[7].tex_umin = 0.0;
    PanelTextures[7].tex_umax = 0.586;

    /* push button side light (green circle) */
    PanelTextures[8].tex_vmin = 1.0 - 0.0781;
    PanelTextures[8].tex_vmax = 1.0 - 0.1094;
    PanelTextures[8].tex_umin = 0.766;
    PanelTextures[8].tex_umax = 0.797;

    /* rotational knob */
    PanelTextures[9].tex_vmax = 1.0 - 0.1254;
    PanelTextures[9].tex_vmin = 1.0 - 0.1871;
    PanelTextures[9].tex_umin = 0.8129;
    PanelTextures[9].tex_umax = 0.8746;

    /* radio toggle ON/UP */
    PanelTextures[10].tex_vmax = 1.0 - 0.203;
    PanelTextures[10].tex_vmin = 1.0 - 0.297;
    PanelTextures[10].tex_umin = 0.578;
    PanelTextures[10].tex_umax = 0.672;

    /* radio toggle OFF/DOWN */
    PanelTextures[11].tex_vmax = 1.0 - 0.203;
    PanelTextures[11].tex_vmin = 1.0 - 0.297;
    PanelTextures[11].tex_umin = 0.703;
    PanelTextures[11].tex_umax = 0.797;

    /* radio NAV cover down */
    PanelTextures[12].tex_vmax = 1.0 - 0.8125;
    PanelTextures[12].tex_vmin = 1.0 - 0.9375;
    PanelTextures[12].tex_umin = 0.0;
    PanelTextures[12].tex_umax = 0.055;

    /* radio NAV cover up */
    PanelTextures[13].tex_vmax = 1.0 - 0.8125;
    PanelTextures[13].tex_vmin = 1.0 - 0.875;
    PanelTextures[13].tex_umin = 0.0625;
    PanelTextures[13].tex_umax = 0.125;

    /* mode FCU */
    PanelTextures[14].tex_vmax = 1.0 - 0.781;
    PanelTextures[14].tex_vmin = 1.0 - 0.8125;
    PanelTextures[14].tex_umin = 0.0;
    PanelTextures[14].tex_umax = 0.0996;

    /* mode RMP2 */
    PanelTextures[15].tex_vmax = 1.0 - 0.781;
    PanelTextures[15].tex_vmin = 1.0 - 0.8125;
    PanelTextures[15].tex_umin = 0.125;
    PanelTextures[15].tex_umax = 0.225;

    /* mode RMP1 */
    PanelTextures[16].tex_vmax = 1.0 - 0.781;
    PanelTextures[16].tex_vmin = 1.0 - 0.8125;
    PanelTextures[16].tex_umin = 0.25;
    PanelTextures[16].tex_umax = 0.3496;

    /* mode OFF */
    PanelTextures[17].tex_vmax = 1.0 - 0.781;
    PanelTextures[17].tex_vmin = 1.0 - 0.8125;
    PanelTextures[17].tex_umin = 0.375;
    PanelTextures[17].tex_umax = 0.475;

    /* inner rotational knob (on radio panel) */
    PanelTextures[18].tex_vmax = 1.0 - 0.125;
    PanelTextures[18].tex_vmin = 1.0 - 0.1875;
    PanelTextures[18].tex_umin = 0.875;
    PanelTextures[18].tex_umax = 0.9375;

    /* blue tri rotational knob */
    PanelTextures[19].tex_vmax = 1.0 - 0.125;
    PanelTextures[19].tex_vmin = 1.0 - 0.1875;
    PanelTextures[19].tex_umin = 0.9375;
    PanelTextures[19].tex_umax = 1.0;

    /* pressure knob marker */
    PanelTextures[20].tex_vmax = 1.0 - 0.0625;
    PanelTextures[20].tex_vmin = 1.0 - 0.125;
    PanelTextures[20].tex_umin = 0.875;
    PanelTextures[20].tex_umax = 0.9375;

    /* light grey rotational knob */
    PanelTextures[21].tex_vmax = 1.0 - 0.19141;
    PanelTextures[21].tex_vmin = 1.0 - 0.25391;
    PanelTextures[21].tex_umin = 0.9365;
    PanelTextures[21].tex_umax = 0.99902;
    
    /* Baro knob */
    PanelTextures[22].tex_vmax = 1.0 - 0.06445;
    PanelTextures[22].tex_vmin = 1.0 - 0.12305;
    PanelTextures[22].tex_umin = 0.93945;
    PanelTextures[22].tex_umax = 0.99805;
    
//	printf("Texture coords initialised\n");
}

/* ------------------------------------------------- */
void BEGIN_PanelLib()
{
    InitialiseTextureCoords();
}
