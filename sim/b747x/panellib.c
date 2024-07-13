/* +------------------------------+---------------------------------+
   | Module      : panellib.c     | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-10      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Panel textures                                   |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <windows.h>
#include <GL/gl.h>

#include "textureid.h"
#include "panellib.h"

/*
    prototypes
 */

unsigned * read_texture(char *, int *, int *, int *);

void Init_Tex_Coords(void);

/* Texture Objects */
GLuint texture_objs[MAX_TEXTURES];

/* texture coordinate data */
#define MAX_TEX_COORDS    25
struct texcoord
{
    float tex_umin; /* left tex coord */
    float tex_umax; /* right tex coord */
    float tex_vmin; /* bottom tex coord */
    float tex_vmax; /* top tex coord */
} tc_list[MAX_TEX_COORDS];


/* Prototypes */
void DrawSprite(int, int, int, int, float, int, int);
void DrawCursor(int, int, int, int, int, int); /* move to header file */
void PanelLib_KnobGrey(int x, int y, int sx, int sy, float angle);


/*---------------------------------------------------------------------------*/
/*
    GenerateTexture :

    Generate an OpenGL Texture Object from an "sgi" formmatted file.
    Returns a texture object identifier, but can be ignored.
    GLuint is an unsigned int
 */
GLuint GenerateTexture(GLuint textureArray[], char *FileName, int textureID, GLint wrap)
{
    unsigned *sprimage;
    int      iwidth, iheight, idepth;

    /* read SGI format rgb image */
    sprimage = read_texture(FileName, &iwidth, &iheight, &idepth);
    if (!sprimage)
    {
        printf("error reading image\n");
        return 0;
    }

    glGenTextures(1, &textureArray[textureID]);
    glBindTexture(GL_TEXTURE_2D, textureArray[textureID]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, iwidth, iheight, 0, GL_RGBA, GL_UNSIGNED_BYTE, (GLubyte *) sprimage);

    free(sprimage);
    return textureArray[textureID];
}


/*---------------------------------------------------------------------------*/
/*
    Pushbutton :

    Draw a pushbutton at x,y with size sx and sy.
    Presently, pushbuttons are automatically rendered on the panel
    background, so there is nothing to display.
 */

void PanelLib_PushButton(int x, int y, int sx, int sy)
{
    /* nothing to display */

    /* some txt in lcd display? */
}

/*---------------------------------------------------------------------------*/
void PanelLib_PushButtonWLight(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 0.0, 255, 8);
    }
}

/*---------------------------------------------------------------------------*/
void PanelLib_PushButtonArrow(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 0.0, 255, 10);
    }
}

/*---------------------------------------------------------------------------*/
/*
    PushSwitch :

    Draw a PushSwitch at x,y, with size sx and sy. state indicates whether
    the pushswitch is active or not.
 */
void PanelLib_PushSwitch(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 0.0, 255, 1);
    }
}

/*---------------------------------------------------------------------------*/
/*
    SwitchGuard :

    Draw a switch guard at x,y, with size sx and sy. if statew is TRUE the
    guard in down.
 */
void PanelLib_SwitchGuard(int x, int y, int sx, int sy, bool state)
{
    if (state)
    {
        DrawSprite(x, y, sx, sy, 0.0, 100, 12);
    }
    else
    {
        DrawSprite(x, y + 60, sx + 9, sy - 64, 0.0, 100, 13);
    }
}

/*---------------------------------------------------------------------------*/
/*
    SelectorKnob :

    Draw a SelectorKnob at x,y with size sx and sy. pos is the position
    of the knob and is converted to a rotation angle. Values of pos are
    0,1,2,3,4,5...

 */
void PanelLib_SelectorKnob(int x, int y, int sx, int sy, int pos)
{
    float rot = pos * 45.0f + 0.02;
    DrawSprite(x, y, sx, sy, -rot, 255, 5);
}

/*---------------------------------------------------------------------------*/
/*
    ThreeWaySwitch :

    Draw a ThreeWaySwitch at x,y with size sx and sy. pos is the position
    of the switch and can be any of POS_LEFT,POS_MID, POS_RIGHT.
 */
void PanelLib_ThreeWaySwitch(int x, int y, int sx, int sy, SWITCH_POS pos)
{
    DrawSprite(x, y, sx, sy, 0.0, 255, pos);
}

/*---------------------------------------------------------------------------*/
/*
    TwoWaySwitch :

    Draw a TwoWaySwitch at x,y with size sx and sy. pos is the position
    of the switch and can be any of POS_UP,POS_DOWN.
 */
void PanelLib_TwoWaySwitch(int x, int y, int sx, int sy, VSWITCH_POS pos)
{
    DrawSprite(x, y, sx, sy, 0.0, 255, pos);
}

/*---------------------------------------------------------------------------*/
/*
    DisplayKnob :

    Draw a Knob at x,y with size sx and sy. rot is the rotation angle of the
    knob in degrees.
 */
void PanelLib_Knob(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSprite(x, y, sx, sy, rot, 255, 9);
}

void PanelLib_KnobInner(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSprite(x, y, sx, sy, rot, 255, 18);
}

void PanelLib_KnobBlue(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSprite(x, y, sx, sy, rot, 255, 19);
}

void PanelLib_KnobGrey(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSprite(x, y, sx, sy, rot, 255, 22);
}

void PanelLib_BaroMark(int x, int y, int sx, int sy, float angle)
{
    float rot = angle + 0.02;
    DrawSprite(x, y, sx, sy, rot, 255, 20);
}

/*---------------------------------------------------------------------------*/
/*
    DrawMode :

    Draw an active mode button at x,y with size sx and sy.
    mode is a number from 0-3 representing the active mode
    from left to right (on-screen)

 */
void PanelLib_DrawMode(int mode)
{
    glPushMatrix();
    //glTranslatef(1000.0, 0.0, 0.0);
	
    switch (mode)
    {
        case 0:
            /* off */
            DrawSprite(17, 5, 102, 32, 0.0, 255, 17);
            break;
        case 1:
            /* rmp1 */
            DrawSprite(137, 5, 102, 32, 0.0, 255, 16);
            break;
        case 2:
            /* rmp2 */
            DrawSprite(257, 5, 102, 32, 0.0, 255, 15);
            break;
        case 3:
            /* fcu */
            DrawSprite(377, 5, 102, 32, 0.0, 255, 14);
            break;
        default:
            break;
    }
    glPopMatrix();
}

/*---------------------------------------------------------------------------*/
/*
    DisplayFCU :

    Procedure for drawing objects on the FCU panel
 */
void PanelLib_DisplayFCU(unsigned char FD, unsigned char LS, unsigned char CSTR,
                         unsigned char WPT, unsigned char VORD, unsigned char NDB,
                         unsigned char ARPT, unsigned char LOC, unsigned char AP1,
                         unsigned char AP2, unsigned char ATHR, unsigned char EXPED,
                         unsigned char APPR, int NAV1, int NAV2, int NAVSELECT,
                         int RANGE, float Baro, float Speed, float Hdg, float Alt,
                         float VS, unsigned char hPa)
{
    glPushMatrix();
    glTranslatef(1000.0, 0.0, 0.0);
	
    /* FCU Panel */
    DrawSprite(0, 56, 768, 200, 0.0, 255, 0);

    PanelLib_PushSwitch(28, 53, 32, 32, FD);
    PanelLib_PushSwitch(66, 53, 32, 32, LS);

    PanelLib_PushSwitch(114, 195, 32, 32, CSTR);
    PanelLib_PushSwitch(151, 195, 32, 32, WPT);
    PanelLib_PushSwitch(188, 195, 32, 32, VORD);
    PanelLib_PushSwitch(225, 195, 32, 32, NDB);
    PanelLib_PushSwitch(262, 195, 32, 32, ARPT);

    PanelLib_PushSwitch(438, 64, 32, 32, LOC);

    PanelLib_PushSwitch(502, 103, 32, 32, AP1);
    PanelLib_PushSwitch(545, 103, 32, 32, AP2);
    PanelLib_PushSwitch(523, 64, 32, 32, ATHR);

    PanelLib_PushSwitch(608, 64, 32, 32, EXPED);
    PanelLib_PushSwitch(698, 64, 32, 32, APPR);

    PanelLib_ThreeWaySwitch(129, 62, 64, 64, NAV1);
    PanelLib_ThreeWaySwitch(219, 62, 64, 64, NAV2);

    PanelLib_SelectorKnob(162, 141, 48, 48, NAVSELECT); /* rotated sprites need centre anchor */
    PanelLib_SelectorKnob(253, 141, 48, 48, RANGE);

    if (hPa)
    {
        PanelLib_BaroMark(62, 126, 64, 64, 45.0);  /* hPa or InHg? */
    }
    else
    {
        PanelLib_BaroMark(62, 126, 64, 64, -45.0);  /* hPa or InHg? */
    }

    PanelLib_Knob(62, 123, 38, 38, Baro);

    PanelLib_Knob(383, 139, 38, 38, Speed);
    PanelLib_KnobBlue(453, 139, 38, 38, Hdg);
    PanelLib_Knob(623, 139, 38, 38, Alt);
    PanelLib_Knob(714, 139, 38, 38, VS);

    /*DrawCursor(714, 139, 24, 24, 255, 21);*/

    glPopMatrix();
}

/*---------------------------------------------------------------------------*/
/*
    DisplayRadio :

    Procedure for drawing objects on the Radio panel
 */
void PanelLib_DisplayRadio(unsigned char VHF1, unsigned char VHF2, unsigned char VHF3,
                           unsigned char HF1, unsigned char SEL, unsigned char HF2,
                           unsigned char AM, unsigned char NAV, unsigned char VOR,
                           unsigned char ILS, unsigned char MLS, unsigned char ADS,
                           unsigned char BFO, unsigned char NavCover,
                           int ToggleSwitch,
                           float InnerKnobPosition, float OuterKnobPosition)
{
    /* Radio Panel */
    glPushMatrix();
    glTranslatef(1000.0, 0.0, 0.0);

    DrawSprite(83, 56, 601, 344, 0.0, 255, 7);
    PanelLib_PushButtonWLight(117, 232, 32, 32, VHF1); /* VHF1 */
    PanelLib_PushButtonWLight(191, 232, 32, 32, VHF2); /* VHF2 */
    PanelLib_PushButtonWLight(271, 232, 32, 32, VHF3); /* VHF3 */

    PanelLib_PushButtonWLight(117, 177, 32, 32, HF1);  /* HF1 */
    PanelLib_PushButtonWLight(233, 177, 16, 16, SEL);  /* SEL */
    PanelLib_PushButtonWLight(271, 177, 32, 32, HF2);  /* HF1 */
    PanelLib_PushButtonWLight(350, 177, 32, 32, AM);   /* AM */

    PanelLib_PushButtonWLight(117, 85, 32, 32, NAV);  /* NAV */
    PanelLib_PushButtonWLight(191, 85, 32, 32, VOR);  /* VOR */
    PanelLib_PushButtonWLight(271, 85, 32, 32, ILS);  /* ILS */
    PanelLib_PushButtonWLight(350, 85, 32, 32, MLS);  /* MLS */
    PanelLib_PushButtonWLight(429, 85, 32, 32, ADS);  /* ADS */
    PanelLib_PushButtonWLight(508, 85, 32, 32, BFO);  /* BFO */

    PanelLib_SwitchGuard(141, 24, 50, 128, NavCover); /* switch guard */

    /* on / off toggle */
    PanelLib_TwoWaySwitch(581, 72, 96, 96, ToggleSwitch);

    /* rotational knob */
    PanelLib_KnobGrey(534, 225, 64, 64, OuterKnobPosition);
    PanelLib_KnobGrey(534, 225, 48, 48, InnerKnobPosition);

    glPopMatrix();
}

/*---------------------------------------------------------------------------*/
/*
    DisplayMode :

    Procedure for drawing objects on the Mode panel
 */
void PanelLib_DisplayMode(int Mode)
{
    glPushMatrix();
    glTranslatef(1000.0, 0.0, 0.0);

    /* Mode Panel */
    DrawSprite(0, 0, 768, 40, 0.0, 255, 6);

    /* draw active button */
    PanelLib_DrawMode(Mode);

    glPopMatrix();
}


/*---------------------------------------------------------------------------*/
/*
    Sprite anchor is centre of object.
 */
void DrawSprite(int x0, int y0, int sx0, int sy0, float R, int Alpha, int txcoordid)
{
    bool  texrot = false;
    float x      = (float) (x0);
    float y      = (float) (y0); /* EFIS landscape coords */
    float sx     = (float) sx0;
    float sy     = (float) sy0;

    if (fabs(R) > 0.01f)
        texrot = true;

    glPushMatrix();
    glColor4ub(255, 255, 255, Alpha);

    if (texrot)
    {
        glTranslatef(x, y, 0.0f);
        glRotatef(R, 0.0f, 0.0f, 1.0f);
        glBegin(GL_QUADS);
        glTexCoord2f(tc_list[txcoordid].tex_umin,
                     tc_list[txcoordid].tex_vmin);     /* bottom left */
        glVertex3f(-sx / 2, -sy / 2, 0.0f);
        glTexCoord2f(tc_list[txcoordid].tex_umax,
                     tc_list[txcoordid].tex_vmin);     /* bottom right */
        glVertex3f(sx / 2, -sy / 2, 0.0f);
        glTexCoord2f(tc_list[txcoordid].tex_umax,
                     tc_list[txcoordid].tex_vmax);     /* top right */
        glVertex3f(sx / 2, sy / 2, 0.0f);
        glTexCoord2f(tc_list[txcoordid].tex_umin,
                     tc_list[txcoordid].tex_vmax);     /* top left */
        glVertex3f(-sx / 2, sy / 2, 0.0f);
        glEnd();
    }
    else
    {
        glBegin(GL_QUADS);
        glTexCoord2f(tc_list[txcoordid].tex_umin,
                     tc_list[txcoordid].tex_vmin);     /* bottom left */
        glVertex3f(x, y, 0.0f);
        glTexCoord2f(tc_list[txcoordid].tex_umax,
                     tc_list[txcoordid].tex_vmin);     /* bottom right */
        glVertex3f(x + sx, y, 0.0f);
        glTexCoord2f(tc_list[txcoordid].tex_umax,
                     tc_list[txcoordid].tex_vmax);     /* top right */
        glVertex3f(x + sx, y + sy, 0.0f);
        glTexCoord2f(tc_list[txcoordid].tex_umin,
                     tc_list[txcoordid].tex_vmax);     /* top left */
        glVertex3f(x, y + sy, 0.0f);
        glEnd();
    }

    glPopMatrix();
}

/*
    DrawCursor - Draw the virtual cursor using a textured quad
 */
/*---------------------------------------------------------------------------*/
void DrawCursor(int x0, int y0, int sx0, int sy0, int Alpha, int txcoordid)
{
    float x  = (float) (1024 - y0); /* from EFIS portrait coords to OpenGL coords */
    float y  = (float) (x0);
    float sx = (float) sy0;
    float sy = (float) sx0;

    glPushMatrix();
    glColor4ub(255, 255, 255, Alpha);
    glBegin(GL_QUADS);
    glTexCoord2f(tc_list[txcoordid].tex_umin,
                 tc_list[txcoordid].tex_vmin); /* top left */
    glVertex3f(x, y, 0.0f);

    glTexCoord2f(tc_list[txcoordid].tex_umax,
                 tc_list[txcoordid].tex_vmin); /* bottom left */
    glVertex3f(x, y + sy, 0.0f);
    glTexCoord2f(tc_list[txcoordid].tex_umax,
                 tc_list[txcoordid].tex_vmax); /* bottom right */
    glVertex3f(x + sx, y + sy, 0.0f);
    glTexCoord2f(tc_list[txcoordid].tex_umin,
                 tc_list[txcoordid].tex_vmax); /* top right */
    glVertex3f(x + sx, y, 0.0f);

    glEnd();
    glPopMatrix();
}

/*---------------------------------------------------------------------------*/
void PanelLib_InitialiseTextures(void)
{
    char buffer_prefix[32];
    char buffer[64];

    /* Generate texture objects */
    sprintf(buffer_prefix, "Textures/");
    sprintf(buffer, "%smosaicV4.sgi", buffer_prefix);
    GenerateTexture(texture_objs, buffer, TEX_MOSAIC, GL_CLAMP);
    Init_Tex_Coords();
}

/*---------------------------------------------------------------------------*/
void Init_Tex_Coords(void)
{
    /* background panel */
    tc_list[0].tex_vmax = 1.0 - 0.0;
    tc_list[0].tex_vmin = 1.0 - 0.194;
    tc_list[0].tex_umin = 0.0;
    tc_list[0].tex_umax = 0.75;

    /* green glow */
    tc_list[1].tex_vmax = 1.0 - 0.0;
    tc_list[1].tex_vmin = 1.0 - 0.0625;
    tc_list[1].tex_umin = 0.751;
    tc_list[1].tex_umax = 0.8124;

    /* three way left*/
    tc_list[2].tex_vmax = 1.0 - 0.219;
    tc_list[2].tex_vmin = 1.0 - 0.281;
    tc_list[2].tex_umin = 0.0234;
    tc_list[2].tex_umax = 0.102;

    /* three way mid*/
    tc_list[3].tex_vmax = 1.0 - 0.219;
    tc_list[3].tex_vmin = 1.0 - 0.281;
    tc_list[3].tex_umin = 0.148;
    tc_list[3].tex_umax = 0.227;

    /* three way right*/
    tc_list[4].tex_vmax = 1.0 - 0.219;
    tc_list[4].tex_vmin = 1.0 - 0.281;
    tc_list[4].tex_umin = 0.273;
    tc_list[4].tex_umax = 0.352;

    /* selector knob */
    tc_list[5].tex_vmax = 1.0 - 0.196;
    tc_list[5].tex_vmin = 1.0 - 0.304;
    tc_list[5].tex_umin = 0.384;
    tc_list[5].tex_umax = 0.491;

    /* mode buttons */
    tc_list[6].tex_vmax = 1.0 - 0.688;
    tc_list[6].tex_vmin = 1.0 - 0.73;
    tc_list[6].tex_umin = 0.0;
    tc_list[6].tex_umax = 0.75;

    /* radio panel */
    tc_list[7].tex_vmax = 1.0 - 0.3125;
    tc_list[7].tex_vmin = 1.0 - 0.648;
    tc_list[7].tex_umin = 0.0;
    tc_list[7].tex_umax = 0.586;

    /* push button side light (green circle) */
    tc_list[8].tex_vmin = 1.0 - 0.0781;
    tc_list[8].tex_vmax = 1.0 - 0.1094;
    tc_list[8].tex_umin = 0.766;
    tc_list[8].tex_umax = 0.797;

    /* rotational knob */
    tc_list[9].tex_vmax = 1.0 - 0.1254;
    tc_list[9].tex_vmin = 1.0 - 0.1871;
    tc_list[9].tex_umin = 0.8129;
    tc_list[9].tex_umax = 0.8746;

    /* radio toggle ON/UP */
    tc_list[10].tex_vmax = 1.0 - 0.203;
    tc_list[10].tex_vmin = 1.0 - 0.297;
    tc_list[10].tex_umin = 0.578;
    tc_list[10].tex_umax = 0.672;

    /* radio toggle OFF/DOWN */
    tc_list[11].tex_vmax = 1.0 - 0.203;
    tc_list[11].tex_vmin = 1.0 - 0.297;
    tc_list[11].tex_umin = 0.703;
    tc_list[11].tex_umax = 0.797;

    /* radio NAV cover down */
    tc_list[12].tex_vmax = 1.0 - 0.8125;
    tc_list[12].tex_vmin = 1.0 - 0.9375;
    tc_list[12].tex_umin = 0.0;
    tc_list[12].tex_umax = 0.055;

    /* radio NAV cover up */
    tc_list[13].tex_vmax = 1.0 - 0.8125;
    tc_list[13].tex_vmin = 1.0 - 0.875;
    tc_list[13].tex_umin = 0.0625;
    tc_list[13].tex_umax = 0.125;

    /* mode FCU */
    tc_list[14].tex_vmax = 1.0 - 0.781;
    tc_list[14].tex_vmin = 1.0 - 0.8125;
    tc_list[14].tex_umin = 0.0;
    tc_list[14].tex_umax = 0.0996;

    /* mode RMP2 */
    tc_list[15].tex_vmax = 1.0 - 0.781;
    tc_list[15].tex_vmin = 1.0 - 0.8125;
    tc_list[15].tex_umin = 0.125;
    tc_list[15].tex_umax = 0.225;

    /* mode RMP1 */
    tc_list[16].tex_vmax = 1.0 - 0.781;
    tc_list[16].tex_vmin = 1.0 - 0.8125;
    tc_list[16].tex_umin = 0.25;
    tc_list[16].tex_umax = 0.3496;

    /* mode OFF */
    tc_list[17].tex_vmax = 1.0 - 0.781;
    tc_list[17].tex_vmin = 1.0 - 0.8125;
    tc_list[17].tex_umin = 0.375;
    tc_list[17].tex_umax = 0.475;

    /* inner rotational knob (on radio panel) */
    tc_list[18].tex_vmax = 1.0 - 0.125;
    tc_list[18].tex_vmin = 1.0 - 0.1875;
    tc_list[18].tex_umin = 0.875;
    tc_list[18].tex_umax = 0.9375;

    /* blue tri rotational knob */
    tc_list[19].tex_vmax = 1.0 - 0.125;
    tc_list[19].tex_vmin = 1.0 - 0.1875;
    tc_list[19].tex_umin = 0.9375;
    tc_list[19].tex_umax = 1.0;

    /* pressure knob marker */
    tc_list[20].tex_vmax = 1.0 - 0.0625;
    tc_list[20].tex_vmin = 1.0 - 0.125;
    tc_list[20].tex_umin = 0.875;
    tc_list[20].tex_umax = 0.9375;

    /* virtual mouse cursor */
    tc_list[21].tex_vmax = 1.0 - 0.0;
    tc_list[21].tex_vmin = 1.0 - 0.0625;
    tc_list[21].tex_umin = 0.875;
    tc_list[21].tex_umax = 0.9375;

    /* light grey rotational knob */
    tc_list[22].tex_vmax = 1.0 - 0.19141;
    tc_list[22].tex_vmin = 1.0 - 0.25391;
    tc_list[22].tex_umin = 0.9365;
    tc_list[22].tex_umax = 0.99902;
}

/*---------------------------------------------------------------------------*/
void BEGIN_Panellib()
{
}
