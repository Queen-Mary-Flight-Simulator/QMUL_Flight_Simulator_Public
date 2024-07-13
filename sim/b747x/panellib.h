/*
    panellib.h
    panel library for NFD
*/

#ifndef panellib_H
#define panellib_H

#ifdef WIN32
#define WINDOWS_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <stdio.h>
#include <stdbool.h>
#include <windows.h>
#include <GL/gl.h>

#include <SIM/textureid.h>

typedef enum
{
    POS_LEFT = 2,
    POS_MID,
    POS_RIGHT,
}SWITCH_POS;

typedef enum
{
    POS_UP = 10,
    POS_DOWN,
}VSWITCH_POS;

extern GLuint texture_objs[MAX_TEXTURES];

/* function prototypes */

void PanelLib_PushButton(int, int, int, int);

void PanelLib_PushButtonWLight(int, int, int, int, bool);

void PanelLib_PushButtonArrow(int, int, int, int, bool);

void PanelLib_PushSwitch(int, int, int, int, bool);

void PanelLib_SwitchGuard(int, int, int, int, bool);

void PanelLib_SelectorKnob(int, int, int, int, int);

void PanelLib_ThreeWaySwitch(int, int, int, int, SWITCH_POS);

void PanelLib_TwoWaySwitch(int, int, int, int, VSWITCH_POS);

void PanelLib_Knob(int, int, int, int, float);

void PanelLib_DisplayFCU(unsigned char, unsigned char, unsigned char,
                         unsigned char, unsigned char, unsigned char,
                         unsigned char, unsigned char, unsigned char,
                         unsigned char, unsigned char, unsigned char,
                         unsigned char, int, int, int,
                         int, float, float, float, float, float, unsigned char);

void PanelLib_DisplayRadio(unsigned char VHF1, unsigned char VHF2, unsigned char VHF3,
                           unsigned char HF1, unsigned char SEL, unsigned char HF2,
                           unsigned char AM, unsigned char NAV, unsigned char VOR,
                           unsigned char ILS, unsigned char MLS, unsigned char ADS,
                           unsigned char BFO, unsigned char NavCover, int ToggleSwitch, 
                           float InnerKnobPosition, float OuterKnobPosition);

void PanelLib_DisplayMode(int);

void PanelLib_DrawMode(int);

void PanelLib_InitialiseTextures(void);

void BEGIN_Panellib();

#endif
