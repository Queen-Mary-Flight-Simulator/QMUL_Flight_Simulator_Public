/*
    panellib.h
    panel library for NFD
*/

#ifndef panellib_H
#define panellib_H

#include <stdbool.h>

#define MAX_TEX_COORDS 25

typedef struct 
{
    float tex_umin;  /* left tex coord   */
    float tex_umax;  /* right tex coord  */
    float tex_vmin;  /* bottom tex coord */
    float tex_vmax;  /* top tex coord    */
} texcoord; 

extern texcoord PanelTextures[MAX_TEX_COORDS];

/* function prototypes */

extern void PanelLib_DisplayFCU(unsigned char FD, unsigned char LS, unsigned char CSTR,
                         unsigned char WPT, unsigned char VORD, unsigned char NDB,
                         unsigned char ARPT, unsigned char LOC, unsigned char AP1,
                         unsigned char AP2, unsigned char ATHR, unsigned char EXPED,
                         unsigned char APPR, int NAV1, int NAV2, int NAVSELECT, int RANGE, 
						 float Baro, float Spd, float Hdg, float Alt, float VSpd,
						 int *Baro_State, int *Spd_State, int *Hdg_State, int *Alt_State, int *VSpd_State,
                         bool hPa, bool alt000);

extern void PanelLib_DisplayRadio(unsigned char VHF1, unsigned char VHF2, unsigned char VHF3,
                           unsigned char HF1, unsigned char SEL, unsigned char HF2,
                           unsigned char AM, unsigned char NAV, unsigned char VOR,
                           unsigned char ILS, unsigned char MLS, unsigned char ADS,
                           unsigned char BFO, unsigned char NavCover, int ToggleSwitch, 
                           float InnerKnobPosition, float OuterKnobPosition);

extern void PanelLib_DisplayMode(int);

extern void BEGIN_PanelLib();

#endif
