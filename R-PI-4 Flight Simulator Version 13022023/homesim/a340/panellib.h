/*
    panellib.h
    panel library for NFD
*/

#ifndef panellib_H
#define panellib_H

/* function prototypes */

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

void BEGIN_PanelLib();

#endif
