#ifndef IOLIB_H
#define IOLIB_H

#include <stdbool.h>
#include <SIM/iodefn.h>

#define Seconds_per_day 86400
#define Ticks_per_day 86400000

extern unsigned int  IOLib_AnalogueData[32];
extern unsigned char IOLib_DigitalDataA;
extern unsigned char IOLib_DigitalDataB;
extern unsigned char IOLib_DigitalDataC;
extern unsigned char IOLib_DigitalDataD;
extern float         IOLib_Temperature;
extern bool          IOLib_Sidestick;

extern void IOLib_UpdateCLS(float v, float e, float a, float r);

extern void IOLib_RepositionCLS(float v, float e, float a, float r);

extern void IOLib_ResetCLS();

extern void IOLib_StartIO();

extern void IOLib_StopIO();

void IOLib_UpdateIO(unsigned char DigitalOutputA, unsigned char DigitalOutputB);

extern void IOLib_LEDS(unsigned char n);

extern void IOLib_Start();

extern void BEGIN_IOLib();

extern void END_IOLib();

#endif
