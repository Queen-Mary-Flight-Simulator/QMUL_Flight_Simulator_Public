#ifndef Clocks_H
#define Clocks_H

#include <stdbool.h>

#define Clocks_FrameRate 50

extern unsigned int Clocks_SysTicks;
extern unsigned int Clocks_ClockTicks;
extern unsigned int Clocks_ClockSecs;
extern unsigned int Clocks_ClockMins;
extern unsigned int Clocks_ClockHours;
extern unsigned int Clocks_TimerSecs;
extern unsigned int Clocks_TimerMins;
extern unsigned int Clocks_TimerHours;

#define Clocks_Forwards 0
#define Clocks_Backwards 1
#define Clocks_Stopped 2
extern unsigned char Clocks_TimerMode;

extern void Clocks_UpdateClocks(bool Held, bool ButtonPressed);
extern void BEGIN_Clocks();

#endif
