/* +------------------------------+---------------------------------+
   | Module      : clocks.c       | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-01              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : clock management library                         |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <stdbool.h>

#include <SIM/clocks.h>

unsigned int  Clocks_SysTicks;
unsigned int  Clocks_ClockTicks;
unsigned int  Clocks_ClockSecs;
unsigned int  Clocks_ClockMins;
unsigned int  Clocks_ClockHours;
unsigned int  Clocks_TimerSecs;
unsigned int  Clocks_TimerMins;
unsigned int  Clocks_TimerHours;
unsigned char Clocks_TimerMode;

bool          ClockStarted;

void          StartClock();

/* ------------------------------------- */
void Clocks_UpdateClocks(bool Held, bool ButtonPressed)
{
    Clocks_SysTicks += 1;
    if (Clocks_SysTicks >= Clocks_FrameRate)
    {
        Clocks_SysTicks = 0;
    }
    if (Held)
    {
        return;
    }
    if (!ClockStarted)
    {
        StartClock();
        ClockStarted = true;
    }

    if (ButtonPressed)
    {
        if (Clocks_TimerMode == Clocks_Forwards)
        {
            Clocks_TimerMode  = Clocks_Backwards;
            Clocks_TimerMins  = 0;
            Clocks_TimerHours = 0;
        }
        else if (Clocks_TimerMode == Clocks_Stopped)
        {
            Clocks_TimerMode = Clocks_Forwards;
        }
    }

    if (Clocks_TimerMode == Clocks_Backwards)
    {
        if (Clocks_TimerSecs <= 1)
        {
            Clocks_TimerSecs = 0;
            Clocks_TimerMode = Clocks_Stopped;
        }
        else if (Clocks_SysTicks && 1)
        {
            Clocks_TimerSecs -= 1;
        }
    }

    Clocks_ClockTicks += 1;
    if (Clocks_ClockTicks >= Clocks_FrameRate)
    {
        Clocks_ClockTicks = 0;

        if (Clocks_TimerMode == Clocks_Forwards)
        {
            Clocks_TimerSecs += 1;
            if (Clocks_TimerSecs >= 60)
            {
                Clocks_TimerSecs = 0;
                Clocks_TimerMins += 1;
                if (Clocks_TimerMins >= 60)
                {
                    Clocks_TimerMins  = 0;
                    Clocks_TimerHours += 1;
                    if (Clocks_TimerHours >= 24)
                    {
                        Clocks_TimerHours = 0;
                    }
                }
            }
        }

        Clocks_ClockSecs += 1;
        if (Clocks_ClockSecs >= 60)
        {
            Clocks_ClockSecs = 0;
            Clocks_ClockMins += 1;
            if (Clocks_ClockMins >= 60)
            {
                Clocks_ClockMins  = 0;
                Clocks_ClockHours += 1;
                if (Clocks_ClockHours >= 24)
                {
                    Clocks_ClockHours = 0;
                }
            }
        }
    }
}

/* ------------------------------------- */
void StartClock()
{
    struct timeval  ticks;
    unsigned int t;

    gettimeofday(&ticks, NULL);
    t = (unsigned int) ticks.tv_sec;

    Clocks_ClockSecs  = t % 60;
    t                 = t / 60;
    Clocks_ClockMins  = t % 60;
    t                 = t / 60;
    Clocks_ClockHours = t % 24;
}

/* ------------------------------------- */
void BEGIN_Clocks()
{
    Clocks_SysTicks   = 0;
    Clocks_ClockTicks = 0;

    ClockStarted      = false;
    Clocks_TimerSecs  = 0;
    Clocks_TimerMins  = 0;
    Clocks_TimerHours = 0;
    Clocks_TimerMode  = Clocks_Forwards;
}
