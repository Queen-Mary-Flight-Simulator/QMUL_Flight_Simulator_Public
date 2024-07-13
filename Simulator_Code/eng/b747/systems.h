#ifndef Systems_H
#define Systems_H

#include <stdbool.h>

#include <SIM/iodefn.h>

extern bool         Systems_Failures[51];
extern bool         Systems_RemoteHold;
extern bool         Systems_Freezing;
extern unsigned int Systems_SysTicks;
extern bool         Systems_EngineFireSound;

extern bool         Systems_AttentionGetter;
extern bool         Systems_EngineFire[4];

extern void  Systems_UpdateSystems();
extern void  BEGIN_Systems();
#endif
