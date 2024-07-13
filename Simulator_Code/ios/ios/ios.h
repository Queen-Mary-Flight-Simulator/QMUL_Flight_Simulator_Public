#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>

typedef void (*IOS_PtrProc)();

extern bool IOS_Mode;

extern void IOS_Display(void);
extern void IOS_Init(IOS_PtrProc Update);
extern void BEGIN_IOS();

#endif
