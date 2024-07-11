#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>

extern bool DEMO_MODE;

extern bool IOS_Mode;

extern void IOS_Update(void);

extern void IOS_Display(void);

extern void IOS_Init();

extern void IOS_CloseWindow();

extern void BEGIN_IOS();

#endif
