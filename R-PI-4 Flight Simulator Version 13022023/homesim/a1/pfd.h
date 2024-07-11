#ifndef PFD_H
#define PFD_H

#include <SIM/glib.h>

#define PFD_X         0
#define PFD_Y         0

#define PFD_AiX       (PFD_X + 596)
#define PFD_AiY       (PFD_Y + 840)
#define PFD_CompassX  PFD_AiX
#define PFD_CompassY  (PFD_AiY - 330)
#define CompassKnobX  (PFD_CompassX + 130)
#define CompassKnobY  (PFD_CompassY - 130)
#define PFD_AsiX      (PFD_AiX - 330)
#define PFD_AsiY      PFD_AiY
#define PFD_TurnSlipX (PFD_AiX - 330)
#define PFD_TurnSlipY PFD_CompassY
#define PFD_GMeterX   (PFD_CompassX + 32)
#define PFD_GMeterY   (PFD_TurnSlipY - 300)
#define PFD_AoAX      (PFD_GMeterX - 270)
#define PFD_AoAY      PFD_GMeterY
#define PFD_ClockX    (PFD_AiX - 476)
#define PFD_ClockY    PFD_GMeterY
#define PFD_AmmeterX  (PFD_CompassX - 62)
#define PFD_AmmeterY  (PFD_Y + 10)
#define PFD_SuctionX  (PFD_AmmeterX + 128 + 5) 
#define PFD_SuctionY  PFD_AmmeterY

#define PFD_AltX      PFD_AiX
#define PFD_AltY      PFD_AiY
#define PFD_RadAltX   (PFD_AiX + 110)
#define PFD_RadAltY   (PFD_PfdY + 200)
#define PFD_BaroX     PFD_AltX
#define PFD_BaroY     (PFD_AltY - 245)
#define PFD_AltSelX   (PFD_AltX + 10)
#define PFD_AltSelY   (PFD_AltY + 236)
#define PFD_VsiX      PFD_AiX
#define PFD_VsiY      PFD_AiY

extern bool PFD_Held;

extern void PFD_Update();

extern void PFD_Display();

extern void PFD_Init();

extern void BEGIN_PFD();

#endif
