#ifndef PFD_H
#define PFD_H

#include <SIM/glib.h>

#define PFD_PfdX     (Glib_SCREENWIDTH / 4)
#define PFD_PfdY     770

#define PFD_AiX      PFD_PfdX
#define PFD_AiY      PFD_PfdY
#define PFD_FMAX     PFD_PfdX - 160
#define PFD_FMAY     PFD_PfdY + 237
#define PFD_AsiX     (PFD_PfdX - 265)
#define PFD_AsiY     PFD_PfdY
#define PFD_AltX     (PFD_PfdX + 170)
#define PFD_AltY     PFD_PfdY
#define PFD_RadAltX  (PFD_AiX + 110)
#define PFD_RadAltY  (PFD_PfdY + 200)
#define PFD_BaroX    PFD_AltX
#define PFD_BaroY    (PFD_AltY - 245)
#define PFD_AltSelX  (PFD_AltX + 10)
#define PFD_AltSelY  (PFD_AltY + 236)
#define PFD_VsiX     (PFD_PfdX + 288)
#define PFD_VsiY     PFD_PfdY
#define PFD_CompassX PFD_AiX
#define PFD_CompassY (PFD_PfdY - 411)

#define PFD_EprX     (PFD_PfdX - 236)
#define PFD_EprY     (PFD_PfdY - 370)
#define PFD_RpmX     PFD_EprX
#define PFD_RpmY     (PFD_EprY - 140)
#define PFD_EgtX     PFD_EprX
#define PFD_EgtY     (PFD_RpmY - 140)

#define PFD_GearX    (PFD_PfdX + 374)
#define PFD_GearY    (PFD_PfdY - 468)
#define PFD_FlapsX   (PFD_PfdX + 392)
#define PFD_FlapsY   (PFD_PfdY - 634)
#define PFD_ParkBrakeX (PFD_PfdX + 365)
#define PFD_ParkBrakeY (PFD_PfdY - 720)

extern bool PFD_Held;

extern void PFD_Update();

extern void PFD_Display();

extern void PFD_Init();

extern void BEGIN_PFD();

#endif
