/*
   ALT        COMPASS
   VSI        BLANK
   RPM        MP
FUELQ FUELP OILP OILT FUELQ
*/

#ifndef NFD_H
#define NFD_H

#include <stdbool.h>
#include <SIM/navdefn.h>
#include <SIM/glib.h>

#define NFD_NfdX          (Glib_SCREENWIDTH / 2)
#define NFD_NfdY          0

#define NFD_AltX          (NFD_NfdX + 180)
#define NFD_AltY          (NFD_NfdY + 840)
#define BaroKnobX         (NFD_AltX + 130)
#define BaroKnobY         (NFD_AltY - 130)
#define NFD_VsiX          NFD_AltX
#define NFD_VsiY          (NFD_AltY - 330)
#define NFD_BlankX        (NFD_VsiX + 330)
#define NFD_BlankY        NFD_VsiY

#define NFD_RpmX          NFD_VsiX
#define NFD_RpmY          (NFD_VsiY - 300)
#define NFD_MpX           (NFD_RpmX + 270)
#define NFD_MpY           (NFD_RpmY)
#define NFD_EgtX          (NFD_MpX + 270)
#define NFD_EgtY          NFD_RpmY

#define NFD_VorX          (NFD_VsiX + 330)
#define NFD_VorY          (NFD_VsiY + 90)
#define ObsKnobX          (NFD_VorX + 110)
#define ObsKnobY          (NFD_VorY - 110)
#define NFD_MagCompassX   (NFD_AltX + 330)
#define NFD_MagCompassY   (NFD_AltY + 30)

#define NFD_RadioX        NFD_NfdX + 370
#define NFD_RadioY        NFD_NfdY + 90

#define NFD_FuelQLX       (NFD_NfdX + 74)
#define NFD_FuelQLY       10
#define NFD_FuelPrsX      (NFD_FuelQLX + 133)
#define NFD_FuelPrsY      NFD_FuelQLY
#define NFD_OilPrsX       (NFD_FuelPrsX + 133)
#define NFD_OilPrsY       NFD_FuelQLY
#define NFD_OilTempX      (NFD_OilPrsX + 133)
#define NFD_OilTempY      NFD_FuelQLY
#define NFD_FuelQRX       (NFD_OilTempX + 133)
#define NFD_FuelQRY       NFD_FuelQLY

#define NFD_FCUX          0  /* not used */
#define NFD_FCUY          0
 
typedef void (*NFD_PtrProc)();

extern NavDefn_FCUMode NFD_NavDisplayMode;

extern unsigned int NFD_NavDisplayRange;

extern void NFD_Display();

extern void NFD_Update();

extern void NFD_Init();

extern void NFD_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right);

extern void NFD_SetOrigin(int x, int y);

extern void BEGIN_NFD();

#endif
