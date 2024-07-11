#ifndef NFD_H
#define NFD_H

#include <stdbool.h>
#include <SIM/navdefn.h>

#define NFD_OFFSET_X      1000
#define NFD_OFFSET_Y      50

#define NFD_NfdX          (386 + NFD_OFFSET_X)
#define NFD_NfdY          (700 + NFD_OFFSET_Y)

#define NFD_CompassX      NFD_NfdX
#define NFD_CompassY      NFD_NfdY
#define NFD_xCompassX     NFD_NfdX
#define NFD_xCompassY     (NFD_NfdY - 170)
#define NFD_WindX         (NFD_NfdX - 273)
#define NFD_WindY         (NFD_NfdY + 248)
#define NFD_GsX           NFD_WindX
#define NFD_GsY           (NFD_NfdY + 271)
#define NFD_TasX          (NFD_NfdX - 210)
#define NFD_TasY          (NFD_NfdY + 271)
#define NFD_LeftNavInfoX  NFD_WindX
#define NFD_LeftNavInfoY  (NFD_NfdY - 246)
#define NFD_RightNavInfoX (NFD_NfdX + 195)
#define NFD_RightNavInfoY NFD_LeftNavInfoY
#define NFD_TopNavInfoX   NFD_RightNavInfoX
#define NFD_TopNavInfoY   (NFD_WindY - 20)
#define NFD_RadioX        (NFD_OFFSET_X + 83)
#define NFD_RadioY        (NFD_OFFSET_Y + 56)
#define NFD_FCUX          (NFD_OFFSET_X + 0)
#define NFD_FCUY          (NFD_OFFSET_Y + 56)

extern NavDefn_FCUMode NFD_NavDisplayMode;

extern unsigned int NFD_NavDisplayRange;

extern void NFD_Init();

extern void NFD_Update();

extern void NFD_Display();

extern void BEGIN_NFD();

#endif
