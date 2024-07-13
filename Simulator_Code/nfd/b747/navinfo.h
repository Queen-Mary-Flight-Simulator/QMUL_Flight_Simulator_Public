#ifndef NavInfo_H
#define NavInfo_H

#include <SIM/navdefn.h>

extern void NavInfo_UpdateLeftNavInfo(NavDefn_FCUNav Mode);

extern void NavInfo_UpdateRightNavInfo(NavDefn_FCUNav Mode);

extern void NavInfo_UpdateTopNavInfo(NavDefn_FCUMode Mode);

extern void NavInfo_UpdateGS(float v);

extern void NavInfo_UpdateTAS(float v);

extern void NavInfo_UpdateWindVector(float WindSpeed, float WindDir);

extern void BEGIN_NavInfo();

#endif
