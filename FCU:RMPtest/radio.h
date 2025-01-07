#ifndef Radio_H
#define Radio_H

#include <SIM/iosdefn.h>
#include <SIM/navdefn.h>

extern NavDefn_RadioPanel Radio_Radios[2];

extern void Radio_RestoreRMP(IosDefn_RestoreVectorRecord v);

extern void Radio_SaveRMP(NavDefn_NavDataPkt *pkt);

extern void Radio_UpdateRMP(int n, int leftb, int middleb, int rightb, int x, int y);

extern void Radio_StopRMP();

extern void BEGIN_Radio();

#endif
