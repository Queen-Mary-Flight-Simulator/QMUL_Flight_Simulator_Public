#ifndef Systems_H
#define Systems_H

#include <stdbool.h>
#include <SIM/iodefn.h>

extern bool Systems_Failures[50];
extern bool Systems_AdfFixedCard;
extern bool Systems_AdfDip;
extern bool Systems_HSI_Installed;
extern bool Systems_VOR_Installed;
extern bool Systems_Radio_Installed;
extern float Systems_SelectedAltitude;
extern bool Systems_MarkerTest;
extern IODefn_SwitchPosition Systems_MasterSwitch;
extern IODefn_SwitchPosition Systems_KeySwitch;
extern bool Systems_RemoteHold;

extern void BEGIN_Systems();

#endif
