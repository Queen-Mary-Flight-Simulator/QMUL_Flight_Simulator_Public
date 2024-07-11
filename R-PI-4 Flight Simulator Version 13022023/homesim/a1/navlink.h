#ifndef NavLink_H
#define NavLink_H

#include <stdbool.h>
#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>

typedef unsigned char         NavLink_SimulatorState;

extern IODefn_IODataPkt       NavLink_IOPkt1;
extern IODefn_IODataPkt       NavLink_IOPkt2;
extern AeroDefn_AeroDataPkt   NavLink_AeroPkt;
extern EngDefn_EngDataPkt     NavLink_EngPkt;
extern NavDefn_NavDataPkt     NavLink_NavPkt;
extern IosDefn_IosDataPkt     NavLink_IosPkt;
extern ProtoDefn_ProtoDataPkt NavLink_ProtoPkt;

extern bool                   NavLink_OctaveMode;
extern unsigned int           NavLink_CmdPtr;
extern bool                   NavLink_Freezing;
extern bool                   NavLink_RemoteHold;
extern bool                   NavLink_Reloading;
extern bool                   NavLink_Restored;
extern char                   NavLink_ReloadFilename[50];
extern bool                   NavLink_Stopping;
extern void                   NavLink_RespondToIos();
extern void                   NavLink_FormPacket();
extern void                   BEGIN_NavLink();

#endif
