#ifndef EngLink_H
#define EngLink_H

#include <stdbool.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/iodefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>

typedef unsigned char         EngLink_SimulatorState;

extern IODefn_IODataPkt       EngLink_IOPkt1;
extern IODefn_IODataPkt       EngLink_IOPkt2;
extern AeroDefn_AeroDataPkt   EngLink_AeroPkt;
extern EngDefn_EngDataPkt     EngLink_EngPkt;
extern NavDefn_NavDataPkt     EngLink_NavPkt;
extern IosDefn_IosDataPkt     EngLink_IosPkt;
extern ProtoDefn_ProtoDataPkt EngLink_ProtoPkt;

extern bool                   EngLink_OctaveMode;
extern bool                   EngLink_Freezing;
extern bool                   EngLink_RemoteHold;
extern bool                   EngLink_ReplayMode;
extern bool                   EngLink_Reloading;
extern char                   EngLink_ReloadFilename[50];
extern bool                   EngLink_Stopping;

extern bool EngLink_RespondToIos();
extern void EngLink_Replay();
extern void EngLink_FormPacket();
extern void EngLink_AddFlightData(unsigned int n, float x);
extern void BEGIN_EngLink();

#endif
