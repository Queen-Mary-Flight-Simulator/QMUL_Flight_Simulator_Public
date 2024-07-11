#ifndef AeroLink_H
#define AeroLink_H

#include <stdbool.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/iodefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>
#include <SIM/igdefn.h>

typedef unsigned char         AeroLink_SimulatorState;

extern IODefn_IODataPkt       AeroLink_IOPkt1;
extern IODefn_IODataPkt       AeroLink_IOPkt2;
extern AeroDefn_AeroDataPkt   AeroLink_AeroPkt;
extern EngDefn_EngDataPkt     AeroLink_EngPkt;
extern NavDefn_NavDataPkt     AeroLink_NavPkt;
extern IosDefn_IosDataPkt     AeroLink_IosPkt;
extern ProtoDefn_ProtoDataPkt AeroLink_ProtoPkt;
extern IGDefn_IGDataPkt       AeroLink_IGPkt;

extern bool                   AeroLink_OctaveMode;
extern bool                   AeroLink_Freezing;
extern bool                   AeroLink_RemoteHold;
extern bool                   AeroLink_ReplayMode;
extern unsigned int           AeroLink_CameraPosition;
extern bool                   AeroLink_Reloading;
extern char                   AeroLink_ReloadFilename[50];
extern bool                   AeroLink_Stopping;

extern bool AeroLink_RespondToIos();
extern void AeroLink_RestoreLast();
extern void AeroLink_Replay();
extern void AeroLink_FormPacket();
extern void AeroLink_AddFlightData(unsigned int n, float x);
extern void BEGIN_AeroLink();

#endif
