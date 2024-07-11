#ifndef IOLink_H
#define IOLink_H

#include <stdbool.h>

#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>

extern IODefn_IODataPkt       IOLink_IOPkt1;
extern IODefn_IODataPkt       IOLink_IOPkt2;
extern AeroDefn_AeroDataPkt   IOLink_AeroPkt;
extern EngDefn_EngDataPkt     IOLink_EngPkt;
extern NavDefn_NavDataPkt     IOLink_NavPkt;
extern IosDefn_IosDataPkt     IOLink_IosPkt;
extern ProtoDefn_ProtoDataPkt IOLink_ProtoPkt;

extern bool                   IOLink_OctaveMode;

extern void IOLink_FormPacket();
extern void BEGIN_IOLink();

#endif
