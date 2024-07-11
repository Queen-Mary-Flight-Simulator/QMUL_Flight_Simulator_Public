#ifndef EngLink_H
#define EngLink_H

#include <stdbool.h>

#include  <SIM/aerodefn.h>
#include  <SIM/engdefn.h>
#include  <SIM/navdefn.h>
#include  <SIM/iodefn.h>
#include  <SIM/iosdefn.h>
#include  <SIM/protodefn.h>

extern AeroDefn_AeroDataPkt    EngLink_AeroPkt;
extern EngDefn_EngDataPkt      EngLink_EngPkt;
extern NavDefn_NavDataPkt      EngLink_NavPkt;
extern IODefn_IODataPkt        EngLink_IOPkt;
extern IosDefn_IosDataPkt      EngLink_IosPkt;
extern ProtoDefn_ProtoDataPkt  EngLink_ProtoPkt;

extern bool                    EngLink_OctaveMode;

extern void BEGIN_EngLink();

#endif
