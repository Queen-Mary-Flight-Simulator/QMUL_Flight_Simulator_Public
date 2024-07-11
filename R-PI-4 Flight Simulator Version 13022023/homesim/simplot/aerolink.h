#ifndef AeroLink_H
#define AeroLink_H

#include <stdbool.h>

#include  <SIM/aerodefn.h>
#include  <SIM/engdefn.h>
#include  <SIM/navdefn.h>
#include  <SIM/iodefn.h>
#include  <SIM/iosdefn.h>
#include  <SIM/protodefn.h>

extern AeroDefn_AeroDataPkt    AeroLink_AeroPkt;
extern EngDefn_EngDataPkt      AeroLink_EngPkt;
extern NavDefn_NavDataPkt      AeroLink_NavPkt;
extern IODefn_IODataPkt        AeroLink_IOPkt;
extern IosDefn_IosDataPkt      AeroLink_IosPkt;
extern ProtoDefn_ProtoDataPkt  AeroLink_ProtoPkt;

extern bool                    AeroLink_OctaveMode;
extern bool                    AeroLink_Freezing;

extern void BEGIN_AeroLink();

#endif
