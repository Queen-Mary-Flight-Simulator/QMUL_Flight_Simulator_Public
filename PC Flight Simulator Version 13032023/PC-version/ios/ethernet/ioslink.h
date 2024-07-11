/*
    ioslink.h
*/

#ifndef IosLink_H
#define IosLink_H

#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>

extern IODefn_IODataPkt              IosLink_IOPkt1;
extern IODefn_IODataPkt              IosLink_IOPkt2;
extern AeroDefn_AeroDataPkt          IosLink_AeroPkt;
extern EngDefn_EngDataPkt            IosLink_EngPkt;
extern NavDefn_NavDataPkt            IosLink_NavPkt;
extern IosDefn_IosDataPkt            IosLink_IosPkt;

void BEGIN_IOSLink();

#endif

