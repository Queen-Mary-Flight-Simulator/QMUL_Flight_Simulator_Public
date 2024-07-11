#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>

#include "aerolink.h"

AeroDefn_AeroDataPkt    AeroLink_AeroPkt;
EngDefn_EngDataPkt      AeroLink_EngPkt;
NavDefn_NavDataPkt      AeroLink_NavPkt;
IODefn_IODataPkt        AeroLink_IOPkt;
IosDefn_IosDataPkt      AeroLink_IosPkt;
ProtoDefn_ProtoDataPkt  AeroLink_ProtoPkt;

bool                    AeroLink_OctaveMode;
bool                    AeroLink_Freezing;

/* ----------------------------------------------------------------------------- */
void BEGIN_AeroLink()
{
    AeroLink_Freezing   = false;
    AeroLink_OctaveMode = false;
}
