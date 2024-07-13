#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <sys/time.h>

#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/navdefn.h>

IODefn_IODataPkt              IosLink_IOPkt1;
IODefn_IODataPkt              IosLink_IOPkt2;
AeroDefn_AeroDataPkt          IosLink_AeroPkt;
EngDefn_EngDataPkt            IosLink_EngPkt;
NavDefn_NavDataPkt            IosLink_NavPkt;
IosDefn_IosDataPkt            IosLink_IosPkt;

void BEGIN_IOSLink(void)
{
}
