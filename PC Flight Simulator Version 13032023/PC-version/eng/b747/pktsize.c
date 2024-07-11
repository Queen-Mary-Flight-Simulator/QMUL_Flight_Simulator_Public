#include <stdio.h>
#include <stdlib.h>

#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>
#include <SIM/igdefn.h>

int main()
{
    printf("IOData Pkt size            = %d\n", sizeof(IODefn_IODataPkt));
    printf("AeroData Pkt size          = %d\n", sizeof(AeroDefn_AeroDataPkt));
    printf("EngData Pkt size           = %d\n", sizeof(EngDefn_EngDataPkt));
    printf("NavData Pkt size           = %d\n", sizeof(NavDefn_NavDataPkt));
    printf("IOSData Pkt size           = %d\n", sizeof(IosDefn_IosDataPkt));
    printf("RestoreVectorRecord size   = %d\n", sizeof(IosDefn_RestoreVectorRecord));
    printf("ProtoData Pkt size         = %d\n", sizeof(ProtoDefn_ProtoDataPkt));
    printf("IGData Pkt size            = %d\n", sizeof(IGDefn_IGDataPkt));
    printf("PlaybackDataPktRecord size = %d\n", sizeof(IosDefn_PlaybackDataPktRecord));
    return 0;
}
