#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <sys/time.h>

#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/udplib.h>
#include <SIM/protodefn.h>
#include <SIM/iolib.h>

#include "iolink.h"

IODefn_IODataPkt       IOLink_IOPkt1;
IODefn_IODataPkt       IOLink_IOPkt2;
AeroDefn_AeroDataPkt   IOLink_AeroPkt;
EngDefn_EngDataPkt     IOLink_EngPkt;
NavDefn_NavDataPkt     IOLink_NavPkt;
IosDefn_IosDataPkt     IOLink_IosPkt;
ProtoDefn_ProtoDataPkt IOLink_ProtoPkt;

unsigned int           FrameNumber;

/* ---------------------------------------------------- */    
void IOLink_FormPacket()
{
    unsigned int i;

    IOLink_IOPkt1.PktNumber = FrameNumber;
    FrameNumber            += 1;
	
    for (i=0; i<=31; i+=1)
    {
        IOLink_IOPkt1.AnalogueData[i] = IOLib_AnalogueData[i];
    }
    IOLink_IOPkt1.DigitalDataA = IOLib_DigitalDataA;
    IOLink_IOPkt1.DigitalDataB = IOLib_DigitalDataB;
    IOLink_IOPkt1.DigitalDataC = IOLib_DigitalDataC;
    IOLink_IOPkt1.DigitalDataD = IOLib_DigitalDataD;
	IOLink_IOPkt1.Temperature  = IOLib_Temperature;
}

/* ---------------------------------------------------- */    
void BEGIN_IOLink()
{
    FrameNumber = 0;
}
