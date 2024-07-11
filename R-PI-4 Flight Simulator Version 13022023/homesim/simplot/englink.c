#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>

#include "aerolink.h"
#include "model.h"
#include "aero.h"
#include "engines.h"
#include "systems.h"
#include "simulate.h"
#include "fcs.h"
#include "iolib.h"

AeroDefn_AeroDataPkt    EngLink_AeroPkt;
EngDefn_EngDataPkt      EngLink_EngPkt;
NavDefn_NavDataPkt      EngLink_NavPkt;
IODefn_IODataPkt        EngLink_IOPkt;
IosDefn_IosDataPkt      EngLink_IosPkt;
ProtoDefn_ProtoDataPkt  EngLink_ProtoPkt;

bool                    EngLink_OctaveMode;

/* ----------------------------------------------------------------------------- */
void BEGIN_EngLink()
{
    EngLink_OctaveMode = false;
    EngLink_AeroPkt.OnTheGround = true;
    EngLink_AeroPkt.MachNumber = 0.0;
}
