/* +------------------------------+---------------------------------+
   | Module      : sounds.c       | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-12      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Generation of aircraft sounds                    |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <SIM/aerodefn.h>
#include <SIM/navdefn.h>
#include <SIM/iodefn.h>
#include <SIM/clocks.h>
#include <SIM/soundlib.h>

#include "englink.h"
#include "engines.h"
#include "sounds.h"
#include "systems.h"

float Kts(float v);
float Feet(float h);

IODefn_GearSelector OldGearSelector;

/* --------------------------------------------------- */  
void SoundSystem(bool Hold)
{
    bool t;
//return; /* *** waiting to fix bug with openal */ 

    OuterMarkerIdent(EngLink_NavPkt.OuterMarker);
    MiddleMarkerIdent(EngLink_NavPkt.MiddleMarker);
    
    t = Feet(EngLink_NavPkt.GroundLevel - (float) EngLink_AeroPkt.Pz) < 500.0 &&  /* below 500 ft */
        EngLink_AeroPkt.GearSelector == IODefn_GearUp &&                          /* with gear up */
        EngLink_AeroPkt.Vd > 1.0;                                                 /* and descending */
    GearWarning(t);
        
    if (EngLink_NavPkt.MorseChannel != 0)
    {
        Morse(EngLink_NavPkt.MorseIdent, EngLink_NavPkt.MorseChannel == 1);
    }
  
    ConfigurationWarning(EngLink_AeroPkt.ConfigWarning);
    StallWarning(EngLink_AeroPkt.Stalling);
    FireWarning(Systems_EngineFireSound);
    
    if (!Hold)
    {
        if (Engines_EngineType == EngDefn_Turbofan)
        {
            AirConditioning(true);
            ElectricalNoise(true);
        }
        
        if (OldGearSelector != EngLink_AeroPkt.GearSelector)
        {
   	        if ((OldGearSelector == IODefn_GearUp) || (OldGearSelector == IODefn_GearDown))
            {
                GearMotor(true);
            }
            OldGearSelector = EngLink_AeroPkt.GearSelector;
        }
    }
    
    Slipstream(Kts(EngLink_AeroPkt.Vc));
    GroundRumble(EngLink_AeroPkt.OnTheGround, Kts(EngLink_AeroPkt.Vc));
    GearBuffet(Kts(EngLink_AeroPkt.Vc), EngLink_AeroPkt.GearPosition);
    
    Sounds(!Hold, Engines_EngineType);

    if (Engines_EngineType == EngDefn_Turbofan)
    {
        int i;
        
        for (i = 0; i <= 3; i += 1)
        {
            JetEngine(i, Engines_Engines[i].Rpm, Engines_ReverseLever[i] > 0.05);
        }
    }
    else if (Engines_EngineType == EngDefn_Piston)
    {
        PistonEngine(1, Engines_Engines[0].Rpm);
    }
    else if (Engines_EngineType == EngDefn_Turboprop)
    {
        TurboPropEngine(1, Engines_Engines[0].Rpm);
    }
}

/* --------------------------------------------------- */  
void BEGIN_Sound()
{
    OldGearSelector = IODefn_GearDown;
}

/* --------------------------------------------------- */  
float Kts(float v)
{
    return v * 1.943844;
}

/* --------------------------------------------------- */  
float Feet(float h)
{
  return h * 3.280840;
}
