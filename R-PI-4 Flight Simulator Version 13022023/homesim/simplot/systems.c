/* SIMPLOT version */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <SIM/maths.h>
#include <SIM/iodefn.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>

#include "iolib.h"
#include "model.h"
#include "systems.h"
#include "aerolink.h"

bool                                     Systems_Failures[51];
unsigned int                             Systems_FlapSetting;
float                                    Systems_FlapPosition;
float                                    Systems_GearPosition;
IODefn_GearSelector                      Systems_GearSelector;
bool                                     Systems_ConfigWarning;

bool                                     Systems_RemoteHold;
bool                                     Systems_Freezing;
bool                                     Systems_EngineFireSound;

bool                                     Systems_AttentionGetter;
bool                                     Systems_EngineFire[4];

/* ----------------------------------------- */
void Systems_UpdateConfigWarning()
{
}

/* ----------------------------------------- */
void Systems_UpdateElevatorTrim()
{
}

/* ----------------------------------------- */
void Systems_UpdateGearSelector(bool Airborne)
{
}

/* ----------------------------------------- */
float Systems_GetFlapPosition()
{
    return Systems_FlapPosition;
}

/* ----------------------------------------- */
float Systems_GetGearPosition()
{
    return Systems_GearPosition;
}

/* ----------------------------------------- */
void BEGIN_Systems()
{
    unsigned int i;

    for (i = 1; i <= 50; i += 1)
    {
        Systems_Failures[i] = false;
    }
    Systems_FlapPosition  = 0.0;
    Systems_GearPosition  = 0.0;
    Systems_GearSelector  = IODefn_GearDown;
    Systems_ConfigWarning = false;

    Systems_RemoteHold = false;
    Systems_Freezing = false;

    Systems_AttentionGetter = false;

    for (i=0; i<=3; i += 1)
    {
        Systems_EngineFire[i] = false;
    }
}
