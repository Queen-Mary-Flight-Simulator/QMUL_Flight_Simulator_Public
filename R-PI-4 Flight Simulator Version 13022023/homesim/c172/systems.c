/* +------------------------------+---------------------------------+
   | Module      : systems.c      | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-16      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 Systems                           |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/iodefn.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>

#include "ios.h"
#include "iolib.h"
#include "model.h"
#include "systems.h"
#include "aerolink.h"

bool                  Systems_Failures[51];
unsigned int          Systems_FlapSetting;
float                 Systems_FlapPosition;
float                 Systems_GearPosition;
IODefn_GearSelector   Systems_GearSelector;
bool                  Systems_ConfigWarning;
bool                  Systems_ParkBrake;

bool                  Systems_RemoteHold;
bool                  Systems_Freezing;
unsigned int          Systems_SysTicks;
bool                  Systems_EngineFireSound;
bool                  Systems_AttentionGetter;
bool                  Systems_EngineFire[4];

bool                  Systems_AdfFixedCard;
bool                  Systems_AdfDip;
bool                  Systems_HSI_Installed;
bool                  Systems_VOR_Installed;
bool                  Systems_Radio_Installed;
float                 Systems_SelectedAltitude;
bool                  Systems_MarkerTest;
IODefn_SwitchPosition Systems_MasterSwitch;
IODefn_SwitchPosition Systems_KeySwitch;

static unsigned int   DoorsOpen;
static bool           OldDoorsOpenLight;
static bool           OldGearTransitLight;
static bool           OldGearDownLight;
static float          etrim;
static float          atrim;
static float          rtrim;
static unsigned int   i;

static unsigned int         FlapSettings[7] = 
{
    0, 1, 5, 10, 20, 25, 30
};
//static unsigned int         FlapDetents[7] = 
//{
//    0, 147, 403, 709, 1140, 1516, 1817
//};

/* ----------------------------------------- */
void  Systems_UpdateGearSelector(bool Airborne)
{
    return; /* n/a to EFS-500 */
}

/* ----------------------------------------- */
void Systems_UpdateConfigWarning()
{
    /* n/a to C172 */
}

/* ----------------------------------------- */
void Systems_UpdateElevatorTrim()
{
    IODefn_ElevatorTrimSwitchPosition s;

    s = IOLib_GetElevatorTrimSwitch();
    if (s == IODefn_ElevatorTrimForwards)
    {
        etrim = etrim + 0.002;
        if (etrim > 1.0)
        {
            etrim = 1.0;
        }
    }
    else if (s == IODefn_ElevatorTrimBackwards)
    {
        etrim = etrim - 0.002;
        if (etrim < -1.0)
        {
            etrim = -1.0;
        }
    }
}

/* ----------------------------------------- */
void Systems_UpdateAileronTrim()
{
}

/* ----------------------------------------- */
void Systems_UpdateRudderTrim()
{
}

/* ----------------------------------------- */
float Systems_GetElevatorTrim()
{
    return etrim;
}

/* ----------------------------------------- */
float Systems_GetAileronTrim()
{
    return atrim;
}

/* ----------------------------------------- */
float Systems_GetRudderTrim()
{
    return rtrim;
}

/* ----------------------------------------- */
float Systems_GetFlapPosition()
{
    int   f;
    float FlapDemand;

    if (!DEMO_MODE)
	{
        f = IOLib_GetFlapSelector();  /* range 0..6 */  // * removed for desktop demo
        Systems_FlapSetting = FlapSettings[f];          // * removed for desktop demo
    }
	
    FlapDemand = (float) (Systems_FlapSetting) / 30.0;
    if (Systems_Failures[1] == false)
    {
        if (fabs(FlapDemand - Systems_FlapPosition) > 0.001)
        {
            if (FlapDemand > Systems_FlapPosition)
            {
                Systems_FlapPosition = Systems_FlapPosition + Systems_FlapDownRate;
            }
            else
            {
                Systems_FlapPosition = Systems_FlapPosition - Systems_FlapUpRate;
            }
            Systems_FlapPosition = Maths_Limit(Systems_FlapPosition, 0.0, 1.0);
        }
    }
    return Systems_FlapPosition;
}

/* ----------------------------------------- */
float Systems_GetGearPosition()
{
    return 0.0;
}

/* ----------------------------------------- */
void BEGIN_Systems()
{
    for (i = 1; i <= 50; i += 1)
    {
        Systems_Failures[i] = false;
    }
    Systems_FlapPosition    = 1.0;
    Systems_GearPosition    = 1.0;
	Systems_ParkBrake       = false;
    Systems_ConfigWarning   = false;
    DoorsOpen               = 0;
    OldDoorsOpenLight       = false;
    OldGearTransitLight     = false;
    OldGearDownLight        = false;
    etrim                   = 0.0;
    atrim                   = 0.0;
    rtrim                   = 0.0;

    Systems_RemoteHold      = false;
    Systems_Freezing        = false;
    Systems_SysTicks        = false;
    Systems_EngineFireSound = false;
    Systems_AttentionGetter = false;
    for (i = 0; i <= 3; i += 1)
    {
        Systems_EngineFire[i] = false;
    }

    Systems_AdfFixedCard = true;
    Systems_AdfDip = false;
    Systems_HSI_Installed = true;
    Systems_VOR_Installed = false;
    Systems_Radio_Installed = true;
    Systems_MarkerTest = false;
    Systems_MasterSwitch = IODefn_On;
    Systems_KeySwitch = IODefn_On;
    Systems_RemoteHold = false;
}
