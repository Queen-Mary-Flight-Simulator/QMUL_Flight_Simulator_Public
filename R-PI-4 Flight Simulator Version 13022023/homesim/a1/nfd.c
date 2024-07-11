/* +------------------------------+---------------------------------+
   | Module      : nfd.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-08      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Navigation Flight Display (NFD)                  |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <sys/time.h>
#include <string.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SIM/iodefn.h>
#include <SIM/glib.h>
#include <SIM/pnglib.h>
#include <SIM/navdefn.h>
#include <SIM/navlib.h>
#include <SIM/maths.h>

#include "aerolink.h"
#include "englink.h"
#include "navlink.h"
#include "ioslink.h"
#include "nav.h"
#include "fcu.h"
#include "panel.h"
#include "nfd.h"
#include "ios.h"
#include "textureid.h"

#include "alt.h"
#include "knobs.h"
#include "vsi.h"
#include "blank.h"
#include "rpm.h"
#include "mp.h"
#include "egt.h"

#include "enginegauges_right.h"
#include "magcompass.h"

NavDefn_FCUMode     NFD_NavDisplayMode;
unsigned int        NFD_NavDisplayRange;

static unsigned int NumberOfSteps;

/* --------------------------------------------------- */
void NFD_Update()
{
    NavLink_FormPacket();

    memcpy(&AeroLink_NavPkt, &NavLink_NavPkt, sizeof(NavDefn_NavDataPkt));
    memcpy(&EngLink_NavPkt,  &NavLink_NavPkt, sizeof(NavDefn_NavDataPkt));
    memcpy(&IosLink_NavPkt,  &NavLink_NavPkt, sizeof(NavDefn_NavDataPkt));

    Nav_UpdateNav();
    Nav_CheckFlightPlan();

    NavLink_RespondToIos();
}

/* --------------------------------------------------- */
void NFD_Display()
{
    NumberOfSteps      = NumberOfSteps + 1;
    Nav_Track          = (float) NavLink_AeroPkt.Yaw - Nav_MagneticVariation;
    NFD_NavDisplayMode = FCU_ModeSelector;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glClear(GL_COLOR_BUFFER_BIT);

    Alt_Altimeter(NFD_AltX, NFD_AltY, -NavLink_AeroPkt.Pz, Knobs_BaroPressure, 0, 0,
                  TEX_ALTITUDE,
                  TEX_ALTITUDE_PRESS,
                  TEX_NEEDLE_ONE,
                  TEX_NEEDLE_FOUR,
                  TEX_KNOB_GENERIC);

    MagneticCompass_Compass(NFD_MagCompassX, NFD_MagCompassY, NavLink_AeroPkt.Yaw - (float) Nav_MagneticVariation,
                            TEX_MAGCOMPASS,
                            TEX_MAGCOMPASS_BG);

    Vsi_Vsi(NFD_VsiX, NFD_VsiY, NavLink_AeroPkt.Vd,
            TEX_VERTSPEED,
            TEX_NEEDLE_ONE);

    Blank_Dial(NFD_BlankX, NFD_BlankY, TEX_BLANK);
	
    Rpm_Rpm(NFD_RpmX, NFD_RpmY, AeroLink_EngPkt.Engines[0].Rpm, TEX_RPM, TEX_NEEDLE_ONE);

    Mp_Mp(NFD_MpX, NFD_MpY, AeroLink_EngPkt.Engines[0].ManifoldPressure, TEX_MP, TEX_NEEDLE_ONE);

    Egt_Egt(NFD_EgtX, NFD_EgtY, AeroLink_EngPkt.Engines[0].Egt, TEX_EGT, TEX_NEEDLE_ONE);

    EngineGauges_LFuelQty(NFD_FuelQLX, NFD_FuelQLY, 0.0,
                          TEX_FUELQTY,
                          TEX_NEEDLE_ONE);

    EngineGauges_FuelPrs(NFD_FuelPrsX, NFD_FuelPrsY, NavLink_EngPkt.Engines[0].Rpm,
                         TEX_FUELPRESS,
                         TEX_NEEDLE_ONE);

    EngineGauges_OilPrs(NFD_OilPrsX, NFD_OilPrsY, NavLink_EngPkt.Engines[0].Rpm,
                        TEX_OILPRESS,
                        TEX_NEEDLE_ONE);

    EngineGauges_OilTemp(NFD_OilTempX, NFD_OilTempY, NavLink_EngPkt.Engines[0].Rpm,
                         NavLink_AeroPkt.Vc,
                         TEX_OILTEMP,
                         TEX_NEEDLE_ONE);

    EngineGauges_RFuelQty(NFD_FuelQRX, NFD_FuelQRY, 0.0,
                          TEX_FUELQTY,
                          TEX_NEEDLE_ONE);

    Panel_CheckPanel();

    Glib_LoadIdentity();
}

/* --------------------------------------------------- */
void NFD_Init()
{
}

/* --------------------------------------------------- */
void BEGIN_NFD()
{
    printf("NFD starting\n");
	
    NumberOfSteps     = 0;
}
