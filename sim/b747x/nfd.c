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

#include <windows.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include <SIM/iodefn.h>
#include <SIM/glib.h>
#include <SIM/pnglib.h>
#include <SIM/navdefn.h>
#include <SIM/navlib.h>

#include "aerolink.h"
#include "englink.h"
#include "navlink.h"
#include "ioslink.h"
#include "nfd-compass.h"
#include "nav.h"
#include "fcu.h"
#include "navinfo.h"
#include "panel.h"
#include "panellib.h"
#include "nfd.h"
#include "radio.h"

NavDefn_FCUMode NFD_NavDisplayMode;
unsigned int    NFD_NavDisplayRange;

int             xOrigin;
int             yOrigin;

unsigned int    NumberOfSteps;

void SetOrigin(int x, int y);

/* --------------------------------------------------- */
void SetOrigin(int x, int y)
{
    glTranslatef((float) (x - xOrigin), (float) (y - yOrigin), 0.0);
    xOrigin = x;
    yOrigin = y;
}

/* --------------------------------------------------- */
void NFD_Update()
{
    NavLink_FormPacket();

    memcpy(&AeroLink_NavPkt, &NavLink_NavPkt, sizeof(NavDefn_NavDataPkt));
    memcpy(&EngLink_NavPkt,  &NavLink_NavPkt, sizeof(NavDefn_NavDataPkt));
    memcpy(&IosLink_NavPkt,  &NavLink_NavPkt, sizeof(NavDefn_NavDataPkt));

    Panel_CheckPanel();
    Nav_UpdateNav();
    Radio_RMP();
    Nav_CheckFlightPlan();

    NavLink_RespondToIos();
}

/* --------------------------------------------------- */
void NFD_Display()
{
    float TAS;

    TAS                = NavLink_AeroPkt.U;
    NumberOfSteps      = NumberOfSteps + 1;
    Nav_Track          = (float) NavLink_AeroPkt.Yaw - Nav_MagneticVariation;
    NFD_NavDisplayMode = FCU_ModeSelector;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glClear(GL_COLOR_BUFFER_BIT);

    switch (NFD_NavDisplayMode)
    {
        case NavDefn_ModeNAV:
        case NavDefn_ModeVOR:
        case NavDefn_ModeILS:
            SetOrigin(NFD_CompassX, NFD_CompassY);
            Compass_Compass((float) NavLink_AeroPkt.Yaw - Nav_MagneticVariation,
                            Nav_HSI_Localiser, Nav_Track, Nav_HSI_Crs, FCU_HDG,
                            FCU_RangeSelector, Nav_HSI_ILSMode, NFD_NavDisplayMode);
            Compass_Rmi(Nav_Rmi_Dir1, FCU_NavSwitch1, Nav_Rmi_Dir2, FCU_NavSwitch2);
            if (NFD_NavDisplayMode == NavDefn_ModeILS)
            {
                Compass_GlideSlope(Nav_HSI_GlideSlope, Nav_HSI_ILSMode, true);
            }
            break;

        case NavDefn_ModeARC:
            SetOrigin(NFD_xCompassX, NFD_xCompassY);
            Glib_ClipWindow(NFD_xCompassX - 294, NFD_xCompassY - 106,
                            294 * 2, Glib_SCREENHEIGHT - 1 - NFD_xCompassY + 118);
            Compass_ExpandedCompass((float) NavLink_AeroPkt.Yaw - Nav_MagneticVariation,
                                    Nav_HSI_Localiser, Nav_Track, Nav_HSI_Crs, FCU_HDG,
                                    FCU_RangeSelector, Nav_HSI_ILSMode, NFD_NavDisplayMode);
            Compass_ExpandedRmi(Nav_Rmi_Dir1, FCU_NavSwitch1, Nav_Rmi_Dir2, FCU_NavSwitch2);
            Glib_ClipWindow(0, 0, Glib_SCREENWIDTH - 1, Glib_SCREENHEIGHT - 1);
            break;

        case NavDefn_ModePLAN:
            SetOrigin(NFD_CompassX, NFD_CompassY);
            Compass_DisplayPlan(FCU_RangeSelector);
            break;

        default:
            break;
    }
	
    SetOrigin(0, 0);
    NavInfo_UpdateLeftNavInfo(FCU_NavSwitch1);
    NavInfo_UpdateRightNavInfo(FCU_NavSwitch2);
    NavInfo_UpdateTopNavInfo(NFD_NavDisplayMode);
    NavInfo_UpdateWindVector((float) NavLink_AeroPkt.WindSpeed, (float) NavLink_AeroPkt.WindDir);
    NavInfo_UpdateGS(Nav_GroundSpeed);
    NavInfo_UpdateTAS(TAS);

    glEnable(GL_TEXTURE_2D);
    Panel_CheckPanel();
    glDisable(GL_TEXTURE_2D);
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

    xOrigin           = 0;
    yOrigin           = 0;
}
