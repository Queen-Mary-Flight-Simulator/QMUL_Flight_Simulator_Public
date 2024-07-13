/* +------------------------------+---------------------------------+
   | Module      : pfd.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-14      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EFIS Primary Flight Display (PFD) |
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
#include <SIM/navdefn.h>
#include <SIM/glib.h>
#include <SIM/pnglib.h>
#include <SIM/weather.h>
#include <SIM/udplib.h>

#include "model.h"
#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "compass.h"
#include "eicas.h"
#include "vsi.h"
#include "aero.h"
#include "fcs.h"
#include "pfd.h"
#include "systems.h"
#include "aerolink.h"
#include "iolib.h"

int          xOrigin;
int          yOrigin;
unsigned int NumberOfSteps;

void PFD_SetOrigin(int x, int y);
void PFD_Display();

/* --------------------------------------------------- */
void PFD_SetOrigin(int x, int y)
{
    glTranslatef((float) (x - xOrigin), (float) (y - yOrigin), 0.0);
    xOrigin = x;
    yOrigin = y;
}

/* --------------------------------------------------- */
void PFD_Display()
{
    float        GSErr;
    float        LocErr;
    bool         Beacon;
    bool         LS;
    float        IAS;
    unsigned int e;
	
    NumberOfSteps = NumberOfSteps + 1;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glClear(GL_COLOR_BUFFER_BIT);

    IAS           = Model_U * sqrt(Weather_DensityRatio);
    if (AeroLink_NavPkt.Mode == NavDefn_ModeILS)
    {
        GSErr  = AeroLink_NavPkt.ILS1.GlideSlopeError;
        LocErr = AeroLink_NavPkt.ILS1.LocaliserError;
        Beacon = AeroLink_NavPkt.ILS1.ILSBeacon;
        LS     = AeroLink_NavPkt.FCU_LS;
    }
    else
    {
        GSErr  = AeroLink_NavPkt.NAV1.GlideSlopeError;
        LocErr = AeroLink_NavPkt.NAV1.LocaliserError;
        Beacon = AeroLink_NavPkt.NAV1.ILSBeacon;
        LS     = AeroLink_NavPkt.FCU_LS;
    }

    PFD_SetOrigin(PFD_AiX, PFD_AiY);
    Ai_AttitudeIndicator(PFD_AiX, PFD_AiY, Model_Pitch, Model_Roll,
                         Model_SideForce / (Aero_Mass * 9.81));

    Ai_Markers(AeroLink_NavPkt.OuterMarker, AeroLink_NavPkt.MiddleMarker,
               false, AeroLink_NavPkt.MarkerTest);
    Ai_FlightDirector(FCS_FD_VBar, FCS_FD_HBar, AeroLink_NavPkt.FCU_FD);
    Ai_GlideSlope(GSErr, Beacon, LS);
    Ai_Localiser(LocErr, Beacon, LS);

    PFD_SetOrigin(PFD_CompassX, PFD_CompassY);
    Compass_PFD_Compass(PFD_CompassX, PFD_CompassY,
                        Model_Yaw - (float) (AeroLink_NavPkt.MagneticVariation),
                        (int) (AeroLink_NavPkt.FCU_HDG), (float) (AeroLink_NavPkt.Track));

    PFD_SetOrigin(PFD_AsiX, PFD_AsiY);
    Asi_Asi(PFD_AsiX, PFD_AsiY, IAS, (unsigned int) (AeroLink_NavPkt.FCU_SPD), Model_UDot,
            (float) (AeroLink_NavPkt.NAV1.GroundSpeed), Model_MachNumber,
            AeroLink_NavPkt.FCU_SPD, AeroLink_NavPkt.FCU_SPD_MACH);

    PFD_SetOrigin(PFD_AltX, PFD_AltY);
    Alt_Altimeter(PFD_AltX, PFD_AltY,
                  -Model_Pz, (unsigned int) (AeroLink_NavPkt.FCU_BaroPressure),
                  AeroLink_NavPkt.FCU_BaroHg, AeroLink_NavPkt.FCU_BaroSTD,
                  AeroLink_NavPkt.FCU_ALT, AeroLink_NavPkt.FCU_Metric_ALT);
    Alt_Baro((int) (AeroLink_NavPkt.FCU_BaroPressure), AeroLink_NavPkt.FCU_BaroHg,
             AeroLink_NavPkt.FCU_BaroSTD);

    PFD_SetOrigin(PFD_RadAltX, PFD_RadAltY);
    Alt_RadioAltimeter(-Model_Pz + (float) (AeroLink_NavPkt.GroundLevel) + Aero_CGHeight);

    PFD_SetOrigin(PFD_VsiX, PFD_VsiY);
    Vsi_Vsi(PFD_VsiX, PFD_VsiY, Model_Vd);

    for (e = 0; e <= 3; e += 1)
    {
        PFD_SetOrigin(PFD_EprX + e * 150, PFD_EprY);
        EICAS_EprGauge(e);
        PFD_SetOrigin(PFD_RpmX + e * 150, PFD_RpmY);
        EICAS_RpmGauge(e);
        PFD_SetOrigin(PFD_EgtX + e * 150, PFD_EgtY);
        EICAS_EgtGauge(e);
    }

    PFD_SetOrigin(PFD_GearX, PFD_GearY);
    EICAS_DisplayGear(Systems_GearPosition);
    PFD_SetOrigin(PFD_FlapsX, PFD_FlapsY);
    EICAS_FlapsIndicator(Systems_FlapPosition, Systems_FlapSetting);
    PFD_SetOrigin(PFD_ParkBrakeX, PFD_ParkBrakeY);
	EICAS_ParkBrake(IOLib_GetParkBrake());

    PFD_SetOrigin(0, 0);
}

/* --------------------------------------------------- */
void PFD_Init()
{
}

/* --------------------------------------------------- */
void BEGIN_PFD()
{
    NumberOfSteps = 0;

    xOrigin = 0;
    yOrigin = 0;
}
