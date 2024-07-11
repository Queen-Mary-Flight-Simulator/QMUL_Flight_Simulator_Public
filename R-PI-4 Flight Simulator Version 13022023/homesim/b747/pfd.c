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

#include <GL/glew.h>
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
#include "ios.h"
#include "fma.h"

unsigned int NumberOfSteps;
bool         PFD_Held = false;

void PFD_Display();

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

    IAS = Model_U * sqrt(Weather_DensityRatio);

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

    Ai_AttitudeIndicator(PFD_AiX, PFD_AiY, Model_Pitch, Model_Roll, Model_SideForce / (Aero_Mass * 9.81));
    Ai_Markers(AeroLink_NavPkt.OuterMarker, AeroLink_NavPkt.MiddleMarker,
               false, AeroLink_NavPkt.MarkerTest);
    Ai_FlightDirector(FCS_FD_VBar, FCS_FD_HBar, AeroLink_NavPkt.FCU_FD);
    Ai_GlideSlope(GSErr, Beacon, LS);
    Ai_Localiser(LocErr, Beacon, LS);

    Compass_PFD_Compass(PFD_CompassX, PFD_CompassY,
                        Model_Yaw - (float) (AeroLink_NavPkt.MagneticVariation),
                        (int) (AeroLink_NavPkt.FCU_HDG), (float) (AeroLink_NavPkt.Track));

    Asi_Asi(PFD_AsiX, PFD_AsiY, IAS, (unsigned int) (AeroLink_NavPkt.FCU_SPD), Model_UDot,
            Model_MachNumber, AeroLink_NavPkt.FCU_SPD, AeroLink_NavPkt.FCU_SPD_MACH);

    Alt_Altimeter(PFD_AltX, PFD_AltY,
                  -Model_Pz, (unsigned int) (AeroLink_NavPkt.FCU_BaroPressure),
                  AeroLink_NavPkt.FCU_BaroHg, AeroLink_NavPkt.FCU_BaroKnob,
                  AeroLink_NavPkt.FCU_ALT, AeroLink_NavPkt.FCU_Metric_Button);
    
    Alt_Baro((int) (AeroLink_NavPkt.FCU_BaroPressure), AeroLink_NavPkt.FCU_BaroHg,
             AeroLink_NavPkt.FCU_BaroKnob);

    Alt_RadioAltimeter(PFD_RadAltX, PFD_RadAltY, -Model_Pz + (float) (AeroLink_NavPkt.GroundLevel) + Aero_CGHeight);

    Vsi_Vsi(PFD_VsiX, PFD_VsiY, Model_Vd);

    FMA_FMA(PFD_FMAX, PFD_FMAY);
	
    for (e = 0; e <= 3; e += 1)
    {
        EICAS_EprGauge(PFD_EprX + e * 160, PFD_EprY, e);
        EICAS_RpmGauge(PFD_RpmX + e * 160, PFD_RpmY, e);
        EICAS_EgtGauge(PFD_EgtX + e * 160, PFD_EgtY, e);
    }

    EICAS_DisplayGear(PFD_GearX, PFD_GearY, Systems_GearPosition);
    EICAS_FlapsIndicator(PFD_FlapsX, PFD_FlapsY, Systems_FlapPosition, Systems_FlapSetting);
    if (DEMO_MODE)
    {
        EICAS_ParkBrake(PFD_ParkBrakeX, PFD_ParkBrakeY, Systems_ParkBrake);
    }
    else
    {
        EICAS_ParkBrake(PFD_ParkBrakeX, PFD_ParkBrakeY, IOLib_GetParkBrake());
    }
    
    Glib_LoadIdentity();
}

/* --------------------------------------------------- */
void PFD_Init()
{
}

/* --------------------------------------------------- */
void BEGIN_PFD()
{
    NumberOfSteps = 0;
}
