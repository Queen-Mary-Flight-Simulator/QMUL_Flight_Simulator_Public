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
#include <SIM/maths.h>

#include "textureid.h"
#include "model.h"
#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "compass.h"
#include "vsi.h"
#include "aero.h"
#include "fcs.h"
#include "pfd.h"
#include "systems.h"
#include "aerolink.h"
#include "iolib.h"
#include "ios.h"
#include "aoa.h"
#include "gmeter.h"
#include "turnslip.h"
#include "clock.h"
#include "enginegauges_left.h"
#include "panel.h"

unsigned int NumberOfSteps;
bool         PFD_Held = false;

void PFD_Display();

/* --------------------------------------------------- */
void PFD_Display()
{
    float        IAS;
    static float turnangle = 0.0;
    
    NumberOfSteps = NumberOfSteps + 1;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glClear(GL_COLOR_BUFFER_BIT);

    IAS = Model_U * sqrt(Weather_DensityRatio);

    Ai_AttitudeIndicator(PFD_AiX, PFD_AiY, Model_Pitch, Model_Roll, TEX_AI, TEX_AI_BG);

    Asi_Asi(PFD_AsiX, PFD_AsiY, IAS, TEX_SPEED, TEX_NEEDLE_ONE);

    Clock_Clock(PFD_ClockX, PFD_ClockY, TEX_CLOCK, TEX_NEEDLE_ONE, TEX_NEEDLE_THREE);

    Aoa_Aoa(PFD_AoAX, PFD_AoAY, Model_Alpha, TEX_AOA, TEX_NEEDLE_ONE);

    GMeter_GMeter(PFD_GMeterX, PFD_GMeterY, Model_GForce, Model_GMin, Model_GMax, TEX_ACCEL, TEX_NEEDLE_ONE);

    Compass_Compass(PFD_CompassX, PFD_CompassY, Model_Yaw - (float) (AeroLink_NavPkt.MagneticVariation), 
                    (int) (AeroLink_NavPkt.FCU_HDG), TEX_COMPASS_STATIC, TEX_COMPASS_DIAL, TEX_COMPASS_AIRCRAFT, TEX_KNOB_GENERIC);
				  
    turnangle = Maths_Integrate(turnangle, 0.5* (Model_R - turnangle)); /* indicated turn */
    TurnSlip_TurnSlip(PFD_TurnSlipX, PFD_TurnSlipY, turnangle, TEX_SIDESLIP, TEX_NEEDLE_ONE, TEX_SLIPBALL, TEX_SLIP_OVERLAY);
  
    EngineGauges_Ammeter(PFD_AmmeterX, PFD_AmmeterY, AeroLink_EngPkt.Engines[0].Rpm, TEX_AMMETER, TEX_NEEDLE_ONE);
    EngineGauges_Suction(PFD_SuctionX, PFD_SuctionY, AeroLink_EngPkt.Engines[0].Rpm, TEX_SUCTION, TEX_NEEDLE_ONE);
//    Blank_Blank(PFD_BlankX, PFD_BlankY, TEX_BLANK);
    
    Panel_CheckPanel();

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
