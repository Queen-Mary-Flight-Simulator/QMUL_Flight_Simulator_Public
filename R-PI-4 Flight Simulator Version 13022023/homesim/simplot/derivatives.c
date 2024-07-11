#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>

#include "aero.h"
#include "aerolink.h"
#include "englink.h"
#include "engines.h"
#include "fcs.h"
#include "maths.h"
#include "model.h"
#include "simulate.h"
#include "systems.h"
#include "stab.h"
#include "iolib.h"
#include "gear.h"

void Derivatives();

/* ----------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    BEGIN_Aero();
    BEGIN_AeroLink();
    BEGIN_EngLink();
    BEGIN_Clocks();
    BEGIN_Engines();
    BEGIN_FCS();
    BEGIN_Stab();
    BEGIN_IOLib();
    BEGIN_Maths();
    BEGIN_Model();
    BEGIN_Gear();
    BEGIN_Systems();
    BEGIN_Weather();

    Model_Flaps = 0.0;  /* flaps up */
	Model_Gear = 1.0;   /* gear down */
    Derivatives();
}

void Derivatives()
{
    printf("Cz0=%f Cz1=%f Cd=%f\n", Aero_AeroCz0(), Aero_AeroCz1(), Aero_AeroCd());
    printf("Cl=%f ClTail=%f Clfw=%f\n", Aero_AeroCl(), Aero_AeroClTail(), Aero_AeroClfw());
    printf("CyBeta=%f CyDr=%f\n", Aero_AeroCyBeta(), Aero_AeroCyDr());
    printf("Cm0=%f CmAlpha=%f CmDe=%f CmQ=%f CmAlphaDot=%f\n", Aero_AeroCm0(), Aero_AeroCmAlpha(), Aero_AeroCmDe(), Aero_AeroCmQ(), Aero_AeroCmAlphaDot());
    printf("ClBeta=%f ClDr=%f ClDa=%f ClP=%f ClR=%f\n", Aero_AeroClBeta(), Aero_AeroClDr(), Aero_AeroClDa(), Aero_AeroClP(), Aero_AeroClR());
    printf("CnBeta=%f CnBetaDot=%f CnDr=%f CnDa=%f CnP=%f CnR=%f\n", Aero_AeroCnBeta(), Aero_AeroCnBetaDot(), Aero_AeroCnDr(), Aero_AeroCnDa(), Aero_AeroCnP(), Aero_AeroCnR());
    printf("Mass=%7.0f Ixx=%7.0f Iyy=%7.0f Izz=%7.0f Ixz=%f\n", Aero_Mass, Aero_Ixx, Aero_Iyy, Aero_Izz, Aero_Ixz);
}
