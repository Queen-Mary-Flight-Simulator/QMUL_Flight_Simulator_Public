/* Boeing 747-400 Display Demo */

#include <stdio.h>
#include <stdbool.h>

#include <SIM/clocks.h>
#include <SIM/maths.h>
#include <SIM/glib.h>
#include <SIM/weather.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>

#include <GLFW/glfw3.h>

#include "pfd.h"
#include "aerolink.h"

#include "model.h"
#include "systems.h"
#include "aero.h"

#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "compass.h"
#include "eicas.h"
#include "vsi.h"
#include "fcs.h"

static float             GearTable[4] = {
    0.0, 0.5, 1.0, 0.5
};

static unsigned int      FlapTable[6] = {
    0, 5, 10, 20, 25, 30
};

static unsigned int      EngineNumber;
static float             dh;
static float             dy;
static float             dv;
static float             dUDot;
static float             dpitch;
static float             droll;
static float             dvc;
static float             dFlaps;
static float             dEpr;
static float             dRpm;
static float             dEgt;
static float             dFf;
static int               dRMI1, dRMI2;
static float             dGSErr;
static float             HSI_Crsx;
static float             dHSI_Crsx;
static float             HSI_Hdgx;
static float             dHSI_Hdgx;
static float             dHSI_Localiser;
static float             dTrack;
static float             dR;
static float             dSideForce;
static float             dDmeDistance;
static unsigned int      iGear;
static unsigned int      iFlapSetting;

static int               timer1;
static int               timer2;

static void SystemUpdate();
void Clocksync();

/* ------------------------------------------- */
static void SystemUpdate()
{
    unsigned int EngineNumber;

    Clocks_ClockTicks = Clocks_ClockTicks + 1;
    if (Clocks_ClockTicks >= 50)
    {
        Clocks_ClockTicks = 0;
        Clocks_TimerSecs  = Clocks_TimerSecs + 1;
        if (Clocks_TimerSecs >= 60)
        {
            Clocks_TimerSecs = 0;
            Clocks_TimerMins = Clocks_TimerMins + 1;
            if (Clocks_TimerMins >= 60)
            {
                Clocks_TimerMins  = 0;
                Clocks_TimerHours = Clocks_TimerHours + 1;
                if (Clocks_TimerHours >= 24)
                {
                    Clocks_TimerHours = 0;
                }
            }
        }
        Clocks_ClockSecs = Clocks_ClockSecs + 1;
        if (Clocks_ClockSecs >= 60)
        {
            Clocks_ClockSecs = 0;
            Clocks_ClockMins = Clocks_ClockMins + 1;
            if (Clocks_ClockMins >= 60)
            {
                Clocks_ClockMins  = 0;
                Clocks_ClockHours = Clocks_ClockHours + 1;
                if (Clocks_ClockHours >= 24)
                {
                    Clocks_ClockHours = 0;
                }
            }
        }
    }
    Model_UDot = -1.0;
    
    Model_Pz = Model_Pz + dh;
    if (Model_Pz < -12000.0 || Model_Pz > 0.0)
    {
        dh = -dh;
    }

    Model_U = Model_U + dUDot;
    if (Model_U < 15.0 || Model_U > 200.0)
    {
        dUDot = -dUDot;
    }
    Model_Vd = Model_Vd + dv;
    if (Model_Vd < -30.0 || Model_Vd > 30.0)
    {
        dv = -dv;
    }
    Model_Yaw = Model_Yaw + dy;
    Model_Yaw = Maths_Normalise(Model_Yaw);
    Model_Pitch = Model_Pitch + dpitch;
    if (Model_Pitch < -0.4 || Model_Pitch > 0.4)
    {
        dpitch = -dpitch;
    }
    Model_Roll = Model_Roll + droll;
    if (Model_Roll < -1.5 || Model_Roll > 1.5)
    {
        droll = -droll;
    }
    Systems_FlapPosition = Systems_FlapPosition + dFlaps;
    if (Systems_FlapPosition < 0.0 || Systems_FlapPosition > 1.0)
    {
        dFlaps = -dFlaps;
    }
    if (Clocks_ClockSecs % 15 == 0 && Clocks_ClockTicks == 1)
    {
        iFlapSetting        = (iFlapSetting + 1) % 6;
        Systems_FlapSetting = FlapTable[iFlapSetting];
    }
    AeroLink_NavPkt.OuterMarker  = Clocks_ClockSecs < 10;
    AeroLink_NavPkt.MiddleMarker = Clocks_ClockSecs > 20 && Clocks_ClockSecs < 30;
    AeroLink_NavPkt.MarkerTest   = Clocks_ClockSecs > 40 && Clocks_ClockSecs < 50;
    if (Clocks_ClockSecs % 15 == 0 && Clocks_ClockTicks == 1)
    {
        iGear                = (iGear + 1) % 4;
        Systems_GearPosition = GearTable[iGear];
    }
    Model_Vc = Model_Vc + dvc;
    if (Model_Vc < 0.0 || Model_Vc > 225.0)
    {
        if (Model_Vc < 0.0)
        {
            Model_Vc = 0.0;
        }
        dvc = -dvc;
    }
    Model_MachNumber = Model_Vc / 340.0;
    for (EngineNumber = 0; EngineNumber <= 3; EngineNumber += 1)
    {
        {
            AeroLink_EngPkt.Engines[EngineNumber].Epr      += dEpr;
            AeroLink_EngPkt.Engines[EngineNumber].Rpm      += dRpm;
            AeroLink_EngPkt.Engines[EngineNumber].Egt      += dEgt;
            AeroLink_EngPkt.Engines[EngineNumber].FuelFlow += dFf;
        }
    }
    if (AeroLink_EngPkt.Engines[0].Epr < 0.0 || AeroLink_EngPkt.Engines[0].Epr > 1.8)
    {
        dEpr = -dEpr;
    }
    if (AeroLink_EngPkt.Engines[0].Rpm < 0.0 || AeroLink_EngPkt.Engines[0].Rpm > 110.0)
    {
        dRpm = -dRpm;
    }
    if (AeroLink_EngPkt.Engines[0].Egt < 0.0 || AeroLink_EngPkt.Engines[0].Egt > 1000.0)
    {
        dEgt = -dEgt;
    }
    if (AeroLink_EngPkt.Engines[0].FuelFlow < 0.0 || AeroLink_EngPkt.Engines[0].FuelFlow > 8000.0)
    {
        dFf = -dFf;
    }
    Model_R = Model_R + dR;
    if (Model_R < -0.1 || Model_R > 0.1)
    {
        dR = -dR;
    }
    Model_SideForce = Model_SideForce + dSideForce;
    if (Model_SideForce < -250000.0 || Model_SideForce > 250000.0)
    {
        dSideForce = -dSideForce;
    }
    AeroLink_NavPkt.NAV1.SlantDistance = AeroLink_NavPkt.NAV1.SlantDistance + dDmeDistance;
    if (AeroLink_NavPkt.NAV1.SlantDistance < -2000.0)
    {
        AeroLink_NavPkt.NAV1.SlantDistance = 20000.0;
    }
    HSI_Crsx = HSI_Crsx + dHSI_Crsx;
    if (HSI_Crsx > 360.0)
    {
        HSI_Crsx = 1.0;
    }
    if (HSI_Crsx < 1.0)
    {
        HSI_Crsx = 360.0;
    }
    AeroLink_NavPkt.HSI_Crs = intround(HSI_Crsx);
    HSI_Hdgx                = HSI_Hdgx + dHSI_Hdgx;
    if (HSI_Hdgx > 360.0)
    {
        HSI_Hdgx = 1.0;
    }
    if (HSI_Hdgx < 1.0)
    {
        HSI_Hdgx = 360.0;
    }
    AeroLink_NavPkt.HSI_Hdg = intround(HSI_Hdgx);
    AeroLink_NavPkt.Track   = Model_Yaw + dTrack;
    if ((float) AeroLink_NavPkt.Track > Model_Yaw + 0.2 || (float) AeroLink_NavPkt.Track < Model_Yaw - 0.2)
    {
        dTrack = -dTrack;
    }
    AeroLink_NavPkt.NAV1.LocaliserError = AeroLink_NavPkt.NAV1.LocaliserError + dHSI_Localiser;
    if (AeroLink_NavPkt.NAV1.LocaliserError < -0.04 || AeroLink_NavPkt.NAV1.LocaliserError > 0.04)
    {
        dHSI_Localiser = -dHSI_Localiser;
    }
    AeroLink_NavPkt.NAV1.GlideSlopeError = AeroLink_NavPkt.NAV1.GlideSlopeError + dGSErr;
    if (AeroLink_NavPkt.NAV1.GlideSlopeError < -0.0153 || AeroLink_NavPkt.NAV1.GlideSlopeError > 0.0153)
    {
        dGSErr = -dGSErr;
    }
    AeroLink_NavPkt.FCU_BaroPressure = 1013;
    //Clocksync();
}

/* ------------------------------------------- */
void Clocksync()
{
    while (1)
    {
        timer2 = ((int) (glfwGetTime() * 1000.0) % 1000) / 20;
        if (timer1 != timer2)
        {
            timer1 = timer2;
            break;	
        }
    }
}

/* ------------------------------------------- */
int main(int argc, char *argv[])
{
    Weather_DensityRatio = 1.0;

    BEGIN_Aero();
    BEGIN_AeroLink();
    BEGIN_Ai();
    BEGIN_Alt();
    BEGIN_Asi();
    BEGIN_Clocks();
    BEGIN_PFD_Compass();
    BEGIN_EICAS();
    BEGIN_FCS();
    BEGIN_Glib();
    BEGIN_Maths();
    BEGIN_Model();
    BEGIN_PFD();
    BEGIN_Systems();
    BEGIN_Vsi();

    Model_OnTheGround        = false;
    AeroLink_NavPkt.FCU_SPD  = 155;
    AeroLink_NavPkt.FCU_SPD_MACH = true;
    AeroLink_NavPkt.FCU_ALT   = 4500;
    Model_Pz                 = -1000.0;
    dh                       = -0.2;
    dUDot                    = 0.05;
    Model_U                  = 60.0;
    Model_Yaw                = 0.0;
    dy                       = 0.002;
    Model_Vd                 = 0.0;
    dv                       = 0.02;
    Model_Pitch              = 0.0;
    Model_Roll               = 0.0;
    dpitch                   = 0.01;
    droll                    = 0.023;
    Model_Vc                 = 60.0;
    dvc                      = 0.05;
    Model_Flaps              = 0.0;
    iFlapSetting             = 0;
    dFlaps                   = 0.001;
    for (EngineNumber = 0; EngineNumber <= 3; EngineNumber += 1)
    {
        AeroLink_EngPkt.Engines[EngineNumber].Epr      = 0.8;
        AeroLink_EngPkt.Engines[EngineNumber].Rpm      = 0.0;
        AeroLink_EngPkt.Engines[EngineNumber].Egt      = 0.0;
        AeroLink_EngPkt.Engines[EngineNumber].FuelFlow = 0.0;
    }
    dEpr                                 = 0.01;
    dRpm                                 = 1.0;
    dEgt                                 = 10.0;
    dFf                                  = 5.0;
    AeroLink_NavPkt.GroundLevel          = 0.0;
    AeroLink_NavPkt.NAV1.GlideSlopeError = 0.0;
    AeroLink_NavPkt.NAV1.ILSBeacon       = true;
    AeroLink_NavPkt.RMI_Dir1             = 0.0;
    AeroLink_NavPkt.RMI_Dir2             = 0.0;
    AeroLink_NavPkt.HSI_Hdg              = 34;
    HSI_Hdgx                             = 34.0;
    dHSI_Hdgx                            = -0.2;
    AeroLink_NavPkt.NAV1.LocaliserError  = 0.0;
    AeroLink_NavPkt.Track                = 0.0;
    AeroLink_NavPkt.HSI_Crs              = 0;
    HSI_Crsx                             = 0.0;
    dHSI_Crsx                            = 0.5;
    dHSI_Localiser                       = 0.00003;
    dTrack                               = 0.1;
    AeroLink_NavPkt.NAV1.SlantDistance   = 50000.0;
    dDmeDistance                         = 1.0;
    Model_R                              = 0.0;
    dSideForce                           = 1000.0;
    Model_SideForce                      = 200000.0;
    dR                                   = 0.002;
    dGSErr                               = 0.00002;
    dRMI1                                = 1;
    dRMI2                                = -1;
    iGear                                = 0;
    PFD_PFDInit(SystemUpdate);

    return 0;
}
