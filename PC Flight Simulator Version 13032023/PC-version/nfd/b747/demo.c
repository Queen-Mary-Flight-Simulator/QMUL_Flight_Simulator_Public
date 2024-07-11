#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#include <SIM/clocks.h>
#include <SIM/navdefn.h>
#include <SIM/maths.h>
#include <SIM/navlib.h>
#include <SIM/glib.h>

#include <GLFW/glfw3.h>

#include "nfd.h"
#include "nav.h"
#include "navlink.h"
#include "radio.h"
#include "fcu.h"
#include "panel.h"
#include "systems.h"
#include "navinfo.h"
#include "nfd-compass.h"
#include "panellib.h"

static float             GearTable[4] = { 0.0, 0.5, 1.0, 0.5 };

static int               NumberOfSteps;
//static int               t1 = 0;
//static int               t2;
static float             dh;
static float             dy;
static float             dv;
static float             dpitch;
static float             droll;
static float             dvc;
static float             dFlaps;
static float             dRmi1, dRmi2;
static float             dGSErr;
static float             HSI_Hdgx;
static float             dHSI_Hdgx;
static float             dHSI_Localiser;
static float             dR;
static float             dDmeDistance;
static float             dTrack;
static float             ddTrack;
static unsigned int      iGear;
static unsigned int      RangeTablePtr;
static float             yaw;
static float             HSI_Crsx;

static int               timer1;
static int               timer2;

static void SystemUpdate();
void Clocksync();

/* --------------------------------------- */
static void SystemUpdate()
{
    NumberOfSteps     = NumberOfSteps + 1;
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
    Nav_Track = (float) NavLink_AeroPkt.Yaw + dTrack;
    dTrack    = dTrack + ddTrack;
    if (dTrack < -0.2 || dTrack > 0.2)
    {
        ddTrack = -ddTrack;
    }
    NavLink_AeroPkt.Pz = (float) NavLink_AeroPkt.Pz + dh;
    if (NavLink_AeroPkt.Pz < -12000.0 || NavLink_AeroPkt.Pz > 0.0)
    {
        dh = -dh;
    }

/* ---------------------------------------- */
    // Radio test case
    NavLink_AeroPkt.Pz = -1100.0;
    NavLink_AeroPkt.Latitude = Maths_Rads(51.1);
	NavLink_AeroPkt.Longitude = Maths_Rads(-0.3);
/* ---------------------------------------- */

    NavLink_AeroPkt.Vd = (float) NavLink_AeroPkt.Vd + dv;
    if (NavLink_AeroPkt.Vd < -30.0 || NavLink_AeroPkt.Vd > 30.0)
    {
        dv = -dv;
    }
    yaw = (float) NavLink_AeroPkt.Yaw + dy;
    yaw = Maths_Normalise(yaw);
    NavLink_AeroPkt.Yaw   = yaw;
    NavLink_AeroPkt.Pitch = (float) NavLink_AeroPkt.Pitch + dpitch;
    if (NavLink_AeroPkt.Pitch < -0.3 || NavLink_AeroPkt.Pitch > 0.3)
    {
        dpitch = -dpitch;
    }
    NavLink_AeroPkt.Roll = (float) NavLink_AeroPkt.Roll + droll;
    if (NavLink_AeroPkt.Roll < -3.0 || NavLink_AeroPkt.Roll > 3.0)
    {
        droll = -droll;
    }
    NavLink_AeroPkt.FlapPosition = (float) NavLink_AeroPkt.FlapPosition + dFlaps;
    if (NavLink_AeroPkt.FlapPosition < 0.0 || NavLink_AeroPkt.FlapPosition > 1.0)
    {
        dFlaps = -dFlaps;
    }
    Nav_OuterMarker  = Clocks_ClockSecs < 10;
    Nav_MiddleMarker = Clocks_ClockSecs > 20 && Clocks_ClockSecs < 30;
    Nav_MarkerTest   = Clocks_ClockSecs > 40 && Clocks_ClockSecs < 50;
    if (Clocks_ClockSecs % 15 == 0 && Clocks_ClockTicks == 1)
    {
        iGear                        = (iGear + 1) % 4;
        NavLink_AeroPkt.GearPosition = GearTable[iGear];
    }
    NavLink_AeroPkt.Vc = (float) NavLink_AeroPkt.Vc + dvc;
    if (NavLink_AeroPkt.Vc < 0.0 || NavLink_AeroPkt.Vc > 225.0)
    {
        if (NavLink_AeroPkt.Vc < 0.0)
        {
            NavLink_AeroPkt.Vc = 0.0;
        }
        dvc = -dvc;
    }
    NavLink_AeroPkt.MachNumber = (float) NavLink_AeroPkt.Vc / 340.0;
/*
    Nav_Rmi_Dir1               = Nav_Rmi_Dir1 + dRmi1;
    Nav_Rmi_Dir2               = Nav_Rmi_Dir2 + dRmi2;
    while (Nav_Rmi_Dir1 < -Maths_PI)
    {
        Nav_Rmi_Dir1 = Nav_Rmi_Dir1 + Maths_TWOPI;
    }
    while (Nav_Rmi_Dir1 > Maths_PI)
    {
        Nav_Rmi_Dir1 = Nav_Rmi_Dir1 - Maths_TWOPI;
    }
    while (Nav_Rmi_Dir2 < -Maths_PI)
    {
        Nav_Rmi_Dir2 = Nav_Rmi_Dir2 + Maths_TWOPI;
    }
    while (Nav_Rmi_Dir2 > Maths_PI)
    {
        Nav_Rmi_Dir2 = Nav_Rmi_Dir2 - Maths_TWOPI;
    }
    HSI_Crsx = HSI_Crsx + 0.01;
    if (HSI_Crsx > 360.0)
    {
        HSI_Crsx = HSI_Crsx - 360.0;
    }
    Nav_HSI_Crs       = intround(HSI_Crsx);
    Nav_HSI_Crs       = Radio_Radios[0].CrsKnob;
*/
    NavLink_AeroPkt.R = (float) NavLink_AeroPkt.R + dR;
    if (NavLink_AeroPkt.R < -0.1 || NavLink_AeroPkt.R > 0.1)
    {
        dR = -dR;
    }
    if (Clocks_ClockSecs % 5 == 0 && Clocks_ClockTicks == 1)
    {
        NavLink_AeroPkt.SideForce = -NavLink_AeroPkt.SideForce;
    }
    Nav_DmeDistance = Nav_DmeDistance + dDmeDistance;
    if (Nav_DmeDistance < -2000.0)
    {
        Nav_DmeDistance = 20000.0;
    }
/*
    HSI_Hdgx = HSI_Hdgx + dHSI_Hdgx;
    if (HSI_Hdgx > 360.0)
    {
        HSI_Hdgx = 1.0;
    }
    if (HSI_Hdgx < 1.0)
    {
        HSI_Hdgx = 360.0;
    }
    Nav_HSI_Hdg       = intround(HSI_Hdgx);
    Nav_HSI_Localiser = Nav_HSI_Localiser + dHSI_Localiser;
    if (Nav_HSI_Localiser < -Maths_PI || Nav_HSI_Localiser > Maths_PI)
    {
        dHSI_Localiser = -dHSI_Localiser;
    }
    Nav_HSI_GlideSlope = Nav_HSI_GlideSlope + dGSErr;
    if (Nav_HSI_GlideSlope < -0.0155 || Nav_HSI_GlideSlope > 0.0155)
    {
        dGSErr = -dGSErr;
    }
*/	

//    while (1)
//    {
//        t2 = (int) (glfwGetTime() * 50.0);
//		if (t1 != t2)
//		{
//		    t1 = t2;
//			break;
//		}
//    }
    
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

/* --------------------------------------- */
int main(int argc, char *argv[])
{
    BEGIN_Clocks();
    BEGIN_Compass();
    BEGIN_FCU();
    BEGIN_Glib();
    BEGIN_Maths();
    BEGIN_Nav();
    BEGIN_NavLib();
    BEGIN_NavLink();
    BEGIN_NavInfo();
    BEGIN_NFD();
    BEGIN_Panel();
    BEGIN_Radio();
    BEGIN_Systems();
    BEGIN_PanelLib();
	
    NumberOfSteps                = 0;
    FCU_NavSwitch1               = NavDefn_NavOFF;
    FCU_NavSwitch2               = NavDefn_NavOFF;
    FCU_ModeSelector             = NavDefn_ModeILS;
    FCU_DataMode                 = NavDefn_DataOFF;
    FCU_HDG                      = 360;
    NavLink_AeroPkt.Pz           = -2000.0;
    NavLink_AeroPkt.WindSpeed    = 10.0;
    NavLink_AeroPkt.WindDir      = 0.5;
    dh                           = -0.5;
    NavLink_AeroPkt.Yaw          = 0.0;
    dy                           = -0.002;
    NavLink_AeroPkt.Vd           = 0.0;
    dv                           = 0.005;
    NavLink_AeroPkt.Pitch        = 0.0;
    NavLink_AeroPkt.Roll         = 0.0;
    dpitch                       = 0.01;
    droll                        = 0.023;
    NavLink_AeroPkt.Vc           = 60.0;
    dvc                          = 0.05;
    NavLink_AeroPkt.FlapPosition = 0.0;
    dFlaps                       = 0.002;
    Nav_HSI_GlideSlope           = 0.0;
    Nav_HSI_ILSMode              = false;
    Nav_Rmi_Dir1                 = 0.0;
    Nav_Rmi_Dir2                 = 0.0;
    Nav_HSI_Hdg                  = 34;
    HSI_Hdgx                     = 34.0;
    Nav_HSI_Crs                  = 0;
    HSI_Crsx                     = 0.0;
    dHSI_Hdgx                    = -0.1;
    Nav_HSI_Localiser            = 0.0;
    dHSI_Localiser               = 0.0005;
    Nav_DmeDistance              = 50000.0;
    dDmeDistance                 = -1.0;
    NavLink_AeroPkt.R            = 0.0;
    NavLink_AeroPkt.SideForce    = 200000.0;
    dR                           = 0.002;
    dGSErr                       = 0.00001;
    dRmi1                        = 0.001;
    dRmi2                        = -0.001;
    dTrack                       = 0.2;
    ddTrack                      = -0.0001;
    RangeTablePtr                = 4;
    iGear                        = 0;

    NFD_NFDInit(SystemUpdate);

    return 0;
}
