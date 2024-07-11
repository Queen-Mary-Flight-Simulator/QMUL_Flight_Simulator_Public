/* master program for homesim
   DJA 23 April 2020 */
   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include <GLFW/glfw3.h>

#include <SIM/iodefn.h>
#include <SIM/glib.h>
#include <SIM/maths.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>
#include <SIM/target.h>
//#include <SIM/udplib.h>
#include <SIM/pnglib.h>
#include <SIM/dted.h>
#include <SIM/navlib.h>
#include <SIM/soundlib.h>
//#include <SIM/jslib.h>

#include "aero.h"
#include "aerolink.h"
#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "compass.h"
#include "fcs.h"
#include "iolib.h"
#include "model.h"
#include "pfd.h"
#include "systems.h"
#include "nfd.h"

#include "rpm.h"
#include "egt.h"
#include "clock.h"
#include "enginegauges_left.h"
#include "enginegauges_right.h"

#include "engines.h"
#include "englink.h"
#include "sounds.h"

#include "fcu.h"
#include "nav.h"
#include "navlink.h"
#include "nfd.h"
#include "panel.h"
#include "radio.h"
#include "magcompass.h"
#include "knobs.h"

#include "approach.h"
#include "dataview.h"
#include "diagnostics.h"
#include "gui.h"
#include "ios.h"
#include "ioslink.h"
#include "map.h"
#include "plot.h"
#include "scan.h"
#include "script.h"

static bool         pkt1found = false;
static bool         reloading = false;
static unsigned int framenumber = 0;

static int   timer1;
static int   timer2;

void Clocksync();

/* ----------------------------------------- */
void PFD_Update()
{
    char         str[80];

    framenumber += 1;
	
    //if (reloading)
    //{
    //    reloading = false;
    //}
    //else
    //{
    //    do
    //    {
    //        p = UDPLib_GetPkt();
    //        if (p == 1)  /* PFD I/O RPi 1 */
    //        {
    //            pkt1found = true;
    //        }
    //    } while (!pkt1found);
    //}

    if (AeroLink_Reloading)
    {
        //UDPLib_Close();
        strcpy(str, "../");
        strcat(str, AeroLink_ReloadFilename);
        chdir(str);
        strcat(str, "/");
        strcat(AeroLink_ReloadFilename, ".exe");
        strcat(str, AeroLink_ReloadFilename);
        execl(str, AeroLink_ReloadFilename, "reload", NULL);
        printf("execl error: %s %s\n", str, AeroLink_ReloadFilename); /* should never happen */
        exit(0);
    }

    AeroLink_FormPacket();
    //UDPLib_SendPkt(&AeroLink_IGPkt, sizeof(AeroLink_IGPkt));  /* only goes to RPi1 and CGI */

    if (AeroLink_Stopping)
	{
	    DTED_Exit();
        IOS_CloseWindow();
		Close_SoundLib();
	    exit(0);
	}

    memcpy(&EngLink_IOPkt1, &AeroLink_IOPkt1,   sizeof(IODefn_IODataPkt));
    memcpy(&NavLink_IOPkt1, &AeroLink_IOPkt1,   sizeof(IODefn_IODataPkt));
    memcpy(&IosLink_IOPkt1, &AeroLink_IOPkt1,   sizeof(IODefn_IODataPkt));

    memcpy(&EngLink_AeroPkt, &AeroLink_AeroPkt, sizeof(AeroDefn_AeroDataPkt));
    memcpy(&NavLink_AeroPkt, &AeroLink_AeroPkt, sizeof(AeroDefn_AeroDataPkt));
    memcpy(&IosLink_AeroPkt, &AeroLink_AeroPkt, sizeof(AeroDefn_AeroDataPkt));

    pkt1found = false;

    PFD_Held = IOLib_GetHoldButton() || AeroLink_RemoteHold;
    Clocks_UpdateClocks(PFD_Held, false);
	Systems_UpdateGearSelector(!Model_OnTheGround);
	Systems_UpdateElevatorTrim();
    Systems_UpdateConfigWarning();

    if (AeroLink_ReplayMode)
    {
        AeroLink_Replay();
    }
    else
    {
        if (!PFD_Held)
        {
			Weather_WeatherModel(true, Model_Pz, Model_U);
            Model_FlightModel();
            Target_TargetDynamics(AeroLink_NavPkt.GroundLevel, Model_Pz, Model_Latitude, Model_Longitude);
            IOLib_UpdateGearLamps(Model_Gear);
        }
		else
		{
		    IOLib_StickShaker(false);  /* turn off stick shaker if held */
		}
    }
    if (IOLib_GetRestoreButton())
    {
        AeroLink_RestoreLast();
    }

    AeroLink_RespondToIos();
	
	Clocksync();
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

/* ----------------------------------------- */
int main(int argc, char *argv[])
{
    unsigned int i;
	
	DEMO_MODE = true;
	
    printf("PFD starting\n");
    
	BEGIN_Clocks();
	BEGIN_DTED();
    BEGIN_Glib();
    BEGIN_Maths();
    BEGIN_NavLib();
	BEGIN_PngLib();
    BEGIN_SoundLib(&argc, argv);
    BEGIN_Target();
    //BEGIN_UDPLib();
    BEGIN_Weather();
	
    BEGIN_Aero();
    BEGIN_AeroLink();
	BEGIN_Ai();
	BEGIN_Asi();
	BEGIN_Compass();
	BEGIN_Rpm();
	BEGIN_Egt();
	BEGIN_Clock();
	BEGIN_EngineGauges_left();
	BEGIN_EngineGauges_right();
	
	BEGIN_FCS();
	BEGIN_IOLib();
    BEGIN_Model();
    BEGIN_PFD();
    BEGIN_Systems();

    BEGIN_Engines();
    BEGIN_EngLink();
	BEGIN_Sound();

 	BEGIN_Compass();
	BEGIN_MagneticCompass();
    BEGIN_FCU();
    BEGIN_Nav();
    BEGIN_NavLink();
    BEGIN_NFD();
    BEGIN_Panel();
    BEGIN_Radio();
	
	BEGIN_Alt();
	BEGIN_Knobs();
	
    BEGIN_Approach();
	BEGIN_Dataview();
	BEGIN_Diagnostics();
    BEGIN_Gui();
	BEGIN_IOS();
    BEGIN_IOSLink();
    BEGIN_Map();
    BEGIN_Plot();
	BEGIN_Scan();
    BEGIN_Script();
//    BEGIN_jsLib();
	
    if (argc > 1)
    {
        if (strcmp(argv[1], "reload") == 0)
        {
            reloading = true;
        }
    }

    //UDPLib_Connect(1, &AeroLink_IOPkt1, sizeof(AeroLink_IOPkt1));

    //UDPLib_Open(3); /* PFD node = 3 */
	
    PFD_Init();
    for (i = 0; i <= 3; i += 1)
    {
        Engines_Engines[i].Epr      = 1.02;
        Engines_Engines[i].Rpm      = 50.0;
        Engines_EngineState[i]      = IODefn_On;
    }
    
    NFD_Init();
	IOS_Init();
	
    return 0;
}
