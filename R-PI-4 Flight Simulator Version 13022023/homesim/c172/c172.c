/* master program for homesim (C172 version)
   DJA 14 November 2022 */
   
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
#include <SIM/fileio.h>
#include <SIM/target.h>
#include <SIM/udplib.h>
#include <SIM/pnglib.h>
#include <SIM/dted.h>
#include <SIM/navlib.h>
#include <SIM/soundlib.h>
#include <SIM/jslib.h>

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
#include "vsi.h"

#include "rpm.h"
#include "egt.h"
#include "clock.h"
#include "turnslip.h"
#include "enginegauges_left.h"
#include "enginegauges_right.h"
#include "vor.h"
#include "adf.h"

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

/* ----------------------------------------- */
void PFD_Update()
{
    unsigned int i;
    char         str[80];

    framenumber += 1;
	
    if (reloading)
    {
        reloading = false;
    }

    jsLib_UpdateIO(0, 0);

    for (i=0; i<=31; i+=1)
    {
        AeroLink_IOPkt1.AnalogueData[i] = jsLib_AnalogueData[i];
    }
    AeroLink_IOPkt1.DigitalDataA = jsLib_DigitalDataA;
    AeroLink_IOPkt1.DigitalDataB = jsLib_DigitalDataB;
    AeroLink_IOPkt1.DigitalDataC = jsLib_DigitalDataC;
    AeroLink_IOPkt1.DigitalDataD = jsLib_DigitalDataD;
	//AeroLink_IOPkt1.Temperature  = IOLib_Temperature;

    if (AeroLink_Reloading)
    {
        UDPLib_Close();
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
    UDPLib_SendPkt(&AeroLink_IGPkt, sizeof(AeroLink_IGPkt));  /* only goes to RPi1 and CGI */

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
    Radio_UpdateRadio();

    if (AeroLink_ReplayMode)
    {
        AeroLink_Replay();
    }
    else
    {
	    //FCS_CheckFCU();  /* update FCU inputs even if held */

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
}

/* ----------------------------------------- */
int main(int argc, char *argv[])
{
    printf("PFD starting\n");
    
	DEMO_MODE = false;
	
    BEGIN_Aero();
    BEGIN_AeroLink();
	BEGIN_Ai();
    BEGIN_Alt();
	BEGIN_Asi();
	BEGIN_Compass();

	BEGIN_FCS();
	BEGIN_IOLib();
    BEGIN_Model();
    BEGIN_PFD();
    BEGIN_Systems();

    BEGIN_Engines();
    BEGIN_EngLink();
	BEGIN_Sound();

    BEGIN_FCU();
    BEGIN_Nav();
    BEGIN_NavLink();
    BEGIN_NFD();
    BEGIN_Panel();
    BEGIN_Radio();

    BEGIN_Approach();
	BEGIN_Dataview();
	BEGIN_Diagnostics();
    BEGIN_Gui();
	BEGIN_IOS();
    BEGIN_IOSLink();
	BEGIN_Scan();
    BEGIN_Script();

	BEGIN_TurnSlip();
	BEGIN_Rpm();
	BEGIN_Egt();
	BEGIN_Clock();
	BEGIN_EngineGauges_left();
	BEGIN_EngineGauges_right();
	BEGIN_Knobs();
	BEGIN_Vsi();
    BEGIN_Vor();
	BEGIN_Adf();
	BEGIN_MagneticCompass();
	
	BEGIN_Clocks();
	BEGIN_DTED();
    BEGIN_Glib();
    BEGIN_Maths();
    BEGIN_NavLib();
    BEGIN_Map();  /* map initialisation comes after navalib */
    BEGIN_Plot();
	BEGIN_PngLib();
	
    BEGIN_SoundLib(&argc, argv);
    BEGIN_Target();
    BEGIN_UDPLib();
    BEGIN_Weather();
	
	BEGIN_FileIO();
    BEGIN_jsLib();
	
    if (argc > 1)
    {
        if (strcmp(argv[1], "reload") == 0)
        {
            reloading = true;
        }
    }

    UDPLib_Open(3); /* PFD node = 3 */
	
	//jsLib_Wait();  /* press red button to start */

    PFD_Init();
    NFD_Init();
	IOS_Init();
	
    return 0;
}
