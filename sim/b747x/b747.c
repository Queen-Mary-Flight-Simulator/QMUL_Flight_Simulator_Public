/* master program for homesim
   DJA 21 Jan 2020 */
   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <windows.h>
#include <stdbool.h>

#include <GLFW/glfw3.h>

#include <SIM/iodefn.h>
#include <SIM/glib.h>
#include <SIM/glibx.h>
#include <SIM/maths.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>
#include <SIM/target.h>
#include <SIM/udplib.h>
#include <SIM/pnglib.h>
#include <SIM/dted.h>
#include <SIM/navlib.h>
#include <SIM/soundlib.h>

#include "aero.h"
#include "aerolink.h"
#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "compass.h"
#include "eicas.h"
#include "fcs.h"
#include "gear.h"
#include "iolib.h"
#include "model.h"
#include "pfd.h"
#include "systems.h"
#include "vsi.h"

#include "engines.h"
#include "englink.h"
#include "sounds.h"

#include "fcu.h"
#include "nav.h"
#include "navinfo.h"
#include "navlink.h"
#include "nfd.h"
#include "nfd-compass.h"
#include "panel.h"
#include "panellib.h"
#include "radio.h"

#include "approach.h"
#include "dataview.h"
#include "diagnostics.h"
#include "gui.h"
#include "ios.h"
#include "ioslink.h"
#include "map.h"
#include "menu.h"
#include "plot.h"
#include "scan.h"
#include "script.h"

bool         PFD_Held = false;
bool         pkt1found = false;
bool         reloading = false;
unsigned int framenumber = 0;

int t1 = 0;
int t2;

/* ----------------------------------------- */
void PFD_Update()
{
    //unsigned int p;
    char str[80];
	
    framenumber += 1;
	
    if (reloading)
    {
        reloading = false;
    }
    else
    {
        //do    // REMOVED FOR QMU VERSION 1
        //{
        //    p = UDPLib_GetPkt();
        //    if (p == 1)  /* PFD I/O RPi 1 */
        //    {
        //        pkt1found = true;
        //    }
        //} while (!pkt1found);
		while (1)  // 50 Hz frame sync added for QMU version 1
		{
		    t2 = (int) (glfwGetTime() * 50.0);
			if (t1 != t2)
			{
			    t1 = t2;
				break;
			}
		}
    }

    Diagnostics_SetTimer1(); /* ***************************** */
    if (AeroLink_Reloading)
    {
        //UDPLib_Close();    // REMOVED FOR QMU VERSION 1
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
    //UDPLib_SendPkt(&AeroLink_IGPkt, sizeof(AeroLink_IGPkt));  /* only goes to RPi1 and CGI */    // REMOVED FOR QMU VERSION 1

    if (AeroLink_Stopping)
	{
	    DTED_Exit();
        IOS_CloseWindow();
		END_SoundLib();
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
}

/* ----------------------------------------- */
int main(int argc, char *argv[])
{
    printf("PFD starting\n");
    
	BEGIN_Clocks();
	BEGIN_DTED();
    BEGIN_Glib();
    BEGIN_Glibx();
    BEGIN_Maths();
    BEGIN_NavLib();
	BEGIN_PngLib();
    BEGIN_SoundLib(&argc, argv);
    BEGIN_Target();
    //BEGIN_UDPLib();    // REMOVED FOR QMU VERSION 1
    BEGIN_Weather();
	
    BEGIN_Aero();
    BEGIN_AeroLink();
	BEGIN_Ai();
    BEGIN_Alt();
	BEGIN_Asi();
	BEGIN_PFD_Compass();
	BEGIN_EICAS();
	BEGIN_FCS();
	BEGIN_Gear();
	BEGIN_IOLib();
    BEGIN_Model();
    BEGIN_PFD();
    BEGIN_Systems();
	BEGIN_Vsi();

    BEGIN_Engines();
    BEGIN_EngLink();
	BEGIN_Sound();

 	BEGIN_Compass();
    BEGIN_FCU();
    BEGIN_Nav();
    BEGIN_NavInfo();
    BEGIN_NavLink();
    BEGIN_NFD();
    BEGIN_Panel();
    BEGIN_Panellib();
    BEGIN_Radio();
	
    BEGIN_Approach();
	BEGIN_Dataview();
	BEGIN_Diagnostics();
    BEGIN_Gui();
	BEGIN_IOS();
    BEGIN_IOSLink();
    BEGIN_Map();
    BEGIN_Menu();
    BEGIN_Plot();
	BEGIN_Scan();
    BEGIN_Script();

    if (argc > 1)
    {
        if (strcmp(argv[1], "reload") == 0)
        {
            reloading = true;
        }
    }

    //UDPLib_Connect(1, &AeroLink_IOPkt1, sizeof(AeroLink_IOPkt1));    // REMOVED FOR QMU VERSION 1

    //UDPLib_Open(3); /* PFD node = 3 */    // REMOVED FOR QMU VERSION 1
	
    Menu_ReadMenus();

    PFD_Init();
	Engines_Init();
    NFD_Init();
	IOS_Init();
	
    return 0;
}
