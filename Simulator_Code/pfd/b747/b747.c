/* B747 master
   DJA 18 Dec 2021 */
   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include <SIM/iodefn.h>
#include <SIM/glib.h>
#include <SIM/maths.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>
#include <SIM/target.h>
#include <SIM/udplib.h>
#include <SIM/pnglib.h>
#include <SIM/dted.h>

#include "aero.h"
#include "aerolink.h"
#include "model.h"
#include "gear.h"
#include "pfd.h"
#include "systems.h"
#include "iolib.h"
#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "compass.h"
#include "eicas.h"
#include "fcs.h"
#include "vsi.h"
#include "diagnostics.h"

static bool Held;
static void MainLoop();
static bool pkt1found = false;
static bool pkt2found = true;  /* dummy RPi */
static bool pkt4found = false;
static bool pkt5found = false;
static bool pkt6found = false;
static bool pkt10found = false;
static bool reloading = false;

static void MainLoop()
{
    unsigned int p;
    char         str[80];

    if (reloading)
    {
        reloading = false;
    }
    else
    {
        do
        {
            p = UDPLib_GetPkt();
            if (p == 1)  /* PFD I/O RPi 1 */
            {
                pkt1found = true;
            }
        } while (!pkt1found);
    }

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
    UDPLib_SendPkt(&AeroLink_AeroPkt, sizeof(AeroLink_AeroPkt));

    do
    {
        p = UDPLib_GetPkt();
        if (p == 4)   /* EICAS */
        {
            pkt4found = true;
        }
        if (p == 5)  /* NFD */
        {
            pkt5found = true;
        }
        if (p == 6)  /* IOS */
        {
            pkt6found = true;
        }
    } while (!(pkt4found && pkt5found && pkt6found));

    if (AeroLink_OctaveMode)
    {
        do
        {
            p = UDPLib_GetPkt();
            if (p == 10)
            {
                pkt10found = true;
            }
        } while (!(pkt10found));
    }

    pkt1found = false;
    pkt2found = true;  /* dummy RPi2 */
    pkt4found = false;
    pkt5found = false;
    pkt6found = false;
    pkt10found = false;

    Held = IOLib_GetHoldButton() || AeroLink_RemoteHold;
    Clocks_UpdateClocks(Held, false);
	Systems_UpdateGearSelector(!Model_OnTheGround);
	Systems_UpdateElevatorTrim();
    Systems_UpdateConfigWarning();
	
    if (AeroLink_ReplayMode)
    {
        AeroLink_Replay();
    }
    else
    {
        if (!Held)
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

    if (AeroLink_RespondToIos())
	{
	    DTED_Exit();
	    exit(0);
	}
}

int main(int argc, char *argv[])
{
    BEGIN_Aero();
    BEGIN_AeroLink();
    BEGIN_Clocks();
    BEGIN_Glib();
    BEGIN_Maths();
    BEGIN_Model();
    BEGIN_Gear();
    BEGIN_FCS();
	BEGIN_EICAS();
    BEGIN_PFD();
    BEGIN_Systems();
    BEGIN_Target();
    BEGIN_Weather();
    BEGIN_UDPLib();
    BEGIN_IOLib();
    BEGIN_Ai();
    BEGIN_Alt();
    BEGIN_Asi();
    BEGIN_PFD_Compass();
    BEGIN_Vsi();
    BEGIN_Diagnostics();
    BEGIN_PngLib();
    BEGIN_DTED();
	
    if (argc > 1)
    {
        if (strcmp(argv[1], "reload") == 0)
        {
            reloading = true;
        }
    }

    printf("PFD starting\n");
    UDPLib_Connect(1, &AeroLink_IOPkt1, sizeof(AeroLink_IOPkt1));
    UDPLib_Connect(2, &AeroLink_IOPkt2, sizeof(AeroLink_IOPkt2));
    UDPLib_Connect(4, &AeroLink_EngPkt, sizeof(AeroLink_EngPkt));
    UDPLib_Connect(5, &AeroLink_NavPkt, sizeof(AeroLink_NavPkt));
    UDPLib_Connect(6, &AeroLink_IosPkt, sizeof(AeroLink_IosPkt));
    UDPLib_Connect(10, &AeroLink_ProtoPkt, sizeof(AeroLink_ProtoPkt));

    UDPLib_Open(3); /* PFD node = 3 */
    PFD_PFDInit(MainLoop);

    return 0;
}
