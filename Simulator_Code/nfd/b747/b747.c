/* +------------------------------+---------------------------------+
   | Module      : b747.c         | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-01      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 Main Module                       |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include <SIM/glib.h>
#include <SIM/clocks.h>
#include <SIM/iodefn.h>
#include <SIM/maths.h>
#include <SIM/navlib.h>
#include <SIM/udplib.h>
#include <SIM/pnglib.h>

#include "nfd.h"
#include "navlink.h"
#include "fcu.h"
#include "radio.h"
#include "nav.h"
#include "navinfo.h"
#include "systems.h"
#include "iolib.h"
#include "nfd-compass.h"
#include "diagnostics.h"
#include "panel.h"
#include "panellib.h"

static bool Held;
static bool pkt1found = false;
static bool pkt2found = true;  /* dummy RPi2 */
static bool pkt3found = false;
static bool pkt4found = false;
static bool pkt6found = false;
static bool pkt10found = false;
static bool reloading = false;

static void MainLoop();

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
            if (p == 1)
            {
                pkt1found = true;
            }
            /* if (p == 2)             */
            /* {                       */
            /*     pkt2found = true;   */
            /* }                       */
            if (p == 3)
            {
                pkt3found = true;
            }
            if (p == 4)
            {
                pkt4found = true;
            }
        } while (!(pkt1found && pkt2found&& pkt3found && pkt4found));
    }

    if (NavLink_Reloading)
    {
        UDPLib_Close();
		FCU_StopFCU();
        strcpy(str, "../");
        strcat(str, NavLink_ReloadFilename);
        chdir(str);
        strcat(str, "/");
        strcat(NavLink_ReloadFilename, ".exe");
        strcat(str, NavLink_ReloadFilename);
        execl(str, NavLink_ReloadFilename, "reload", NULL);
        printf("execl failed %s %s\n", str, NavLink_ReloadFilename);    /* should never get here */
        exit(0);
    }

    NavLink_FormPacket();
    UDPLib_SendPkt(&NavLink_NavPkt, sizeof(NavLink_NavPkt));

    do
    {
        p = UDPLib_GetPkt();
        if (p == 6)
        {
            pkt6found = true;
        }
    } while (!pkt6found);

/* MATLAB / OCTAVE */

    if (NavLink_OctaveMode)
    {
        do
        {
            p = UDPLib_GetPkt();
            if (p == 10)
            {
                pkt10found = true;
            }
        } while (!pkt10found);
    }

    pkt1found = false;
    pkt2found = true;  /* dummy RPi2 */
    pkt3found = false;
    pkt4found = false;
    pkt6found = false;
    pkt10found = false;

    Systems_KeySwitch = IOLib_GetKeySwitch();
    if (Systems_KeySwitch == IODefn_Off)
    {
	    printf("Key SWitch OFF\n");
        UDPLib_Close();
        exit(1);
    }

    Held = IOLib_GetHoldButton();

    Clocks_UpdateClocks(Held, false);
    Panel_CheckPanel();
    Nav_UpdateNav();
    Radio_RMP();
    Nav_CheckFlightPlan();

    NavLink_RespondToIos();
}

int main(int argc, char *argv[])
{
    BEGIN_Clocks();
    BEGIN_FCU();
    BEGIN_Glib();
    BEGIN_Maths();
    BEGIN_Nav();
    BEGIN_NavLib();
    BEGIN_NavLink();
    BEGIN_NavInfo();
    BEGIN_NFD();
 	BEGIN_Compass();
    BEGIN_Panel();
    BEGIN_Radio();
    BEGIN_Systems();
    BEGIN_UDPLib();
    BEGIN_IOLib();
	BEGIN_PanelLib();
    BEGIN_Diagnostics();
    BEGIN_PngLib();

    if (argc > 1)
    {
        if (strcmp(argv[1], "reload") == 0)
        {
            reloading = true;
        }
    }

    printf("NFD starting\n");

    UDPLib_Connect(1, &NavLink_IOPkt1, sizeof(NavLink_IOPkt1));
    UDPLib_Connect(2, &NavLink_IOPkt2, sizeof(NavLink_IOPkt2));
    UDPLib_Connect(3, &NavLink_AeroPkt, sizeof(NavLink_AeroPkt));
    UDPLib_Connect(4, &NavLink_EngPkt, sizeof(NavLink_EngPkt));
    UDPLib_Connect(6, &NavLink_IosPkt, sizeof(NavLink_IosPkt));
    UDPLib_Connect(10, &NavLink_ProtoPkt, sizeof(NavLink_ProtoPkt));

    UDPLib_Open(5); /* NFD node = 5 */
    NFD_NFDInit(MainLoop);

    return 0;
}
