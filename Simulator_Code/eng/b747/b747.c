/* +------------------------------+---------------------------------+
   | Module      : b747.c         | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-06      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/lfs/eicas/b747           |
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
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include <SIM/iodefn.h>
#include <SIM/maths.h>
#include <SIM/clocks.h>
#include <SIM/udplib.h>
#include <SIM/soundlib.h>

#include "engines.h"
#include "englink.h"
#include "systems.h"
#include "iolib.h"
#include "sounds.h"

static bool Held;
static void MainLoop();
static bool pkt1found = false;
static bool pkt2found = true; /* dummy RPi2 */
static bool pkt3found = false;
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
            /* if (p == 2)  EICAS I/O RPi 2 */
            /* {                            */
            /*     pkt2found = true;        */
            /* }                            */
            if (p == 3)  /* PFD */
            {
                pkt3found = true;
            }
        } while (!(pkt1found && pkt2found && pkt3found));
    }

    if (EngLink_Reloading)
    {
        UDPLib_Close();
        strcpy(str, "../");
        strcat(str, EngLink_ReloadFilename);
        chdir(str);
        strcat(str, "/");
        strcat(EngLink_ReloadFilename, ".exe");
        strcat(str, EngLink_ReloadFilename);
        execl(str, EngLink_ReloadFilename, "reload", NULL);
        printf("execl error: %s %s\n", str, EngLink_ReloadFilename); /* should never happen */
        exit(0);
    }

    EngLink_FormPacket();
    UDPLib_SendPkt(&EngLink_EngPkt, sizeof(EngLink_EngPkt));

    do
    {
        p = UDPLib_GetPkt();
        if (p == 5)  /* NFD */
        {
            pkt5found = true;
        }
        if (p == 6)  /* IOS */
        {
            pkt6found = true;
        }
    } while (!(pkt5found && pkt6found));

    if (EngLink_OctaveMode)
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
    pkt3found = false;
    pkt5found = false;
    pkt6found = false;
    pkt10found = false;

    Held = IOLib_GetHoldButton() || EngLink_RemoteHold;
    Clocks_UpdateClocks(Held, false);
    Systems_UpdateSystems();
    if (EngLink_ReplayMode)
    {
        EngLink_Replay();
    }
    else
    {
        Engines_EngineModel(Held);
		SoundSystem(Held);
    }

    if (EngLink_RespondToIos())
	{
	    exit(0);  /* should shut down everything  */
	}
}

int main(int argc, char *argv[])
{
    BEGIN_EngLink();
	BEGIN_IOLib();
    BEGIN_Clocks();
    BEGIN_Engines();
    BEGIN_Maths();
    BEGIN_Systems();
    BEGIN_UDPLib();
    BEGIN_SoundLib(&argc, argv);
	BEGIN_Sound();
	
    if (argc > 1)
    {
        if (strcmp(argv[1], "reload") == 0)
        {
            reloading = true;
        }
    }

    printf("EICAS starting\n");
    UDPLib_Connect(1, &EngLink_IOPkt1, sizeof(EngLink_IOPkt1));
    UDPLib_Connect(2, &EngLink_IOPkt2, sizeof(EngLink_IOPkt2));
    UDPLib_Connect(3, &EngLink_AeroPkt, sizeof(EngLink_AeroPkt));
    UDPLib_Connect(5, &EngLink_NavPkt, sizeof(EngLink_NavPkt));
    UDPLib_Connect(6, &EngLink_IosPkt, sizeof(EngLink_IosPkt));
    UDPLib_Connect(10, &EngLink_ProtoPkt, sizeof(EngLink_ProtoPkt));

    UDPLib_Open(4); /* EICAS node = 4 */
    
	while (1)
	{
    	MainLoop();
    }
	
    return 0;
}
