#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>

#include <SIM/iodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/aerodefn.h>
#include <SIM/navdefn.h>
#include <SIM/udplib.h>
#include <SIM/iolib.h>

#include "iolink.h"

struct timeval frametime;
    
unsigned int Timer1;
unsigned int Timer2;

bool                  Held;
IODefn_SwitchPosition MasterSwitch;
IODefn_SwitchPosition KeySwitch;

unsigned int rxpkts;
unsigned int txpkts;
unsigned int frames;

void MainLoop();

/* --------------------------------------------- */
void MainLoop()
{
    unsigned int   p;

    bool           pkt2found;  /* I/O system 2 */
    bool           pkt3found;  /* PFD */
    bool           pkt4found;  /* ENG */
    bool           pkt5found;  /* NAV */
    bool           pkt6found;  /* IOS */
    bool           pkt10found; /* Matlab */

    rxpkts = 0;
    txpkts = 0;
    frames = 0;

    gettimeofday(&frametime, NULL);
    Timer1 = frametime.tv_usec / 20000L;

    while (1)
    {
        KeySwitch = (IOLib_DigitalDataD & 0x40) == 0;

        IOLink_FormPacket();
        UDPLib_SendPkt(&IOLink_IOPkt1, sizeof(IOLink_IOPkt1));
     	txpkts += 1;

        if (KeySwitch == IODefn_Off)
        {
            break;
        }

        pkt2found = true;  /* dummy RPi */
        pkt3found = false;
        pkt4found = false;
        pkt5found = false;
        pkt6found = false;
        pkt10found = false;

        do
        {
            p = UDPLib_GetPkt();
            //if (p == 2) 
            //{
            //    pkt2found = true;
            //    rxpkts += 1;
            //}
            if (p == 3) 
            {
                pkt3found = true;
                rxpkts += 1;
            }
            if (p == 4) 
            {
                pkt4found = true;
                rxpkts += 1;
            }
            if (p == 5) 
            {
                pkt5found = true;
                rxpkts += 1;
            }
            if (p == 6) 
            {
                pkt6found = true;
                rxpkts += 1;
            }
        } while (!(pkt2found && pkt3found && pkt4found && pkt5found && pkt6found));

        if (IOLink_OctaveMode)
        {
            do
            {
                p = UDPLib_GetPkt();
                if (p == 10)
                {
                    pkt10found = true;
                    rxpkts += 1;
                }
            } while (!pkt10found);
        }

        Held = ((IOLib_DigitalDataB & 0x02) == 0);

        if ((IOLib_DigitalDataB & 0x02) == 0 || IOLink_Restored)  /* restore button pressed */
		{
		    if (!IOLib_Sidestick)
            {
                IOLib_RepositionCLS(IOLink_LastRestore, IOLink_AeroPkt.ElevatorTrim, IOLink_AeroPkt.AileronTrim, IOLink_AeroPkt.RudderTrim);
            }
			IOLink_Restored = false;
        }
		else
        {
            IOLib_UpdateCLS(IOLink_AeroPkt.Vc, IOLink_AeroPkt.ElevatorTrim, 
                            IOLink_AeroPkt.AileronTrim, IOLink_AeroPkt.RudderTrim);
        }

        IOLib_UpdateIO(IOLink_AeroPkt.DigitalDataOutA, IOLink_AeroPkt.DigitalDataOutB);

        if (IOLink_RespondToIos(Held))
		{
		    break;
		}

        while (1)
        {
            gettimeofday(&frametime, NULL);
            Timer2 = frametime.tv_usec / 20000L;  /* frame ticks */
            if (Timer1 != Timer2)
            {
                Timer1 = Timer2;
                break;
            }
        }

        frames += 1;
    }
}

/* --------------------------------------------- */
int main(int argc, char *argv[])
{
    struct timeval tv1;
    struct timeval tv2;
    float          etime;
    time_t         simtime;
    unsigned int   h, m, s;

    while (1)
    {
        BEGIN_IOLink();
        BEGIN_IOLib();
        BEGIN_UDPLib();

        IOLib_Start();
        printf("I/O System starting\n");
        
        if ((IOLib_DigitalDataD & 0x40) != 0)
        {
            printf("Key Switch OFF\n");
            exit(1);
        }

        UDPLib_Connect(2, &IOLink_IOPkt2, sizeof(IOLink_IOPkt2));
        UDPLib_Connect(3, &IOLink_AeroPkt, sizeof(IOLink_AeroPkt));
        UDPLib_Connect(4, &IOLink_EngPkt, sizeof(IOLink_EngPkt));
        UDPLib_Connect(5, &IOLink_NavPkt, sizeof(IOLink_NavPkt));
        UDPLib_Connect(6, &IOLink_IosPkt, sizeof(IOLink_IosPkt));
        UDPLib_Connect(10, &IOLink_ProtoPkt, sizeof(IOLink_ProtoPkt));

        UDPLib_Open(1);  /* IO node = 1 */

        IOLib_ResetCLS();  /* remove power - just in case stick is active */

        gettimeofday(&tv1, NULL);
        MainLoop();
        gettimeofday(&tv2, NULL);

        IOLib_ResetCLS();  /* remove power from the stick */

        UDPLib_Close();
        END_IOLib();

        simtime = time(NULL);
        printf("%s", asctime(localtime(&simtime)));
        printf("Pkts received: %u\n", rxpkts);
        printf("Pkts sent: %u\n", txpkts);
        printf("frames: %u\n", frames);
        etime = (float) (tv2.tv_sec - tv1.tv_sec) + (float) ((int) (tv2.tv_usec) - (int) (tv1.tv_usec)) / 1000000.0;
        h = (unsigned int) etime / 3600;
        m = (((unsigned int) (etime)) % 3600) / 60;
        s = (unsigned int) etime % 60;
        printf("elapsed time: %u:%u:%u\n", h, m, s);
        printf("frame rate: %f fps\n", (float) frames / etime);
    }
	
    return 0;
}
