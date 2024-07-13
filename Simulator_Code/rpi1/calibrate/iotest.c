#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#include <SIM/udplib.h>
#include <SIM/iodefn.h>
#include <SIM/iolib.h>

#include "iolink.h"
  
void MainLoop();
void ShutDownIOSystem();

/* --------------------------------------------- */
void MainLoop()
{
    struct timeval tv;
    unsigned int   Timer1;
    unsigned int   Timer2;

    gettimeofday(&tv, NULL);
    Timer1 = tv.tv_usec / 20000L;  /* frame ticks */

    IOLib_UpdateIO(0xff, 0xff);

    while (1)
    {
        IOLib_UpdateIO(IOLink_AeroPkt.DigitalDataOutA, IOLink_AeroPkt.DigitalDataOutB);
		
        IOLink_FormPacket();
        UDPLib_SendPkt(&IOLink_IOPkt1, sizeof(IOLink_IOPkt1));

        while (1)
        {
            gettimeofday(&tv, NULL);
            Timer2 = tv.tv_usec / 20000L;  /* frame ticks */
            if (Timer1 != Timer2)
            {
                Timer1 = Timer2;
				break;
            }
        }
    }

    ShutDownIOSystem();		
}

/* --------------------------------------------- */
int main(int argc, char *argv[])
{
    BEGIN_IOLink();
    BEGIN_IOLib(); 
    BEGIN_UDPLib();

    printf("I/O test starting\n");

    UDPLib_Open(1);  /* IO node = 1 */
  
    MainLoop();

    return 0;
}

/* --------------------------------------------- */
void ShutDownIOSystem()
{
    UDPLib_Close();
    END_IOLib();
}
