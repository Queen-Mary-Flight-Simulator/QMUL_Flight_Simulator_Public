#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef WIN32
  #pragma pack(push,2)
#endif

#include <SIM/iodefn.h>
#include <SIM/igdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/udplib.h>

/* interface packet from the desktop simulator */
IODefn_IODataPkt IOPkt1;
IGDefn_IGDataPkt IGPkt;

unsigned int  Frames = 0;

int           UDP_Start(void);
int           UDP_Recv(void);

/* ------------------------------------------------------------------ */
int main()
{
    int i;
    
    printf("calling UDP_Start\n");
    UDP_Start();
    printf("UDP_Start OK\n");

    for (i=1; i<= 1000; i+=1)
	{
        //printf("calling UDP_Recv\n");
        UDP_Recv();
        //printf("UDP_Recv OK\n");
        //printf(".");
		//fflush(stdout);
	}
    return 1;
}

/* ------------------------------------------------------------------ */
int UDP_Start(void)
{
/*  open sockets from RPi 192.168.1.1 and IG 192.168.1.2 for IG 192.168.1.8 */    
    BEGIN_UDPLib();
    UDPLib_Connect(1, &IOPkt1, sizeof(IOPkt1));
    UDPLib_Connect(3, &IGPkt, sizeof(IGPkt));	
    UDPLib_Open(8); /* IG node = 8 */

    return 1;
}

/* ------------------------------------------------------------------ */
int UDP_Recv(void)
{
    unsigned int p;
    bool         pkt1found = false;
    bool         pkt3found = false;

    do
    {
        p = UDPLib_GetPkt();
        if (p == 1)  /* PFD I/O RPi 1 */
        {
            pkt1found = true; /* now wait for pkt3 */
        }
        
        if (p == 3)
        {
            pkt3found = true;
        }
    } while (!(pkt1found && pkt3found));
    
    Frames += 1;
    return 1;
}
