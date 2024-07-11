/* +------------------------------+---------------------------------+
   | Module      : udplib.c       | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-02-07      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : UDP packet transfers library                     |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

/* NB Read-only version */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#include <sys/types.h>
//#include <sys/socket.h>
//#include <arpa/inet.h>
//#include <unistd.h>

#include <winsock2.h>
#include <windows.h>

#include <SIM/udplib.h>

#define DEFAULT_PORT    54321
#define MAX_BUFFERS     32

typedef struct
{
    void         *pktadr;
    unsigned int pktsize;
} pktrecord;

static char               GROUP_IP []     = "192.168.1.0";

static struct sockaddr_in s_addr_in;
static int                socket_in;
static pktrecord          pktbuffers[MAX_BUFFERS + 1];

static char               buff[1500];
static unsigned int       CurrentNode = 0;

static unsigned int       group_ip;

static unsigned int       Get_Group(unsigned int x);
static unsigned int       Get_Node(unsigned int x);

/* ------------------------------------------------ */
static unsigned int Get_Group(unsigned int x)
{
    union
    {
        unsigned int c;
        char         b[4];
    } x32;

    x32.c    = htonl(x);
    x32.b[0] = x32.b[1];
    x32.b[1] = x32.b[2];
    x32.b[2] = x32.b[3];
    x32.b[3] = 0;
    return x32.c;
}

/* ------------------------------------------------ */
static unsigned int Get_Node(unsigned int x)
{
    return htonl(x) % 256;
}

/* ------------------------------------------------ */
void UDPLib_Connect(unsigned int node, void *pkt, unsigned int pktlen)
{
    pktbuffers[node].pktadr  = pkt;
    pktbuffers[node].pktsize = pktlen;
}

/* ------------------------------------------------ */
void UDPLib_SendPkt(void *pkt, unsigned int n)
{
    return;
}

/* ------------------------------------------------ */
unsigned int UDPLib_GetPkt()
{
    unsigned int n = 0;
    unsigned int retval;
    int          addr_len = sizeof(s_addr_in);
	
    while (1)
    {
        retval = recvfrom(socket_in, buff, sizeof(buff), 0,
                          (struct sockaddr *) &s_addr_in, &addr_len);
        if (retval < 0)
        {
            printf("recvfrom error\n");
            exit(1);
        }

        if (Get_Group(s_addr_in.sin_addr.s_addr) == group_ip)
        {
            n = Get_Node(s_addr_in.sin_addr.s_addr);
            if (n != CurrentNode)  /* don't copy reflected pkt */
            {
                if (retval != pktbuffers[n].pktsize)
                {
                    printf("UDPLib_GetPkt: node %d bad size: %d (%d)\n", n, retval, pktbuffers[n].pktsize);
                    exit(1);
                }
                if (pktbuffers[n].pktadr != NULL)
                {
                    memcpy(pktbuffers[n].pktadr, &buff, pktbuffers[n].pktsize);
                    return n;
                }
                else
                {
                    printf("UDPLib_Getpkt: Unknown pkt: %x8\n", (unsigned int) htonl(s_addr_in.sin_addr.s_addr));
                }
            }
        }
    }

    return n;
}

/* ------------------------------------------------ */
void UDPLib_Open(unsigned int n)
{
    WSADATA wsaData;
    int     res;
	
    res = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (res != 0)
    {
        printf("WSAStartup failed: %d\n", res);
        exit(1);
    }
    CurrentNode               = n;
    s_addr_in.sin_family      = AF_INET;
    s_addr_in.sin_addr.s_addr = htonl(INADDR_ANY);
    s_addr_in.sin_port        = htons(DEFAULT_PORT);
    socket_in                 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_in < 0)
    {
        printf("socket open error\n");
        exit(1);
    }
    if (bind(socket_in, (struct sockaddr *) &s_addr_in, sizeof(s_addr_in)) < 0)
    {
        printf("bind error\n");
        exit(1);
    }
}

/* ------------------------------------------------ */
void UDPLib_Close()
{
    closesocket(socket_in);
    WSACleanup();
}

/* ------------------------------------------------ */
void BEGIN_UDPLib()
{
    unsigned int i;

    for (i = 0; i <= MAX_BUFFERS; i = i + 1)
    {
        pktbuffers[i].pktadr  = NULL;
        pktbuffers[i].pktsize = 0;
    }

    group_ip = Get_Group(inet_addr(GROUP_IP));
}
