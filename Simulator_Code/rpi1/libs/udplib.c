#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
  #include <winsock2.h>
#else
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <unistd.h>
#endif

#include <SIM/udplib.h>

#define DEFAULT_PORT 54321
#define MAX_BUFFERS  10

typedef struct
{
    void         *pktadr;
    unsigned int pktsize;
} pktrecord;

static char BROADCAST_IP [] = "192.168.1.255";
static char GROUP_IP []     = "192.168.1.0";

static int                retval;
static struct sockaddr_in s_addr_in;
static struct sockaddr_in s_addr_out;
static int                socket_in;
static int                socket_out;
static unsigned int       addr_len;
static int                permission;
static pktrecord          pktbuffers[MAX_BUFFERS+1];

static char               buff[1600];
static unsigned int       CurrentNode = 0;

static unsigned int       group_ip;

static unsigned int Get_Group(unsigned int x);
static unsigned int Get_Node(unsigned int x);

/* ------------------------------------------------ */
static unsigned int Get_Group(unsigned int x)
{
  union {
      unsigned int c;
      char b[4];
  } x32;

  x32.c = htonl(x);
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
    pktbuffers[node].pktadr = pkt;
    pktbuffers[node].pktsize = pktlen;
}

/* ------------------------------------------------ */
void UDPLib_SendPkt(void *pkt, unsigned int n)
{
  retval = sendto(socket_out, pkt, n, 0, 
                  (struct sockaddr *) &s_addr_out, sizeof(s_addr_out));
  if (retval < 0) {
    printf("send error\n");
    exit(1);
  }
}

/* ------------------------------------------------ */
unsigned int UDPLib_GetPkt()
{
    unsigned int n = 0;

    for (;;) 
    {
        retval = recvfrom(socket_in, &buff, sizeof(buff), 0, 
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
                    printf("UDPLib_Getpkt: Unknown pkt: %x8\n", htonl(s_addr_in.sin_addr.s_addr));
                }
            }
        }
    }

    return n;
}

/* ------------------------------------------------ */
void UDPLib_Open(unsigned int n)
{
  CurrentNode = n;
  permission = 1;
  addr_len = sizeof(s_addr_in);
  s_addr_in.sin_family = AF_INET;
  s_addr_in.sin_addr.s_addr = htonl(INADDR_ANY);
  s_addr_in.sin_port = htons(DEFAULT_PORT);
  socket_in = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_in < 0) {
    printf("socket open error\n");
    exit(1);
  }
  if (bind(socket_in, (struct sockaddr *) &s_addr_in, sizeof(s_addr_in)) < 0) {
    printf("bind error\n");
    exit(1);
  }
  s_addr_out.sin_family = AF_INET;
  s_addr_out.sin_addr.s_addr = inet_addr(BROADCAST_IP);
  s_addr_out.sin_port = htons(DEFAULT_PORT);
  socket_out = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_out < 0) {
    printf("socket open error\n");
    exit(1);
  }
  if (setsockopt(socket_out, SOL_SOCKET, SO_BROADCAST, &permission, sizeof(permission)) < 0) {
    printf("permission error\n");
    exit(1);
  }
}

/* ------------------------------------------------ */
void UDPLib_Close()
{
  close(socket_in);
  close(socket_out);
}

/* ------------------------------------------------ */
void BEGIN_UDPLib()
{
    unsigned int i;

    for (i=0; i<=MAX_BUFFERS; i=i+1) 
    {
        pktbuffers[i].pktadr = NULL;
        pktbuffers[i].pktsize = 0;
    }

    group_ip = Get_Group(inet_addr(GROUP_IP));
}
