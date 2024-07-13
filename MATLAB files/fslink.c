#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <winsock.h>
#include <string.h>

#pragma pack(push,2)
#include "iodefn.h"
#include "aerodefn.h"
#include "navdefn.h"
#include "iosdefn.h"
#include "protodefn.h"
#pragma pack(pop)

#include "fslink.h"
#include "mex.h"

#define ONERAD        57.29577951
#define DEFAULT_PORT  54321
#define BROADCAST_IP  "192.168.1.255"

// Mex subfunction numerical id's
#define MEX_UDP_START 1
#define MEX_UDP_STOP 2
#define MEX_UDP_SEND 3
#define MEX_UDP_RECV 4
#define MEX_UDP_DATASET 5
#define MEX_UDP_DATAGET 6

#define MEX_OUTPUT_DATANUM 27

union IPaddress
{   unsigned int       int32;
    unsigned char      bytes[4];
};

/* globals containing socket related data */
int                 socket_in  = 0;
int                 socket_out = 0;
unsigned short      port;
struct sockaddr_in  s_addr_in;
struct sockaddr_in  s_addr_out;
unsigned int        group_ip;
int                 permission = 1;
char                buf[1508];

/* structs to store flight simulator packets */
IODefn_IODataPkt       IOPkt;
AeroDefn_AeroDataPkt   AeroPkt;
NavDefn_NavDataPkt     NavPkt;
IosDefn_IosDataPkt     IosPkt;
ProtoDefn_ProtoDataPkt ProtoPkt;

/* winsock data comms struct */
WSADATA wsadata;


/* ---------------------------------------------------- */

unsigned int GetGroup( unsigned int addr )
{
    union IPaddress x;

    x.int32 = htonl(addr);
    x.bytes[0] = x.bytes[1];
    x.bytes[1] = x.bytes[2];
    x.bytes[2] = x.bytes[3];
    x.bytes[3] = 0;
    return x.int32;
}

/* ---------------------------------------------------- */

unsigned int GetNode( unsigned int addr )

{
    return htonl(addr) % 256;
}

/* ---------------------------------------------------- */

int UDP_Start(unsigned short port)
{
	/* Start winsock */
	if (WSAStartup(MAKEWORD(2, 0), &wsadata) != 0)
    {
        return -6;
    }

	/*
        Incoming data socket
    */
    memset(&s_addr_in, 0, sizeof(s_addr_in));
    s_addr_in.sin_family      = AF_INET;
    s_addr_in.sin_addr.s_addr = htonl(INADDR_ANY);
    s_addr_in.sin_port        = htons(DEFAULT_PORT);

    socket_in = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_in < 0) {
        return 0;
    }
    if (bind(socket_in, (struct sockaddr *) &s_addr_in, sizeof(s_addr_in)) < 0) {
        return 0;
    }

    group_ip = GetGroup(inet_addr(BROADCAST_IP));

    /*
        Outgoing data socket
    */
    memset(&s_addr_out, 0, sizeof(s_addr_out));
    s_addr_out.sin_family      = AF_INET;
    s_addr_out.sin_addr.s_addr = inet_addr(BROADCAST_IP);
	s_addr_out.sin_port        = htons(DEFAULT_PORT);

    socket_out = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_out < 0) {
        return 0;
    }
    if (setsockopt(socket_out, SOL_SOCKET, SO_BROADCAST, (void *) &permission, sizeof(permission)) < 0) {
        perror("permission");
        return 0;
    }

    mexPrintf("UDP_Start : Matlab UDP connection ready for datagrams %d\n", port);
    return 1;
}

/* ---------------------------------------------------- */

void UDP_Stop(void)
{
	closesocket(socket_in);
    closesocket(socket_out);
	WSACleanup();
}

/* ---------------------------------------------------- */

/* sends datagram */
int UDP_Send(void)
{
	int addr_len = sizeof(s_addr_out);
	int retval;

    /* Should be checking return was size of data to be sent */
    retval = sendto(socket_out, &ProtoPkt, sizeof(ProtoPkt), 0, (struct sockaddr *)&s_addr_out, addr_len);
    if (retval < 0) {
        perror("send");
        return 0;
    }

	return 1;
}

/* ---------------------------------------------------- */

/* receives datagram */
int UDP_Recv(void)
{
	int retval;
    int addr_len = sizeof(s_addr_in);
    int pkt1found = 0;
    int pkt2found = 0;
    int pkt3found = 0;
    int pkt4found = 0;

    /* Loop until IO, Aero, Nav, Ios UDP's have arrived */

	// Sync with IO packet so this function returns with a complete / consistent frame
    do
	{
	    retval = recvfrom(socket_in, buf, sizeof(buf), 0, (struct sockaddr *) &s_addr_in, &addr_len);
		if (GetGroup(s_addr_in.sin_addr.s_addr) == group_ip)
        {
			if (GetNode(s_addr_in.sin_addr.s_addr) == 1)
			{
			    pkt1found = 1;
				if (retval != sizeof(IOPkt)) {
					perror("IOPkt");
					return -14;
				} else {
					memcpy(&IOPkt, buf, retval);
				}
			}
		}
	} while (pkt1found == 0);

	// With the IO packet previously received, receive the packets from the other nodes in the frame
	do {
		retval = recvfrom(socket_in, buf, sizeof(buf), 0, (struct sockaddr *) &s_addr_in, &addr_len);
		if (retval < 0) {
			perror("recvfrom:");
			return -13;
		}

		if (GetGroup(s_addr_in.sin_addr.s_addr) == group_ip)
		{
			switch (GetNode(s_addr_in.sin_addr.s_addr))
			{
			// case 1: SYNC WITH PKT 1 ABOVE TO ENSURE WE ARE ONLY SYNC'ED WITH PACKETS FROM THIS FRAME
			case 2:
				pkt2found = 1;
				if (retval != sizeof(AeroPkt)) {
					perror("AeroPkt");
					return -15;
				} else {
					memcpy(&AeroPkt, buf, retval);
					if (AeroPkt.Shutdown) {
						UDP_Stop();
					}
				}
			break;

			case 3:
				pkt3found = 1;
				if (retval != sizeof(NavPkt)) {
					perror("NavPkt");
					return -16;
				} else {
					memcpy(&NavPkt, buf, retval);
				}
			break;

			case 4:
				pkt4found = 1;
				if (retval != sizeof(IosPkt)) {
					perror("IosPkt");
					return -17;
				} else {
					memcpy(&IosPkt, buf, retval);
				}
			break;

			default:
			break;

			} /* switch */
		}

	} while((pkt2found == 0) || (pkt3found == 0) || (pkt4found == 0));

    /* All simulator UDPs have arrived ... continue */

    return 1;
}

/* ---------------------------------------------------- */

void UDP_Data_Set(double *data)
{
    /* Set Tx data from Matlab array object */
    
	ProtoPkt.Header.PktNumber      = AeroPkt.PktNumber; 
    ProtoPkt.Header.PktType        = PROTO_TYPE_MATLAB;
    ProtoPkt.Data.Matlab.Aileron   = (float) data[0]; // 0.0 -> 1.0
    ProtoPkt.Data.Matlab.Elevator  = (float) data[1]; // as above   
    ProtoPkt.Data.Matlab.Rudder    = (float) data[2]; // as above   
    ProtoPkt.Data.Matlab.Throttle  = (float) data[3]; // as above
/*
	mexPrintf ("Setting Proto Data\nAileron %f Elevator %f Rudder %f Throttle %f\n",
	           ProtoPkt.Data.Matlab.Aileron,
			   ProtoPkt.Data.Matlab.Elevator,
			   ProtoPkt.Data.Matlab.Rudder,
			   ProtoPkt.Data.Matlab.Throttle);
*/
}

/* ---------------------------------------------------- */

void UDP_Data_Get(double *data)
{
	/* copy data from most recently received UDPs */
	
    data[0] = AeroPkt.Roll;
    data[1] = AeroPkt.Pitch;
	data[2] = AeroPkt.Yaw;
	data[3] = AeroPkt.P;
	data[4] = AeroPkt.Q;
	data[5] = AeroPkt.R;
	data[6] = AeroPkt.Pz;
	data[7] = AeroPkt.Alpha;
    data[8] = AeroPkt.Beta;
	data[9] = AeroPkt.U;
    data[10] = AeroPkt.V;
    data[11] = AeroPkt.W;
    data[12] = AeroPkt.UDot;
    data[13] = AeroPkt.VDot;
    data[14] = AeroPkt.WDot;
    data[15] = AeroPkt.Vn;
    data[16] = AeroPkt.Ve;
    data[17] = AeroPkt.Vd;
    data[18] = AeroPkt.Elevator;
    data[19] = AeroPkt.Aileron;
    data[20] = AeroPkt.Rudder;
    data[21] = AeroPkt.ElevatorTrim;
    data[22] = AeroPkt.AileronTrim;
    data[23] = AeroPkt.RudderTrim;
    data[24] = IOPkt.EngineLever;
    data[25] = (double) AeroPkt.OctaveMode;
    data[26] = (double) IOPkt.KeySwitch;

}

/* ---------------------------------------------------- */

/*
    Params:
    1. Number of return arguments
    2. Array of pointers to return arguments
    3. Number of input arguments
    4. Array of pointers to input arguments
*/
void mexFunction (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
    int functionId;
	int rv = 0;
	unsigned short port = 0;
	double *inMatrix;       /* 1xN input matrix */
	mwSize ncols;           /* size of input matrix */
	double *outMatrix;      /* output matrix  - to store data returned from simulator */

/*
    mexPrintf ("Packet Size\n");
    mexPrintf ("IOData Pkt size   = %d\n", sizeof(IODefn_IODataPkt));
    mexPrintf ("AeroData Pkt size = %d\n", sizeof(AeroDefn_AeroDataPkt));
    mexPrintf ("NavData Pkt size = %d\n", sizeof(NavDefn_NavDataPkt));
    mexPrintf ("IOSData Pkt size = %d\n", sizeof(IosDefn_IosDataPkt));
    mexPrintf ("I have %d inputs and %d outputs\n", nrhs, nlhs);
*/

	/* The M-FILE script passes in the subfunction ID which is used in the switch yard */
	functionId = (int)mxGetScalar(prhs[0]);
	switch(functionId) {
		case MEX_UDP_START:
			mexPrintf("Calling Function MEX_UDP_START\n");
			port = (unsigned short)mxGetScalar(prhs[1]);
			mexPrintf("Port : %d\n", (int)port);
			rv = UDP_Start(port);
			mexPrintf("rv = %d\n", rv);
		break;
		case MEX_UDP_STOP:
			mexPrintf("Calling Function MEX_UDP_STOP\n");
			UDP_Stop();
		break;
		case MEX_UDP_SEND:
			//mexPrintf("Calling Function MEX_UDP_SEND\n");
			rv = UDP_Send();
			//mexPrintf("rv = %d\n", rv);
		break;
		case MEX_UDP_RECV:
			//mexPrintf("Calling Function MEX_UDP_RECV\n");
			rv = UDP_Recv();
			//mexPrintf("UDP_Recv() rv = %d\n", rv);
		break;
		case MEX_UDP_DATASET:
			//mexPrintf("Calling Function MEX_UDP_DATASET\n");
			/* create a pointer to the real data in the input matrix  */
			inMatrix = mxGetPr(prhs[1]);
			/* get dimensions of the input matrix */
            ncols = mxGetN(prhs[1]);
			UDP_Data_Set(inMatrix);
		break;
		case MEX_UDP_DATAGET:
			//mexPrintf("Calling Function MEX_UDP_DATAGET\n");
			/* create the output matrix */
            plhs[0] = mxCreateDoubleMatrix(1,(mwSize)MEX_OUTPUT_DATANUM,mxREAL);
			/* get a pointer to the real data in the output matrix */
            outMatrix = mxGetPr(plhs[0]);
			UDP_Data_Get(outMatrix);
		break;
	}
}
