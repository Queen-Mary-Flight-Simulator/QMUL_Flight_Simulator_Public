#pragma pack(push,2)
#include "iodefn.h"
#include "aerodefn.h"
#include "navdefn.h"
#include "iosdefn.h"
#include "protodefn.h"
#pragma pack(pop)

#include "mex.h"

// Testing a static global persistent variable. In theory, this should work for sockets.
static int counter = 0;

void Function1(void) {
    mexPrintf ("Function1\n");
}

void mexFunction (int nlhs, mxArray *plhs[],
                   int nrhs, const mxArray *prhs[])
{
    mexPrintf ("Packet Size\n");
    mexPrintf ("IOData Pkt size   = %d\n", sizeof(IODefn_IODataPkt));
    mexPrintf ("AeroData Pkt size = %d\n", sizeof(AeroDefn_AeroDataPkt));
    mexPrintf ("NavData Pkt size = %d\n", sizeof(NavDefn_NavDataPkt));
    mexPrintf ("IOSData Pkt size = %d\n", sizeof(IosDefn_IosDataPkt));
    mexPrintf ("I have %d inputs and %d outputs\n", nrhs, nlhs);
    counter++;
    Function1();
}
