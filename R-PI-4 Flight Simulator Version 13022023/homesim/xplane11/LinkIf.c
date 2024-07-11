/*
 * LinkIf.c
 * 
 * This plugin registers a new view with the sim that receives UDPs from a custom Link Trainer
 * RPi ADC interface and commands the camera to provide a view from the Link Trainer inputs.  We do this by:
 * 
 * 1. Registering hotkey F8 to engage the view.
 * 2. Setting the view to external when we are engaged.
 * 3. Registering a new camera control function that ends when a new view is picked.
 * 4. Opening a socket to receive data from the Link Training custom interface.
 *
 * Notes: Based on the XPlane plugin example "Camera.c". Additional code by Graham Spence, June 2017.


Values below are {analogue voltage, ADC value, variable value}

1. sin(HDG): -5V = 0 = -1.0, 0V = 16384 = 0.5, 5V = 32767 = 1.0
2. cos(HDG): -5V = 0 = -1.0, 0V = 16384 = 0.5, 5V = 32767 = 1.0
3. sin(TRK): -5V = 0 = -1.0, 0V = 16384 = 0.5, 5V = 32767 = 1.0
4. cos(TRK): -5V = 0 = -1.0, 0V = 16384 = 0.5, 5V = 32767 = 1.0
5. Airspeed: 0V = 16384 = 0 Kt, 5V = 32767 = 200 Kt
6. Ground-speed: 0V = 16384 = 0 Kt, 5V = 32767 = 200 Kt
7. Altitude: 0V = 16384 = 0 Ft, 5V = 32767 = 10000 Ft
8. sin(Altitude fine): -5V = 0 = -1.0, 0V = 16384 = 0.5, 5V = 32767 = 1.0
9. cos(Altitude fine): -5V = 0 = -1.0, 0V = 16384 = 0.5, 5V = 32767 = 1.0
10. Roll: -5V = 0 = -45 degs, 0V = 16384 = 0 degs, 5V = 32767 = +45 degs
11. Pitch: -5V = 0 = -45.0 degs, 0V = 16384 = 0 degs, 5V = 32767 = +45 degs
12. RESET or HOLD button pressed (tbc).

 */

#define LIN 1

#include <winsock2.h>
#include <windows.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h> // Using fixed width types for the incoming packet data structure
#include <sys/time.h>
#include <stdbool.h>

/* XPlane SDK Headers */
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMCamera.h"
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMGraphics.h"
//#include "XPLMDataRefs.h"

#define EARTHRADIUS     6378137.0
#define DEG2RAD         (M_PI / 180.0)
#define RAD2DEG         (180.0 / M_PI)
#define DEFAULTLAT      (51.8763 * DEG2RAD) // London Luton
#define DEFAULTLONG     (-0.3717 * DEG2RAD)

/* Struct for the data arriving from the RPi interface. */

/* XPlane 10 is 64 bit and the plugin is being compiled with a 64 bit compiler,
   so IODataPkt is be defined with exact width types to assist with interpreting
   packets formed on 32-bit systems */

typedef struct
{
    uint32_t PktNumber;

    uint16_t AnalogueData[32];
    unsigned char c1;
    unsigned char c2;
    unsigned char c3;
    unsigned char c4;
    
    float T;
    
    uint32_t TimeStamp;
    
} IODataPkt;


/* XPlane internal variables */
XPLMHotKeyID gHotKey = NULL;
/*
XPLMDataRef		gPlaneX = NULL;
XPLMDataRef		gPlaneY = NULL;
XPLMDataRef		gPlaneZ = NULL;
*/
XPLMDataRef		gRefLat = NULL;
XPLMDataRef		gRefLong = NULL;

/* globals containing socket related data */
WSADATA             wsadata;
int                 socket_in = 0;
struct sockaddr_in  s_addr_in;
char                s_buff[1500];
uint16_t            port;

/* Link Trainer RPi Interface box packet struct object */
IODataPkt IOPkt;
double    lastIterationTimestamp;
float     lastAltitudeFine;
int       altitudeThousands = 0;
bool      bReadAltCoarse = false;

/* Camera position at attitude from previous iteration */
double dOldLat, dOldLong;


/* Function declarations, being C etc */
void	MyHotKeyCallback(void *inRefcon);    
int 	LinkTrainerViewerFunc(XPLMCameraPosition_t *outCameraPosition,  
                              int                   inIsLosingControl,    
                              void                 *inRefcon);
int     UDP_Start(void);
void    UDP_Stop(void);
int     UDP_Recv(void);
void    AbortPlugin(void);
double  GetDecimalTime(void);
int     ReadConfigFile(void);

BOOL WINAPI __declspec(dllexport) DllMain( HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved )
//BOOL APIENTRY DllMain( HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved )
{
    switch (ul_reason_for_call)
    {
        case DLL_PROCESS_ATTACH:
        case DLL_THREAD_ATTACH:
        case DLL_THREAD_DETACH:
        case DLL_PROCESS_DETACH:
            break;
    }
    return TRUE;
}


__declspec(dllexport) int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{
	strcpy(outName, "Link Trainer");
	strcpy(outSig, "trenchard.linktrainer.camera");
	strcpy(outDesc, "A plugin that adds a camera view for the Trenchard Museum Link Trainer.");

	/* Prefetch the sim variables we will use. */
    
    /*
	gPlaneX = XPLMFindDataRef("sim/flightmodel/position/local_x");
	gPlaneY = XPLMFindDataRef("sim/flightmodel/position/local_y");
	gPlaneZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    */
    
    /* SDK documentation says not to write to these datarefs, but elsewhere, it says to write to these to force a scenary change! */
    gRefLat = XPLMFindDataRef("sim/flightmodel/position/lat_ref");
    gRefLong = XPLMFindDataRef("sim/flightmodel/position/long_ref");
    
	/* Register our hot key for the new view. */
	gHotKey = XPLMRegisterHotKey(XPLM_VK_F8,
                                 xplm_DownFlag, 
				                "Link Trainer External View",
				                MyHotKeyCallback,
				                NULL);
 
	/* Note, don't quit the plugin here (if errors) with XPLMDisablePlugin() as it can't be called until after this function completes */
    
    return 1;
}

__declspec(dllexport) void XPluginStop(void)
{
	XPLMUnregisterHotKey(gHotKey);
    UDP_Stop();
}

__declspec(dllexport) void XPluginDisable(void)
{
    UDP_Stop();
}

__declspec(dllexport) int XPluginEnable(void)
{
	return 1;
}

__declspec(dllexport) void XPluginReceiveMessage(
					XPLMPluginID	inFromWho,
					long			inMessage,
					void *			inParam)
{
}

void MyHotKeyCallback(void * inRefcon)
{
	/* This is the hotkey callback.  First we simulate a joystick press and
	 * release to put us in 'free view 1'.  This guarantees that no panels
	 * are showing and we are an external view. */
	XPLMCommandButtonPress(xplm_joy_v_fr1);
	XPLMCommandButtonRelease(xplm_joy_v_fr1);
    
	/* Now we control the camera until the view changes. */
	XPLMControlCamera(xplm_ControlCameraUntilViewChanges, LinkTrainerViewerFunc, NULL);
    
    /* Open a socket to receive UDPs from the Link Trainer custom interface */
    
    if( UDP_Start() == 0)
    {
        // Exit and disable plugin
        XPLMDebugString("LinkIf Hotkey Callback : UDP_Start() : RETURNED 0\n");
        UDP_Stop();
        XPLMPluginID id = XPLMGetMyID();
        XPLMDisablePlugin(id);
    }
    else
    {
        XPLMDebugString("LinkIf Hotkey Callback : UDP_Start() : SUCCESS\n");
    }
    
    /* Default configuration for starting conditions. If file can't be opened, use the following defaults */
    if (ReadConfigFile() == 0)
    {
        /* Initialise XPlane camera to either the default position or from configuration file */
        dOldLat = DEFAULTLAT;
        dOldLong = DEFAULTLONG;
        
        /* Initialise the packet fields with some default data */
        /*
        IOPkt.PktNumber = 1;
        IOPkt.pitch = 16384; // 0 deg
        IOPkt.roll = 16384; // 0 deg
        IOPkt.altitude_coarse = 24576; // 5000 ft. Using fine altitude
        IOPkt.altitude_fine_sin = 32767; // 1 IS THIS BASED ON COMPASS BEARING OR UNIT CIRCLE?
        IOPkt.altitude_fine_cos = 16384; // 0 IS THIS BASED ON COMPASS BEARING OR UNIT CIRCLE?
        IOPkt.airspeed = 16384; // 0 kts
        IOPkt.groundspeed = 18384; // 0 kts
        // North
        //IOPkt.shdg = 32767; // sin(hdg) => 1 [ 0 deg, North ]
        //IOPkt.chdg = 16384; // cos(hdg) => 0 [ 0 deg, North ]
        //IOPkt.strk = 32767;
        //IOPkt.ctrk = 16384;
        // East
        //IOPkt.shdg = 16384;
        //IOPkt.chdg = 32767;
        //IOPkt.strk = 16384;
        //IOPkt.ctrk = 32767;
        // West
        IOPkt.shdg = 16384;
        IOPkt.chdg = 0;
        IOPkt.strk = 16384;
        IOPkt.ctrk = 0;
        IOPkt.spare1 = 0;
        IOPkt.spare2 = 0;
        IOPkt.TimeStamp = 0;
        */
        
        IOPkt.PktNumber = 1;
        IOPkt.AnalogueData[10] = 16384; // 0 deg
        IOPkt.AnalogueData[9] = 16384; // 0 deg
        IOPkt.AnalogueData[6] = 24576; // 5000 ft. Using fine altitude
        IOPkt.AnalogueData[7] = 32767; // 1 IS THIS BASED ON COMPASS BEARING OR UNIT CIRCLE?
        IOPkt.AnalogueData[8] = 16384; // 0 IS THIS BASED ON COMPASS BEARING OR UNIT CIRCLE?
        IOPkt.AnalogueData[4] = 16384; // 0 kts
        IOPkt.AnalogueData[5] = 18384; // 0 kts
        // North
        //IOPkt.shdg = 32767; // sin(hdg) => 1 [ 0 deg, North ]
        //IOPkt.chdg = 16384; // cos(hdg) => 0 [ 0 deg, North ]
        //IOPkt.strk = 32767;
        //IOPkt.ctrk = 16384;
        // East
        //IOPkt.shdg = 16384;
        //IOPkt.chdg = 32767;
        //IOPkt.strk = 16384;
        //IOPkt.ctrk = 32767;
        // West
        IOPkt.AnalogueData[0] = 16384;
        IOPkt.AnalogueData[1] = 0;
        IOPkt.AnalogueData[2] = 16384;
        IOPkt.AnalogueData[3] = 0;
        //IOPkt.spare1 = 0;
        //IOPkt.spare2 = 0;
        IOPkt.TimeStamp = 0;
        
        lastAltitudeFine = 0.0f; // 0..999.999
        altitudeThousands = 0;
    }

    // Record start time
    lastIterationTimestamp = GetDecimalTime();
    
    XPLMDebugString("LinkIf Hotkey Callback : Defaults Initialised\n");
}

/*
 * LinkTrainerViewerFunc
 * 
 * This is the actual camera control function, the real worker of the plugin.  It is 
 * called each time X-Plane needs to draw a frame.
 *
 * It was registered with XPLMControlCamera via hotkey callback function.
 */
int LinkTrainerViewerFunc(XPLMCameraPosition_t *outCameraPosition,   
                          int                   inIsLosingControl,    
                          void                 *inRefcon)
{
	if (outCameraPosition && !inIsLosingControl)
	{
        char buffer[256];
        float hdg, trk, pitch, roll;
		float kgs, x, y, fVe, fVn, currentAltitude;
        float altc, altfs, altfc, fineAltDeg, fineAltFt;
        double dLatDot, dLongDot;
        double dNewLat, dNewLong;
        double localX, localY, localZ;
        
		/*
         * Check if a new Link Trainer input packet has arrived.
         */
		
        if(UDP_Recv())
        {
            // Record time for this successful packet receipt? Why? The simulation continues regardless of packets arriving...
            //XPLMDebugString("+");
            
            // For the very first successful packet read, read the coarse altitude and use this as a base for
            // the fine altitutde thousands count
            if (bReadAltCoarse == false)
            {
                altc = (( (float)IOPkt.AnalogueData[6] - 16384.0f) / 16384.0f) * 10000.0f;  // 10000 ft converted to m
                altitudeThousands = (int)(altc / 1000.0f);
                bReadAltCoarse = true;
            }
        }
        else
        {
            // Don't update timestamp, so next step has a larger elaspsed time (since last packet) in the event of a lost packet.
            //XPLMDebugString("-");
        }
        
        //
        // Transform analogue input data to simulation / XPlane values. Data comes from last packet received.
        // If no data was received, use the previous packet values
        //
        
        // XPlane pitch, roll, and yaw are rotations from a camera facing flat north in degrees (negative-Z axis).
        // Positive pitch means nose up, positive roll means roll right, and positive yaw means yaw right, all in degrees.
        //
         
        // Pitch and Roll
        pitch = (( (float)IOPkt.AnalogueData[10] - 16384.0f) / 16384.0) * 45.0f;
        roll = (( (float)IOPkt.AnalogueData[9] - 16384.0f) / 16384.0) * 45.0f;
        
        // Altitude (coarse)
        altc = (( (float)IOPkt.AnalogueData[6] - 16384.0f) / 16384.0f) * 10000.0f;  // 10000 ft converted to m
        
        // Altitude (fine)
        altfs = ((float)IOPkt.AnalogueData[7] - 16384.0f) / 16384.0f;
        altfc = ((float)IOPkt.AnalogueData[8] - 16384.0f) / 16384.0f;
        fineAltDeg = atan2f(altfs,altfc) * RAD2DEG;
        fineAltDeg = fmodf((fineAltDeg + 360.0f), 360.0f); // 0..355.999999
        fineAltFt = fineAltDeg * 2.7777778f; // 0..999.99999
        
        // Potentiometer wired wrong way? When altitude increments from 0, we see the analogue input decrementing from 1000
        fineAltFt = 1000.0f - fineAltFt;
        
        // Using fine altitude only, so detect thoushands rollover
        // Test for rollover up to next 1000 ft level
        if (lastAltitudeFine >= 800.0f && fineAltFt < 200.0f && altitudeThousands < 9)
        {
            // Roll-up detected
            altitudeThousands = altitudeThousands + 1;
            //altitudeThousands = altitudeThousands - 1; // Reversed on test box
        }
        else if (lastAltitudeFine < 200.0f && fineAltFt >= 800.0f && altitudeThousands > 0)
        {
            // Roll-down detected. Only allow roll-down if above 1000 ft
            altitudeThousands = altitudeThousands - 1;
            //altitudeThousands = altitudeThousands + 1; // Reversed on test box
        }
        //else
        //{
            // No change in thoushands
        //}
   
        // Combine thoushands and fine altitude to give current altitude
        currentAltitude = (((float)altitudeThousands * 1000.0f) + fineAltFt);
        //sprintf(buffer, "current %f\n", currentAltitude);
        //XPLMDebugString(buffer);
        
     

        // Record current value of fine altitude for next iteration
        lastAltitudeFine = fineAltFt;
        
        
        // Ground speed: 0v = 0 Kt, +5V = 200 Kt
        kgs = ((float)(IOPkt.AnalogueData[5] - 16384.0f) / 16384.0f) * 102.888888f; // 200 kts converted to m/s
            
        // Heading and track in radians
        y = ((float)IOPkt.AnalogueData[0] - 16384.0f) / 16384.0f;
        x = ((float)IOPkt.AnalogueData[1] - 16384.0f) / 16384.0f;
        hdg = atan2f(y, x);
        y = ((float)IOPkt.AnalogueData[2] - 16384.0f) / 16384.0f;
        x = ((float)IOPkt.AnalogueData[3] - 16384.0f) / 16384.0f;
        trk = atan2f(y, x);
        
        // Convert to degrees +-180
        hdg = hdg * RAD2DEG;
        // Convert unit circle to compass bearing
        //hdg = 90.0f - hdg;
        //trk = trk * RAD2DEG;
        // Not sure yet if XPlane wants +-180 or 0-355, so uncomment below for 0..360
        hdg = fmodf((hdg + 360.0f), 360.0f);
        //trk = fmodf((trk + 360.0f), 360.0f);
        
        // Other commands from RPi Interface
        //IOPkt.spare1 // HOLD
        //IOPkt.spare2 // RESET (to runway - or starting position & orientation)
        
        
        //
        // Compute the new position of the link trainer. Dead reckoning using data from last packet and elapsed time.
        //
        
        trk = (90.0f * DEG2RAD) - trk;
        
        // Calculate VEast and VNorth from ground speed (m/s) and track (radians).
        fVn = sin( trk ) * kgs;
        fVe = cos( trk ) * kgs;
        
        // Rate of change of longitude and latitude. lat and long in radians please
        dLongDot = fVe / (EARTHRADIUS * cos(dOldLat));
        dLatDot = fVn / EARTHRADIUS;
        
        // Update latitude and longitude
        double currentTimeStamp = GetDecimalTime();
        double elapsedTime = currentTimeStamp - lastIterationTimestamp;
        dNewLong = dOldLong + dLongDot * elapsedTime;
        dNewLat = dOldLat + dLatDot * elapsedTime;
        
        /*
        Latitude and longitude are in decimal degrees, and altitude is in meters above MSL.
        Note, XPlane local coord sys moves to keep a/c near a reference origin so local
        x,y,z can be floats. We have no control over the local origin, so must provide
        XPlane with lat/longs and it will determine the correct x,y,z.
        */
        XPLMWorldToLocal((dNewLat * RAD2DEG),
                         (dNewLong * RAD2DEG),
                         currentAltitude * 0.3048f, // altc - coarse altitude can be used if fine altitude not working
                         //altc, // Significant jitter observed at low altitudes. Higher altitudes masks it - use fine altitude
                         &localX,
                         &localY,
                         &localZ);
                         
		/* Fill out the camera position info. */
		outCameraPosition->x       = (float)localX;
		outCameraPosition->y       = (float)localY;
		outCameraPosition->z       = (float)localZ;
		outCameraPosition->pitch   = pitch;
		outCameraPosition->heading = hdg;
		outCameraPosition->roll    = roll;		
        outCameraPosition->zoom    = 1.0f;
        
        // Update old lat longs with new values ready for next iteration
        dOldLat = dNewLat;
        dOldLong = dNewLong;
        
        // Store current time stamp so elapsed time can be determined next iteration
        lastIterationTimestamp = currentTimeStamp;
	}
	
	/* Return 1 to indicate we want to keep controlling the camera. */
	return 1;
}                  

/*
 * UDP_Start
 *
 * Start Winsock and open a socket to receive data from the Link Trainer RPi interface.
 */
int UDP_Start(void)
{
    int result;
    char buffer[256];
    int permission = 1;
	
	/*
        Start winsock
    */
	result = WSAStartup(MAKEWORD(2, 2), &wsadata); // 2,2 is latest winsock spec. Returns 0 for success.
    if (result != 0)
    {
        sprintf(buffer, "LinkIf : UDP_Start : WSAStartup failed %d\n", result); // The error code is the windows error code
        //XPLMDebugString(const char *inString); // Autoflushes output
        XPLMDebugString(buffer);
        return 0;
    }

	/*
        Open incoming data socket
    */
    memset(&s_addr_in, 0, sizeof(s_addr_in));
    s_addr_in.sin_family      = AF_INET;
    s_addr_in.sin_addr.s_addr = htonl(INADDR_ANY);
    s_addr_in.sin_port        = htons(54321);
    socket_in                 = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // Returns a socket descriptor, otherwise INVALID_SOCKET with error found using WSAGetLastError()
    if (socket_in == INVALID_SOCKET)
    {
        sprintf(buffer, "LinkIf : UDP_Start : socket() failed : socket_in : %d\n", WSAGetLastError());
        XPLMDebugString(buffer);
        WSACleanup();
        return 0;
    }
    
    /*
        Allow rebinding. Usually not recommended, but to restart the simulation, the DLL gets reset and bind will fail.
        The DLL isn't unloaded, so in theory the open socket will still be available. This could be tracked and then
        there wouldn't be a need to reopen the socket every restart.
    */
    if (setsockopt(socket_in, SOL_SOCKET, SO_REUSEADDR, (void *) &permission, sizeof(permission)) == SOCKET_ERROR)
    {   
        sprintf(buffer, "LinkIf : UDP_Start : SO_REUSEADDR failed : could not set socket rebinding : %d\n", WSAGetLastError());
        XPLMDebugString(buffer);
        WSACleanup();
        return 0;
    }
    
    /*
        Bind socket
    */
    result = bind(socket_in, (struct sockaddr *) &s_addr_in, sizeof(s_addr_in));
    if (result == SOCKET_ERROR) // No error, bind returns 0
    {
        sprintf(buffer, "LinkIf : UDP_Start : bind() failed : socket_in : %d\n", WSAGetLastError());
        XPLMDebugString(buffer);
        UDP_Stop();
        return 0;
    }
    
    return 1;
}

/*
 * UDP_Recv
 *
 * Check if there are any incoming bytes available. If there are sufficient bytes, read the next packet.
 * Keep reading packets to 'catch-up' to prevent the incoming socket buffer filling and dropping packets.
 * This also ensures that XPlane has received the latest data if there is a queue of packets.
 */
int UDP_Recv(void)
{
	int rv, rvpeek;
    int addr_len = sizeof(s_addr_in);
	char buffer[256];
    int copyPkt = 0; // flag the function should attempt to copy the packet data. 0 = no copy
    u_long peekBytes;
    
    /*
        Loop recvfrom until the current packet and any later packets have been read.
        Simple / crude non-blocking implementation via MSG_PEEK.
    */
    for(;;)
    {
        // MSG_PEEK looks at only the next recv size i.e. packet size, not the amount in the rx buffer which might be more
        // But if zero bytes waiting, MSG_PEEK blocks! Ah.... this won't work.
        // Try FION_READ
        /*
        rvpeek = recvfrom(socket_in, s_buff, sizeof(s_buff), MSG_PEEK, (struct sockaddr *) &s_addr_in, &addr_len);
        if (rvpeek == SOCKET_ERROR)
        {
            sprintf(buffer, "LinkIf : UDP_Recv : recvfrom MSG_PEEK error : %d\n", WSAGetLastError());
            XPLMDebugString(buffer);
            AbortPlugin();
            return 0;
        }
        */
    
        // Get size of data in the rx socket buffer
        ioctlsocket(socket_in, FIONREAD, &peekBytes);
        
        // If there is more than the packet size available, go ahead and receive packet
        //if (rvpeek >= sizeof(IOPkt))
        if (peekBytes >= sizeof(IOPkt))
        {
            rv = recvfrom(socket_in, s_buff, sizeof(s_buff), 0, (struct sockaddr *) &s_addr_in, &addr_len);
            if (rv == SOCKET_ERROR)
            {
                sprintf(buffer, "LinkIf : UDP_Recv : recvfrom error : %d\n", WSAGetLastError());
                XPLMDebugString(buffer);
                AbortPlugin();
                return 0;
            }
            // Pkt received - flag for processing. Will turn 1 when at least one pkt received
            copyPkt = 1;
        }
        else
        {
            // Escape loop as no or incomplete data i.e. no complete input packet available
            // If return here, it excludes previously received packets in this loop - the last
            // one still needs processing.
            break;
        }
    }

    // Should we go ahead and copy packet data?
    if (copyPkt == 1)
    {
        // Check the size of the received data
        if (rv != sizeof(IOPkt))
        {
           // Possibly not an IOPkt. Inn theory, this should't happen
           XPLMDebugString("LinkIf : UDP_Recv : Unexpected packet size\n");
           return 0;
        }
        
        // Copy the data into the working struct
        memcpy(&IOPkt, s_buff, rv);
    }
    else
    {
        // Unreachable?
        return 0;
    }
    
    return 1;
}

/*
 * UDP_Stop
 *
 * Close the socket and stop Winsock.
 */
void UDP_Stop(void)
{
    XPLMDebugString("LinkIf : UDP_Stop : Cleaning up socket\n");
	closesocket(socket_in);
	WSACleanup();
}

/*
 * GetDecimalTime
 *
 * Get the current time of day in seconds as a double
 */
double GetDecimalTime(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return ( (double)tv.tv_sec + (double)(tv.tv_usec/1000000.0) ); 
}

/*
 * AbortPlugin. Something has gone wrong so bring down the socket and disable the plugin
 */
void AbortPlugin(void)
{
    XPLMDebugString("LinkIf : AbortPlugin\n");
    UDP_Stop();
    XPLMPluginID id = XPLMGetMyID();
    XPLMDisablePlugin(id);
}

/*
 * ReadConfigFile
 */
int ReadConfigFile(void)
{
    FILE *fp;
    char buffer[80];
    int initspd, inithdg;

    fp = fopen("Resources\\plugins\\LinkPlugin\\64\\link.conf", "r");
    if (fp == NULL)
    {
        XPLMDebugString("LinkIf : ReadConfigFile : COULD NOT OPEN FILE link.conf WILL USE DEFAULTS\n");
        return 0;
    }

    fscanf(fp, "IF_PORT=%hu", &port);
    fgets(buffer,80, fp);
    fscanf(fp, "START_LAT=%lf", &dOldLat);
    fgets(buffer,80, fp);
    fscanf(fp, "START_LONG=%lf", &dOldLong);
    fgets(buffer,80, fp);
    fscanf(fp, "START_ALT_FTX1000=%d", &altitudeThousands);
    fgets(buffer,80, fp);
    fscanf(fp, "START_SPD_KTS=%d", &initspd);
    fgets(buffer,80, fp);
    fscanf(fp, "START_HDG_DEG=%d", &inithdg);
    fgets(buffer,80, fp);
    fclose(fp);

    // Ensure the initial config units are correct for the simulation
    dOldLat = dOldLat * DEG2RAD;
    dOldLong = dOldLong * DEG2RAD;
    lastAltitudeFine = 0.0f;
    inithdg = 90 - inithdg;
    
    // Some init values need converting to expected analogue values
    IOPkt.AnalogueData[10] = 16384; // 0 deg
    IOPkt.AnalogueData[9] = 16384; // 0 deg
    IOPkt.AnalogueData[5] = (uint16_t) ((((float)initspd / 200.0f) * 16384.0f) + 16384.0f);
    IOPkt.AnalogueData[4] = IOPkt.AnalogueData[5];
    IOPkt.AnalogueData[6] = (uint16_t) ((((float)altitudeThousands / 10000.0f) * 16384.0f) + 16384.0f);
    IOPkt.AnalogueData[7] = 32767; // 1 IS THIS BASED ON COMPASS BEARING OR UNIT CIRCLE?
    IOPkt.AnalogueData[8] = 16384; // 0
    
    // Detemine analogue inputs from start heading in degrees. Compass bearing or unit circle from Link trainer?
    // convert to compass bearing? work out sin cos of input hdg, convert to analogue
    //inithdg = 90 - inithdg;
    IOPkt.AnalogueData[0] = (uint16_t) ((sin((float)inithdg * DEG2RAD) * 16384.0f) + 16384.0f);
    IOPkt.AnalogueData[1] = (uint16_t) ((cos((float)inithdg * DEG2RAD) * 16384.0f) + 16384.0f);
    IOPkt.AnalogueData[2] = IOPkt.AnalogueData[0];
    IOPkt.AnalogueData[3] = IOPkt.AnalogueData[1];
    //IOPkt.spare1 = 0;
    //IOPkt.spare2 = 0;
    IOPkt.TimeStamp = 0;
    
    // Output config file values to log, along with corresponding analogue values
    XPLMDebugString("LinkIf : ReadConfigFile : Starting data...");
    sprintf(buffer, "LinkIf Port : %d\n", port);
    XPLMDebugString(buffer);
    sprintf(buffer, "LinkIf Lat Long : %f %f\n", dOldLat * RAD2DEG, dOldLong * RAD2DEG);
    XPLMDebugString(buffer);
    sprintf(buffer, "LinkIf Spd Hdg : %d %d\n", initspd, 90 - inithdg);
    XPLMDebugString(buffer);
    sprintf(buffer, "LinkIf Altitude(000) : %d\n", altitudeThousands);
    XPLMDebugString(buffer);
    
    return 1;
}
