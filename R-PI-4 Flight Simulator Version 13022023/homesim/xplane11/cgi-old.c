/*
 * simcgi.c
 * 
 * This plugin registers a new view in XPlane-11 based on data in the UDP packet (cgipkt) sent by the PC
 * The XPlane camera is commended to provide a view using the aircraft position and attitude.  We do this by:
 * 
 * 1. Registering hotkey F8 to engage the view.
 * 2. Setting the view to external when we are engaged.
 * 3. Registering a new camera control function that ends when a new view is picked.
 * 4. Opening a socket to receive data from the Desktop simulator.
 *
 * Notes: Based on the XPlane plugin example "Camera.c". 
          Additional code by Graham Spence, June 2017.
          Desktop simulator version by DJA, March 2020.
*/

//#define IBM 1   /* must go before the include statements to indicate this is a Linux version */

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* XPlane SDK Headers */
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMCamera.h"
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMGraphics.h"
//#include "XPLMDataRefs.h"

/*
    Flight Simulator headers.
    Note, change default packing on Windows for UDP structs arriving
    from Linux / RPi. 
*/

#ifdef WIN32
  #pragma pack(push,2)
#endif

#include <SIM/iodefn.h>
#include <SIM/igdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/udplib.h>

#ifdef WIN32
  #pragma pack(pop)
#endif

#define DEG2RAD         (M_PI / 180.0)
#define RAD2DEG         (180.0 / M_PI)

/* XPlane internal variables */
XPLMHotKeyID gHotKey = NULL;
XPLMDataRef  PhysicsEngine = NULL;
XPLMDataRef  JoystickOverride = NULL;

/* interface packet from the desktop simulator */
IODefn_IODataPkt IOPkt1;
IGDefn_IGDataPkt IGPkt;

/* Function declarations, being C etc */
void	MyHotKeyCallback(void *inRefcon);    
int 	SimCGIViewerFunc(XPLMCameraPosition_t *outCameraPosition,  
                         int                  inIsLosingControl,    
                         void                 *inRefcon);
int     UDP_Start(void);
void    UDP_Stop(void);
int     UDP_Recv(void);

/* ------------------------------------------------------------------ */
BOOL APIENTRY DllMain( HANDLE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved )
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

/* ------------------------------------------------------------------ */
PLUGIN_API int XPluginStart(
                        char *		outName,
                        char *		outSig,
                        char *		outDesc)
{
    int t[1] = { 1 };
    
    strcpy(outName, "Desktop simulator CGI");
    strcpy(outSig, "simcgi.camera");
    strcpy(outDesc, "A plugin providing an IG channel for the Desktop Simulator.");

    PhysicsEngine = XPLMFindDataRef("sim/operation/override/override_planepath");
    if (PhysicsEngine == NULL)
    {
	    printf("Cannot access Physics engine override\n");
		exit(-1);
    }
	JoystickOverride = XPLMFindDataRef("sim/operation/override/override_joystick");
	if (JoystickOverride == NULL)
	{
	    printf("Cannot access joystick override\n");
		exit(-1);
	}
    
    /* Register our hot key for the new view. */
    gHotKey = XPLMRegisterHotKey(XPLM_VK_F8,
                                 xplm_DownFlag, 
                                "SimCGI External View",
                                MyHotKeyCallback,
                                NULL);

    XPLMSetDatavi(PhysicsEngine, t, 0, 1);  /* disable the physics engine */
    XPLMSetDatai(JoystickOverride, 1);
	
    UDP_Start();

    /* Note, don't quit the plugin here (if errors) with XPLMDisablePlugin() as it can't be called until after this function completes */
    
    return 1;
}

/* ------------------------------------------------------------------ */
PLUGIN_API void XPluginStop(void)
{
    XPLMUnregisterHotKey(gHotKey);
    UDP_Stop();
}

/* ------------------------------------------------------------------ */
PLUGIN_API void XPluginDisable(void)
{
    UDP_Stop();
}

/* ------------------------------------------------------------------ */
PLUGIN_API int XPluginEnable(void)
{
    return 1;
}

/* ------------------------------------------------------------------ */
PLUGIN_API void XPluginReceiveMessage(
                    XPLMPluginID	inFromWho,
                    long			inMessage,
                    void *			inParam)
{
}

/* ------------------------------------------------------------------ */
void MyHotKeyCallback(void * inRefcon)
{
    /* This is the hotkey callback.  First we simulate a joystick press and
     * release to put us in 'free view 1'.  This guarantees that no panels
     * are showing and we are an external view. */
//    XPLMCommandButtonPress(xplm_joy_v_fr1);
//    XPLMCommandButtonRelease(xplm_joy_v_fr1);
    
    /* Now we control the camera until the view changes. */
    XPLMControlCamera(xplm_ControlCameraUntilViewChanges, SimCGIViewerFunc, NULL);

    /* Default starting conditions */

    IGPkt.PktNumber = 1;
    IGPkt.Pitch = 0.0;
    IGPkt.Roll = 0.0; 
    IGPkt.Yaw = 0.0;
    IGPkt.Latitude = 53.3623 * DEG2RAD;
    IGPkt.Longitude = -2.2802 * DEG2RAD;

    XPLMDebugString("Hotkey Callback: Defaults Initialised\n");
}

/*
 * SimCGIViewerFunc
 * This is the actual camera control function, the real worker of the plugin.  It is 
 * called each time X-Plane needs to draw a frame.
 *
 * It was registered with XPLMControlCamera via hotkey callback function.
 */
/* ------------------------------------------------------------------ */
int SimCGIViewerFunc(XPLMCameraPosition_t *outCameraPosition,   
                     int                  inIsLosingControl,    
                     void                 *inRefcon)
{
    if (outCameraPosition && !inIsLosingControl)
    {
        double localX, localY, localZ;
        
        /*
         * Check if a new packet has arrived.
         */
        
        UDP_Recv();
        
        // Commands to be added
        //IGPkt.spare1 // HOLD
        //IGPkt.spare2 // RESET (to runway - or starting position & orientation)
        
        /*
        Latitude and longitude are in decimal degrees, and altitude is in meters above MSL.
        Note, XPlane local coord sys moves to keep a/c near a reference origin so local
        x,y,z can be floats. We have no control over the local origin, so must provide
        XPlane with lat/longs and it will determine the correct x,y,z.
        */
        XPLMWorldToLocal(IGPkt.Latitude * RAD2DEG,
                         IGPkt.Longitude * RAD2DEG,
                         -IGPkt.Pz,
                         &localX,
                         &localY,
                         &localZ);
                         
        /* Fill out the camera position info. */
        outCameraPosition->x       = (float) (localX + IGPkt.Ey);          /* east component */
      //outCameraPosition->y       = (float) localY - IGPkt.Ez + 12.5;     /* up component (12.5 Manchester database adjustment) */
        outCameraPosition->y       = (float) (localY - IGPkt.Ez - 0.75);   /* up component (0.75 Heathrow database adjustment) */
        outCameraPosition->z       = (float) (localZ - IGPkt.Ex);          /* south component */
        outCameraPosition->pitch   = IGPkt.Pitch * RAD2DEG;
        outCameraPosition->heading = IGPkt.Yaw * RAD2DEG;
        outCameraPosition->roll    = IGPkt.Roll * RAD2DEG;		
        outCameraPosition->zoom    = 1.0f;
    }
    
    /* Return 1 to indicate we want to keep controlling the camera. */
    return 1;
}                  

/*
 * UDP_Start
 * Start Winsock and open a socket to receive data from the Desktop simulator interface.
 */
/* ------------------------------------------------------------------ */
int UDP_Start(void)
{
/*  open sockets from RPi 192.168.1.1 and IG 192.168.1.2 for IG 192.168.1.8 */    
    BEGIN_UDPLib();
    UDPLib_Connect(1, &IOPkt1, sizeof(IOPkt1));
    UDPLib_Connect(3, &IGPkt, sizeof(IGPkt));	
    UDPLib_Open(8); /* IG node = 8 */

    XPLMDebugString("Hotkey Callback: UDP_Start() : SUCCESS\n");
        
    return 1;
}

/*
 * UDP_Recv
 * Check if there are any incoming bytes available. If there are sufficient bytes, read the next packet.
 * Keep reading packets to 'catch-up' to prevent the incoming socket buffer filling and dropping packets.
 * This also ensures that XPlane has received the latest data if there is a queue of packets.
 */
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
            pkt1found = true;
        }
        if (p == 3)  /* EICAS I/O RPi 2 */
        {
            pkt3found = true;
        }
    } while (!(pkt1found && pkt3found));
    
    return 1;
}

/*
 * UDP_Stop
 * Close the socket and stop Winsock.
 */
/* ------------------------------------------------------------------ */
void UDP_Stop(void)
{
    XPLMDebugString("UDP_Stop: Cleaning up socket\n");
    UDPLib_Close();
}