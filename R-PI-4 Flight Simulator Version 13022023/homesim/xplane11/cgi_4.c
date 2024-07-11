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
XPLMHotKeyID gHotKey          = NULL;
XPLMDataRef  PhysicsEngine    = NULL;
XPLMDataRef  JoystickOverride = NULL;
XPLMDataRef  TargetXRef       = NULL;
XPLMDataRef  TargetYRef       = NULL;
XPLMDataRef  TargetZRef       = NULL;
XPLMDataRef  TargetPitchRef   = NULL;
XPLMDataRef  TargetRollRef    = NULL;
XPLMDataRef  TargetYawRef     = NULL;

/* interface packet from the desktop simulator */
IODefn_IODataPkt IOPkt1;
IGDefn_IGDataPkt IGPkt;

unsigned int     CmdPtr;
float            cloudbase      = 15000.0;
float            visibility     = 35000.0f;
float            visrate        = 0.0f;
int              TargetLoaded   = 0;
unsigned int     CameraPosition = 0;
int              HUDMode        = 0;  /* off by default */

long long unsigned int Timer0;
long long unsigned int Timer1;
long long unsigned int Timer2;
long long unsigned int Timer3;
long long unsigned int Timer4;
long long unsigned int freq;
unsigned int           PktsLost = 0;
unsigned int           Frames = 0;
/* Function declarations, being C etc */
void	      MyHotKeyCallback(void *inRefcon);    
int 	      SimCGIViewerFunc(XPLMCameraPosition_t *outCameraPosition,  
                               int                  inIsLosingControl,    
                               void                 *inRefcon);
int           UDP_Start(void);
void          UDP_Stop(void);
int           UDP_Recv(void);

unsigned char GetByte();
float         GetReal();
unsigned int  GetWord();
int           GetInt();
bool          GetBoolean();
void          DecodeIosPkt(void);
XPLMDataRef   GetRef(const char str[]);

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

    PhysicsEngine    = GetRef("sim/operation/override/override_planepath");
    JoystickOverride = GetRef("sim/operation/override/override_joystick");
    TargetXRef       = GetRef("sim/flightmodel/position/local_x");
    TargetYRef       = GetRef("sim/flightmodel/position/local_y");
    TargetZRef       = GetRef("sim/flightmodel/position/local_z");
    TargetPitchRef   = GetRef("sim/flightmodel/position/theta");
    TargetRollRef    = GetRef("sim/flightmodel/position/phi");
    TargetYawRef     = GetRef("sim/flightmodel/position/psi");
	
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

/*  open sockets from RPi 192.168.1.1 and IG 192.168.1.3 for IG 192.168.1.8 */    
    BEGIN_UDPLib();
    UDPLib_Connect(1, &IOPkt1, sizeof(IOPkt1));
    UDPLib_Connect(3, &IGPkt, sizeof(IGPkt));	
    UDPLib_Open(8); /* IG node = 8 */

    QueryPerformanceFrequency((void *) &freq);
    QueryPerformanceCounter((void *) &Timer0);
    Timer1 = Timer0;
	Timer3 = Timer0;
	
    XPLMDebugString("Hotkey Callback: Defaults Initialised\n");
}

/*
 * SimCGIViewerFunc
 * This is the actual camera control function, the real worker of the plugin 
 * and is called each time X-Plane renders a frame.
 *
 * It was registered with XPLMControlCamera via a hotkey callback function.
 */
/* ------------------------------------------------------------------ */
int SimCGIViewerFunc(XPLMCameraPosition_t *outCameraPosition,   
                     int                  inIsLosingControl,    
                     void                 *inRefcon)
{
    double AircraftX, AircraftY, AircraftZ;
    double AircraftPitch, AircraftRoll, AircraftYaw;
    double TargetX, TargetY, TargetZ;
    double TargetPitch, TargetRoll, TargetYaw;
        
    if (!(outCameraPosition && !inIsLosingControl))
    {
	    return 1;
	}	
	
QueryPerformanceCounter((void *) &Timer4);
printf("%d %f\n", Frames, (float) (Timer4 - Timer3) / 10000.0);

    UDP_Recv();
QueryPerformanceCounter((void *) &Timer3); // ***
        
    DecodeIosPkt();

    if (fabs(IGPkt.Latitude) < 0.01 && fabs(IGPkt.Longitude < 0.01))
	{
	    return 1;  /* aircraft not positioned */
	}
	
    /* N.B. Beware - XPlane crashes if the aircraft or target is positoned at (0, 0)
	   Latitude and longitude are in decimal degrees, altitude is in metres above MSL.
       Note, XPlane local coords are moved to keep a/c near a reference origin so local
       x,y,z can be floats. We have no control over the local origin, so must provide
       XPlane with lat/longs and it will determine the correct x,y,z. */
	   
    XPLMWorldToLocal(IGPkt.Latitude * RAD2DEG,
                     IGPkt.Longitude * RAD2DEG,
                     -IGPkt.Pz,
                     &AircraftX,
                     &AircraftY,
                     &AircraftZ);
    AircraftPitch = IGPkt.Pitch;
    AircraftRoll  = IGPkt.Roll;
    AircraftYaw   = IGPkt.Yaw;
        
	if (CameraPosition == 0)  /* place the camera in the aircraft cockpit */
	{
		/* set the camera position to the pilot eye-point */
		outCameraPosition->x       = (float) (AircraftX + IGPkt.Ey);                         /* east component */
		outCameraPosition->y       = (float) (AircraftY - IGPkt.Ez + IGPkt.AirportOffset);   /* up component (0.75 Heathrow database adjustment) */
		outCameraPosition->z       = (float) (AircraftZ - IGPkt.Ex);                         /* south component */
		outCameraPosition->pitch   = AircraftPitch * RAD2DEG;
		outCameraPosition->heading = AircraftYaw * RAD2DEG;
		outCameraPosition->roll    = AircraftRoll * RAD2DEG;		
		outCameraPosition->zoom    = 1.0f;
		
        if (fabs(IGPkt.TLatitude) < 0.01 && fabs(IGPkt.TLongitude < 0.01))  /* XPlane-11 will use the default target position */
	    {
	        return 1;  /* Target not positioned */
    	}
	    else
    	{
			XPLMWorldToLocal(IGPkt.TLatitude * RAD2DEG,
							 IGPkt.TLongitude * RAD2DEG,
							 -IGPkt.TPz,
							 &TargetX,
							 &TargetY,
							 &TargetZ);
			TargetPitch = IGPkt.TPitch;
			TargetRoll = IGPkt.TRoll;
			TargetYaw = IGPkt.TYaw;

			XPLMSetDataf(TargetXRef, TargetX);
			XPLMSetDataf(TargetYRef, TargetY);
			XPLMSetDataf(TargetZRef, TargetZ);
			XPLMSetDataf(TargetPitchRef, TargetPitch * RAD2DEG);
			XPLMSetDataf(TargetRollRef, TargetRoll * RAD2DEG);
			XPLMSetDataf(TargetYawRef, TargetYaw * RAD2DEG);
			return 1;
		}
    }
    else   /* position the aircraft as the target */
	{
		XPLMSetDataf(TargetXRef, AircraftX);
		XPLMSetDataf(TargetYRef, AircraftY);
		XPLMSetDataf(TargetZRef, AircraftZ);
		XPLMSetDataf(TargetPitchRef, AircraftPitch * RAD2DEG);
		XPLMSetDataf(TargetRollRef, AircraftRoll * RAD2DEG);
		XPLMSetDataf(TargetYawRef, AircraftYaw * RAD2DEG);

        outCameraPosition->pitch   = 0.0;
        outCameraPosition->roll    = 0.0;
        outCameraPosition->zoom    = 1.0f;
	
		if (CameraPosition == 1)  /* move the camera to the rear view */
		{
			outCameraPosition->x = AircraftX - 100.0 * sin(AircraftYaw);
			outCameraPosition->y = AircraftY;
			outCameraPosition->z = AircraftZ + 100.0 * cos(AircraftYaw);
			outCameraPosition->heading = AircraftYaw * RAD2DEG;
		}		
		else if (CameraPosition == 2) /* move the camera to the side view */
		{
			outCameraPosition->x = AircraftX + 100.0 * cos(AircraftYaw);
			outCameraPosition->y = AircraftY;
			outCameraPosition->z = AircraftZ + 100.0 * sin(AircraftYaw);
			outCameraPosition->heading = (AircraftYaw - M_PI / 2.0) * RAD2DEG;
		}
		else if (CameraPosition == 3) /* move the camera to the wing man */
		{
			outCameraPosition->x = AircraftX + 50.0 * cos(AircraftYaw + M_PI * 0.25);
			outCameraPosition->y = AircraftY;
			outCameraPosition->z = AircraftZ + 50.0 * sin(AircraftYaw + M_PI * 0.25);
			outCameraPosition->heading = (AircraftYaw - M_PI / 4.0) * RAD2DEG;
		}
		else if (CameraPosition == 4) /* move the camera to the god's-eye */
		{
			outCameraPosition->x = AircraftX;
			outCameraPosition->y = 1000.0;
			outCameraPosition->z = AircraftZ;
			outCameraPosition->pitch = -(M_PI / 2.0) * RAD2DEG;
			outCameraPosition->heading = 0.0;
		}
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
    XPLMDebugString("Hotkey Callback: UDP_Start() : SUCCESS\n");
        
    return 1;
}

/*
 * UDP_Sync
 * reads queued packets until packets are arriving every 20ms
 */
/* ------------------------------------------------------------------ */
void UDP_Sync(void)
{
    bool         pkt1found = false;
    bool         pkt3found = false;
    unsigned int npkts = 0;
	
    QueryPerformanceCounter((void *) &Timer1);

    do
    {
        unsigned int p = UDPLib_GetPkt();
		
        if (p == 1)  /* PFD I/O RPi 1 */
        {
            QueryPerformanceCounter((void *) &Timer2);
            pkt1found = true; /* now wait for pkt3 */
        }
        
        if (p == 3)
        {
            pkt3found = true;
        }

        if (((Timer2 - Timer1) / 10) < 20000)  /* frame < 20 ms? */
        {
		    pkt1found = false;
			pkt3found = false;
			Timer1 = Timer2;
			npkts += 1;
		}
    } while (!(pkt1found && pkt3found));
	
	printf("UDP_Sync: %d pkts\n", npkts);
}

/*
 * UDP_Recv
 * normally just read pkt1 and pkt3
 * if frame time > 22 ms discard pkt1 and pkt3 and wait for next pkt1
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
            QueryPerformanceCounter((void *) &Timer2);
            if (((Timer2 - Timer1) / 10) > 22000)  /* frame > 22 ms? */
            {
                do
                {
                    p = UDPLib_GetPkt();  /* wait for pkt3 */
                    if (p == 3)
                    {
                        pkt3found = true;
                    }
                } while (!pkt3found);
                pkt1found = false;  /* discard pkt1 */
                pkt3found = false;  /* discard pkt3 */
                Timer1 = Timer2;    /* remember time of last pkt1 */
                PktsLost += 1;
                continue;
            }
            else
            {
                pkt1found = true; /* now wait for pkt3 */
                Timer1 = Timer2;  /* remember time of last pkt1 */
            }
        }
        
        if (p == 3)
        {
            pkt3found = true;
        }
    } while (!(pkt1found && pkt3found));
    
    Frames += 1;
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

    QueryPerformanceCounter((void *) &Timer2);
    printf("Frames: %d:\nUDP packets lost: %d\nFrame rate %f fps\n", Frames, PktsLost, (float) Frames * 1.0e7 / (float) (Timer2 - Timer0));
    UDPLib_Close();
}

/* ------------------------------------------------ */
XPLMDataRef GetRef(const char str[])
{
    XPLMDataRef r;
    r = XPLMFindDataRef(str);
    if (r == NULL)
    {
        printf("Cannot access %s\n", str);
        exit(-1);
    }
    return r;
}

/* ------------------------------------------------ */
unsigned char GetByte()
{
    unsigned char x;

    x      = IGPkt.CmdBuff[CmdPtr];
    CmdPtr = CmdPtr + 1;
    return x;
}

/* ------------------------------------------------ */
float GetReal()
{
    union
    {
        float r;
        char  b[4];
    } x32;

    x32.b[0] = GetByte();
    x32.b[1] = GetByte();
    x32.b[2] = GetByte();
    x32.b[3] = GetByte();
    return x32.r;
}

/* ------------------------------------------------ */
unsigned int GetWord()
{
    union
    {
        unsigned short int c;
        char               b[2];
    } x16;

    x16.b[0] = GetByte();
    x16.b[1] = GetByte();
    return (unsigned int) x16.c;
}

/* ------------------------------------------------ */
int GetInt()
{
    union
    {
        short int i;
        char      b[2];
    } x16;

    x16.b[0] = GetByte();
    x16.b[1] = GetByte();
    return (int) x16.i;
}

/* ------------------------------------------------ */
bool GetBoolean()
{
    unsigned char x;

    x = GetByte();
    return(x != 0);
}

/* ---------------------------------------------------- */
void DecodeIosPkt(void)

{
    unsigned short int IosCmd;
    
    CmdPtr = 0;
    
    while (1)
    {
        IosCmd = GetWord();

        switch (IosCmd)
        {
            case IosDefn_EndOfPkt: 
                return;

            case IosDefn_Exit:  /* ignore */
                break;

            case IosDefn_SetCloudbase:
                cloudbase = GetReal();
                break;
    
            case IosDefn_SetVisibility:
                visibility = GetReal();
                break;
      
            case IosDefn_SetVisRate:
                visrate = GetReal();
                break;
      
            case IosDefn_LoadTargetFile:
                TargetLoaded = 1;
                break;
      
            case IosDefn_SwitchTargetOff:
                TargetLoaded = 0;
                break;

            case IosDefn_SwitchHUDOff:
                HUDMode = GetBoolean();
                break;
            
            case IosDefn_PlaybackCamera:
                CameraPosition = GetInt();
                break;

            default: 
                return;
        }
    }
}
