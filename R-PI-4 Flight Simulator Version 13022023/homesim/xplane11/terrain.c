/*
 * HellWorld.c
 * 
 * This plugin implements the canonical first program.  In this case, we will 
 * create a window that has the text hello-world in it.  As an added bonus
 * the  text will change to 'This is a plugin' while the mouse is held down
 * in the window.  
 * 
 * This plugin demonstrates creating a window and writing mouse and drawing
 * callbacks for that window.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <string.h>
#include <math.h>

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"

#include <SIM/dted.h>

#define DEG2RAD         (M_PI / 180.0)
#define RAD2DEG         (180.0 / M_PI)

/* XPlane internal variables */
XPLMHotKeyID gHotKey = NULL;

XPLMDataRef		AircraftX = NULL;
XPLMDataRef		AircraftY = NULL;
XPLMDataRef		AircraftZ = NULL;


/*
 * Global Variables.  We will store our single window globally.  We also record
 * whether the mouse is down from our mouse handler.  The drawing handler looks
 * at this information and draws the appropriate display.
 * 
 */

XPLMWindowID	gWindow = NULL;
int				gClicked = 0;

double          DTEDlatmin  = 53.327;
double          DTEDlatmax  = 53.372;
double          DTEDlongmin = -2.320;
double          DTEDlongmax = -2.250;
double          *Posts;
DTED_Record     DTED;

XPLMProbeRef    probe;

void MyDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon);    

void MyHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus);    

int MyHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon);    

void GenDTED();		
void WriteDTED(char Filename[]);
double GetSpotHeight(double latitude, double longitude);

/*
 * XPluginStart
 * 
 * Our start routine registers our window and does any other initialization we 
 * must do.
 * 
 */
PLUGIN_API int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{
	/* First we must fill in the passed in buffers to describe our
	 * plugin to the plugin-system. */

	strcpy(outName, "HelloWorld");
	strcpy(outSig, "xplanesdk.examples.helloworld");
	strcpy(outDesc, "A plugin that makes a window.");

	/* Now we create a window.  We pass in a rectangle in left, top,
	 * right, bottom screen coordinates.  We pass in three callbacks. */

	gWindow = XPLMCreateWindow(
					50, 600, 300, 200,			/* Area of the window. */
					1,							/* Start visible. */
					MyDrawWindowCallback,		/* Callbacks */
					MyHandleKeyCallback,
					MyHandleMouseClickCallback,
					NULL);						/* Refcon - not used. */
					
	/* We must return 1 to indicate successful initialization, otherwise we
	 * will not be called back again. */

	return 1;
}

/*
 * XPluginStop
 * 
 * Our cleanup routine deallocates our window.
 * 
 */
PLUGIN_API void	XPluginStop(void)
{
	XPLMDestroyWindow(gWindow);
}

/*
 * XPluginDisable
 * 
 * We do not need to do anything when we are disabled, but we must provide the handler.
 * 
 */
PLUGIN_API void XPluginDisable(void)
{
}

/*
 * XPluginEnable.
 * 
 * We don't do any enable-specific initialization, but we must return 1 to indicate
 * that we may be enabled at this time.
 * 
 */
PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

/*
 * XPluginReceiveMessage
 * 
 * We don't have to do anything in our receive message handler, but we must provide one.
 * 
 */
PLUGIN_API void XPluginReceiveMessage(
					XPLMPluginID	inFromWho,
					long			inMessage,
					void *			inParam)
{
}

/*
 * MyDrawingWindowCallback
 * 
 * This callback does the work of drawing our window once per sim cycle each time
 * it is needed.  It dynamically changes the text depending on the saved mouse
 * status.  Note that we don't have to tell X-Plane to redraw us when our text
 * changes; we are redrawn by the sim continuously.
 * 
 */
void MyDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon)
{
	int		left, top, right, bottom;
	float	color[] = { 1.0, 1.0, 1.0 }; 	/* RGB White */
	
	/* First we get the location of the window passed in to us. */
	XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
	
	/* We now use an XPLMGraphics routine to draw a translucent dark
	 * rectangle that is our window's shape. */
	XPLMDrawTranslucentDarkBox(left, top, right, bottom);

	/* Finally we draw the text into the window, also using XPLMGraphics
	 * routines.  The NULL indicates no word wrapping. */
	XPLMDrawString(color, left + 5, top - 20, 
		(char*)(gClicked ? "I'm a plugin" : "Hello world"), NULL, xplmFont_Basic);
		
}                                   

/*
 * MyHandleKeyCallback
 * 
 * Our key handling callback does nothing in this plugin.  This is ok; 
 * we simply don't use keyboard input.
 * 
 */
void MyHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus)
{
}                                   

/*
 * MyHandleMouseClickCallback
 * 
 * Our mouse click callback toggles the status of our mouse variable 
 * as the mouse is clicked.  We then update our text on the next sim 
 * cycle.
 * 
 */
int MyHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon)
{
/* ************************************************** */    
    GenDTED();
    WriteDTED("MANCHESTER.DTD");
    free(Posts);
 /* ************************************************* */    
	/* If we get a down or up, toggle our status click.  We will
	 * never get a down without an up if we accept the down. */
	if ((inMouse == xplm_MouseDown) || (inMouse == xplm_MouseUp))
		gClicked = 1 - gClicked;
	
	/* Returning 1 tells X-Plane that we 'accepted' the click; otherwise
	 * it would be passed to the next window behind us.  If we accept
	 * the click we get mouse moved and mouse up callbacks, if we don't
	 * we do not get any more callbacks.  It is worth noting that we 
	 * will receive mouse moved and mouse up even if the mouse is dragged
	 * out of our window's box as long as the click started in our window's 
	 * box. */
	return 1;
}

/* ------------------------------------------------------------------ */
void GenDTED()		
{
    int wx1;
    int wy1;
    int wx2;
    int wy2;
    int i;
    int j;
    int p;
    int dsize;
    printf("Generate Terrain\n");
    probe = XPLMCreateProbe(xplm_ProbeY);
	
	/* Prefetch the sim variables we will use. */
    
	AircraftX = XPLMFindDataRef("sim/flightmodel/position/local_x");
	AircraftY = XPLMFindDataRef("sim/flightmodel/position/local_y");
	AircraftZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    
    wx1 = (int) (DTEDlongmin * 100000.0);
    wy1 = (int) (DTEDlatmin  * 100000.0);
    wx2 = (int) (DTEDlongmax * 100000.0);
    wy2 = (int) (DTEDlatmax  * 100000.0);
    
    DTED.bx1 = (float) ((wx1 / 10) * 10);
    DTED.by1 = (float) ((wy1 / 10) * 10);
    DTED.bx2 = (float) ((wx2 / 10) * 10);
    DTED.by2 = (float) ((wy2 / 10) * 10);

    DTED.xPosts = abs(DTED.bx2 - DTED.bx1) / 10 + 1;
    DTED.yPosts = abs(DTED.by2 - DTED.by1) / 10 + 1;
    dsize = DTED.xPosts * DTED.yPosts * sizeof(double);
	
    printf("GenDTED: bx1=%f by1=%f bx2=%f by2=%f xPosts=%d yPosts=%d\n", 
                 DTED.bx1, DTED.by1, DTED.bx2, DTED.by2, DTED.xPosts, DTED.yPosts);

    Posts = (double *) malloc(dsize);
    if (!Posts)
    {
        printf("Error: Insufficient memory for DTED (%d K bytes)\n", dsize / 1000);
        exit(1);
    }

    p = 0;

    for (i=0; i<DTED.xPosts; i+=1)
    {
        for (j=0; j<DTED.yPosts; j+=1)
        {
            float x = DTED.bx1 + (float) (i * 10);
            float y = DTED.by1 + (float) (j * 10);
            double longitude = x * DEG2RAD / 100000.0;
			double latitude  = y * DEG2RAD / 100000.0;
						
            Posts[p] = GetSpotHeight(latitude, longitude);
            p += 1;
        }
    }

	XPLMDestroyProbe(probe);
}

/* ------------------------------------------------------------------ */
double GetSpotHeight(double latitude, double longitude)
{
    double          x, y, z;
	double          xlat, xlong, h;
	int             res;
	XPLMProbeInfo_t probeinfo;

    XPLMWorldToLocal(latitude * RAD2DEG,
                     longitude * RAD2DEG,
                     5000.0,
                     &x,
                     &y,
                     &z);

    XPLMSetDataf(AircraftX, x);
    XPLMSetDataf(AircraftY, y);
    XPLMSetDataf(AircraftZ, z);
	
    probeinfo.structSize = sizeof(XPLMProbeInfo_t);
                    
	res = XPLMProbeTerrainXYZ(probe, x, y, z, &probeinfo);

    XPLMLocalToWorld(probeinfo.locationX,
                     probeinfo.locationY,
                     probeinfo.locationZ,
                     &xlat,
                     &xlong,
                     &h);

    if (res == xplm_ProbeHitTerrain)
	{
	    return h;
    }
	else
	{
	    return 0.0;
	}
}

/* ------------------------------------------------------------------ */
void WriteDTED(char Filename[])
{
    FILE *f;

    if ((f = fopen(Filename, "wb")) == NULL)
    {
        printf("Error opening DTED file %s\n", Filename);
        return;
    }

    fwrite(&DTED, sizeof(DTED), 1, f);
    fwrite(Posts, DTED.xPosts * DTED.yPosts * sizeof(double), 1, f);

    fclose(f);
}
