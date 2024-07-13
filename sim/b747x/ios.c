#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include <SIM/maths.h>
#include <SIM/pnglib.h>
#include <SIM/udplib.h>
#include <SIM/glibx.h>
#include <SIM/navlib.h>

#include "ios.h"
#include "menu.h"
#include "map.h"
#include "aerolink.h"
#include "englink.h"
#include "navlink.h"
#include "ioslink.h"
#include "approach.h"
#include "plot.h"
#include "script.h"
#include "gui.h"
#include "dataview.h"
#include "scan.h"
#include "iolib.h"
#include "diagnostics.h"
#include "pfd.h"
#include "nfd.h"
#include "engines.h"
#include "panellib.h"

#define DIAGNOSTICS 0

bool         IOS_Mode;
unsigned int NumberOfSteps;
GLFWwindow*  window;

void appSetupRenderingContext(void);
void error_callback(int error, const char* description);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

/* ------------------------------------------------------------- */
void appSetupRenderingContext(void)
{
    if (!IOS_Mode)
	{
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClearDepth(1.0);
        glDepthFunc(GL_LEQUAL);
    }
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.0);
}

/* ------------------------------------------------------------- */
extern void IOS_Update(void)
{
    Plot_CheckPlayback();

    memcpy(&AeroLink_IosPkt, &IosLink_IosPkt, sizeof(IosDefn_IosDataPkt));
    memcpy(&EngLink_IosPkt,  &IosLink_IosPkt, sizeof(IosDefn_IosDataPkt));
    memcpy(&NavLink_IosPkt,  &IosLink_IosPkt, sizeof(IosDefn_IosDataPkt));

    /* Data recording. Copy io,aero and nav to data pkt */
    memcpy(&IosLink_IosPlotDataPkt.IOPkt1,  &IosLink_IOPkt1,  sizeof(IosLink_IOPkt1));
    memcpy(&IosLink_IosPlotDataPkt.IOPkt2,  &IosLink_IOPkt2,  sizeof(IosLink_IOPkt2));
    memcpy(&IosLink_IosPlotDataPkt.AeroPkt, &IosLink_AeroPkt, sizeof(IosLink_AeroPkt));
    memcpy(&IosLink_IosPlotDataPkt.EngPkt,  &IosLink_EngPkt,  sizeof(IosLink_EngPkt));
    memcpy(&IosLink_IosPlotDataPkt.NavPkt,  &IosLink_NavPkt,  sizeof(IosLink_NavPkt));
    Plot_SaveData();

    Gui_CheckMouse();

    Map_UpdateTrackList();

    IosLink_CheckWayPoints(); /* any FP waypoints to add? */

    IosLink_CheckSystemChange();

    if (Script_ScriptError == true)
    {
        // Show error message dialog
        Script_ScriptError = false;
    }

    if (Script_ScriptEnabled)
    {
        Script_ExecuteScript(IosLink_ScriptFilename);
    }
}

/* ------------------------------------------------------------- */
void IOS_Display(void)
{
    int    x1 = Glibx_SCREENWIDTH - 40;
	int    y1 = Glibx_SCREENHEIGHT - 40;
	int    x2 = x1 + 20;
	int    y2 = y1 + 20;

    NumberOfSteps = NumberOfSteps + 1;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glClear(GL_COLOR_BUFFER_BIT);
    Glibx_AntiAliasing(true);

    Glibx_ClipWindow(0, 0, Glibx_SCREENWIDTH - 1, Glibx_SCREENHEIGHT - 1);
    glColor3f(1.0f, 1.0f, 1.0f);
    glRecti(Glibx_SCREENWIDTH / 2, 0, Glibx_SCREENWIDTH, Glibx_SCREENHEIGHT - 1);
	
    if (IosLink_IOSMode == MapDisplay)
    {
        Map_DrawMap();
    }
    else if (IosLink_IOSMode == ApproachDisplay)
    {
        Approach_ShowApproach();
    }
    else if (IosLink_IOSMode == FlightDataDisplay)
    {
        Plot_ShowPlot();
    }
    else if (IosLink_IOSMode == RawDataDisplay)
    {
        DataView_Display();
    }

    Gui_Menu();

	Glibx_Colour(Glibx_GREY);
	Glibx_Draw(x1, y1, x1, y2);
	Glibx_Draw(x1, y2, x2, y2);
	Glibx_Draw(x2, y2, x2, y1);
	Glibx_Draw(x2, y1, x1, y1);
	Glibx_Draw(x1, y1, x2, y2);
	Glibx_Draw(x1, y2, x2, y1);
}

/* ------------------------------------------------------------- */
void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

/* ---------------------------------- */
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_P && action == GLFW_PRESS)
    {
        PngLib_SavePngFile("temp.png");
    }
	else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)  // ADDED FOR QMU VERSION 1
	{
    	AeroLink_RemoteHold = !AeroLink_RemoteHold;
	}
}

/* ---------------------------------- */
void IOS_Init()
{
    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    window = glfwCreateWindow(Glibx_SCREENWIDTH, Glibx_SCREENHEIGHT, "IOS", glfwGetPrimaryMonitor(), NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
	glfwRestoreWindow(window);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, Gui_Mouse_Button_Callback);
    glfwSetCursorPosCallback(window, Gui_Mouse_Cursor_Callback);
    glfwSetScrollCallback(window, Gui_Mouse_Scroll_Callback);

    PanelLib_InitialiseTextures(); /* must be called after GLFW is initialised */
    appSetupRenderingContext();

    while (!glfwWindowShouldClose(window))
    {
        IosLink_SendCmd(IosDefn_EndOfPkt);
        IosLink_CmdPtr = 0;

	    PFD_Update();
		Engines_Update();
		NFD_Update();
        IOS_Update();
		
        glViewport(0, 0, Glibx_SCREENWIDTH, Glibx_SCREENHEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0.0, (float) Glibx_SCREENWIDTH, 0.0, (float) Glibx_SCREENHEIGHT);
        glMatrixMode(GL_MODELVIEW);

        if (IOS_Mode)
		{
		    PFD_Display();
            IOS_Display();
        }
		else
		{
		    PFD_Display();
			NFD_Display();
		}

    Diagnostics_SetTimer2(); /* ***************************** */
        if (DIAGNOSTICS)
        {
            glViewport(0, 0, Glibx_SCREENWIDTH, Glibx_SCREENHEIGHT);
            Diagnostics_Diagnostics(960, 330, 750);
        }
		
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    IOS_CloseWindow();
}

/* ---------------------------------- */
void IOS_CloseWindow()
{
    glfwDestroyWindow(window);
    glfwTerminate();
}

/* ---------------------------------- */
void BEGIN_IOS()
{
    printf("IOS starting\n");
	
    IOS_Mode            = true;
    NumberOfSteps       = 0;
}
