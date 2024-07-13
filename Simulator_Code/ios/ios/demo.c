#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SIM/maths.h>
#include <SIM/pnglib.h>
//#include <SIM/udplib.h>
#include <SIM/glib.h>
#include <SIM/navlib.h>

#include "map.h"
#include "ioslink.h"
#include "approach.h"
#include "plot.h"
#include "script.h"
#include "gui.h"
#include "dataview.h"
#include "scan.h"
#include "iolib.h"
#include "diagnostics.h"

#define DIAGNOSTICS 0

static char         snapfilename[] = "snap00.png";
static bool         pkt1found      = false;
static bool         pkt2found      = true;  /* dummmy RPi2 */
static bool         pkt3found      = false;
static bool         pkt4found      = false;
static bool         pkt5found      = false;
static bool         pkt10found     = false;
static unsigned int FrameNumber;
static GLFWwindow   * window;

unsigned int NumberOfSteps;
double       t0;
double       t1;
double       t2;

/* prototypes */
void Update(void);
void appSetupRenderingContext(void);
void Display(void);
void error_callback(int error, const char* description);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void ShutDown(void);

/* ---------------------------------- */
void appSetupRenderingContext(void)
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LEQUAL);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.0);
}

/* ---------------------------------- */
void Update(void)
{
//    unsigned int p;

//    do
//    {
//        p = UDPLib_GetPkt();
//        if (p == 1)
//        {
//            pkt1found = true;
//        }
        //if (p == 2)
        //{
        //    pkt2found = true;
        //}
//        if (p == 3)
//        {
//            pkt3found = true;
//        }
//        if (p == 4)
//        {
//            pkt4found = true;
//        }
//        if (p == 5)
//        {
//            pkt5found = true;
//        }
//    } while (!(pkt1found && pkt2found && pkt3found && pkt4found && pkt5found));

    Plot_CheckPlayback();

    IosLink_SendCmd(IosDefn_EndOfPkt);
    IosLink_IosPkt.PktNumber = FrameNumber;

    FrameNumber += 1;
//    UDPLib_SendPkt(&IosLink_IosPkt, sizeof(IosLink_IosPkt));

    if (IosLink_ExitPending)
	{
	    ShutDown();
	}
/* MATLAB / OCTAVE */

//    if (IosLink_AeroPkt.OctaveMode)
//    {
//        do
//        {
//            p = UDPLib_GetPkt();
//            if (p == 10)
//            {
//                pkt10found = true;
//            }
//        } while (!pkt10found);
//    }

    pkt1found = false;
    pkt2found = true; /* dummy RPi2 */
    pkt3found = false;
    pkt4found = false;
    pkt5found = false;
    pkt10found = false;

    IosLink_CmdPtr = 0;

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

    if (Script_Error == true)
    {
        // Show error message dialog
        Script_Error = false;
    }

    if (Script_Enabled)
    {
        Script_ExecuteScript(IosLink_ScriptFilename);
    }

    //if (IosLink_IOPkt.KeySwitch == IODefn_Off)
    //{
    //    ShutDown();
    //}
}

/* ---------------------------------- */
void Display(void)
{
    Glib_AntiAliasing(true);

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
}

/* ------------------------------------------------------------- */
void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

/* ---------------------------------- */
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        float  etime;
        double t1 = glfwGetTime();

        etime = (float) (t1 - t0);
        printf("Frames: %d\n", NumberOfSteps);
        printf("Time: %f\n", etime);
        printf("Frame Rate: %f fps\n", (float) NumberOfSteps / etime);
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
    else if (key == GLFW_KEY_P && action == GLFW_PRESS)
    {
        snapfilename[5] = snapfilename[5] + 1;
        if (snapfilename[5] > '9')
        {
            snapfilename[5] = '0';
            snapfilename[4] = snapfilename[4] + 1;
        }
        PngLib_SavePngFile(snapfilename, 0, 0, 1024, 768);
    }
}

/*----------------------------------------------------------------------------*/
void ShutDown(void)
{
    printf("IOS Shutdown!\n");
    if (Script_Enabled == true)
    {
        Script_SaveScript(IosLink_ScriptFilename); // In case script running
    }
//    UDPLib_Close();
    exit(0);
}

/* ---------------------------------- */
int main(int argc, char *argv[])
{
    GLenum glew_status;
    
    BEGIN_IOSLink();
    BEGIN_NavLib();  /* must go before BEGIN_Map */
    BEGIN_Map();
    BEGIN_Approach();
    BEGIN_Plot();
    BEGIN_Script();
    BEGIN_Maths();
    BEGIN_PngLib();
    BEGIN_Gui();
	BEGIN_IOLib();
//    BEGIN_UDPLib();

    FrameNumber         = 0;
    t0                  = glfwGetTime();
    NumberOfSteps       = 0;
    glfwSetErrorCallback(error_callback);

//    UDPLib_Connect(1, &IosLink_IOPkt1, sizeof(IosLink_IOPkt1));
//    UDPLib_Connect(2, &IosLink_IOPkt2, sizeof(IosLink_IOPkt2));
//    UDPLib_Connect(3, &IosLink_AeroPkt, sizeof(IosLink_AeroPkt));
//    UDPLib_Connect(4, &IosLink_EngPkt, sizeof(IosLink_EngPkt));
//    UDPLib_Connect(5, &IosLink_NavPkt, sizeof(IosLink_NavPkt));
//    UDPLib_Connect(10, &IosLink_ProtoPkt, sizeof(IosLink_ProtoPkt));

//    UDPLib_Open(6);  /* IOS node = 6 */

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    window = glfwCreateWindow(Glib_SCREENWIDTH, Glib_SCREENHEIGHT, "IOS", glfwGetPrimaryMonitor(), NULL);
//    window = glfwCreateWindow(Glib_SCREENWIDTH, Glib_SCREENHEIGHT, "IOS", NULL, NULL);
    
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwRestoreWindow(window);
    glfwSwapInterval(0);

    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, Gui_Mouse_Button_Callback);
    glfwSetCursorPosCallback(window, Gui_Mouse_Cursor_Callback);
    glfwSetScrollCallback(window, Gui_Mouse_Scroll_Callback);

    glew_status = glewInit();
    if (GLEW_OK != glew_status) 
    {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
        exit(-1);
    }

    if (!GLEW_VERSION_2_0) 
    {
        fprintf(stderr, "No support for OpenGL 2.0 found\n");
        exit(-1);
    }

    appSetupRenderingContext();
    Glib_Info();
    Glib_Init();
    Glib_Errors();
    
    #ifdef _WIN32
      Glib_LoadFont("c:/windows/fonts/arial.ttf", 1, 12);  /* font no. 1 */
      Glib_LoadFont("c:/windows/fonts/arial.ttf", 2, 24);  /* font no. 2 */
    #else
      Glib_LoadFont("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf", 1, 12);
      Glib_LoadFont("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf", 2, 24);
    #endif
    
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT20, 20);
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT30, 32);

    //Glib_LoadTexture("Textures/mosaicV4.png", 1);
    Glib_LoadTexture("Textures/MapSymbols32px.png", 2);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    Glib_AntiAliasing(true);

    while (!glfwWindowShouldClose(window))
    {
        Diagnostics_SetTimer1(); /* ***************************** */
        NumberOfSteps += 1;

        Update();
       
        glViewport(0, 0, Glib_SCREENWIDTH, Glib_SCREENHEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);

        Glib_LoadIdentity();
        Glib_SetTexture(2);
        Diagnostics_SetTimer2(); /* ***************************** */
        Display();
        
        Diagnostics_SetTimer3(); /* ***************************** */
        if (DIAGNOSTICS)
        {
             Diagnostics_Diagnostics(0, 10, 750);
        }
        Glib_Flush();  /* process any pending objects */

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    Glib_Close();
    glfwDestroyWindow(window);
    glfwTerminate();
}
