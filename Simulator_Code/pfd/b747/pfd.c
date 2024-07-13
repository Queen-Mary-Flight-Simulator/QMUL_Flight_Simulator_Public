/* +------------------------------+---------------------------------+
   | Module      : pfd.c          | Version         : 2.1           | 
   | Last Edit   : 02-04-2022     | Reference Number: 02-01-14      |
   +------------------------------+---------------------------------+
   | Computer    : DELL Optiplex 7010                               |
   | Directory   : /dja/homesim/pfd/b747                            |
   | Compiler    : gcc 9.3.0                                        |
   | OS          : Windows10                                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Boeing 747-400 EFIS Primary Flight Display (PFD) |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <sys/time.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SIM/iodefn.h>
#include <SIM/navdefn.h>
#include <SIM/glib.h>
#include <SIM/pnglib.h>
#include <SIM/weather.h>

#include "model.h"
#include "ai.h"
#include "alt.h"
#include "asi.h"
#include "compass.h"
#include "eicas.h"
#include "vsi.h"
#include "aero.h"
#include "fcs.h"
#include "pfd.h"
#include "systems.h"
#include "aerolink.h"
#include "iolib.h"
#include "diagnostics.h"
#include "fma.h"

#define DIAGNOSTICS 1

static GLFWwindow* window;

static int       MouseX;
static int       MouseY;
static bool      MouseLeftButton;
static bool      MouseMiddleButton;
static bool      MouseRightButton;

unsigned int     NumberOfSteps;
double           t0;

void Display();
void appSetupRenderingContext();
void error_callback(int error, const char* description);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
                            
/* ------------------------------------------------------------- */
void appSetupRenderingContext(void)
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LEQUAL);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.0);
}

/* --------------------------------------------------- */
void Display()
{
    float        GSErr;
    float        LocErr;
    bool         Beacon;
    bool         LS;
    float        IAS;
    unsigned int e;
    
    IAS           = Model_U * sqrt(Weather_DensityRatio);
    if (AeroLink_NavPkt.Mode == NavDefn_ModeILS)
    {
        GSErr  = AeroLink_NavPkt.ILS1.GlideSlopeError;
        LocErr = AeroLink_NavPkt.ILS1.LocaliserError;
        Beacon = AeroLink_NavPkt.ILS1.ILSBeacon;
        LS     = AeroLink_NavPkt.FCU_LS;
    }
    else
    {
        GSErr  = AeroLink_NavPkt.NAV1.GlideSlopeError;
        LocErr = AeroLink_NavPkt.NAV1.LocaliserError;
        Beacon = AeroLink_NavPkt.NAV1.ILSBeacon;
        LS     = AeroLink_NavPkt.FCU_LS;
    }

    Ai_AttitudeIndicator(PFD_AiX, PFD_AiY, Model_Pitch, Model_Roll, Model_SideForce / (Aero_Mass * 9.81));
    Ai_Markers(AeroLink_NavPkt.OuterMarker, AeroLink_NavPkt.MiddleMarker,
               false, AeroLink_NavPkt.MarkerTest);
    Ai_FlightDirector(FCS_FD_VBar, FCS_FD_HBar, AeroLink_NavPkt.FCU_FD);
    Ai_GlideSlope(GSErr, Beacon, LS);
    Ai_Localiser(LocErr, Beacon, LS);

    Compass_PFD_Compass(PFD_CompassX, PFD_CompassY,
                        Model_Yaw - (AeroLink_NavPkt.MagneticVariation),
                        (int) (AeroLink_NavPkt.FCU_HDG), (float) (AeroLink_NavPkt.Track));

    Asi_Asi(PFD_AsiX, PFD_AsiY, IAS, (unsigned int) (AeroLink_NavPkt.FCU_SPD), Model_UDot,
            Model_MachNumber, AeroLink_NavPkt.FCU_SPD, AeroLink_NavPkt.FCU_SPD_MACH);

    Alt_Altimeter(PFD_AltX, PFD_AltY,
                  -Model_Pz, (unsigned int) (AeroLink_NavPkt.FCU_BaroPressure),
                  AeroLink_NavPkt.FCU_BaroHg, AeroLink_NavPkt.FCU_BaroKnob,
                  AeroLink_NavPkt.FCU_ALT, AeroLink_NavPkt.FCU_Metric_Button);
    Alt_Baro((int) (AeroLink_NavPkt.FCU_BaroPressure), AeroLink_NavPkt.FCU_BaroHg,
             AeroLink_NavPkt.FCU_BaroKnob);

    Alt_RadioAltimeter(PFD_RadAltX, PFD_RadAltY, -Model_Pz + (float) (AeroLink_NavPkt.GroundLevel) + Aero_CGHeight);

    Vsi_Vsi(PFD_VsiX, PFD_VsiY, Model_Vd);

    FMA_FMA(PFD_FMAX, PFD_FMAY);
	
    for (e = 0; e <= 3; e += 1)
    {
        EICAS_EprGauge(PFD_EprX + e * 150, PFD_EprY, e);
        EICAS_RpmGauge(PFD_RpmX + e * 150, PFD_RpmY, e);
        EICAS_EgtGauge(PFD_EgtX + e * 150, PFD_EgtY, e);
    }

    EICAS_DisplayGear(PFD_GearX, PFD_GearY, Systems_GearPosition);
    EICAS_FlapsIndicator(PFD_FlapsX, PFD_FlapsY, Systems_FlapPosition, Systems_FlapSetting);
    EICAS_ParkBrake(PFD_ParkBrakeX, PFD_ParkBrakeY, IOLib_GetParkBrake());
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
        double t2 = glfwGetTime();
        float etime = (float) (t2 - t0);

        printf("Frames: %d\n", NumberOfSteps);
        printf("Time: %f\n", etime);
        printf("Frame Rate: %f fps\n", (float) NumberOfSteps / etime);
        glfwSetWindowShouldClose(window, GL_TRUE);
        exit(1);
    }
    else if (key == GLFW_KEY_P && action == GLFW_PRESS)
    {
        PngLib_SavePngFile("temp.png", 0, 0, 1024, 768);
    }
}

/* --------------------------------------------------- */
void PFD_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right)
{
    *x = MouseX;
    *y = MouseY;
    *left = MouseLeftButton;
    *middle = MouseMiddleButton;
    *right = MouseRightButton;
}

/* ---------------------------------- */
void Mouse_Button_Callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        MouseLeftButton = (action == GLFW_PRESS);
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        MouseRightButton = (action == GLFW_PRESS);
    }
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    {
        MouseMiddleButton = (action == GLFW_PRESS);
    }
}

/* ---------------------------------- */
void Mouse_Cursor_Callback(GLFWwindow* window, double xpos, double ypos)
{
    MouseX = (int) xpos;
    MouseY = Glib_SCREENHEIGHT - (int) ypos;
}

/* --------------------------------------------------- */
void PFD_PFDInit(PFD_PtrProc Update)
{
    GLenum glew_status;
 
    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }

    window = glfwCreateWindow(Glib_SCREENWIDTH, Glib_SCREENHEIGHT, "Display", glfwGetPrimaryMonitor(), NULL);
    
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwRestoreWindow(window);
    glfwSwapInterval(0);
    
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, Mouse_Button_Callback);
    glfwSetCursorPosCallback(window, Mouse_Cursor_Callback);

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

    #ifdef _WIN32   /* no standard fonts for PFD */
      Glib_LoadFont("c:/windows/fonts/arial.ttf", 1, 12);  /* font no. 1 */
      Glib_LoadFont("c:/windows/fonts/arial.ttf", 2, 24);  /* font no. 2 */
    #else
      Glib_LoadFont("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf", 1, 12);
      Glib_LoadFont("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf", 2, 24);
    #endif

    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT8,  11);
    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT12, 16);
    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT16, 20);
    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT24, 32);
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT20, 18);
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT30, 32);

    Glib_LoadTexture("Textures/PFD.png", 3);
	
    while (!glfwWindowShouldClose(window))
    {
        NumberOfSteps += 1;

        Diagnostics_SetTimer1(); /* ***************************** */
        Update();
        
        if (AeroLink_Stopping)
        {
            glfwSetWindowShouldClose(window, GL_TRUE);
        }

		glClear(GL_COLOR_BUFFER_BIT);

        Glib_LoadIdentity();

        Glib_SetTexture(3);
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

    glfwDestroyWindow(window);

    glfwTerminate();
    exit(EXIT_SUCCESS);
}

/* --------------------------------------------------- */
void BEGIN_PFD()
{
    t0 = glfwGetTime();

    NumberOfSteps = 0;

    MouseX = 0;
    MouseY = 0;
    MouseLeftButton = false;
    MouseMiddleButton = false;
    MouseRightButton = false;
}
