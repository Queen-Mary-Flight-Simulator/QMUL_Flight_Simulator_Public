/* +------------------------------+---------------------------------+
   | Module      : nfd.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-08      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Navigation Flight Display (NFD)                  |
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
#include <SIM/glib.h>
#include <SIM/pnglib.h>
#include <SIM/navdefn.h>
#include <SIM/navlib.h>

#include "navlink.h"
#include "nfd-compass.h"
#include "nav.h"
#include "fcu.h"
#include "navinfo.h"
#include "panel.h"
#include "nfd.h"

#include "diagnostics.h"

#define DIAGNOSTICS 1

NavDefn_FCUMode    NFD_NavDisplayMode;
unsigned int       NFD_NavDisplayRange;

static GLFWwindow* window;

static int       MouseX;
static int       MouseY;
static bool      MouseLeftButton;
static bool      MouseMiddleButton;
static bool      MouseRightButton;
unsigned int     NumberOfSteps;
double           t0;

void appSetupRenderingContext(void);
void Display();
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
    float TAS = NavLink_AeroPkt.U;
    float Gspeed = sqrt((float) NavLink_AeroPkt.Vn * (float) NavLink_AeroPkt.Vn +
                        (float) NavLink_AeroPkt.Ve * (float) NavLink_AeroPkt.Ve);

    NumberOfSteps      = NumberOfSteps + 1;
    Nav_Track          = NavLink_AeroPkt.Yaw - Nav_MagneticVariation;
	//printf("MagVar=%f Yaw=%f \n",Nav_MagneticVariation,NavLink_AeroPkt.Yaw);
    NFD_NavDisplayMode = FCU_ModeSelector;

    switch (NFD_NavDisplayMode)
    {
        case NavDefn_ModeNAV:
        case NavDefn_ModeVOR:
        case NavDefn_ModeILS:
            Compass_Compass(NFD_CompassX, NFD_CompassY, NavLink_AeroPkt.Yaw - 				Nav_MagneticVariation,
                            Nav_HSI_Localiser, Nav_Track, Nav_HSI_Crs, FCU_HDG,
                            FCU_RangeSelector, Nav_HSI_ILSMode, NFD_NavDisplayMode);

            Compass_Rmi(NFD_CompassX, NFD_CompassY, Nav_Rmi_Dir1, FCU_NavSwitch1, Nav_Rmi_Dir2, FCU_NavSwitch2);
            if (NFD_NavDisplayMode == NavDefn_ModeILS)
            {
                Compass_GlideSlope(NFD_CompassX, NFD_CompassY, Nav_HSI_GlideSlope, Nav_HSI_ILSMode, true);
            }
            break;

        case NavDefn_ModeARC:
            Glib_ClipWindow(NFD_xCompassX - 294, NFD_xCompassY - 106,
                            294 * 2, Glib_SCREENWIDTH - 1 - NFD_xCompassY + 118);
            Compass_ExpandedCompass(NFD_xCompassX, NFD_xCompassY, NavLink_AeroPkt.Yaw - Nav_MagneticVariation,
                                    Nav_HSI_Localiser, Nav_Track, Nav_HSI_Crs, FCU_HDG,
                                    FCU_RangeSelector, Nav_HSI_ILSMode, NFD_NavDisplayMode);
            Compass_ExpandedRmi(NFD_xCompassX, NFD_xCompassY, Nav_Rmi_Dir1, FCU_NavSwitch1, Nav_Rmi_Dir2, FCU_NavSwitch2);
            Glib_RemoveClipWindow();
            break;

        case NavDefn_ModePLAN:
            Compass_DisplayPlan(NFD_CompassX, NFD_CompassY, FCU_RangeSelector);
            break;

        default:
            break;
    }
    
    Glib_LoadIdentity();  /* reset to absolute coords */

    NavInfo_UpdateLeftNavInfo(FCU_NavSwitch1);
    NavInfo_UpdateRightNavInfo(FCU_NavSwitch2);
    NavInfo_UpdateTopNavInfo(NFD_NavDisplayMode);
    NavInfo_UpdateWindVector((float) NavLink_AeroPkt.WindSpeed, (float) NavLink_AeroPkt.WindDir);
    NavInfo_UpdateGS(Gspeed);
    NavInfo_UpdateTAS(TAS);

    Panel_CheckPanel();   /* must be the last call */
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
        //printf("Frames: %d\n", NumberOfSteps);
        //printf("Time: %f\n", etime);
        //printf("Frame Rate: %f fps\n", (float) NumberOfSteps / etime);
        glfwSetWindowShouldClose(window, GL_TRUE);
        exit(1);
    }
    else if (key == GLFW_KEY_P && action == GLFW_PRESS)
    {
        PngLib_SavePngFile("temp.png", 1920/2, 0, 1920/2, 1080);
    }
    else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        switch (NFD_NavDisplayMode)
        {
            case NavDefn_ModeILS:
                FCU_ModeSelector = NavDefn_ModeVOR;
                break;
            case NavDefn_ModeVOR:
                FCU_ModeSelector = NavDefn_ModeNAV;
                break;
            case NavDefn_ModeNAV:
                FCU_ModeSelector = NavDefn_ModeARC;
                break;
            case NavDefn_ModeARC:
                FCU_ModeSelector = NavDefn_ModePLAN;
                break;
            case NavDefn_ModePLAN:
                FCU_ModeSelector = NavDefn_ModeILS;
                break;
            default:
                FCU_ModeSelector = NavDefn_ModeILS;
                break;
        }
    }
}

/* --------------------------------------------------- */
void NFD_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right)
{
    *x      = MouseX;
    *y      = MouseY;
    *left   = MouseLeftButton;
    *middle = MouseMiddleButton;
    *right  = MouseRightButton;
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
    MouseX = Glib_SCREENHEIGHT - (int) ypos; /* portrait */
    MouseY = Glib_SCREENWIDTH - (int) xpos;
//    MouseX = xpos;  /* landscape */
//    MouseY = Glib_SCREENHEIGHT - (int) ypos;
}

/* --------------------------------------------------- */
void NFD_NFDInit(NFD_PtrProc Update)
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
    
    #ifdef _WIN32  /* no graphics fonts for NFD */
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
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT20, 20);
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT30, 32);
    
    Glib_LoadTexture("Textures/mosaicv6.png", 1);

    while (!glfwWindowShouldClose(window))
    {
        NumberOfSteps = NumberOfSteps + 1;

        Diagnostics_SetTimer1(); /* ***************************** */
        Update();

        if (NavLink_Stopping)
        {
            glfwSetWindowShouldClose(window, GL_TRUE);
        
        }

        glClear(GL_COLOR_BUFFER_BIT);

        Glib_LoadIdentity();

        Glib_SetTexture(1);
        Diagnostics_SetTimer2(); /* ***************************** */
        Display();

        Diagnostics_SetTimer3(); /* ***************************** */
        if (DIAGNOSTICS)
        {
             Diagnostics_Diagnostics(0, 50, 750);
        }
        Glib_Flush();  /* process any pending objects */
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    Glib_Close();

    Glib_Errors();
    
    glfwDestroyWindow(window);

    glfwTerminate();
    exit(EXIT_SUCCESS);
}

void BEGIN_NFD()
{
    t0                = glfwGetTime();
    NumberOfSteps     = 0;

    MouseX            = 0;
    MouseY            = 0;
    MouseLeftButton   = false;
    MouseMiddleButton = false;
    MouseRightButton  = false;
}
