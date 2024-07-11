/*  Dataview is the offline plotting of .dat files
    It is mostly based on gui.c and plot.c from the IOS modules with
        all the output code removed and menu.dat changed to fdr.dat
*/   
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SIM/maths.h>
#include <SIM/pnglib.h>
#include <SIM/glib.h>
#include <SIM/navlib.h>

#include "link.h"
#include "plot.h"
#include "gui.h"

#define diagnostics  0

static char         snapfilename[] = "snap00.png";
static unsigned int FrameNumber;
static GLFWwindow   *window;

void appSetupRenderingContext(void);
void error_callback(int error, const char* description);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void Update(void);
void Display(void);

/* ---------------------------------- */
void Update(void)
{
    IosLink_IosPkt.PktNumber = FrameNumber;
    FrameNumber += 1;

    Gui_CheckMouse();
}

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
void Display(void)
{
    glClearColor(0.83, 0.83, 0.83, 1.0); /* off white */
    glClear(GL_COLOR_BUFFER_BIT);

    Glib_LoadIdentity();

    Plot_ShowPlot();
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
        PngLib_SavePngFile(snapfilename, 0, 0, 768, 1024);
    }
}

/* ---------------------------------- */
int main(int argc, char *argv[])
{
    GLenum glew_status;

    FrameNumber = 0;

    BEGIN_Glib();
    BEGIN_Link();
    BEGIN_Plot();
    BEGIN_Maths();
    BEGIN_PngLib();
    BEGIN_Gui();

    IosLink_IOSMode = FlightDataDisplay;

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
	{
        exit(EXIT_FAILURE);
    }

    window = glfwCreateWindow(Glib_SCREENWIDTH, Glib_SCREENHEIGHT, "DATAVIEW", glfwGetPrimaryMonitor(), NULL);
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
    //Glib_LoadTexture("Textures/MapSymbols32px.png", 2);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    Glib_AntiAliasing(true);

    glViewport(0, 0, Glib_SCREENWIDTH, Glib_SCREENHEIGHT);

    while (!glfwWindowShouldClose(window))
    {
        Update();

        Display();

        Glib_Flush();  /* process any pending objects */

        glfwSwapBuffers(window);
        glfwPollEvents();
        if (IosLink_Stopping)
        {
            break;
        }
    }

    Glib_Close();
    glfwDestroyWindow(window);
    glfwTerminate();

    Plot_EndPlot();
    exit(EXIT_SUCCESS);

    return 0;
}
