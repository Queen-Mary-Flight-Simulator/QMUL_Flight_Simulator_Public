/* Version for Thrustmaster controls
   DJA 27 Aug 2021
   assumption: RPi is 192.168.1.1
   RPi samples the analogue and digital inputs and transmits a pkt
   PC reads the pkt and displays analogue data as dials and digital data as buttons
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SIM/udplib.h>
#include <SIM/glib.h>
#include <SIM/iodefn.h>
#include <SIM/pnglib.h>

#include "display.h"

IODefn_IODataPkt IOPkt1;
GLFWwindow*      window;

void         Update(void);
void         key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void         appSetupRenderingContext(void);
void         error_callback(int error, const char* description);
void         Display();

/* ---------------------------------- */
void appSetupRenderingContext(void)
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LEQUAL);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
}

/* -----------------------------------------------*/
void Update(void)
{
    unsigned int p;
    bool pkt1found = false;

    do
    {
        p = UDPLib_GetPkt();
        if (p == 1) 
        {
            pkt1found = true;
        }
    } while (!pkt1found);
}

/* -----------------------------------------------*/
int main(int argc, char *argv[])
{
    GLenum glew_status;
    
    printf("I/O test starting\n");

    BEGIN_Display();
    BEGIN_Glib();
    BEGIN_UDPLib();
    
    UDPLib_Connect(1, &IOPkt1, sizeof(IOPkt1));

    UDPLib_Open(3);  /* iotest node = 3 */
    
    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
    
    window = glfwCreateWindow(Glib_SCREENWIDTH, Glib_SCREENHEIGHT, "IO test", glfwGetPrimaryMonitor(), NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwRestoreWindow(window);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window, key_callback);

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
    
//    Glib_LoadFont("c:/windows/fonts/arial.ttf", 1, 12);  /* font no. 1 */
//    Glib_LoadFont("c:/windows/fonts/arial.ttf", 2, 24);  /* font no. 2 */
    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT12, 12);
    
//	Glib_SetFont(Glib_GFONT10, 6);
    Glib_SetFont(Glib_EFONT12, 6);
    
    while (!glfwWindowShouldClose(window))
    {
        Update();

        glViewport(0, 0, Glib_SCREENWIDTH, Glib_SCREENHEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);

        Glib_LoadIdentity();

        Display(IOPkt1);

        Glib_Flush();
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    Glib_Close();
    glfwDestroyWindow(window);
    glfwTerminate();
    UDPLib_Close();
    
    return 0;
}

/* -----------------------------------------------*/
void Display()
{
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    Display_Update(IOPkt1);
}

/* ------------------------------------------------------------- */
void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

/* ---------------------------------- */
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        if (key == GLFW_KEY_ESCAPE)
        {
            glfwSetWindowShouldClose(window, GL_TRUE);
            exit(1);
        }
        else if (key == GLFW_KEY_P)
        {
            PngLib_SavePngFile("temp.png", 0, 0, 2000/2, 1080);
        }
    }
}
