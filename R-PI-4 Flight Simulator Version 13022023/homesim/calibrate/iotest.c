/* Version for Thrustmaster controls
   DJA 30 Dec 2021
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SIM/glib.h>
#include <SIM/iodefn.h>
#include <SIM/pnglib.h>
#include <SIM/jslib.h>

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
    unsigned int i;

    for (i=0; i<=31; i+=1)
    {
        IOPkt1.AnalogueData[i] = jsLib_AnalogueData[i];
    }
    IOPkt1.DigitalDataA = jsLib_DigitalDataA;
    IOPkt1.DigitalDataB = jsLib_DigitalDataB;
    IOPkt1.DigitalDataC = jsLib_DigitalDataC;
    IOPkt1.DigitalDataD = jsLib_DigitalDataD;
	//IOPkt1.Temperature  = IOLib_Temperature;

    jsLib_UpdateIO(0, 0);

}

/* -----------------------------------------------*/
int main(int argc, char *argv[])
{
    GLenum glew_status;
    
    printf("I/O test starting\n");

    BEGIN_Display();
    BEGIN_Glib();
    BEGIN_jsLib();
    
    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
    
    window = glfwCreateWindow(Glib_SCREENWIDTH, Glib_SCREENHEIGHT, "IO test", 0, NULL);
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
    Glib_Init("mosaicV4.sgi");
    Glib_Errors();
    
    Glib_LoadFont("../fonts/B612-Bold.ttf",  Glib_EFONT12, 16);
    Glib_SetFont(Glib_EFONT12, 6);
	Glib_LoadTexture("diag.png", 1);
    
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
