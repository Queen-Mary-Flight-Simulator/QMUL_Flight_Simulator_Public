#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <GLFW/glfw3.h>
#include <SIM/glib.h>

#define  SCREENWIDTH  768
#define  SCREENHEIGHT 1024

GLFWwindow*  window;
float        pointer = 0.0;
float        dpointer = 1.0;

void         key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void         error_callback(int error, const char* description);
void         Update();
void         Display();

/* ---------------------------------- */
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
        exit(1);
    }
}

/* ---------------------------------- */
void Update()
{
    pointer += dpointer;
	if (pointer > 160.0 || pointer < -160.0)
	{
	    dpointer = -dpointer;
	}
}

/* ---------------------------------- */
void Display()
{
    int a;
	float f;
	float x1, y1;
	float x2, y2;
	
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1.0, 1.0, 1.0);

    for (a = -160; a <= 160; a+=20)
    {
	    f = (float) a * M_PI / 180.0;
		x1 = 400.0 + sin(f) * 250.0; 
		y1 = 400.0 + cos(f) * 250.0; 
		x2 = 400.0 + sin(f) * 300.0; 
		y2 = 400.0 + cos(f) * 300.0;
        Glib_Draw(x1, y1, x2, y2); 
    }  

    f = (float) pointer * M_PI / 180.0;
    x1 = 400.0;
	y1 = 400.0;
	x2 = 400.0 + sin(f) * 300.0;
	y2 = 400.0 + cos(f) * 300.0;
	Glib_Draw(x1, y1, x2, y2);
}

/* ---------------------------------- */
int main(int argc, char *argv[])
{
    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
    
    window = glfwCreateWindow(SCREENWIDTH, SCREENHEIGHT, "test", 0, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window, key_callback);

    glViewport(0, 0, SCREENWIDTH, SCREENHEIGHT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, (float) SCREENWIDTH, 0.0, (float) SCREENHEIGHT, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	
	glLineWidth(4.0);
	
    while (!glfwWindowShouldClose(window))
    {
        Update();
		Display();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    
    return 0;
}
