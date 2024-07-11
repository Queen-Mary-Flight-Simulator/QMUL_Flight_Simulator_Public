#include <stdbool.h>

#include <GLFW/glfw3.h>
#include <SIM/glib.h>
#include <SIM/mouselib.h>

int   MouseX;
int   MouseY;
bool  MouseLeftButton;
bool  MouseMiddleButton;
bool  MouseRightButton;
float MouseScrollFactor;

/* --------------------------------------------------- */
void MouseLib_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right, float *scrollfactor)
{
    *x            = MouseX;
    *y            = MouseY;
    *left         = MouseLeftButton;
    *middle       = MouseMiddleButton;
    *right        = MouseRightButton;
	*scrollfactor = MouseScrollFactor;
}

/* ---------------------------------- */
void Mouse_Button_Callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        MouseLeftButton = (action == GLFW_PRESS);
    }
    
	if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        MouseRightButton = (action == GLFW_PRESS);
    }
}

/* ---------------------------------- */
void Mouse_Cursor_Callback(GLFWwindow* window, double xpos, double ypos)
{
    MouseX = (int) xpos;
    MouseY = Glib_SCREENHEIGHT - (int) ypos;
}

/* ---------------------------------- */
void Mouse_Scroll_Callback(GLFWwindow* window, double xpos, double ypos)
{
    MouseScrollFactor -= (float) ypos * 10.0f;
    if (MouseScrollFactor < 1.0)
    {
        MouseScrollFactor = 1.0;
    }
}

/* ---------------------------------- */
void MouseLib_AttachMouse(GLFWwindow* window)
{
    glfwSetMouseButtonCallback(window, Mouse_Button_Callback);
    glfwSetCursorPosCallback(window, Mouse_Cursor_Callback);
    glfwSetScrollCallback(window, Mouse_Scroll_Callback);
}

/* ---------------------------------- */
void BEGIN_MouseLib()
{
    MouseX            = 0;
    MouseY            = 0;
    MouseLeftButton   = false;
    MouseMiddleButton = false;
    MouseRightButton  = false;
    MouseScrollFactor = 200.0;
}
