#ifndef GUI_H
#define GUI_H

#include <GLFW/glfw3.h>

extern int Gui_MouseX;
extern int Gui_MouseY;
extern bool Gui_MouseLeftButton;
extern bool Gui_MouseRightButton;
extern float Gui_Mouse_Scroll;

void Gui_Mouse_Button_Callback(GLFWwindow* window, int button, int action, int mods);
void Gui_Mouse_Cursor_Callback(GLFWwindow* window, double xpos, double ypos);
void Gui_Mouse_Scroll_Callback(GLFWwindow* window, double xpos, double ypos);
void Gui_CheckMouse(void);
void Gui_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right, float *scrollfactor);
void Gui_Menu(void);

void BEGIN_Gui(void);

#endif
