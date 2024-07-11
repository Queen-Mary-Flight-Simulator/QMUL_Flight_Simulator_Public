#ifndef MouseLib_H
#define MouseLib_H

#include <GLFW/glfw3.h>

extern void MouseLib_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right, float *scrollfactor);

extern void MouseLib_AttachMouse(GLFWwindow* window);

extern void BEGIN_MouseLib();

#endif
