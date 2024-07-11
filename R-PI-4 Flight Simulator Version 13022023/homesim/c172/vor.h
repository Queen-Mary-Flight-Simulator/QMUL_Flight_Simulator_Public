#ifndef Vor_H
#define Vor_H

#include "GL/gl.h"

#include <stdbool.h>

extern void Vor_Vor(int vorX, int vorY, float Localiser, float Glideslope, int Crs, bool ILSMode, bool Status,
                    GLuint texobj1, GLuint texobj2, GLuint texobs);

extern void BEGIN_Vor();

#endif
