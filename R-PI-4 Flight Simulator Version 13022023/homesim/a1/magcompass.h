#ifndef MagCompass_H
#define MagCompass_H

#include <stdbool.h>
#include <GL/gl.h>

#define MagCompass_Height 320
#define MagCompass_Width  320

extern void MagneticCompass_Compass(int x0, int y0, float, GLuint, GLuint);

extern void BEGIN_MagneticCompass();

#endif
