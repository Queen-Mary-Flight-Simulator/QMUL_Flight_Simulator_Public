#ifndef Alt_H
#define Alt_H

#include <stdbool.h>
#include <GL/gl.h>

extern void Alt_Altimeter(int AltX, int AltY, float z, unsigned int Baro, bool BaroHg, unsigned int BaroMode,
                          GLuint texobj_dial, GLuint texobj_dial_press, GLuint texobj_needle,
                          GLuint texobj_needle_hour, GLuint texobj_baro);

extern void Alt_Baro(unsigned int Baro, bool BaroHg, unsigned int BaroMode);

extern void BEGIN_Alt();

#endif
