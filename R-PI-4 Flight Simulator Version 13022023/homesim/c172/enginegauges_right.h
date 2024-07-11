#ifndef FuelGauge_H
#define FuelGauge_H

#include <GL/gl.h>

extern void EngineGauges_LFuelQty(int x0, int y0, float fuelq, GLuint texobj_gauge, GLuint texobj_needle);
extern void EngineGauges_RFuelQty(int x0, int y0, float fuelq, GLuint texobj_gauge, GLuint texobj_needle);
extern void EngineGauges_FuelPrs(int x0, int y0, float rpm, GLuint texobj_gauge, GLuint texobj_needle);
extern void EngineGauges_OilPrs(int x0, int y0, float rpm, GLuint texobj_gauge, GLuint texobj_needle);
extern void EngineGauges_OilTemp(int x0, int y0, float rpm, float Vc, GLuint texobj_gauge, GLuint texobj_needle);
extern void BEGIN_FuelGauge(int x0, int y0, float fuelq);

extern void BEGIN_EngineGauges_right();

#endif
