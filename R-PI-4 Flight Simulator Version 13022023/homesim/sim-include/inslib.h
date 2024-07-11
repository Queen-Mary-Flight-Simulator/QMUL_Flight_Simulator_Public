#ifndef InsLib_H
#define InsLib_H

#include <stdbool.h>

extern bool InsLib_Active;

extern void InsLib_Align(double a_latitude, double a_longitude, double a_altitude, double a_Vn, double a_Ve, double a_Vd, double a_pitch, double a_roll, double a_yaw);

extern void InsLib_Update(double xb, double yb, double zb, double p, double q, double r);

extern void InsLib_GetData(double *a_latitude, double *a_longitude, double *a_altitude, double *a_pitch, double *a_roll, double *a_yaw);

extern void BEGIN_InsLib();

#endif
