#ifndef Weather_H
#define Weather_H

#include <stdbool.h>

extern float Weather_Rho;
extern float Weather_Pressure;
extern float Weather_PressureAltitude;
extern float Weather_DensityRatio;
extern float Weather_Turbulence_Level;
extern float Weather_Turbulence_Intermittency;
extern float Weather_WindN, Weather_WindE;
extern unsigned int Weather_RegionalQNH;
extern float Weather_GroundTemperature;
extern float Weather_SpeedOfSound;
extern float Weather_ISADeviation;
extern float Weather_CloudBase;
extern bool  Weather_InCloud, Weather_OldInCloud;
extern bool  Weather_OutCloud, Weather_OldOutCloud;
extern float Weather_Visibility, Weather_CloudVis;
extern float Weather_UTurb, Weather_VTurb, Weather_WTurb;
extern bool  Weather_DayMode;
extern float Weather_Temperature(float h);
extern float Weather_Mach(float h, float v);
extern void Weather_SetWind(float ws, float wdir, bool reset);
extern void Weather_WeatherModel(bool turbulence, float Pz, float U);
extern void Weather_Gusts(float *ug, float *vg, float *wg, float v, float h);
extern float Weather_Mach_to_Kts(float Z, float MachNo);
extern float Weather_Mach_to_Metres(float Z, float MachNo);
extern float Weather_Metres_to_Mach(float Z, float v);

extern void BEGIN_Weather();

#endif
