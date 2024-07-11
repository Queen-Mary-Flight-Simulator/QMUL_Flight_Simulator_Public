#ifndef Alt_H
#define Alt_H

#include <stdbool.h>

extern void Alt_Altimeter(int AltX, int AltY, float z, 
                          unsigned int Baro, bool BaroHg, unsigned int BaroMode,
                          unsigned short int FCUAlt, bool Metric);

extern void Alt_Baro(unsigned int Baro, bool BaroHg, unsigned int BaroMode);

extern void Alt_RadioAltimeter(int RadAtlX, int RadAltY, float h);

extern void BEGIN_Alt();

#endif
