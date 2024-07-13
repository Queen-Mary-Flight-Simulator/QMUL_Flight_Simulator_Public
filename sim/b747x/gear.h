#ifndef Gear_H
#define Gear_H

#include <stdbool.h>

extern float Gear_Fx, Gear_Fy, Gear_Fz;
extern float Gear_Mx, Gear_My, Gear_Mz;

extern bool Gear_WeightOnWheels();
extern void Gear_GearModel();
extern void BEGIN_Gear();
#endif
