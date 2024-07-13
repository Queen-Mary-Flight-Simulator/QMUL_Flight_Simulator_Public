#ifndef Asi_H
#define Asi_H

#include <stdbool.h>

extern void Asi_Asi(int AsiX, int AsiY, float IAS, unsigned int IAS_Ref, float UDot, 
                    float GroundSpeed, float MachNumber, unsigned int BugSpeed,
                    bool MachMode);

extern void Asi_VSpeeds(float IAS, float Mass, unsigned int FlightMode);

extern void BEGIN_Asi();

#endif
