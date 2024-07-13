#ifndef Compass_NFD_H
#define Compass_NFD_H

#include <stdbool.h>
#include <SIM/navdefn.h>

extern void Compass_Compass(int CompassX, int CompassY, float Hdg, float Bearing, float Track, unsigned int CrsPointer,
                            unsigned int HdgBug, unsigned int Range, bool ILSMode,
                            NavDefn_FCUMode Mode);

extern void Compass_Rmi(int CompassX, int CompassY, float Rmi1, NavDefn_FCUNav n1, float Rmi2, NavDefn_FCUNav n2);

extern void Compass_ExpandedRmi(int CompassX, int CompassY, float Rmi1, NavDefn_FCUNav n1, float Rmi2, NavDefn_FCUNav n2);

extern void Compass_GlideSlope(int CompassX, int CompassY, float GlideSlopeError, bool ILSMode, bool Status);

extern void Compass_ExpandedCompass(int CompassX, int CompassY, float Hdg, float Bearing, float Track, 
                                    unsigned int CrsPointer, unsigned int HdgBug, 
                                    unsigned int Range, bool ILSMode, 
                                    NavDefn_FCUMode Mode);

extern void Compass_DisplayPlan(int CompassX, int CompassY, unsigned int Range);

extern void Compass_Wrn(int x, int y, unsigned int n, unsigned int w, bool LeadingZeros);

extern void BEGIN_Compass();

#endif
