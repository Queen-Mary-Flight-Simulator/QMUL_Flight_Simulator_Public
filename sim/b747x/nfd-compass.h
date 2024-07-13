#ifndef Compass_NFD_H
#define Compass_NFD_H

#include <stdbool.h>
#include <SIM/navdefn.h>

extern void Compass_Compass(float Hdg, float Bearing, float Track, unsigned int CrsPointer,
                            unsigned int HdgBug, unsigned int Range, bool ILSMode,
                            NavDefn_FCUMode Mode);

extern void Compass_Rmi(float Rmi1, NavDefn_FCUNav n1, float Rmi2, 
                        NavDefn_FCUNav n2);

extern void Compass_ExpandedRmi(float Rmi1, NavDefn_FCUNav n1, float Rmi2,
                                NavDefn_FCUNav n2);

extern void Compass_GlideSlope(float GlideSlopeError, bool ILSMode, bool Status);

extern void Compass_ExpandedCompass(float Hdg, float Bearing, float Track, 
                                    unsigned int CrsPointer, unsigned int HdgBug, 
                                    unsigned int Range, bool ILSMode, 
                                    NavDefn_FCUMode Mode);

extern void Compass_DisplayPlan(unsigned int Range);

extern void Compass_Wrn(int x, int y, unsigned int n, unsigned int w, bool LeadingZeros);

extern void BEGIN_Compass();

#endif
