#ifndef Ai_H
#define Ai_H

#include <stdbool.h>

extern void Ai_AttitudeIndicator(int AiX, int AiY, float Pitch, float Roll, float Slip);

extern void Ai_FlightDirector(int x, int y, bool Enabled);

extern void Ai_GlideSlope(float GlideSlopeError, bool ILSMode, bool Selected);

extern void Ai_Localiser(float LocaliserError, bool ILSMode, bool Selected);

extern void Ai_Markers(bool OMLamp, bool MMLamp, bool IMLamp, bool LampTest);

extern void BEGIN_Ai();

#endif
