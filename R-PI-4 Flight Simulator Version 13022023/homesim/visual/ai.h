#ifndef Ai_H
#define Ai_H

#ifdef __cplusplus
extern "C" {
#endif

extern void Ai_AttitudeIndicator(float Pitch, float Roll, float yaw);

extern void Ai_FlightPathVector(float gamma, float beta, float gammadot, float betadot);

extern void BEGIN_Ai();

#ifdef __cplusplus
}
#endif

#endif
