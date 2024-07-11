#ifndef Model_H
#define Model_H

#include <stdbool.h>

#define Model_G                  9.81
#define Model_RollingFriction    0.016
#define Model_EarthRadius        6378137.0

extern double Model_Latitude, Model_Longitude;
extern double Model_dLatitude, Model_dLongitude;
extern float  Model_Pitch;
extern float  Model_Roll;
extern float  Model_Yaw;
extern float  Model_Vn, Model_Ve, Model_Vd;
extern float  Model_Pz;
extern float  Model_U, Model_V, Model_W;
extern float  Model_UDot, Model_VDot, Model_WDot;
extern float  Model_P, Model_Q, Model_R;
extern float  Model_PDot, Model_QDot, Model_RDot;
extern float  Model_Pmt, Model_Rmt, Model_Ymt;
extern float  Model_Alpha, Model_Beta;
extern float  Model_AlphaWing;
extern float  Model_AlphaDot, Model_BetaDot;
extern float  Model_Cl;
extern float  Model_ClTail;
extern float  Model_Clfw;
extern float  Model_Cd;
extern float  Model_CyBeta;
extern float  Model_CyDr;
extern float  Model_Cz0;
extern float  Model_Cz1;
extern float  Model_Cm0;
extern float  Model_CmAlpha;
extern float  Model_CmDe;
extern float  Model_CmQ;
extern float  Model_CmAlphaDot;
extern float  Model_ClBeta;
extern float  Model_ClDr;
extern float  Model_ClDa;
extern float  Model_ClP;
extern float  Model_ClR;
extern float  Model_CnBeta;
extern float  Model_CnBetaDot;
extern float  Model_CnDr;
extern float  Model_CnDa;
extern float  Model_CnP;
extern float  Model_CnR;
extern float  Model_Lift, Model_Thrust, Model_Drag, Model_SideForce;
extern float  Model_DiffBraking;
extern float  Model_GForce, Model_GMin, Model_GMax;
extern float  Model_XForce, Model_YForce, Model_ZForce;
extern float  Model_A11, Model_A12, Model_A13;
extern float  Model_A21, Model_A22, Model_A23;
extern float  Model_A31, Model_A32, Model_A33;
extern float  Model_Vc, Model_VcDot;
extern float  Model_MachNumber;
extern float  Model_Elevator;
extern float  Model_Aileron;
extern float  Model_Rudder;
extern float  Model_ElevatorTrim;
extern float  Model_AileronTrim;
extern float  Model_RudderTrim;
extern float  Model_LeftBrake, Model_RightBrake;
extern float  Model_Flaps;
extern float  Model_Gear;
extern bool   Model_Stalling;
extern bool   Model_OnTheGround;
extern float  Model_e0, Model_e1, Model_e2, Model_e3;
extern float  Model_Ex, Model_Ey, Model_Ez;

extern void   Model_FlightModel();
extern void   Model_SetDCM();
extern void   Model_SetQuarternions();
extern bool   Model_Autotrim(float * de);
extern void   Model_ResetFlightModel();
extern void   Model_Pushback(float HDG, bool reset);

extern void BEGIN_Model();
#endif
