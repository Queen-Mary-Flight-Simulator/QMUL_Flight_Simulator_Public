#ifndef Aero_H
#define Aero_H

#include <SIM/aerodefn.h>

#define ONERAD (180.0 / M_PI)

#define Aero_AutoPilot false

#define Aero_s	16.17
#define Aero_b	10.92
#define Aero_CBar	1.48
#define Aero_OuterEngineArm	0.0
#define Aero_InnerEngineArm	0.0
#define Aero_AcHeight	(-2.0)
#define Aero_WingIncidence	0.026
#define Aero_Cg_Gear_Arm	0.20
#define Aero_Gear_Tail_Arm	4.2
#define Aero_Gear_Offset	1.1
#define Aero_CGHeight	(-1.0)
#define Aero_AerialHeight	(-0.5)
#define Aero_EyeXStation	3.0
#define Aero_EyeYStation	(-0.5)
#define Aero_EyeZStation	(Aero_AcHeight - Aero_CGHeight)
#define Aero_ElevatorGain	(9.0 / ONERAD)
#define Aero_AileronGain	(14.0 / ONERAD)
#define Aero_RudderGain	(10.0 / ONERAD)
#define Aero_ElevatorTrimGain	(9.0 / ONERAD)
#define Aero_AileronTrimGain	(14.0 / ONERAD)
#define Aero_RudderTrimGain	(10.0 / ONERAD)
#define Aero_EngineType	AeroDefn_Piston
#define Aero_NumberOfEngines	1
extern float Aero_Ixx, Aero_Iyy, Aero_Izz;
extern float Aero_Ixz, Aero_Ixy, Aero_Iyz;
extern float Aero_Mass;
extern float Aero_CgPosition;

extern float Aero_AeroMaxAlpha();
extern float Aero_AeroCz0();
extern float Aero_AeroCz1();
extern float Aero_AeroCl();
extern float Aero_AeroClTail();
extern float Aero_AeroClfw();
extern float Aero_AeroCd();
extern float Aero_AeroCyBeta();
extern float Aero_AeroCyDr();
extern float Aero_AeroCm0();
extern float Aero_AeroCmAlpha();
extern float Aero_AeroCmDe();
extern float Aero_AeroCmQ();
extern float Aero_AeroCmAlphaDot();
extern float Aero_AeroClBeta();
extern float Aero_AeroClDr();
extern float Aero_AeroClDa();
extern float Aero_AeroClP();
extern float Aero_AeroClR();
extern float Aero_AeroCnBeta();
extern float Aero_AeroCnBetaDot();
extern float Aero_AeroCnDr();
extern float Aero_AeroCnDa();
extern float Aero_AeroCnP();
extern float Aero_AeroCnR();
extern void BEGIN_Aero();

#endif
