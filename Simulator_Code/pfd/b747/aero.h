#ifndef Aero_H
#define Aero_H

#include <math.h>

#include <SIM/aerodefn.h>

#define ONERAD                   (180.0 / M_PI)

#define Aero_s                   510.95
#define Aero_b                   59.74
#define Aero_CBar                8.321
#define Aero_Cg_Gear_Arm         3.0
#define Aero_Gear_Tail_Arm       33.0
#define Aero_Gear_Offset         5.5
#define Aero_AcHeight            (-8.76)
#define Aero_CGHeight            (-5.1816)  /* 17 ft. previous values: (-4.05) (-4.78) */
#define Aero_EyeXStation         25.19
#define Aero_EyeYStation         0.51
#define Aero_EyeZStation         (Aero_AcHeight - Aero_CGHeight)
#define Aero_WingIncidence       0.035
#define Aero_ElevatorGain        (23.0 / ONERAD)
#define Aero_AileronGain         (40.0 / ONERAD)
#define Aero_RudderGain          (25.0 / ONERAD)
#define Aero_TillerGain          (75.0 / ONERAD)
#define Aero_ElevatorTrimGain    (20.0 / ONERAD)
#define Aero_AileronTrimGain     (40.0 / ONERAD)
#define Aero_RudderTrimGain      (25.0 / ONERAD)
#define Aero_AutoPilot           true

#define MaxLines                 20
#define MaxPts                   100

typedef struct
{
    unsigned int np;
    float        xa[MaxPts];
    float        ya[MaxPts];
    float        y2[MaxPts];
} SData;

typedef struct
{
    unsigned int nLines;
    float        Minz;
    float        Maxz;
    SData        Lines[MaxLines];
} SplineData;

extern float Aero_Ixx, Aero_Iyy, Aero_Izz;
extern float Aero_Ixz, Aero_Iyz, Aero_Ixy;
extern float Aero_Mass;
extern float Aero_CgPosition;

extern float Lookup1(SplineData s, unsigned int n, float x);
extern float Lookup2(SplineData s, float var1, float var2);

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
