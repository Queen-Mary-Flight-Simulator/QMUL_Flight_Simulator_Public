#ifndef Engines_H
#define Engines_H

#include <stdbool.h>

#include <SIM/engdefn.h>
#include <SIM/iodefn.h>

#define Engines_Left	0
#define Engines_Right	1
#define Engines_None	2
typedef unsigned char Engines_FuelSelector;
#define Engines_BothMags	3
#define Engines_LeftMag	1
#define Engines_RightMag	2
#define Engines_OffMag	0
typedef unsigned char Engines_MagnetoPosition;

typedef struct 
{
    float Thrust;
    float Rpm;
    float Beta;
    float ManifoldPressure;
    float FuelFlow;
    float Egt;
    float Epr;
} Engines_EngineData;

extern Engines_EngineData    Engines_Engines[4];
extern float                 Engines_EngineThrustX;
extern float                 Engines_EngineThrustY;
extern float                 Engines_EngineThrustZ;
extern float                 Engines_EnginePMT;
extern float                 Engines_EngineRMT;
extern float                 Engines_EngineYMT;
extern bool                  Engines_SingleEngineMode;
extern float                 Engines_ThrottleLever[4];
extern float                 Engines_ReverseLever[4];
extern float                 Engines_FuelQuantityLeft;
extern float                 Engines_FuelQuantityRight;
extern IODefn_SwitchPosition Engines_EngineState[4];
extern EngDefn_Propulsion    Engines_EngineType;

#define Engines_MaxFuel	250.0

extern void Engines_Update();
extern void Engines_EngineModel(bool held);
 
extern void Engines(Engines_EngineData      *Engine, 
                    float                   ThrottleLever, 
                    float                   PitchLever, 
                    float                   MixtureLever, 
                    Engines_MagnetoPosition Magneto, 
                    IODefn_SwitchPosition   Starter, 
                    bool                    Failed, 
                    bool                    FuelOut);
extern void BEGIN_Engines();

#endif
