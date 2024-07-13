#ifndef Engines_H
#define Engines_H

#include <stdbool.h>
#include <SIM/iodefn.h>
#include <SIM/engdefn.h>

#define Engines_MaxFuel 10000.0

typedef struct 
{
    float Thrust;
    float Epr;
    float Rpm;
    float FuelFlow;
    float Egt;
    float Beta;
    float ManifoldPressure;
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
extern float                 Engines_EprDemand[4];
extern IODefn_SwitchPosition Engines_EngineState[4];
extern EngDefn_Propulsion    Engines_EngineType;

extern void Engines_Update();

extern void Engines_EngineModel(bool Held);

extern void Engines_Init();

extern void BEGIN_Engines();

#endif
