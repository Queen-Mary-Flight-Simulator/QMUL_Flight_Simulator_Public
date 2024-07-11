#include <math.h>
#include <stdbool.h>
#include <string.h>

#include <SIM/engdefn.h>
#include <SIM/iodefn.h>
#include <SIM/maths.h>
#include <SIM/weather.h>

#include "ios.h"
#include "systems.h"
#include "engines.h"
#include "aerolink.h"
#include "englink.h"
#include "navlink.h"
#include "ioslink.h"
#include "iolib.h"
#include "pfd.h"
#include "sounds.h"
#include "model.h"

Engines_EngineData    Engines_Engines[4];
float                 Engines_EngineThrustX;
float                 Engines_EngineThrustY;
float                 Engines_EngineThrustZ;
float                 Engines_EnginePMT;
float                 Engines_EngineRMT;
float                 Engines_EngineYMT;
bool                  Engines_SingleEngineMode;
float                 Engines_ThrottleLever[4];
float                 Engines_ReverseLever[4];
float                 Engines_FuelQuantityLeft;
float                 Engines_FuelQuantityRight;
IODefn_SwitchPosition Engines_EngineState[4];
EngDefn_Propulsion    Engines_EngineType;

float CtTable[72] =
  /* 0.0    0.2    0.4    0.6    0.8    1.0    1.2    1.4  J */
    {0.017,-0.005,-0.040,-0.078,-0.091,-0.093,-0.097,-0.103,  /*  0 */
     0.043, 0.023,-0.010,-0.050,-0.082,-0.095,-0.103,-0.110,  /*  5 */
     0.072, 0.053, 0.023,-0.013,-0.063,-0.090,-0.113,-0.135,  /* 10 */
     0.100, 0.085, 0.059, 0.026,-0.012,-0.054,-0.100,-0.142,  /* 15 */
     0.116, 0.112, 0.094, 0.064, 0.030,-0.007,-0.051,-0.097,  /* 20 */
     0.114, 0.117, 0.116, 0.102, 0.069, 0.037, 0.000,-0.037,  /* 25 */
     0.107, 0.110, 0.113, 0.117, 0.107, 0.078, 0.047, 0.015,  /* 30 */
     0.097, 0.100, 0.103, 0.107, 0.112, 0.113, 0.093, 0.068,  /* 35 */
     0.085, 0.087, 0.092, 0.095, 0.100, 0.107, 0.114, 0.122}; /* 40 */

float CpTable[72] =
  /* 0.0    0.2    0.4    0.6    0.8    1.0    1.2    1.4  J */
    {0.015, 0.020, 0.025, 0.028, 0.020, 0.021, 0.022, 0.023,  /*  0 */
     0.014, 0.016, 0.017, 0.018, 0.012, 0.007, 0.007, 0.007,  /*  5 */
     0.021, 0.022, 0.020, 0.015, 0.008,-0.005,-0.012,-0.013,  /* 10 */
     0.036, 0.036, 0.034, 0.024, 0.012,-0.003,-0.023,-0.057,  /* 15 */
     0.062, 0.060, 0.057, 0.048, 0.022, 0.011,-0.013,-0.037,  /* 20 */
     0.097, 0.092, 0.087, 0.080, 0.064, 0.043, 0.016,-0.012,  /* 25 */
     0.141, 0.132, 0.125, 0.117, 0.108, 0.090, 0.062, 0.036,  /* 30 */
     0.198, 0.182, 0.167, 0.157, 0.149, 0.143, 0.130, 0.108,  /* 35 */
     0.261, 0.247, 0.223, 0.205, 0.195, 0.190, 0.186, 0.181}; /* 40 */

float BetaSpacing[9] =
    {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0};

float JSpacing[9] =
    {0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6};

float CtValue (float beta, float j);
float CpValue (float beta, float j);
float Cval (float beta, float j, float CTable[]);
float ThrottleLeverProfile(float Lever);
float MixtureLeverProfile(float Lever);
void Engine(Engines_EngineData     *Engine, 
            float                   ThrottleLever, 
            float                   PitchLever, 
            float                   MixtureLever, 
            Engines_MagnetoPosition Magneto, 
            IODefn_SwitchPosition   Starter, 
            bool                    Failed, 
            bool                    FuelOut);

/* --------------------------------------------- */
float CtValue(float beta, float j)
{
    return Cval(beta, j, CtTable);
}

/* --------------------------------------------- */
float CpValue(float beta, float j)
{
    return Cval(beta, j, CpTable);
}

/* --------------------------------------------- */
float Cval(float beta, float j, float CTable[])
{
    unsigned int n1, p1, p2, q1;
    float beta1, x1, y1, y2, y3, y4, y5, y6;

    beta = Maths_Limit(beta, 0.0, 40.0);
    j = Maths_Limit(j, 0.0, 1.4);

    n1 = (unsigned int) (beta / 5.0);
    if (n1 > 7) 
    {
        n1 = 7;
    }
    p1 = n1 * 8;
    p2 = p1 + 8;
    q1 = (unsigned int) (j / 0.2);
    if (q1 > 7) 
    {
        q1 = 7;
    }

    beta1 = BetaSpacing[n1];
    x1 = JSpacing[q1];
    y1 = CTable[p1 + q1];
    y2 = CTable[p2 + q1];
    y4 = CTable[p1 + q1 + 1];
    y5 = CTable[p2 + q1 + 1];
    y3 = y1 + (beta - beta1) * (y2 - y1) / 5.0;
    y6 = y4 + (beta - beta1) * (y5 - y4) / 5.0;
    return y3 - (j - x1) * (y3 - y6) / 0.2;
}

/* --------------------------------------------------- */
void Engines_Update()
{
    if (EngLink_ReplayMode)
    {
        EngLink_Replay();
    }
    EngLink_FormPacket();
    
    memcpy(&AeroLink_EngPkt, &EngLink_EngPkt, sizeof(EngDefn_EngDataPkt));
    memcpy(&NavLink_EngPkt,  &EngLink_EngPkt, sizeof(EngDefn_EngDataPkt));
    memcpy(&IosLink_EngPkt,  &EngLink_EngPkt, sizeof(EngDefn_EngDataPkt));

    Engines_EngineModel(PFD_Held);
  	SoundSystem(PFD_Held);
    EngLink_RespondToIos();
}

/* --------------------------------------------- */
void Engines_EngineModel(bool Held)
{
    unsigned char FuelSwitch[2];
    unsigned char EngineMagneto[2];
    unsigned char EngineStart[2];
    float         MixtureLever[2];
    bool          FuelOut = false;

    FuelSwitch[0] = Engines_Left;
    FuelSwitch[1] = Engines_Left;

    EngineMagneto[0] = (IOLib_GetFuelSwitch(0) == IODefn_On ? Engines_LeftMag : Engines_OffMag);
    EngineMagneto[1] = (IOLib_GetFuelSwitch(2) == IODefn_On ? Engines_RightMag : Engines_OffMag);

    EngineStart[0] = IOLib_GetStarterSwitch(0);

    Engines_ThrottleLever[0] = IOLib_GetEngineLever(0);
    MixtureLever[0] = 1.0;

    FuelOut = false;
    Engine(&Engines_Engines[0], Engines_ThrottleLever[0], 1.0, MixtureLever[0],
           EngineMagneto[0] | EngineMagneto[1], EngineStart[0], Systems_Failures[10], FuelOut);

    Engines_EngineThrustX = Engines_Engines[0].Thrust;
    Engines_EngineThrustY = 0.0;
    Engines_EngineThrustZ = 0.0;
    Engines_EnginePMT = 0.0;
    Engines_EngineRMT = 0.0;
    Engines_EngineYMT = 0.0;

    if (FuelSwitch[0] == Engines_Right) 
    {
        Engines_FuelQuantityRight = Maths_Integrate(Engines_FuelQuantityRight, -Engines_Engines[0].FuelFlow);
    } 
    else if (FuelSwitch[0] == Engines_Left) 
    {
        Engines_FuelQuantityLeft = Maths_Integrate(Engines_FuelQuantityLeft, -Engines_Engines[0].FuelFlow);
    }
    if (Engines_FuelQuantityLeft < 0.0) 
    {
        Engines_FuelQuantityLeft = 0.0;
    }
}

/* --------------------------------------------- */
float ThrottleLeverProfile(float Lever)
{
    return Lever * 2.0 - Lever * Lever;
}

/* --------------------------------------------- */
float MixtureLeverProfile(float Lever)
{
    return Lever * 2.0 - Lever * Lever;
}

/* --------------------------------------------- */
void Engine(Engines_EngineData     *Engine, 
            float                   ThrottleLever, 
            float                   PitchLever, 
            float                   MixtureLever, 
            Engines_MagnetoPosition Magneto, 
            IODefn_SwitchPosition   Starter, 
            bool                    Failed, 
            bool                    FuelOut)
{
    float MP;
    float DeltaMP;
    float PowerFactor;
    float FAR;
    float EnginePowerLoss;
    float StaticHP;
    float EnginePower;
    float PropPower;
    float Torque;
    float RpmDot;
    float J;
    float Ct;
    float Cp;

    DeltaMP = (0.04635 * ThrottleLeverProfile(ThrottleLever) - 0.0469) * Engine->Rpm;
    MP = Weather_PressureAltitude + DeltaMP;
    MP = Maths_Limit(MP, 0.0, 40.0);

    FAR = MixtureLeverProfile(MixtureLever) * Weather_DensityRatio * 0.1;
    if (FAR <= 0.0625) 
    {
        Engine->Egt = MP * 1143.0 / 28.5 + 5714.0 * FAR;
    } 
    else 
    {
        Engine->Egt = MP * 1857.0 / 28.5 - 5714.0 * FAR;
    }

    if (FAR <= 0.0625) 
    {
        PowerFactor = 64.0 * FAR - 3.2;
    } 
    else 
    if (FAR <= 0.08) 
    {
        PowerFactor = 11.4 * FAR + 0.088;
    } 
    else if (FAR <= 0.10) 
    {
        PowerFactor = 1.12 - 1.5 * FAR;
    } 
    else 
    {
        PowerFactor = 4.85 - 38.8 * FAR;
    }

    if (PowerFactor < 0.0001 || Failed || Magneto == Engines_OffMag || FuelOut) 
    {
        PowerFactor = 0.0;
    }

    EnginePowerLoss = 0.0413 * Engine->Rpm * Engine->Rpm / 2700.0;
    StaticHP = MP * (0.0039 * Engine->Rpm - 1.0);
    if (Magneto != Engines_BothMags) 
    {
        StaticHP = StaticHP * 0.95;
    }

    EnginePower = StaticHP * PowerFactor - EnginePowerLoss;

    Engine->FuelFlow = (0.235 * StaticHP + 0.0125 * Engine->Rpm - 9.69) *
                        MixtureLever * 0.000225;
    if (Engine->FuelFlow < 0.0 || PowerFactor <= 0.0001) 
    {
        Engine->FuelFlow = 0.0;
    }

    if (Engine->Rpm < 300.0) 
    {
        J = 16.9 * 1.9440 * Model_Vc / 300.0;
    } 
    else 
    {
        J = 16.9 * 1.9440 * Model_Vc / Engine->Rpm;
    }

    Cp = CpValue(Engine->Beta, J);
    Ct = CtValue(Engine->Beta, J);

    PropPower = Cp * Engine->Rpm * Engine->Rpm * Engine->Rpm / 6430041.0 / Weather_DensityRatio;
    if (Engine->Rpm < 300.0) 
    {
        Torque = (EnginePower - PropPower) * 7120.91 / 300.0;
    } 
    else 
    {
        Torque = (EnginePower - PropPower) * 7120.91 / Engine->Rpm;
    }

    RpmDot = Torque * 5.0;
    RpmDot = Maths_Limit(RpmDot, -500.0, 500.0);

    if (PowerFactor <= 0.0001) 
    {
        RpmDot = -1000.0;
        Engine->Egt = 0.0;
    }

    if (Engine->Rpm < 650.0 && Starter == IODefn_On && !Failed)
    {
        RpmDot = 500.0;
    }

    Engine->Thrust = 0.0038121 * Engine->Rpm * Engine->Rpm * Ct / Weather_DensityRatio;

    if (Model_OnTheGround) 
    {
        Engine->Thrust = Engine->Thrust * 0.7;
    }

    Engine->Rpm = Maths_Integrate(Engine->Rpm, RpmDot);
    Engine->Rpm = Maths_Limit(Engine->Rpm, 0.0, 3000.0);
    Engine->ManifoldPressure = Maths_Integrate(Engine->ManifoldPressure, MP - Engine->ManifoldPressure);
    Engine->ManifoldPressure = Maths_Limit(Engine->ManifoldPressure, 0.0, 40.0);
    Engine->Beta = 20.0;
}

/* --------------------------------------------- */
void BEGIN_Engines()
{
    Engines_SingleEngineMode = false;
    
	Engines_Engines[0].Thrust           = 0.0;
	Engines_Engines[0].Rpm              = 0.0;
	Engines_Engines[0].ManifoldPressure = 0.0;
	Engines_Engines[0].FuelFlow         = 0.0;
	Engines_Engines[0].Egt              = 0.0;
    Engines_Engines[0].Beta             = 20.0;

	Engines_ThrottleLever[0]            = 0.0;
	Engines_EngineState[0]              = IODefn_Off;
    
    Engines_FuelQuantityLeft  = 220.0;
    Engines_FuelQuantityRight = 220.0;
    Engines_EngineThrustX     = 0.0;
    Engines_EngineThrustY     = 0.0;
    Engines_EngineThrustZ     = 0.0;
    Engines_EnginePMT         = 0.0;
    Engines_EngineRMT         = 0.0;
    Engines_EngineYMT         = 0.0;
    Engines_EngineType        = EngDefn_Piston;
}
