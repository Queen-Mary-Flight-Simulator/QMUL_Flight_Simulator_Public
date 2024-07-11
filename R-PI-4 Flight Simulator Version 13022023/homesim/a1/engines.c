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
float                 Engines_ThrottleLever[4];
float                 Engines_EngineThrustX;
float                 Engines_EngineThrustY;
float                 Engines_EngineThrustZ;
float                 Engines_EnginePMT;
float                 Engines_EngineRMT;
float                 Engines_EngineYMT;
bool                  Engines_SingleEngineMode;
//float                 Engines_ThrottleLever[4];
float                 Engines_ReverseLever[4];
float                 Engines_FuelQuantityLeft;
float                 Engines_FuelQuantityRight;
IODefn_SwitchPosition Engines_EngineState[4];
EngDefn_Propulsion    Engines_EngineType;

float CtTable[108] = 
  /* 0.0    0.2    0.4    0.6    0.8    1.0    1.2    1.4    1.6    1.8    2.0    2.2 J */
  {0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,  /*  0 */
   0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,  /*  5 */
   0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,  /* 10 */
   0.092, 0.084, 0.065, 0.037, 0.007,-0.022,-0.050,-0.080,-0.108,-0.137,-0.167,-0.196,  /* 15 */
   0.099, 0.096, 0.089, 0.064, 0.036, 0.006,-0.023,-0.054,-0.084,-0.115,-0.144,-0.175,  /* 20 */
   0.118, 0.114, 0.110, 0.095, 0.071, 0.043, 0.013,-0.017,-0.045,-0.073,-0.102,-0.131,  /* 25 */
   0.115, 0.110, 0.106, 0.100, 0.101, 0.080, 0.053, 0.024,-0.007,-0.038,-0.068,-0.098,  /* 30 */
   0.139, 0.132, 0.126, 0.119, 0.113, 0.108, 0.089, 0.065, 0.038, 0.010,-0.020,-0.048,  /* 35 */
   0.141, 0.135, 0.128, 0.123, 0.120, 0.114, 0.112, 0.101, 0.080, 0.057, 0.030, 0.005}; /* 40 */

float CpTable[108] = 
  /* 0.0    0.2    0.4    0.6    0.8    1.0    1.2    1.4    1.6    1.8    2.0    2.2 J */
  {0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,  /*  0 */
   0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,  /*  5 */
   0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,  /* 10 */
   0.043, 0.042, 0.037, 0.027, 0.008,-0.020,-0.065,-0.114,-0.160,-0.207,-0.255,-0.310,  /* 15 */
   0.064, 0.062, 0.058, 0.049, 0.033, 0.010,-0.024,-0.070,-0.118,-0.166,-0.213,-0.260,  /* 20 */
   0.093, 0.091, 0.088, 0.081, 0.070, 0.049, 0.020,-0.018,-0.067,-0.116,-0.163,-0.210,  /* 25 */
   0.137, 0.132, 0.128, 0.120, 0.111, 0.098, 0.073, 0.039,-0.008,-0.055,-0.103,-0.150,  /* 30 */
   0.205, 0.197, 0.187, 0.176, 0.164, 0.150, 0.130, 0.105, 0.069, 0.023,-0.028,-0.080,  /* 35 */
   0.250, 0.242, 0.233, 0.223, 0.213, 0.202, 0.190, 0.174, 0.149, 0.117, 0.071, 0.008}; /* 40 */

float BetaSpacing[9] = 
  {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0};

float JSpacing[12] =
  {0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2};

float CtValue (float beta, float j);
float CpValue (float beta, float j);
float Cval (float beta, float j, float CTable[]);
float PitchLeverProfile(float Lever);
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
    j = Maths_Limit(j, 0.0, 2.2);

    n1 = (unsigned int) (beta / 5.0);  /* 0..7 */
    if (n1 > 7) 
    {
        n1 = 7;
    }
    p1 = n1 * 12;
    p2 = p1 + 12;
    q1 = (unsigned int) (j / 0.2);  /* 0..10 */
    if (q1 > 10) 
    {
        q1 = 10;
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
    float         PitchLever[2];
    float         MixtureLever[2];
    bool          FuelOut = false;

    FuelSwitch[0] = Engines_Left;
    FuelSwitch[1] = Engines_Left;

    EngineMagneto[0] = (IOLib_GetFuelSwitch(0) == IODefn_On ? Engines_LeftMag : Engines_OffMag);
    EngineMagneto[1] = (IOLib_GetFuelSwitch(2) == IODefn_On ? Engines_RightMag : Engines_OffMag);

    EngineStart[0] = IOLib_GetStarterSwitch(0);

    Engines_ThrottleLever[0] = IOLib_GetEngineLever(0);
    
	PitchLever[0]   = 1.0;
	MixtureLever[0] = 1.0;

    FuelOut = false;
    
	Engine(&Engines_Engines[0], Engines_ThrottleLever[0], PitchLever[0], MixtureLever[0],
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
float PitchLeverProfile(float Lever)
{
  return Lever * Lever;
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
	float RpmMax;
	float BetaDot;
    float J;
    float Ct;
    float Cp;

    RpmMax = 2700.0 * PitchLeverProfile(PitchLever);
    
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
    else if (FAR <= 0.08) 
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

    EnginePowerLoss = 1.3125 * (0.0413 * Engine->Rpm * Engine->Rpm / 2700.0);
    StaticHP = 1.3125 * (MP * (0.0039 * Engine->Rpm - 1.0));
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
    Ct = 1.3125 * CtValue(Engine->Beta, J);

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
        RpmDot = -100.0;
        Engine->Egt = 0.0;
		if (PitchLever < 0.2)
		{
		    RpmDot = RpmDot - 500.0 * (1.0 - PitchLever * 5.0);
		}
    }

    if (Engine->Rpm < 650.0 && Starter == IODefn_On && !Failed)
    {
        RpmDot = 500.0;
    }

    BetaDot = (Engine->Rpm - RpmMax) / 10.0 + RpmDot / 20.0;
    BetaDot = Maths_Limit(BetaDot, -50.0, 50.0);

    Engine->Thrust = 0.0038121 * Engine->Rpm * Engine->Rpm * Ct / Weather_DensityRatio;

    if (Model_OnTheGround) 
    {
        Engine->Thrust = Engine->Thrust * 0.7;
    }

    Engine->Rpm = Maths_Integrate(Engine->Rpm, RpmDot);
    Engine->Rpm = Maths_Limit(Engine->Rpm, 0.0, 3000.0);
    Engine->ManifoldPressure = Maths_Integrate(Engine->ManifoldPressure, MP - Engine->ManifoldPressure);
    Engine->ManifoldPressure = Maths_Limit(Engine->ManifoldPressure, 0.0, 40.0);
    Engine->Beta = Maths_Integrate(Engine->Beta, BetaDot);
    Engine->Beta = Maths_Limit(Engine->Beta, 15.0, 40.0);
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
