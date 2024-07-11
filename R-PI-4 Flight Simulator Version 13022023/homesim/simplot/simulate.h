#ifndef Simulate_H
#define Simulate_H

#include <stdbool.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/iodefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>

#define Simulate_Off_Mode	     1
#define Simulate_Elevator_Mode	 2
#define Simulate_Aileron_Mode	 3
#define Simulate_Rudder_Mode	 4
#define Simulate_Engine_Mode	 5
#define Simulate_Spoiler_Mode	 6
#define Simulate_ParkBrake_Mode	 7
#define Simulate_LeftBrake_Mode  8
#define Simulate_RightBrake_Mode 9
#define Simulate_MaxInputs       (Simulate_RightBrake_Mode)

typedef unsigned char Simulate_InputModeType;
#define Simulate_Step_Shape     1
#define Simulate_Pulse_Shape	2
#define Simulate_Doublet_Shape	3
#define Simulate_Ramp_Shape	    4
#define Simulate_Sine_Shape     5

typedef unsigned char Simulate_InputShapeType;

typedef void (*PtrProc) (int);

typedef struct
{
    bool         active;
	unsigned int shape;
    float        delay;
    float        width;
    float        amplitude;
} InputType;

extern unsigned int       Simulate_SimSteps;
extern bool               Simulate_AutoTrimMode;
extern float              Simulate_Altitude;
extern float              Simulate_Speed;
extern float              Simulate_Heading;
extern float              Simulate_VSpeed;
extern float              Simulate_Elevator;
extern float              Simulate_Aileron;
extern float              Simulate_Rudder;
extern float              Simulate_EngineLever;
extern bool               Simulate_IAS;
extern float              Simulate_Spoiler;
extern float              Simulate_ParkBrake;
extern float              Simulate_LeftBrake;
extern float              Simulate_RightBrake;
extern float              Simulate_Latitude;
extern float              Simulate_Longitude;
extern IosDefn_PlaybackDataPktRecord *Simulate_FlightDataBlk;
extern int                Simulate_FlightDataSize;
extern float              Simulate_Distance;
extern float              Simulate_Mass;

extern unsigned short int Simulate_FCU_HDG;
extern unsigned short int Simulate_FCU_ALT;
extern unsigned short int Simulate_FCU_SPD;
extern short int          Simulate_FCU_VS;
extern bool               Simulate_FCU_HDG_Hold;
extern bool               Simulate_FCU_ALT_Hold;
extern bool               Simulate_FCU_SPD_Hold;
extern bool               Simulate_FCU_VS_Hold;

extern void Simulate_Simulate();
extern void Simulate_SetInputs(unsigned int input, unsigned int shape, float delay, float period, float ampltitude);
extern void Simulate_PrintOutputs(int n);
extern void BEGIN_Simulate();
#endif
