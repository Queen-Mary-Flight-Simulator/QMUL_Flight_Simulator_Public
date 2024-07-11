/* 
+------------------------------+---------------------------------+
| Module      : engdefn.h      | Version         : 1.1           |
| Last Edit   : 19-13-17       | Reference Number: 02-01-01      |
|------------------------------+---------------------------------+
| Computer    : DELL4                                            |
| Directory   : /c/mingw/include/sim                             |
| Compiler    : gcc 5.4                                          |
| OS          : Windows10                                        |
|----------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
|----------------------------------------------------------------+
| Description : engine model system data packet definition       |
|                                                                |
|----------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#pragma pack(push,2)

#ifndef EngDefn_H
#define EngDefn_H

#include <stdbool.h>

#define EngDefn_UnknownEngine    0
#define EngDefn_Piston           1
#define EngDefn_Turboprop        2
#define EngDefn_Turbofan         3
#define EngDefn_Turbojet         4

typedef unsigned char   EngDefn_Propulsion;

#define EngDefn_Left     0
#define EngDefn_Right    1
#define EngDefn_None     2

typedef unsigned char   EngDefn_FuelSelector;

typedef struct
{
    float Thrust;
    float Epr;
    float Rpm;
    float FuelFlow;
    float Egt;
    float Beta;
    float ManifoldPressure;
} EngDefn_EngineData;

typedef struct
{
    unsigned int       PktNumber;

    unsigned int       NumberOfEngines;

    EngDefn_EngineData Engines[4];
    float              EngineLevers[4];
    float              ReverseLevers[4];
    float              FuelQuantityLeft;
    float              FuelQuantityRight;

    float              EngineThrustX;
    float              EngineThrustY;
    float              EngineThrustZ;
    float              EnginePMT;
    float              EngineRMT;
    float              EngineYMT;

    EngDefn_Propulsion EngineType;

    bool               EngineFireSound;

	unsigned char      DigitalDataOutA;
	unsigned char      DigitalDataOutB;
    unsigned char      Filler1[2];
	
    unsigned short int TimeOfDay;
    unsigned int       TimeStamp;
} EngDefn_EngDataPkt;
#endif

#pragma pack(pop)
