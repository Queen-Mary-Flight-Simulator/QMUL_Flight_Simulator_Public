/* 
+------------------------------+---------------------------------+
| Module      : iodefn.h       | Version         : 1.1           |
| Last Edit   : 13-01-17       | Reference Number: 01-01-01      |
+------------------------------+---------------------------------+
| Computer    : RPi1 & 2                                         |
| Directory   : /usr/include/SIM/                                |
| Compiler    : gcc 4.9.2                                        |
| OS          : Raspbian                                         |
+----------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
+----------------------------------------------------------------+
| Description : input/output data packet definition              |
|                                                                |
+----------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#pragma pack(push,2)

#ifndef IODefn_H
#define IODefn_H

#define IODefn_GearOff     0
#define IODefn_GearUp      1
#define IODefn_GearDown    2

typedef unsigned char   IODefn_GearSelector;

#define IODefn_Off    0
#define IODefn_On     1

typedef unsigned char   IODefn_SwitchPosition;

#define IODefn_ElevatorTrimOff          0
#define IODefn_ElevatorTrimForwards     1
#define IODefn_ElevatorTrimBackwards    2

typedef unsigned char   IODefn_ElevatorTrimSwitchPosition;

#define IODefn_AileronTrimOff              0
#define IODefn_AileronTrimRightWingDown    1
#define IODefn_AileronTrimLeftWingDown     2

typedef unsigned char   IODefn_AileronTrimSwitchPosition;

#define IODefn_RudderTrimOff               0
#define IODefn_RudderTrimLeft              1
#define IODefn_RudderTrimRight             2

typedef unsigned char   IODefn_RudderTrimSwitchPosition;

#define IODefn_ATOff          0
#define IODefn_ATForwards     1
#define IODefn_ATBackwards    2

typedef unsigned char   IODefn_ATDirection;

#define IODefn_APLOff         0
#define IODefn_APLNoseDown    1
#define IODefn_APLNoseUp      2

typedef unsigned char   IODefn_APLSwitchPosition;

typedef struct
{
    unsigned int       PktNumber;

    short unsigned int AnalogueData[32];
    unsigned char      DigitalDataA;
    unsigned char      DigitalDataB;
    unsigned char      DigitalDataC;
    unsigned char      DigitalDataD;
    
    float              Temperature;

    unsigned int       TimeStamp;
} IODefn_IODataPkt;
#endif

#pragma pack(pop)
