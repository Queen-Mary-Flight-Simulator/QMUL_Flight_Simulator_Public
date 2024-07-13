/* 
+-------------------------------+---------------------------------+
| Module      : navdefn.h       | Version         : 1.1           |
| Last Edit   : 13-01-17        | Reference Number: 03-01-01      |
+-------------------------------+---------------------------------+
| Computer    : DELL5                                             |
| Directory   : /c/mingw/include/sim/                             |
| Compiler    : gcc 5.3.0                                         |
| OS          : Windows10                                         |
+-----------------------------------------------------------------+
| Authors     : D J Allerton                                      |
|             :                                                   |
+-----------------------------------------------------------------+
| Description : navigation system data packet definition          |
|                                                                 |
+-----------------------------------------------------------------+
| Revisions   : none                                              |
|                                                                 |
+-----------------------------------------------------------------+ */

#pragma pack(push,2)

#ifndef NavDefn_H
#define NavDefn_H

#include <stdbool.h>

#define NavDefn_ModeILS  0
#define NavDefn_ModeVOR  1
#define NavDefn_ModeNAV  2
#define NavDefn_ModeARC  3
#define NavDefn_ModePLAN 4
typedef unsigned char    NavDefn_FCUMode;

#define NavDefn_DataOFF  0
#define NavDefn_DataCSTR 1
#define NavDefn_DataWPT  2
#define NavDefn_DataVORD 3
#define NavDefn_DataNDB  4
#define NavDefn_DataARPT 5
typedef unsigned char    NavDefn_FCUData;

#define NavDefn_NavADF   0
#define NavDefn_NavOFF   1
#define NavDefn_NavVOR   2
typedef unsigned char    NavDefn_FCUNav;

#define NavDefn_DME      0
#define NavDefn_ILS      1
#define NavDefn_VOR      2
#define NavDefn_TAC      3
#define NavDefn_NDB      4
typedef unsigned char    NavDefn_BeaconMode;

typedef unsigned int     NavDefn_BeaconType;

typedef struct
{
    unsigned int Active;
    unsigned int Stby;
} NavDefn_RadioKnob;

typedef struct
{
    NavDefn_RadioKnob  ComVHF1;
    NavDefn_RadioKnob  ComVHF2;
    NavDefn_RadioKnob  ComVHF3;
    NavDefn_RadioKnob  ComHF1;
    NavDefn_RadioKnob  ComHF2;
    NavDefn_RadioKnob  ComAM;
    NavDefn_RadioKnob  NavVOR;
    NavDefn_RadioKnob  NavILS;
    NavDefn_RadioKnob  NavMLS;
    NavDefn_RadioKnob  NavADF;
    unsigned short int CrsKnob;
    bool               NavGuard;
    bool               PowerSwitch;
    bool               PushSwitches[14];
    unsigned short int Mode;
} NavDefn_RadioPanel;

typedef struct
{
    float         LocaliserError;
    float         GlideSlopeError;
    float         SlantDistance;
    float         GroundSpeed;
    float         GroundDistance;
    float         BearingToStation;
    float         RunwayQdm;
    bool          ILSBeacon;
    bool          BeaconStatus;
    unsigned char Filler1[2];
} NavDefn_NavData;

typedef struct
{
    unsigned int           PktNumber;

    unsigned short int     BaroPressure1;
    unsigned short int     BaroPressure2;
    float                  MagneticVariation;

    float                  RMI_Dir1;
    float                  RMI_Dir2;

    unsigned short int     CurrentRunway;
    float                  RunwayQDM;
    float                  RunwayLatitude;
    float                  RunwayLongitude;
    float                  GroundLevel;

    bool                   OuterMarker;
    bool                   MiddleMarker;
    bool                   InnerMarker;
    bool                   MarkerTest;

    float                  Track;
    unsigned short int     TrackRange;

    NavDefn_NavData        NAV1;
    NavDefn_NavData        NAV2;
    NavDefn_NavData        ADF1;
    NavDefn_NavData        ADF2;
    NavDefn_NavData        ILS1;
    NavDefn_NavData        WayPoint;

    unsigned int           MorseChannel;
    char                   MorseIdent[4];
	
    short int              HSI_Crs;
    short int              HSI_Hdg;
    short int              VOR_Obs;
    unsigned char          Filler3[2];

    NavDefn_RadioPanel     SavedRadios[1];

    unsigned short int     FCU_BaroPressure;
    unsigned short int     FCU_HDG;
    unsigned short int     FCU_ALT;
    unsigned short int     FCU_SPD;
    short int              FCU_VS;

    bool                   FCU_FD;
    bool                   FCU_LS;
    bool                   FCU_LOC;
    bool                   FCU_AP1;
    bool                   FCU_AP2;
    bool                   FCU_ATHR;
    bool                   FCU_EXPED;
    bool                   FCU_APPR;
    bool                   FCU_SPD_MACH;
    bool                   FCU_HDG_TRK;

    bool                   FCU_BaroHg;
    bool                   FCU_BaroSTD;
    bool                   FCU_HDG_Hold;
    bool                   FCU_ALT_Hold;
    bool                   FCU_SPD_Hold;
    bool                   FCU_VS_Hold;
    bool                   FCU_Metric_ALT;
	
    NavDefn_FCUNav         NavAid1;
    NavDefn_FCUNav         NavAid2;
    NavDefn_FCUMode        Mode;
    NavDefn_FCUData        Data;

    unsigned char          Filler4;
} NavDefn_NavDataPkt;
#endif

#pragma pack(pop)
