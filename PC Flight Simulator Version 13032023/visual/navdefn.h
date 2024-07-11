/* +------------------------------+---------------------------------+
   | Module      : navdefn.h      | Version         : 1.1           | 
   | Last Edit   : 19-11-07       | Reference Number: 03-01-01      |
   +------------------------------+---------------------------------+
   | Computer    : Viglen2                                          |
   | Directory   : /dja/nfd                                         |
   | Compiler    : gcc 3.2                                          |
   | OS          : RedHat 8.0                                       |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : navigation system data packet definition         |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#ifndef NavDefn_H
#define NavDefn_H

#include "cbool.h"

#define NavDefn_MapILS 0
#define NavDefn_MapVOR 1
#define NavDefn_MapNAV 2
#define NavDefn_MapARC 3
#define NavDefn_MapPLAN 4
typedef unsigned char NavDefn_MapModeType;

#define NavDefn_DispOFF 0
#define NavDefn_DispCSTR 1
#define NavDefn_DispWPT 2
#define NavDefn_DispVORD 3
#define NavDefn_DispNDB 4
#define NavDefn_DispARPT 5
typedef unsigned char NavDefn_DispModeType;

#define NavDefn_NavADF 0
#define NavDefn_NavOFF 1
#define NavDefn_NavVOR 2
typedef unsigned char NavDefn_NavaidModeType;

#define NavDefn_DME 0
#define NavDefn_ILS 1
#define NavDefn_VOR 2
#define NavDefn_TAC 3
#define NavDefn_NDB 4
typedef unsigned char NavDefn_BeaconMode;

typedef unsigned int NavDefn_BeaconType;

typedef struct {
    unsigned int Active;
    unsigned int Stby;
} NavDefn_RadioKnob;

typedef struct {
    NavDefn_RadioKnob NavVOR;
    NavDefn_RadioKnob NavILS;
    NavDefn_RadioKnob NavADF;
    NavDefn_RadioKnob ComHF1;
    NavDefn_RadioKnob ComHF2;
    NavDefn_RadioKnob ComVHF1;
    NavDefn_RadioKnob ComVHF2;
    NavDefn_RadioKnob ComVHF3;
    NavDefn_RadioKnob ComAM;
    unsigned short int CrsKnob;
    boolean NavGuard;
    boolean PowerSwitch;
    boolean PushSwitches[14];
    unsigned char Filler1[2];
} NavDefn_RadioPanel;

typedef struct {
    float LocaliserError;
    float GlideSlopeError;
    float SlantDistance;
    float GroundSpeed;
    float GroundDistance;
    float BearingToStation;
    float RunwayQdm;
    boolean ILSBeacon;
    boolean BeaconStatus;
    unsigned char Filler2[2];
} NavDefn_NavData;

typedef struct {
    unsigned int PktNumber;

    unsigned short int BaroPressure1;
    unsigned short int BaroPressure2;
    float MagneticVariation;
    
    float RMI_Dir1;
    float RMI_Dir2;
    
    unsigned short int CurrentRunway;
    short int RunwayQDM;
    float RunwayLatitude;
    float RunwayLongitude;
    float GroundLevel;
    
    boolean OuterMarker;
    boolean MiddleMarker;
    boolean InnerMarker;
    boolean MarkerTest;
    
    float Track;
    unsigned short int TrackRange;
    unsigned char Filler3[2];

    NavDefn_NavData NAV1;
    NavDefn_NavData NAV2;
    NavDefn_NavData ADF1;
    NavDefn_NavData ADF2;
    NavDefn_NavData ILS1;
    NavDefn_NavData WayPoint;
    
    short int HSI_Crs;
    short int HSI_Hdg;
    short int VOR_Obs;
    unsigned char Filler4[2];

    NavDefn_RadioPanel SavedRadios[2];
    
    unsigned short int FCURange;
    unsigned short int FCUBaroPressure;
    unsigned short int FCUHdg;
    unsigned short int FCUAlt;
    unsigned short int FCUSpeed;
    short int FCUVS;
    short unsigned int FCUBaroMode;

    boolean FD;
    boolean LS;
    boolean LOC;
    boolean AP1;
    boolean AP2;
    boolean ATHR;
    boolean EXPED;
    boolean APPR;
    boolean SPD_MACH;
    boolean HDG_TRK;
    
    boolean FCUBaroHg;
    boolean FCUHdgHold;
    boolean FCUAltHold;
    boolean FCUSpeedHold;
    boolean FCUVSHold;

    NavDefn_NavaidModeType NavAid1;
    NavDefn_NavaidModeType NavAid2;
    NavDefn_MapModeType MapMode;
    NavDefn_DispModeType DispMode;
    
    boolean MorseMode;
    unsigned char Filler5[2];
    char MorseString[5];
    unsigned char Filler6[3];
} NavDefn_NavDataPkt;

#endif
