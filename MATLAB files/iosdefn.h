/* +------------------------------+---------------------------------+
   | Module      : iosdefn.h      | Version         : 1.1           | 
   | Last Edit   : 19-11-07       | Reference Number: 04-01-01      |
   +------------------------------+---------------------------------+
   | Computer    : Viglen3                                          |
   | Directory   : /dja/ios                                         |
   | Compiler    : gcc 3.2                                          |
   | OS          : RedHat 8.0                                       |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : instructor station data packet definition        |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#ifndef IosDefn_H
#define IosDefn_H

#include "aerodefn.h"
#include "navdefn.h"
#include "iodefn.h"
#include "cbool.h"

#define IosDefn_MaxFileNameSize 20

#define IosDefn_EndOfPkt 0

#define IosDefn_Exit 31
#define IosDefn_PrintScreen 32
#define IosDefn_Restore 34
#define IosDefn_Save 35
#define IosDefn_SetMode 36
#define IosDefn_Script 37
#define IosDefn_SetOctaveMode 38
#define IosDefn_LoadFlightPlan 39
#define IosDefn_SetFlightPlanMode 40

#define IosDefn_RePositionAircraft 61
#define IosDefn_MapCentre 62
#define IosDefn_MapCompass 63
#define IosDefn_MapFind 64
#define IosDefn_MapTrack 65
#define IosDefn_MapScale 66
#define IosDefn_MapReset 67
#define IosDefn_MapZoomOut 68
#define IosDefn_MapZoomIn 69

#define IosDefn_SetTurbulence 91
#define IosDefn_SetWindSpeed 92
#define IosDefn_SetWindDir 93
#define IosDefn_SetQNH 94
#define IosDefn_SetMagneticVariation 95
#define IosDefn_SetCloudbase 96
#define IosDefn_SetVisibility 97
#define IosDefn_SetGroundTemperature 98
#define IosDefn_SetTimeOfDay 99
#define IosDefn_SetGustIntermittency 100

#define IosDefn_SetAircraftAltitude 121
#define IosDefn_SetAircraftHeading 122
#define IosDefn_SetAircraftSpeed 123
#define IosDefn_SetRunHoldFreeze 124
#define IosDefn_SetRMICardType 125
#define IosDefn_SetMorseMode 126
#define IosDefn_SetSingleEngineMode 127
#define IosDefn_SetCgPosition 128
#define IosDefn_SetRightFuelQuantity 129
#define IosDefn_SetLeftFuelQuantity 130
#define IosDefn_SetAdfDip 131
#define IosDefn_SetFlightControls 132
#define IosDefn_SetAutopilotAltitude 133
#define IosDefn_SetAutopilotHeading 134
#define IosDefn_SetAutopilotSpeed 135
#define IosDefn_SetAutopilotVSpeed 136
#define IosDefn_AutopilotAltitudeOn 137
#define IosDefn_AutopilotHeadingOn 138
#define IosDefn_AutopilotSpeedOn 139
#define IosDefn_AutopilotVSpeedOn 140
#define IosDefn_AutolandOn 141

#define IosDefn_SetFlapsMode 151
#define IosDefn_SetGearMode 152
#define IosDefn_SetNav1LocaliserMode 153
#define IosDefn_SetNav1GlideSlopeMode 154
#define IosDefn_SetNav2LocaliserMode 155
#define IosDefn_SetNav2GlideSlopeMode 156
#define IosDefn_SetRMI1Mode 157
#define IosDefn_SetRMI2Mode 158
#define IosDefn_SetDMEMode 159
#define IosDefn_SetEngine1Mode 160
#define IosDefn_SetEngine2Mode 161
#define IosDefn_SetEngine3Mode 162
#define IosDefn_SetEngine4Mode 163
#define IosDefn_SetASIMode 164
#define IosDefn_SetAIMode 165
#define IosDefn_SetVSIMode 166
#define IosDefn_SetAltimeterMode 167
#define IosDefn_SetTurnMode 168
#define IosDefn_SetHSICardMode 169
#define IosDefn_SetRMICardMode 170
#define IosDefn_SetEngine1Fire 171
#define IosDefn_SetEngine2Fire 172
#define IosDefn_SetEngine3Fire 173
#define IosDefn_SetEngine4Fire 174

#define IosDefn_LoadTargetFile 181
#define IosDefn_SwitchTargetOff 182
#define IosDefn_SetTargetPosition 183
#define IosDefn_SetTargetDistance 184
#define IosDefn_SetTargetSpeed 185
#define IosDefn_SetTargetHeading 186
#define IosDefn_SetTargetTurnRate 187
#define IosDefn_SetTargetAltitude 188
#define IosDefn_SetTargetClimbRate 189
#define IosDefn_LoadHUDFile 190
#define IosDefn_SwitchHUDOff 191
#define IosDefn_LoadConeFile 192
#define IosDefn_SwitchConeOff 193

#define IosDefn_MapMode 211
#define IosDefn_ApproachMode 212
#define IosDefn_FlightDataMode 213
#define IosDefn_SetApproachRange 214
#define IosDefn_ApproachReset 215

#define IosDefn_PlotRecord 241
#define IosDefn_PlotSave 242
#define IosDefn_PlotNextPage 243
#define IosDefn_PlotPreviousPage 244
#define IosDefn_PlotNextMark 245
#define IosDefn_PlotPreviousMark 246
#define IosDefn_PlotMark 247
#define IosDefn_PlotGoto 248
#define IosDefn_SetPlotTime 249
#define IosDefn_SetKp 250
#define IosDefn_SetKi 251
#define IosDefn_SetKd 252
#define IosDefn_Dataview 253
#define IosDefn_GNUplot 254

#define IosDefn_PlaybackRecord 301
#define IosDefn_PlaybackReplay 302
#define IosDefn_PlaybackCamera 303
#define IosDefn_SetPlaybackTime 304
#define IosDefn_LoadPlaybackFile 305

#define IosDefn_WakeEnableUpdate      331
#define IosDefn_WakeSetAltitude       332
#define IosDefn_WakeSetHeading        333
#define IosDefn_WakeSetDistance       334
#define IosDefn_WakeSetVertSlope      335
#define IosDefn_WakeRestore           336

char IosDefn_FileName[IosDefn_MaxFileNameSize + 1];

typedef struct {
    double Latitude;
    double Longitude;
    float  Pz;
    float  Pitch;
    float  Roll;
    float  Yaw;
    float  U;
    float  V;
    float  W;
    float  P;
    float  Q;
    float  R;

    unsigned short int     CurrentRunway;
    unsigned char          Filler1[2];
    NavDefn_RadioPanel     SavedRadios[2];

    unsigned short int     FCURange;
    unsigned short int     FCUBaroPressure;
    unsigned short int     FCUHdg;
    unsigned short int     FCUAlt;
    unsigned short int     FCUSpeed;
    short int              FCUVS;
    short unsigned int     FCUBaroMode;

    boolean                FD;
    boolean                LS;
    boolean                LOC;
    boolean                AP1;
    boolean                AP2;
    boolean                ATHR;
    boolean                EXPED;
    boolean                APPR;
    boolean                SPD_MACH;
    boolean                HDG_TRK;

    boolean                FCUBaroHg;
    boolean                FCUHdgHold;
    boolean                FCUAltHold;
    boolean                FCUSpeedHold;
    boolean                FCUVSHold;

    NavDefn_NavaidModeType NavAid1;
    NavDefn_NavaidModeType NavAid2;
    NavDefn_MapModeType    MapMode;
    NavDefn_DispModeType   DispMode;
    unsigned char          Filler2[3];
} IosDefn_RestoreVectorRecord;

typedef struct {
    AeroDefn_AeroDataPkt AeroPkt;
    IODefn_IODataPkt     IOPkt;
    NavDefn_NavDataPkt   NavPkt;
} IosDefn_PlaybackDataPktRecord;

typedef struct {
    unsigned int PktNumber;
    IosDefn_PlaybackDataPktRecord PlaybackDataPkt;
    unsigned char                 CmdBuff[52];
    IosDefn_RestoreVectorRecord   RestoreVector;
} IosDefn_IosDataPkt;

#endif

