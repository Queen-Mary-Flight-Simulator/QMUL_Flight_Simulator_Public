/* 
+------------------------------+---------------------------------+
| Module      : iosdefn.h      | Version         : 1.1           |
| Last Edit   : 13-01-17       | Reference Number: 04-01-01      |
+------------------------------+---------------------------------+
| Computer    : DELL-6                                           |
| Directory   : /c/mingw/include/sim                             |
| Compiler    : gcc 5.3.0                                        |
| OS          : Windows10                                        |
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

#pragma pack(push,2)

#ifndef IosDefn_H
#define IosDefn_H

#include <stdbool.h>

#include "aerodefn.h"
#include "engdefn.h"
#include "navdefn.h"
#include "iodefn.h"

#define IosDefn_MaxFileNameSize          20

#define IosDefn_EndOfPkt                 0

#define IosDefn_Exit                     31
#define IosDefn_SetMode                  32
#define IosDefn_SetRunHoldFreeze         33
#define IosDefn_PrintScreen              34
#define IosDefn_Models                   35
#define IosDefn_Visual                   36
#define IosDefn_SetOctaveMode            37
#define IosDefn_LoadDTED                 38

#define IosDefn_Restore                  61
#define IosDefn_Save                     62
#define IosDefn_RePositionAircraft       63
#define IosDefn_SetAircraftAltitude      64
#define IosDefn_SetAircraftHeading       65
#define IosDefn_SetAircraftSpeed         66

#define IosDefn_MapCentre                91
#define IosDefn_MapCompass               92
#define IosDefn_MapFind                  93
#define IosDefn_MapTrack                 94
#define IosDefn_MapScale                 95
#define IosDefn_MapReset                 96

#define IosDefn_SetTurbulence            101
#define IosDefn_SetWindSpeed             102
#define IosDefn_SetWindDir               103
#define IosDefn_SetQNH                   104
#define IosDefn_SetMagneticVariation     105
#define IosDefn_SetCloudbase             106
#define IosDefn_SetVisibility            107
#define IosDefn_SetGroundTemperature     108
#define IosDefn_SetTimeOfDay             109
#define IosDefn_SetVisRate               110

#define IosDefn_SetRMICardType           131
#define IosDefn_SetMorseMode             132
#define IosDefn_SetSingleEngineMode      133
#define IosDefn_SetCgPosition            134
#define IosDefn_SetFlightControls        135
#define IosDefn_SwitchHUDOff             136
#define IosDefn_SetFlaps                 137
#define IosDefn_SetGear                  138
#define IosDefn_SetParkBrake             139
#define IosDefn_SetLeftFuelQuantity      140
#define IosDefn_SetRightFuelQuantity     141
#define IosDefn_SetAdfDip                142
#define IosDefn_SetDate                  143
#define IosDefn_Autotrim                 144

#define IosDefn_FailFlaps                161
#define IosDefn_FailGear                 162
#define IosDefn_FailNav1Localiser        163
#define IosDefn_FailNav1GlideSlope       164
#define IosDefn_FailNav2Localiser        165
#define IosDefn_FailNav2GlideSlope       166
#define IosDefn_FailRMI1                 167
#define IosDefn_FailRMI2                 168
#define IosDefn_FailDME                  169
#define IosDefn_FailEngine1              170
#define IosDefn_FailEngine2              171
#define IosDefn_FailEngine3              172
#define IosDefn_FailEngine4              173
#define IosDefn_FailASI                  174
#define IosDefn_FailAI                   175
#define IosDefn_FailVSI                  176
#define IosDefn_FailAltimeter            177
#define IosDefn_FailTurn                 178
#define IosDefn_FailCompass              179
#define IosDefn_FailFD                   180
#define IosDefn_Engine1Fire              181
#define IosDefn_Engine2Fire              182
#define IosDefn_Engine3Fire              183
#define IosDefn_Engine4Fire              184

#define IosDefn_SetAutopilotAltitude     211
#define IosDefn_SetAutopilotHeading      212
#define IosDefn_SetAutopilotSpeed        213
#define IosDefn_SetAutopilotVSpeed       214
#define IosDefn_AutolandOn               215
#define IosDefn_AutopilotAltitudeOn      216
#define IosDefn_AutopilotHeadingOn       217
#define IosDefn_AutopilotSpeedOn         218
#define IosDefn_AutopilotVSpeedOn        219

#define IosDefn_LoadFlightPlan           241
#define IosDefn_SetFlightPlanMode        242
#define IosDefn_Script                   243

#define IosDefn_LoadTargetFile           271
#define IosDefn_SwitchTargetOff          272
#define IosDefn_SetTargetPosition        273
#define IosDefn_SetTargetDistance        274
#define IosDefn_SetTargetSpeed           275
#define IosDefn_SetTargetHeading         276
#define IosDefn_SetTargetTurnRate        277
#define IosDefn_SetTargetAltitude        278
#define IosDefn_SetTargetClimbRate       279
#define IosDefn_SetTargetConflict        280
#define IosDefn_Pushback                 281

#define IosDefn_SetApproachRange         301
#define IosDefn_ApproachReset            302

#define IosDefn_PlotRecord               331
#define IosDefn_PlotSave                 332
#define IosDefn_PlotNextPage             333
#define IosDefn_PlotPreviousPage         334
#define IosDefn_PlotNextMark             335
#define IosDefn_PlotPreviousMark         336
#define IosDefn_PlotMark                 337
#define IosDefn_PlotGoto                 338
#define IosDefn_SetPlotTime              339
#define IosDefn_SetKp                    340
#define IosDefn_SetKi                    341
#define IosDefn_SetKd                    342
#define IosDefn_GNUplot                  343

#define IosDefn_PlotRudder               361
#define IosDefn_PlotAileron              362
#define IosDefn_PlotElevator             363
#define IosDefn_PlotAirspeed             364
#define IosDefn_PlotAltitude             365
#define IosDefn_PlotBetaRate             366
#define IosDefn_PlotBeta                 367
#define IosDefn_PlotAlphaRate            368
#define IosDefn_PlotAlpha                369
#define IosDefn_PlotYawRate              370
#define IosDefn_PlotRollRate             371
#define IosDefn_PlotPitchRate            372
#define IosDefn_PlotYaw                  373
#define IosDefn_PlotRoll                 374
#define IosDefn_PlotPitch                375
#define IosDefn_PlotG                    376
#define IosDefn_PlotLocErr               377
#define IosDefn_PlotGSErr                378
#define IosDefn_PlotRateOfClimb          379
#define IosDefn_PlotFlightData1          380
#define IosDefn_PlotFlightData2          381
#define IosDefn_PlotFlightData3          382

#define IosDefn_PlaybackRecord           391
#define IosDefn_PlaybackReplay           392
#define IosDefn_PlaybackCamera           393
#define IosDefn_SetPlaybackTime          394
#define IosDefn_LoadPlaybackFile         395

#define IosDefn_Radio                    421
#define IosDefn_ILS1                     422
#define IosDefn_ILS2                     423
#define IosDefn_VOR1                     424
#define IosDefn_VOR2                     425
#define IosDefn_ADF1                     426
#define IosDefn_ADF2                     427

typedef struct
{
    double                 Latitude;
    double                 Longitude;
    float                  Pz;
    float                  Pitch;
    float                  Roll;
    float                  Yaw;
    float                  U;
    float                  V;
    float                  W;
    float                  P;
    float                  Q;
    float                  R;

    unsigned short int     CurrentRunway;
    unsigned char          Filler1[2];
    NavDefn_RadioPanel     SavedRadios[2];

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
    NavDefn_FCUData        DataMode;
    unsigned char          Filler2;
} IosDefn_RestoreVectorRecord;

typedef struct
{
    AeroDefn_AeroDataPkt AeroPkt;
    EngDefn_EngDataPkt   EngPkt;
    IODefn_IODataPkt     IOPkt1;
    IODefn_IODataPkt     IOPkt2;
    NavDefn_NavDataPkt   NavPkt;
	bool                 FrameMarker;
	char                 Filler[3];
} IosDefn_PlaybackDataPktRecord;

typedef struct
{
    unsigned int                  PktNumber;
    IosDefn_PlaybackDataPktRecord PlaybackDataPkt;
    unsigned char                 CmdBuff[52];
    IosDefn_RestoreVectorRecord   RestoreVector;
} IosDefn_IosDataPkt;
#endif

#pragma pack(pop)
