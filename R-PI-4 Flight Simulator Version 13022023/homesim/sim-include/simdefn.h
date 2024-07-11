/*
   simdefn.h

   I/O
   Aero
   Nav
 */
#pragma pack(push,2)

#ifndef SIMDEFN_H
#define SIMDEFN_H

#include <stdbool.h>

#define MaxWayPoints             50

#define SimDefn_GearUp           0
#define SimDefn_GearDown         1
#define SimDefn_GearOff          2
typedef unsigned char            SimDefn_GearSelectorPosition;
typedef unsigned char            SimDefn_Propulsion;

#define SimDefn_On               0
#define SimDefn_Off              1
typedef unsigned char            SimDefn_SwitchPosition;

#define SimDefn_TrimForwards     0
#define SimDefn_TrimBackwards    1
#define SimDefn_TrimLeft         2
#define SimDefn_TrimRight        3
#define SimDefn_TrimOff          4

#define SimDefn_UnknownEngine    0
#define SimDefn_Piston           1
#define SimDefn_Turboprop        2
#define SimDefn_Jet              3

typedef unsigned char            SimDefn_TrimSwitchPosition;

#define SimDefn_MapModeILS       0
#define SimDefn_MapModeVOR       1
#define SimDefn_MapModeNAV       2
#define SimDefn_MapModeARC       3
#define SimDefn_MapModePLAN      4
typedef unsigned char            SimDefn_MapModeType;

#define SimDefn_DispOFF          0
#define SimDefn_DispCSTR         1
#define SimDefn_DispWPT          2
#define SimDefn_DispVORD         3
#define SimDefn_DispNDB          4
#define SimDefn_DispARPT         5
typedef unsigned char            SimDefn_DispModeType;

#define SimDefn_NavModeADF       0
#define SimDefn_NavModeOFF       1
#define SimDefn_NavModeVOR       2
typedef unsigned char            SimDefn_NavaidModeType;

#define SimDefn_DME_Mode         0
#define SimDefn_ILS_Mode         1
#define SimDefn_VOR_Mode         2
#define SimDefn_NDB_Mode         3
typedef unsigned char            SimDefn_BeaconMode;

typedef unsigned int             SimDefn_BeaconType;

typedef struct
{
    float Thrust;
    float Epr;
    float Rpm;
    float FuelFlow;
    float Egt;
    float Beta;
    float ManifoldPressure;
} SimDefn_EngineData;

typedef struct
{
    unsigned int  SelectedBeacon;
    float         LocaliserError;
    float         GlideSlopeError;
    float         SlantDistance;
    float         GroundSpeed;
    float         GroundDistance;
    float         BearingToStation;
    float         RunwayQdm;
    float         RmiPointer;
    bool          ILSBeacon;
    bool          BeaconStatus;
    unsigned char Filler2[2];
} SimDefn_NavData;

typedef struct
{
    char         WayPointID[4];
    float        WayPointLatitude;
    float        WayPointLongitude;
    unsigned int WayPointAltitude;
    unsigned int WayPointSpeed;
} SimDefn_WayPointRecord;

typedef struct
{
    unsigned int                 PktNumber;
    float                        ElevatorPosition;
    float                        AileronPosition;
    float                        RudderPosition;
    float                        FlapSelector;
    float                        EngineLever;
    float                        ReverseLever;
    float                        LeftBrake;
    float                        RightBrake;

    SimDefn_SwitchPosition       LampsTestButton;
    SimDefn_SwitchPosition       WarningCancelButton;
    SimDefn_GearSelectorPosition GearSelector;
    SimDefn_TrimSwitchPosition   RudderTrimSwitch;
    SimDefn_SwitchPosition       ThrottlePushButton;
    SimDefn_SwitchPosition       ThrottleSwitch;
    SimDefn_SwitchPosition       TriggerSwitch;
    SimDefn_TrimSwitchPosition   AileronTrimSwitch;
    SimDefn_TrimSwitchPosition   ElevatorTrimSwitch;
    bool                         HoldButtonPressed;
    bool                         RestoreButtonPressed;
    bool                         FreezeButtonPressed;
    bool                         ClockButtonPressed;
    SimDefn_SwitchPosition       AutomaticResetSwitch;
    SimDefn_SwitchPosition       RightBoostPumpSwitch;
    SimDefn_SwitchPosition       LeftBoostPumpSwitch;
    SimDefn_SwitchPosition       StarterSwitch;
    SimDefn_SwitchPosition       IgnitionSwitch;
    SimDefn_SwitchPosition       MasterSwitch;
    SimDefn_SwitchPosition       ColumnButton;
    SimDefn_SwitchPosition       ParkBrake;
    SimDefn_SwitchPosition       KeySwitch;
    bool                         StickMoving;
    unsigned char                Filler3;

    float                        Pitch;
    float                        Roll;
    float                        Yaw;

    float                        P;
    float                        Q;
    float                        R;

    float                        PDot;
    float                        QDot;
    float                        RDot;

    float                        Vn;
    float                        Ve;
    float                        Vd;

    float                        Pz;
    double                       Latitude;
    double                       Longitude;

    float                        U;
    float                        V;
    float                        W;

    float                        UDot;
    float                        VDot;
    float                        WDot;

    float                        Rmt;
    float                        Pmt;
    float                        Ymt;

    float                        Alpha;
    float                        Beta;
    float                        AlphaDot;
    float                        BetaDot;

    float                        Cl;
    float                        Cd;

    float                        Lift;
    float                        Thrust;
    float                        Drag;
    float                        SideForce;

    float                        XForce;
    float                        YForce;
    float                        ZForce;

    float                        Vc;
    float                        MachNumber;

    float                        WindSpeed;
    float                        WindDir;

    float                        Ixx;
    float                        Iyy;
    float                        Izz;
    float                        Ixz;
    float                        Iyz;
    float                        Ixy;
    float                        Mass;

    float                        Ex;
    float                        Ey;
    float                        Ez;
    float                        Wheelz;

    float                        TPitch;
    float                        TRoll;
    float                        TYaw;
    float                        TPz;
    double                       TLatitude;
    double                       TLongitude;

    float                        Rho;
    float                        OAT;
    unsigned int                 QNH;

    short unsigned int           NumberOfEngines;
    short unsigned int           TimeOfDay;

    SimDefn_EngineData           Engines[4];
    float                        EngineLevers[6];
    float                        FuelQuantityLeft;
    float                        FuelQuantityRight;
    float                        EngineThrust;
    float                        EngineMoment;

    SimDefn_Propulsion           EngineType;

    bool                         Stalling;
    bool                         EngineFireSound;
    bool                         ConfigWarning;
    bool                         OnTheGround;
    bool                         OctaveMode;
    bool                         ReloadingModel;
    bool                         Shutdown;

    float                        FlapPosition;
    unsigned int                 FlapSelection;
    float                        GearPosition;
    float                        CGHeight;

    float                        Elevator;
    float                        Aileron;
    float                        Rudder;
    float                        ElevatorTrim;
    float                        AileronTrim;
    float                        RudderTrim;

    float                        FlightData[3];

    unsigned short int           BaroPressure1;
    unsigned short int           BaroPressure2;
    float                        MagneticVariation;

    float                        RMI_Dir1;
    float                        RMI_Dir2;

    unsigned short int           CurrentRunway;
    short int                    RunwayQDM;
    float                        RunwayLatitude;
    float                        RunwayLongitude;
    float                        GroundLevel;
    float                        GroundSpeed;

    bool                         OuterMarker;
    bool                         MiddleMarker;
    bool                         InnerMarker;
    bool                         MarkerTest;

    float                        Track;
    unsigned short int           TrackRange;
    unsigned char                Filler4[2];

    unsigned int                 FlightPlan_Segment;
    unsigned int                 NumberOfWayPoints;
    SimDefn_WayPointRecord       WayPoints[MaxWayPoints];

    float                        HSI_Localiser;
    float                        HSI_GlideSlope;
    float                        VOR_Localiser;
    float                        VOR_GlideSlope;
    bool                         HSI_ILSMode;
    bool                         HSI_Status;
    bool                         VOR_ILSMode;
    bool                         VOR_Status;
    float                        DME_Distance;

    SimDefn_NavData              NAV1;
    SimDefn_NavData              NAV2;
    SimDefn_NavData              ADF1;
    SimDefn_NavData              ADF2;
    SimDefn_NavData              ILS1;
    SimDefn_NavData              WayPoint;

    short int                    HSI_Crs;
    short int                    HSI_Hdg;
    short int                    VOR_Obs;
    unsigned char                Filler5[2];

    int                          FD_VBar;
    int                          FD_HBar;

    unsigned short int           FCURange;
    unsigned short int           FCUBaroPressure;
    unsigned short int           FCUHdg;
    unsigned short int           FCUAlt;
    unsigned short int           FCUSpeed;
    short int                    FCUVS;
    short unsigned int           FCUBaroMode;

    bool                         FD;
    bool                         LS;
    bool                         LOC;
    bool                         AP1;
    bool                         AP2;
    bool                         ATHR;
    bool                         EXPED;
    bool                         APPR;
    bool                         SPD_MACH;
    bool                         HDG_TRK;

    bool                         FCUBaroHg;
    bool                         FCUHdgHold;
    bool                         FCUAltHold;
    bool                         FCUSpeedHold;
    bool                         FCUVSHold;

    SimDefn_NavaidModeType       NavAid1;
    SimDefn_NavaidModeType       NavAid2;
    SimDefn_MapModeType          MapMode;
    SimDefn_DispModeType         DispMode;

    unsigned char                Filler6[3];
} SimDefn_ModelDataPkt;
#endif

#pragma pack(pop)
