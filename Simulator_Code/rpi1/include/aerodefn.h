/* 
+------------------------------+---------------------------------+
| Module      : aerodefn.h     | Version         : 1.1           |
| Last Edit   : 13-01-17       | Reference Number: 02-01-01      |
+------------------------------+---------------------------------+
| Computer    : DELL3                                            |
| Directory   : /mingw/include/SIM/                              |
| Compiler    : gcc 5.3.0                                        |
| OS          : Windows 10                                       |
+----------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
+----------------------------------------------------------------+
| Description : flight model system data packet definition       |
|                                                                |
+----------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#pragma pack(push,2)

#ifndef AeroDefn_H
#define AeroDefn_H

#include <stdbool.h>
#include <SIM/iodefn.h>

typedef struct
{
    unsigned int PktNumber;

    float        Pitch;
    float        Roll;
    float        Yaw;

    float        P;
    float        Q;
    float        R;

    float        PDot;
    float        QDot;
    float        RDot;

    float        Vn;
    float        Ve;
    float        Vd;

    float        Pz;
    double       Latitude;
    double       Longitude;

    float        U;
    float        V;
    float        W;

    float        UDot;
    float        VDot;
    float        WDot;

    float        Rmt;
    float        Pmt;
    float        Ymt;

    float        Alpha;
    float        Beta;
    float        AlphaDot;
    float        BetaDot;

    float        Cl;
    float        Cd;

    float        Lift;
    float        Thrust;
    float        Drag;
    float        SideForce;

    float        XForce;
    float        YForce;
    float        ZForce;

    float        Vc;
    float        MachNumber;

    float        WindSpeed;
    float        WindDir;

    float        Ixx;
    float        Iyy;
    float        Izz;
    float        Ixz;
    float        Iyz;
    float        Ixy;
    float        Mass;

    float        Ex;
    float        Ey;
    float        Ez;
    float        Wheelz;

    float        TPitch;
    float        TRoll;
    float        TYaw;
    float        TPz;
    double       TLatitude;
    double       TLongitude;

    float        Rho;
    float        OAT;

    unsigned short int TimeOfDay;

    bool         Stalling;
    bool         ConfigWarning;
    bool         OnTheGround;
    bool         OctaveMode;
    bool         ReloadingModel;

    bool         APSpeedMode;
    float        APThrottlePosition;

    float        FlapPosition;
    float        FlapSetting;
    float        GearPosition;
    IODefn_GearSelector GearSelector;

    unsigned char DigitalDataOutA;
    unsigned char DigitalDataOutB;
    char         Filler1;
    
    float        Elevator;
    float        Aileron;
    float        Rudder;
    float        ElevatorTrim;
    float        AileronTrim;
    float        RudderTrim;

    float        LeftBrake;
    float        RightBrake;
    float        CGHeight;
    unsigned int TimeStamp;

    float        FlightData[3];
} AeroDefn_AeroDataPkt;
#endif

#pragma pack(pop)
