/* +------------------------------+---------------------------------+
   | Module      : aerodefn.h     | Version         : 1.1           | 
   | Last Edit   : 19-11-07       | Reference Number: 02-01-01      |
   +------------------------------+---------------------------------+
   | Computer    : Viglen1                                          |
   | Directory   : /dja/pfd                                         |
   | Compiler    : gcc 3.2                                          |
   | OS          : RedHat 8.0                                       |
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

#ifndef AeroDefn_H
#define AeroDefn_H

#include "cbool.h"

#define AeroDefn_UnknownEngine   0
#define AeroDefn_Piston 1
#define AeroDefn_Turboprop 2
#define AeroDefn_Jet 3

typedef unsigned char AeroDefn_Propulsion;

typedef struct {
    float Thrust;
    float Epr;
    float Rpm;
    float FuelFlow;
    float Egt;
    float Beta;
    float ManifoldPressure;
} AeroDefn_EngineData;

typedef struct {
    unsigned int PktNumber;

    float Pitch;
    float Roll;
    float Yaw;
   
    float P;
    float Q;
    float R;
   
    float PDot;
    float QDot;
    float RDot;
   
    float Vn;
    float Ve;
    float Vd;
   
    float Pz;
    double Latitude;
    double Longitude;
   
    float U;
    float V;
    float W;
   
    float UDot;
    float VDot;
    float WDot;
   
    float Rmt;
    float Pmt;
    float Ymt;
   
    float Alpha;
    float Beta;
    float AlphaDot;
    float BetaDot;
   
    float Cl;
    float Cd;
   
    float Lift;
    float Thrust;
    float Drag;
    float SideForce;
   
    float XForce;
    float YForce;
    float ZForce;
   
    float Vc;
    float MachNumber;
   
    float WindSpeed;
    float WindDir;
   
    float Ixx;
    float Iyy;
    float Izz;
    float Ixz;
    float Iyz;
    float Ixy;
    float Mass;
   
    float Ex;
    float Ey;
    float Ez;
    float Wheelz;

    float TPitch;
    float TRoll;
    float TYaw;
    float TPz;
    double TLatitude;
    double TLongitude;
   
    float Rho;
    float OAT;
   
    short unsigned int NumberOfEngines;
    short unsigned int TimeOfDay;
   
    AeroDefn_EngineData Engines[4];
    float EngineLevers[6];
    float FuelQuantityLeft;
    float FuelQuantityRight;
    float EngineThrust;
    float EngineMoment;

    AeroDefn_Propulsion EngineType;
   
    boolean Stalling;
    boolean EngineFireSound;
    boolean ConfigWarning;
    boolean OnTheGround;
    boolean OctaveMode;
    boolean ReloadingModel;
    boolean Shutdown;
   
    float FlapPosition;
    float GearPosition;
   
    float Elevator;
    float Aileron;
    float Rudder;
    float ElevatorTrim;
    float AileronTrim;
    float RudderTrim;
   
    unsigned int TimeStamp;
   
    float FlightData[3];
} AeroDefn_AeroDataPkt;

#endif
