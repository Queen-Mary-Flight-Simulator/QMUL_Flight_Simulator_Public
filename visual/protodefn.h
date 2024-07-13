#ifndef PROTODEFN_H
#define PROTODEFN_H

#define PROTO_TYPE_MATLAB 0
#define PROTO_TYPE_VORTEX 1
#define PROTO_TYPE_ATC    2
#define PROTO_TYPE_BUFFER 3
#define PROTO_TYPE_WAYPTS 4

typedef struct {
  unsigned int PktNumber;
  unsigned int PktType;
} ProtoHeader_t;

typedef struct {
  float Aileron;
  float Elevator;
  float Rudder;
  float Throttle;
} ProtoMatlab_t;

// CONFIRM WITH LATEST CODE - maybe other data
typedef struct {
  float D_CL;        // Change in lift force coeff
  float D_CY;        //    "    " side   "     "
  float D_Cl;        //    "    " rolling moment coeff
  float D_Cm;        //    "    " pitching moment coeff
  float D_Cn;        //    "    " yawing moment coeff
} ProtoVortex_t;

// ATC data structure??? 1500 bytes. For 50 ac, 30 bytes/ac, ~7 floats per a/c
typedef struct {
  float latitude;   // radians
  float longitude;  // radians
  float speed;      // knots
  float altitude;   // TBC - feet for now
  float heading;    // degrees (+- PI)
} ProtoAircraft_t;

typedef struct {
  ProtoAircraft_t Aircraft[50]; // totalling 1000 bytes
} ProtoATC_t;

typedef char ProtoBuffer_t[1500];

typedef struct {
  char         WayPointID[4];
  float        WayPointLatitude;
  float        WayPointLongitude;
  unsigned int WayPointAltitude;
  unsigned int WayPointSpeed;
} ProtoWaypoint_t;

typedef struct {
  unsigned int Num;
  ProtoWaypoint_t List[50];
} ProtoWaypointList_t;

typedef union {
  ProtoMatlab_t       Matlab;
  ProtoVortex_t       Vortex;
  ProtoATC_t          Traffic;
  ProtoBuffer_t       Buffer;
  ProtoWaypointList_t Waypoints;
} Data_t;

typedef struct {
  ProtoHeader_t Header;
  Data_t        Data;
} Protodefn_ProtoDataPkt;

#endif
