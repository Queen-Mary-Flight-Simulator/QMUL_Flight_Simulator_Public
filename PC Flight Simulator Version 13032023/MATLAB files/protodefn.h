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


typedef struct {
  float D_LX;
  float D_LY;
  float D_LZ;
  float D_AR;
  float D_AP;
  float D_AY;
  float VtxSegCurrX;
  float VtxSegCurrY;
  float VtxSegCurrZ;
  float VtxSegCurrRoll;
  float VtxSegCurrPitch;
  float VtxSegCurrYaw;
  float VtxDirVecX;
  float VtxDirVecY;
  float VtxDirVecZ;
  float TailUltimateLoad;
  float TailLoad;
  float TailUltimateBM;
  float TailBM;
  unsigned int VtxTimeNum;
} ProtoVortex_t;

// ATC data structure
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

typedef char ProtoBuffer_t[1000];

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
} Mode_t;

typedef struct {
  ProtoHeader_t Header;
  Mode_t        Data;
} ProtoDefn_ProtoDataPkt;

#endif
