#ifndef Nav_H
#define Nav_H

#include <stdbool.h>

#define Nav_MaxWayPoints 50

typedef struct 
{
    unsigned int SelectedBeacon;
    float LocaliserError;
    float GlideSlopeError;
    float SlantDistance;
    float Gspeed;
    float GroundDistance;
    float BearingToStation;
    float RmiPointer;
    bool BeaconStatus;
    bool IlsFrequency;
    float RunwayQdm;
} Nav_NavRecord;

typedef struct 
{
    char WayPointID[4];
    float WayPointLatitude;
    float WayPointLongitude;
    unsigned int WayPointAltitude;
    unsigned int WayPointSpeed;
} Nav_WayPointRecord;

extern unsigned int Nav_MorseChannel;
extern char Nav_MorseIdent[4];

extern unsigned int Nav_FlightPlan_Segment;
extern unsigned int Nav_NumberOfWayPoints;
extern Nav_WayPointRecord Nav_WayPoints[Nav_MaxWayPoints];

extern unsigned int Nav_CurrentRunway;
extern Nav_NavRecord Nav_ILS1;
extern Nav_NavRecord Nav_VOR1;
extern Nav_NavRecord Nav_VOR2;
extern Nav_NavRecord Nav_ADF1;
extern Nav_NavRecord Nav_ADF2;
extern Nav_NavRecord Nav_WayPoint;
extern float Nav_GroundLevel;
extern float Nav_Rmi_Dir1, Nav_Rmi_Dir2;
extern int Nav_HSI_Crs, Nav_HSI_Hdg, Nav_VOR_Obs;
extern float Nav_HSI_Localiser, Nav_HSI_GlideSlope;
extern bool Nav_HSI_ILSMode, Nav_HSI_Status;
extern float Nav_VOR_Localiser, Nav_VOR_GlideSlope;
extern bool Nav_VOR_ILSMode, Nav_VOR_Status;
extern float Nav_DmeDistance, Nav_GroundSpeed;
extern bool Nav_OuterMarker, Nav_MiddleMarker;
extern bool Nav_InnerMarker, Nav_MarkerTest;
extern float Nav_Track;
extern unsigned int Nav_TrackRange;
extern float Nav_MagneticVariation;
extern unsigned int Nav_BaroPressure1;
extern unsigned int Nav_BaroPressure2;
extern unsigned int Nav_RegionalQNH;

extern void Nav_UpdateNav();
extern void Nav_CheckFlightPlan();

extern void BEGIN_Nav();

#endif
