#pragma pack(push,2)

#ifndef IGDefn_H
#define IGDefn_H

#include <stdbool.h>

typedef struct
{
    unsigned int  PktNumber;

    float         Pitch;
    float         Roll; 
    float         Yaw;
    float         U;
    float         Rho; 
    float         Pz;
	double        Latitude;
	double        Longitude;
    float         Alpha;
    float         Beta;
    float         Q;
    float         AlphaDot;
    float         BetaDot; 
    float         UDot;
    bool          Stopping;
	char          filler1[3];
    float         TPitch;
    float         TRoll;
    float         TYaw;
	float         TPz;
    double        TLatitude;
    double        TLongitude; 
    float         Ey;
    float         Ex;
    float         Ez;
    int           TimeOfDay;

    float         MagneticVariation; 
    unsigned int  FCU_BaroPressure;
    unsigned int  CurrentRunway; 
    double        RunwayLatitude;
    double        RunwayLongitude;
    double        RunwayQDM;
    float         GroundLevel;

    unsigned char CmdBuff[52];

} IGDefn_IGDataPkt;
#endif

#pragma pack(pop)
