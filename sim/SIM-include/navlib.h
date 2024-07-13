#ifndef NavLib_H
#define NavLib_H

#define EarthRadius 6378137.0

#define NavLib_MaxBeacons	1000
#define NavLib_MaxRunways	1000
#define NavLib_MaxWayPoints	50

#define NavLib_DME	1
#define NavLib_ILS	2
#define NavLib_VOR	4
#define NavLib_NDB	8

#define NavLib_NameSize 20

typedef unsigned char NavLib_BeaconMode;
typedef unsigned int NavLib_BeaconType;

typedef struct
{
    float             BeaconLatitude;     /* rads */
    float             BeaconLongitude;    /* rads */
    unsigned int      Frequency;          /* freq * 100 */
	float             Qdm;                /* degs 1-360 iff ILS */
    unsigned int      Range;              /* not available */
	unsigned int      Elevation;          /* m */
	unsigned int      Runway;             /* 23R etc */
    char              Ident[6];
    NavLib_BeaconType Navaid;
} NavLib_BeaconRecord;

typedef struct 
{
    float             RunwayLatitude;     /* rads */
    float             RunwayLongitude;    /* rads */
    float             RunwayEndLatitude;  /* rads */
    float             RunwayEndLongitude; /* rads */
	unsigned int      Frequency;          /* iff ILS */
	float             Qdm;                /* runway QDM true */
    unsigned int      Length;             /* m */
    unsigned int      Width;              /* not used */
    unsigned int      Elevation;          /* m */
	unsigned int      Runway;             /* 23R etc */
	char              Name[NavLib_NameSize];
} NavLib_RunwayRecord;

typedef struct 
{
    char WayPointID[4];
    float             WayPointLatitude;
    float             WayPointLongitude;
    unsigned int      WayPointAltitude;
    unsigned int      WayPointSpeed;
} NavLib_WayPointRecord;

extern unsigned int NavLib_NumberOfBeacons;
extern unsigned int NavLib_NumberOfRunways;
extern unsigned int NavLib_NumberOfWayPoints;
extern unsigned int NavLib_NextWayPoint;

extern NavLib_BeaconRecord NavLib_Beacons[NavLib_MaxBeacons];
extern NavLib_RunwayRecord NavLib_Runways[NavLib_MaxRunways];
extern NavLib_WayPointRecord NavLib_WayPoints[NavLib_MaxWayPoints];

extern float NavLib_Distance(float Lat1, float Long1, float Lat2, float Long2);

extern float NavLib_Bearing(float Lat1, float Long1, float Lat2, float Long2);

extern void NavLib_ReadBeacons(float minlat, float maxlat, float minlong, float maxlong);

extern void NavLib_ReadRunways(float minlat, float maxlat, float minlong, float maxlong);

extern void NavLib_ReadFlightPlan(char []);

extern void NavLib_InitNavLib();

extern unsigned int NavLib_LookupChannel(float Latitude, float Longitude, unsigned int f, NavLib_BeaconType BType);

extern unsigned int NavLib_NearestRunway(float latitude, float longitude, int freq);

extern void BEGIN_NavLib(void);

#endif
