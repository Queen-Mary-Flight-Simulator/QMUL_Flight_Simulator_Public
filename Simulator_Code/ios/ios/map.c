#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <SIM/iosdefn.h>
#include <SIM/maths.h>
#include <SIM/glib.h>
#include <SIM/navlib.h>

#include "map.h"
#include "ioslink.h"

#define MaxTrackLength 8000
#define MaxTracks      20
#define MaxCompasses   20

#define ONERAD  (180.0 / M_PI)
#define DEG6    (6.0 / ONERAD)
#define DEG90   (90.0 / ONERAD)
#define PIBY2   (M_PI / 2.0)
#define TWOPI   (M_PI * 2.0)
#define D12     (TWOPI * EarthRadius / 30.0)

typedef struct 
{
    int    TrackLength;
    int    DisplayedPoints;
    int    FirstPoint;
    int    NextPoint;
    int    NextDisplayedPoint;
    float xPoints[MaxTrackLength];
    float yPoints[MaxTrackLength];

} AircraftTrackRecord;

typedef struct 
{
    unsigned int Size;
    float        CLat[MaxCompasses];
    float        CLong[MaxCompasses];
} CompassList;

typedef struct
{
    int Cx;
    int Cy;
    int Coords;  /* bool */
    int Dme;     /* bool */
} LatLongCoordinates;

typedef struct
{
    unsigned int Size;
    float        Lat1[MaxTracks];
    float        Long1[MaxTracks];
    float        Lat2[MaxTracks];
    float        Long2[MaxTracks];
} TrackList;

typedef struct
{
    float x;
    float y;
    float Hdg;
} ArrowRecord;

typedef struct
{
    int x;
	int y;
} DrawnILSRecord;

/* variables */

float                 Map_MapScaleFactor  = 0.0;

AircraftTrackRecord   *AircraftTrackList = NULL;
float                 AircraftLatitude;
float                 AircraftLongitude;
float                 AircraftHeading;
float                 TargetLatitude;
float                 TargetLongitude;
float                 TargetHeading;

float                 MapMinLatitude  = PIBY2;
float                 MapMaxLatitude  = -PIBY2;
float                 MapMinLongitude = TWOPI;
float                 MapMaxLongitude = -TWOPI;
float                 MapLatitude     = 0.0;
float                 MapLongitude    = 0.0;

CompassList           Compasses;

static float          Lambert_N;       /* convergence factor */
static float          Lambert_K;       /* constant term */
static float          msf;             /* map scale factor */
static float          MapOffset;       /* y offset of screen in metres */

static float          MagneticVariation;

static TrackList      Tracks;
static ArrowRecord    Arrow;

static float          OldLatitude;
static float          OldLongitude;
static float          OldYaw;
static unsigned int   OldPktNumber;

static DrawnILSRecord DrawnILS[1000];
static unsigned int   nDrawnILS;

void         ShowTarget(void);
unsigned int NextTrackPoint(unsigned int);
unsigned int PrevTrackPoint(unsigned int);
int          Min(int, int);
int          Max(int, int);

/* ------------------------------------------------------------- */
void Map_SetMapCentre(float Latitude, float Longitude)
{
    float Chi;
    float lat1;
    float lat2;
    float r;
    float r1;
    float r2;

    MapLatitude  = Latitude;
    MapLongitude = Longitude;
    Chi              = PIBY2 - fabs(Latitude);
    Lambert_N        = cos(Chi);
    Lambert_K        = tan(Chi) * pow(tan(Chi / 2.0), -Lambert_N);
    lat1             = Chi - DEG6;
    lat2             = Chi + DEG6;
    r                = Lambert_K * pow(tan(Chi / 2.0), Lambert_N);
    r1               = Lambert_K * pow(tan(lat1 / 2.0), Lambert_N);
    r2               = Lambert_K * pow(tan(lat2 / 2.0), Lambert_N);
    msf              = D12 / fabs(r1 - r2);
    MapOffset        = r * msf;
}

/* ------------------------------------------------------------- */
void Map_GlobeToScreen(float Latitude, float Longitude, float *x, float *y)
{
    float Chi;
    float Theta;
    float r;
    float t;

    Chi   = PIBY2 - fabs(Latitude);
    Theta = Lambert_N * (Longitude - MapLongitude);
    r     = Lambert_K * pow(tan(Chi / 2.0), Lambert_N);
    *x    = (r * sin(Theta) * msf / Map_MapScaleFactor) + MapCentreX;
    t     = (r * cos(Theta) * msf - MapOffset) / Map_MapScaleFactor;
    if (MapLatitude > 0.0)
    {
        *y = MapCentreY - t;
    }
    else
    {
        *y = MapCentreY + t;
    }
}

/* ------------------------------------------------------------- */
void Map_ScreenToGlobe(float x, float y, float *Latitude, float *Longitude)
{
    float Chi;
    float Theta;
    float r;
    float dx;
    float dy;

    dx = (x - MapCentreX) * Map_MapScaleFactor;
    if (MapLatitude > 0.0)
    {
        dy = (MapCentreY - y) * Map_MapScaleFactor + MapOffset;
    }
    else
    {
        dy = (y - MapCentreY) * Map_MapScaleFactor + MapOffset;
    }
    r          = sqrt(dx * dx + dy * dy) / msf;
    Theta      = asin((x - MapCentreX) * Map_MapScaleFactor / (r * msf));
    *Longitude = Theta / Lambert_N + MapLongitude;
    Chi        = 2.0 * atan(pow(r / Lambert_K, 1.0 / Lambert_N));
    *Latitude  = PIBY2 - Chi;
    if (MapLatitude < 0.0)
    {
        *Latitude = -(*Latitude);
    }
}

/* --------------------------------------------- */
long entier(float n)
{
    return (long) floor(n);
}

/* --------------------------------------------- */
void Map_Init(void)
{
    unsigned int i;
    float        Lat, Long;

    for (i = 1; i < NavLib_NumberOfBeacons; i = i + 1)
    {
        Lat  = NavLib_Beacons[i].BeaconLatitude;
        Long = NavLib_Beacons[i].BeaconLongitude;

        if (Lat < MapMinLatitude)
        {
            MapMinLatitude = Lat;
        }
        else if (Lat > MapMaxLatitude)
        {
            MapMaxLatitude = Lat;
        }

        if (Long < MapMinLongitude)
        {
            MapMinLongitude = Long;
        }
        else if (Long > MapMaxLongitude)
        {
            MapMaxLongitude = Long;
        }
    }
}

/* --------------------------------------------- */
void Map_SetMapMagneticVariation(float d)
{
    MagneticVariation = d;
}

/* --------------------------------------------- */
float Map_GetMagneticVariation(void)
{
    return MagneticVariation;
}

/* --------------------------------------------- */
void Map_ChangeMap(unsigned int Cmd, int Mouse_Mx, int Mouse_My)
{
    float Lat, Long;

    switch (Cmd)
    {
    case IosDefn_MapFind:
        Map_SetMapCentre(AircraftLatitude, AircraftLongitude);
        break;

    case IosDefn_MapCentre:
        // Get mouse xy
        Map_ScreenToGlobe((float) (Mouse_Mx), (float) (Mouse_My), &Lat, &Long);
        Map_SetMapCentre(Lat, Long);
        break;

    case IosDefn_MapCompass:
        if (Compasses.Size < MaxCompasses)
        {
            Map_ScreenToGlobe((float) (Mouse_Mx), (float) (Mouse_My), &Lat, &Long);
            Compasses.CLat[Compasses.Size]  = Lat;
            Compasses.CLong[Compasses.Size] = Long;
            Compasses.Size                      = Compasses.Size + 1;
        }
        break;

    case IosDefn_RePositionAircraft:
        Map_ScreenToGlobe((float) (Mouse_Mx), (float) (Mouse_My), &Lat, &Long);
        IosLink_SendPosition(IosDefn_RePositionAircraft, Lat, Long);
        break;

    case IosDefn_SetTargetPosition:
        Map_ScreenToGlobe((float) (Mouse_Mx), (float) (Mouse_My), &Lat, &Long);
        IosLink_SendPosition(IosDefn_SetTargetPosition, Lat, Long);
        break;

    case IosDefn_MapReset:
        Compasses.Size = 0;
        Tracks.Size        = 0;
        Map_InitialiseTrackList();
        break;

    case IosDefn_MapScale:
        break;

    case IosDefn_MapTrack:
        break;
    }
}

/* --------------------------------------------- */
void Map_AddTrack(int mx1, int my1, int mx2, int my2)
{
    float Lat, Long;

    if (Tracks.Size < MaxTracks)
    {
        Map_ScreenToGlobe((float) (mx1), (float) (my1), &Lat, &Long);
        Tracks.Lat1[Tracks.Size + 1]  = Lat; // skipping zeroth element...
        Tracks.Long1[Tracks.Size + 1] = Long;
        Map_ScreenToGlobe((float) (mx2), (float) (my2), &Lat, &Long);
        Tracks.Lat2[Tracks.Size + 1]  = Lat;
        Tracks.Long2[Tracks.Size + 1] = Long;
        Tracks.Size                  += 1;
        if (Tracks.Size >= MaxTracks)
        {
            printf("Too many tracks (%d)\n", MaxTracks);
            exit(-1);
        }
    }
}

/* --------------------------------------------- */
void Circle(int x, int y, int points, int size)
{
    int   i;
    float angle;
    float x1 = (float) x;
    float y1 = (float) (y + size);
    float x2, y2;
    
    for (i = 0; i <= points; i++)
    {
        angle = (TWOPI * (float) i) / (float) points;
        x2 = x + sin(angle) * (float) size;
        y2 = y + cos(angle) * (float) size;
        Glib_Draw((int) x1, (int) y1, (int) x2, (int) y2);
        x1 = x2;
        y1 = y2;
    }
}

/* --------------------------------------------- */
void Map_CompassRose(int x, int y)
{
    unsigned int a;
    float        ax;
    float        r;
    float        x1, y1, x2, y2;
    float        dm;

    char *cangles[] = { "", "30", "60", "90", "120", "150", "180", "210", "240", "270", "300", "330" };

    dm = -MagneticVariation;

    Glib_LineWidth(3.0);
    Glib_Colour(Glib_GREY);
    Circle(x, y, 40, 100);
    Glib_LineWidth(1.0);

    for (a = 0; a < 360; a += 10)
    {
        ax = Maths_Rads(90.0 - (float) a) + dm;
        if (a == 0)
        {
            r = 0.0;
        }
        else if (a % 30 == 0)
        {
            r = 80.0;
        }
        else
        {
            r = 90.0;
        }
        x1 = x + (int) (r * cos(ax));
        y1 = y + (int) (r * sin(ax));
        x2 = x + (int) (100.0 * cos(ax));
        y2 = y + (int) (100.0 * sin(ax));
        Glib_Draw(x1, y1, x2, y2);
    }

    Glib_SetFont(Glib_GFONT10, 6);
    Glib_Colour(Glib_BLACK);
    
    Glib_Translate((float) x, (float) y);
    for (a=30; a<=330; a+=30)
    {
	    Glib_PushMatrix();
        Glib_Rotate(360.0 - (float) a - Maths_Degrees(MagneticVariation));
        Glib_Chars(cangles[a/30], (a > 100) ? -10 : -6, 70);
	    Glib_PopMatrix();
    }
    Glib_LoadIdentity();  /* restore absolute coords */
}

/* --------------------------------------------- */
void DrawTrack(float Lat1, float Long1, float Lat2, float Long2)
{
    float x1, y1, x2, y2;

    Map_GlobeToScreen(Lat1, Long1, &x1, &y1);
    Map_GlobeToScreen(Lat2, Long2, &x2, &y2);
    Glib_Colour(Glib_GREY);
    Glib_Draw(x1, y1, x2, y2);
}

/* --------------------------------------------- */
void Map_SetMapScaleFactor(float s) //(* units metres/pixel *)
{
    if (s < 20.0)
    {
        Map_MapScaleFactor = 20.0;
    }
    else if (s > 5000.0)
    {
        Map_MapScaleFactor = 5000.0;
    }
    else
    {
        Map_MapScaleFactor = s;
    }
}

/* --------------------------------------------- */
float Map_GetMapScaleFactor(void)  //(* units metres/pixel *)
{
    return Map_MapScaleFactor;
}

/* --------------------------------------------- */
void DrawWayPoint(int x1, int y1)
{
    Glib_Colour(Glib_GREEN);
    Glib_Draw(x1 - 5, y1, x1 + 5, y1);
    Glib_Draw(x1, y1 - 5, x1, y1 + 5);
}

/* --------------------------------------------- */
void DrawRunway(int x1, int y1, NavLib_RunwayRecord r)
{
    float q;
    float Len;
    float dLat;
    float dLong;
    float sx1, sy1;
    float sx2, sy2;
    float dx, dy;

    //float tx = 7.0 * 0.125; /* blue circle */
    //Glib_DrawTexture(x1-16, y1-16, 32, 32, tx, 0.0, tx+0.125, 1.0, 1.0); omit runway circle, DJA 26/10/20
    
    q = r.Qdm + MagneticVariation;
    q = Maths_Normalise(q);
    Len = (float)r.Length;
    dLat = (Len * cos(q)) / EarthRadius;
    dLong = (Len * sin(q)) / (cos(r.RunwayLatitude) * EarthRadius);
    Map_GlobeToScreen(r.RunwayLatitude + dLat, r.RunwayLongitude + dLong, &sx2, &sy2);
    sx1 = (float) x1;
    sy1 = (float) y1;
    dx = sx1 - sx2;
    dy = sy1 - sy2;

    if ( sqrt(dx * dx + dy * dy) > 20.0 ) 
    {
        Glib_Colour(Glib_GREY);
        Glib_Draw(sx1, sy1, sx2, sy2);
    }
}

/* --------------------------------------------- */
void DrawBeacon(int x, int y, NavLib_BeaconRecord n)
{
    char              str[6];
    float             tx;
    NavLib_BeaconType t = n.Navaid;  /* no TACAN or VORTAC in Navigraph database */
    char              Ident[6];
    float             qdm = n.Qdm + MagneticVariation;
    
    qdm = Maths_Normalise(qdm);
    memcpy(Ident, &n.Ident, 6);
    
    Glib_Colour(Glib_BLUE);
    Glib_SetFont(Glib_GFONT10, 6);

    if (t & NavLib_NDB)
	{
        tx = 3.0 * 0.125;	
        Glib_DrawTexture(x-16, y-16, 32, 32, tx, 0.0, tx+0.125, 1.0, 1.0);
        Glib_Chars(Ident, x + 10, y + 10);
    }
    else if (t & NavLib_VOR)
	{
        if (t & NavLib_DME)
		{
            tx = 2.0 * 0.125;
            Glib_DrawTexture(x-16, y-16, 32, 32, tx, 0.0, tx+0.125, 1.0, 1.0);
		}
        tx = 1.0 * 0.125;
        Glib_DrawTexture(x-16, y-16, 32, 32, tx, 0.0, tx+0.125, 1.0, 1.0);
        Glib_Chars(Ident, x - 12, y - 22);
    }
    else if (t & NavLib_ILS)
	{
		int i;
		int y2 = y;
		
		for (i=1; i<=nDrawnILS; i+=1)
		{
		    float dx = (float) (x - DrawnILS[i].x);
		    float dy = (float) (y - DrawnILS[i].y);
		    float d = sqrt(dx * dx + dy * dy);

    		if (d < 5.0)
	    	{
		        y2 -= 25;
				break;
	    	}
		}
	    nDrawnILS += 1;
		DrawnILS[nDrawnILS].x = x;
		DrawnILS[nDrawnILS].y = y;
		
		tx = 6.0 * 0.125;
		Glib_DrawTexture(x-16, y-16, 32, 32, tx, 0.0, tx+0.125, 1.0, 1.0);
		
		sprintf(str, "%s", Ident);
		if (qdm > -DEG90 && qdm < DEG90)
		{
			Glib_Chars(str, x - Glib_StringSize(Ident) - 9, y2 + 6);
		}
		else
		{
			Glib_Chars(str, x + 12, y2 - 10);
		}
    }
}

/* --------------------------------------------- */
void Map_DrawMap(void)
{
    int          LatMin;      /* degrees */
    int          LatMax;      /* degrees */
    int          LongMin;     /* degrees */
    int          LongMax;     /* degrees */
    int          Lat, Long;
    float        sx1, sy1, sx2, sy2;
    unsigned int i;
    char         str[50];

    Map_SetMapMagneticVariation(IosLink_NavPkt.MagneticVariation);
	
    nDrawnILS = 0;
	
    LatMin = (int) (Maths_Degrees(MapMinLatitude)) * 2;
    if (LatMin < 0)
    {
        LatMin -= 2;
    }
    LatMax = (int) (Maths_Degrees(MapMaxLatitude)) * 2;
    if (LatMax > 0)
    {
        LatMax += 2;
    }
    LongMin = (int) (Maths_Degrees(MapMinLongitude)) * 2;
    if (LongMin < 0)
    {
        LongMin -= 2;
    }
    LongMax = (int) (Maths_Degrees(MapMaxLongitude)) * 2;
    if (LongMax > 0)
    {
        LongMax += 2;
    }

    Glib_LineWidth(1.0);
    Glib_SetFont(Glib_GFONT10, 6);
    Glib_Colour(Glib_GREY);
    Glib_ClipWindow(MapMinX, MapMinY, MapWidth, MapDepth);
    
    for (Long = LongMin; Long <= LongMax; Long += 1)
    {
        Map_GlobeToScreen(Maths_Rads((float) (LatMin) * 0.5), Maths_Rads((float) (Long) * 0.5), &sx1, &sy1);
        Map_GlobeToScreen(Maths_Rads((float) (LatMax) * 0.5), Maths_Rads((float) (Long) * 0.5), &sx2, &sy2);
        Glib_Draw(sx1, sy1, sx2, sy2);

        if ((Long % 2) == 0)
        {
            float x0;
            float a, b;
            a  = (sy2 - sy1) / (sx2 - sx1);
            b  = sy1 - a * sx1;
            x0 = -b / a;

            sprintf(str, "%d%c", abs(Long) / 2, (Long >= 0) ? 'E' : 'W');
            if (sy1 < 0.0 && sy2 > 0.0 && x0 > 0.0 && x0 < (float) Glib_SCREENWIDTH)
            {
                Glib_Chars(str, (int) x0 + 3, 5);
            }
            else
            {
                Glib_Chars(str, (int) sx1 + 3, (int) sy1 + 4);
            }
            x0 = (sy2 - b) / a;
            if (sy2 > (float) Glib_SCREENHEIGHT && sy1 < (float) Glib_SCREENHEIGHT && x0 > 0.0 && x0 < (float) Glib_SCREENWIDTH)
            {
                x0 = ((float) Glib_SCREENHEIGHT - b) / a;
                Glib_Chars(str, (int) x0 + 3, Glib_SCREENHEIGHT - 13);
            }
            else
            {
                Glib_Chars(str, (int) sx2 + 3, (int) sy2 - 10);
            }
        }
    }

    for (Lat = Min(LatMin, LatMax); Lat <= Max(LatMin, LatMax); Lat += 1)
    {
        for (Long = LongMin; Long <= LongMax - 1; Long += 1)
        {
            Map_GlobeToScreen(Maths_Rads((float) (Lat) * 0.5), Maths_Rads((float ) (Long) * 0.5), &sx1, &sy1);
            Map_GlobeToScreen(Maths_Rads((float) (Lat) * 0.5), Maths_Rads((float) (Long) * 0.5 + 0.5), &sx2, &sy2);
            Glib_Draw(sx1, sy1, sx2, sy2);

            if (((Lat % 2) == 0) && (((LatMax > 0) && (Lat < LatMax)) || ((LatMin < 0) && (Lat == LatMin))))
            {
                float a  = (sy2 - sy1) / (sx2 - sx1);
                float b  = sy1 - a * sx1;
                float y0 = a * sx1 + b;

                sprintf(str, "%2d%c", abs(Lat) / 2, (Lat >= 0) ? 'N' : 'S');
                if (sx1 < MapMinX && sx2 > MapMinX && y0 > 0.0 && y0 < (float) Glib_SCREENWIDTH)
                {
                    Glib_Chars(str, MapMinX + 3, (int) y0);
                }
                else if (Long == LongMin)
                {
                    Glib_Chars(str, (int) sx1, (int) sy1);
                }

                y0 = a * sx2 + b;
                if (sx1 < (float) SCREENWIDTH && sx2 > (float) SCREENWIDTH && y0 > 0.0 && y0 < (float) Glib_SCREENHEIGHT)
                {
                    Glib_Chars(str, SCREENWIDTH - Glib_StringSize(str) - 5, (int) y0);
                }
                else
                {
                    if (Long == (LongMax - 1))
                    {
                        Glib_Chars(str, (int) sx2 - strlen(str) * 6, (int) sy2);
                    }
                }
            }
        }
    }

    for (i = 1; i <= NavLib_NumberOfBeacons; i+=1)
    {
        Map_GlobeToScreen(NavLib_Beacons[i].BeaconLatitude, NavLib_Beacons[i].BeaconLongitude, &sx1, &sy1);
        if ((sx1 >= MapMinX) && (sx1 <= MapMaxX) && (sy1 >= MapMinY) && (sy1 <= MapMaxY))
        {
            DrawBeacon(entier(sx1), entier(sy1), NavLib_Beacons[i]);
        }
    }

    for (i = 1; i <= NavLib_NumberOfRunways; i+=1)
    {
        Map_GlobeToScreen(NavLib_Runways[i].RunwayLatitude, NavLib_Runways[i].RunwayLongitude, &sx1, &sy1);
        if ((sx1 >= MapMinX) && (sx1 <= MapMaxX) && (sy1 >= MapMinY) && (sy1 <= MapMaxY))
        {
            DrawRunway(entier(sx1), entier(sy1), NavLib_Runways[i]);
        }
    }

    for (i = 1; i <= NavLib_NumberOfWayPoints; i+=1)
    {
        Map_GlobeToScreen(Maths_Rads(NavLib_WayPoints[i].WayPointLatitude), Maths_Rads(NavLib_WayPoints[i].WayPointLongitude), &sx2, &sy2);
        DrawWayPoint(entier(sx2), entier(sy2));
        if (i >= 2)
        {
            //glEnable(GL_LINE_STIPPLE);
            //Glib_Flush();
            //glLineStipple(4, 0xAAAA);
            Glib_Draw(sx1, sy1, sx2, sy2);
            //Glib_Flush();
            //glDisable(GL_LINE_STIPPLE);
        }
        sx1 = sx2;
        sy1 = sy2;
    }

    if (Compasses.Size > 0)
    {
        for (i = 0; i < Compasses.Size; i+=1)
        {
		    float x, y;
			
			Map_GlobeToScreen(Compasses.CLat[i], Compasses.CLong[i], &x, &y);
            Map_CompassRose(x, y);
        }
    }

    if (Tracks.Size > 0)
    {
        for (i = 1; i <= Tracks.Size; i+=1)
        {
            DrawTrack(Tracks.Lat1[i], Tracks.Long1[i], Tracks.Lat2[i], Tracks.Long2[i]);
        }
    }

    Map_ShowAircraftTrack();

    ShowTarget();

    Glib_RemoveClipWindow();
}

/* --------------------------------------------- */
int Min(int a, int b)
{
    if (a < b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

/* --------------------------------------------- */
int Max(int a, int b)
{
    if (a > b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

/* --------------------------------------------- */
void Map_DisplayCoordinates(int Mx, int My)
{
    float Lat, Long;
    int   Mins;
    char  str[50];

    Map_ScreenToGlobe((float) Mx, (float) My, &Lat, &Long);
    Mins = entier(60.0 * Maths_Degrees(fabs(Lat)));

    Glib_Colour(Glib_BLACK);
    Glib_SetFont(Glib_GFONT10, 8);
    sprintf(str, "% 02d %02d%c", Mins / 60, Mins % 60, (Lat >= 0.0) ? 'N' : 'S');
    Glib_Chars(str, Mx + 16, My - 30);

    Mins = entier(60.0 * Maths_Degrees(Long));
    sprintf(str, (abs(Mins) >= 600) ? "%03d %02d%c" : " %02d %02d%c",
            abs(Mins) / 60, abs(Mins) % 60, (Mins < 0) ? 'W' : 'E');
    Glib_Chars(str, Mx + 16, My - 46);
}

/* --------------------------------------------- */
void Map_DisplayDmeDistance(int x1, int y1, int x2, int y2)
{
    float        Lat1, Long1;
    float        Lat2, Long2;
    unsigned int d; /* nm * 10 */
    int          b;
    char         str[50];

    Map_ScreenToGlobe((float) x1, (float) y1, &Lat1, &Long1);
    Map_ScreenToGlobe((float) x2, (float) y2, &Lat2, &Long2);
    d = (unsigned int) (NavLib_Distance(Lat1, Long1, Lat2, Long2) * 0.0053996);
    b = entier(Maths_Degrees(NavLib_Bearing(Lat1, Long1, Lat2, Long2) - MagneticVariation));

//    Map_DisplayCoordinates(x2, y2);

    Glib_SetFont(Glib_GFONT10, 8);
    Glib_Colour(Glib_BLACK);
    if (d > 999)
    {
        Glib_Chars("--.-", x2 + 16, y2 - 62);
    }
    else
    {
        sprintf(str, " %2d.%d NM", d / 10, d % 10);
        Glib_Chars(str, x2 + 16, y2 - 62);
    }
    while (b < 1)
    {
        b = b + 360;
    }
    while (b > 360)
    {
        b = b - 360;
    }
    sprintf(str, " %03d degs", b);
    Glib_Chars(str, x2 + 16, y2 - 78);
}

/* ---------------------------------------------------- */
void ArrowHead(float x, float y, float Hdg, bool filled)
{
    float a, a1, a2;
    float x1, y1, x2, y2;

    a  = 1.570796 - Hdg; /* 90 - Hdg */
    a1 = a - 2.7925268;  /* a - 160 */
    a2 = a + 2.7925268;  /* a + 160 */

    x1 = x + 30.0 * cos(a1);
    y1 = y + 30.0 * sin(a1);
    x2 = x + 30.0 * cos(a2);
    y2 = y + 30.0 * sin(a2);

    if (filled)
    {
        Glib_Triangle(x, y, x1, y1, x2, y2);
    }
    else
    {
        Glib_Draw(x, y, x1, y1);
        Glib_Draw(x, y, x2, y2);
        Glib_Draw(x1, y1, x2, y2);
    }
}

/* ---------------------------------------------------- */
void Map_UpdateTrackList(void)
{
    if (IosLink_AeroPkt.PktNumber > (OldPktNumber + 50))
    {
        if ((fabs(IosLink_AeroPkt.Latitude - OldLatitude) > 0.000015695) ||
            (fabs(IosLink_AeroPkt.Longitude - OldLongitude) > 0.000015695))
        {
            OldLatitude  = IosLink_AeroPkt.Latitude;
            OldLongitude = IosLink_AeroPkt.Longitude;
            OldPktNumber = IosLink_AeroPkt.PktNumber;

            if (AircraftTrackList->TrackLength >= MaxTrackLength)
            {       /* track buffer is full - discard oldest point */
                AircraftTrackList->FirstPoint = NextTrackPoint(AircraftTrackList->FirstPoint);
                AircraftTrackList->TrackLength--;
                if (AircraftTrackList->DisplayedPoints > 0)
                {
                    AircraftTrackList->DisplayedPoints--;
                }
            }

            AircraftLatitude                                             = IosLink_AeroPkt.Latitude;
            AircraftLongitude                                            = IosLink_AeroPkt.Longitude;
            AircraftTrackList->xPoints[AircraftTrackList->NextPoint] = IosLink_AeroPkt.Latitude;
            AircraftTrackList->yPoints[AircraftTrackList->NextPoint] = IosLink_AeroPkt.Longitude;
            AircraftTrackList->NextPoint                                 = NextTrackPoint(AircraftTrackList->NextPoint);
            AircraftTrackList->TrackLength++;
            //printf("Map_UpdateTrackList() Trk Len: %d\n", AircraftTrackList->TrackLength);
        }
        TargetLatitude  = IosLink_AeroPkt.TLatitude;
        TargetLongitude = IosLink_AeroPkt.TLongitude;
        TargetHeading   = IosLink_AeroPkt.TYaw;
    }

    if (fabs(IosLink_AeroPkt.Yaw - OldYaw) > 0.02)
    {
        AircraftHeading = IosLink_AeroPkt.Yaw;
        OldYaw              = IosLink_AeroPkt.Yaw;
    }

    Arrow.Hdg = AircraftHeading;
}

/* ---------------------------------------------------- */
void Map_ShowAircraftTrack(void)
{
    float x0, y0;
    float x1, y1;
    bool firstpoint = true;
	
    AircraftTrackList->DisplayedPoints    = 0;
    AircraftTrackList->NextDisplayedPoint = AircraftTrackList->FirstPoint;

    if (AircraftTrackList->DisplayedPoints < AircraftTrackList->TrackLength)
    {
        Glib_Colour(Glib_BLUE);
        do
        {
            if (AircraftTrackList->NextDisplayedPoint != AircraftTrackList->FirstPoint)
            {
                Map_GlobeToScreen(AircraftTrackList->xPoints[AircraftTrackList->NextDisplayedPoint],
                                  AircraftTrackList->yPoints[AircraftTrackList->NextDisplayedPoint],
                                  &x1, &y1);
				if (firstpoint)
				{
					x0 = x1;
					y0 = y1;
					firstpoint = false;
				}
                else if ((fabs(x1 - x0) > 1.0) || (fabs(y1 - y0) > 1.0))
                {
                    Glib_Draw(x0, y0, x1, y1);
					x0 = x1;
					y0 = y1;
                }
            }
            AircraftTrackList->NextDisplayedPoint = NextTrackPoint(AircraftTrackList->NextDisplayedPoint);
            AircraftTrackList->DisplayedPoints += 1;
        } while (AircraftTrackList->DisplayedPoints < AircraftTrackList->TrackLength);
    }

    Glib_Colour(Glib_BLACK);
    Map_GlobeToScreen(AircraftLatitude, AircraftLongitude, &Arrow.x, &Arrow.y);
    ArrowHead(Arrow.x, Arrow.y, AircraftHeading, false);
    Arrow.Hdg = AircraftHeading;
}

/* ---------------------------------------------------- */
void ShowTarget(void)
{
    float sx, sy;

    Map_GlobeToScreen(TargetLatitude, TargetLongitude, &sx, &sy);
    ArrowHead(sx, sy, TargetHeading, true);
}

/* ---------------------------------------------------- */
unsigned int NextTrackPoint(unsigned int n)
{
    if (n >= MaxTrackLength)
    {
        return 1;
    }
    else
    {
        return n + 1;
    }
}

/* ---------------------------------------------------- */
unsigned int PrevTrackPoint(unsigned int n)
{
    if (n <= 1)
    {
        return MaxTrackLength;
    }
    else
    {
        return n - 1;
    }
}

/* --------------------------------------------- */
void Map_InitialiseTrackList(void)
{
    if (AircraftTrackList != NULL)
    {
        free(AircraftTrackList);
    }
    AircraftTrackList = malloc(sizeof(AircraftTrackRecord));
    if (!AircraftTrackList)
    {
        printf("Memory error : AircraftTrackList\n");
        exit(1);
    }

    AircraftTrackList->TrackLength        = 0;
    AircraftTrackList->DisplayedPoints    = 0;
    AircraftTrackList->FirstPoint         = 1;
    AircraftTrackList->NextPoint          = 1;
    AircraftTrackList->NextDisplayedPoint = 1;

    OldLatitude  = 0.0;
    OldLongitude = 0.0;
    OldYaw       = 0.0;
    OldPktNumber = 0;
}

/* --------------------------------------------- */
void BEGIN_Map()
{
    MagneticVariation = 0.0;
    Map_SetMapScaleFactor(Map_Default_Scale_Factor);
    Map_Init();
    Map_SetMapCentre((MapMinLatitude + MapMaxLatitude) * 0.5,
                     (MapMinLongitude + MapMaxLongitude) * 0.5);
    AircraftLatitude  = (MapMinLatitude + MapMaxLatitude) * 0.5;
    AircraftLongitude = (MapMinLongitude + MapMaxLongitude) * 0.5;
    AircraftHeading   = 0.0;
    TargetLatitude    = 0.0;
    TargetLongitude   = 0.0;
    TargetHeading     = 0.0;
    Map_InitialiseTrackList();
}
