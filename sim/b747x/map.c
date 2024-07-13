#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <SIM/iosdefn.h>
#include <SIM/maths.h>
#include <SIM/glibx.h>
#include <SIM/navlib.h>

#include "map.h"
#include "ioslink.h"

#define DEG6    (6.0 / Maths_ONERAD)
#define D12     (Maths_TWOPI * EarthRadius / 30.0)

#define MAXPOINTS 200  /* maximum line segment vertices in any character */
#define EOC        -1  /* end of character */

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
    float        Lat1[Map_MaxTracks];
    float        Long1[Map_MaxTracks];
    float        Lat2[Map_MaxTracks];
    float        Long2[Map_MaxTracks];
} TrackList;

typedef struct
{
    float x;
    float y;
    float Hdg;
} ArrowRecord;

/* variables */

Map_AircraftTrackRecord *Map_AircraftTrackList = NULL;
float                   Map_AircraftLatitude;
float                   Map_AircraftLongitude;
float                   Map_AircraftHeading;
float                   Map_TargetLatitude;
float                   Map_TargetLongitude;
float                   Map_TargetHeading;

float                   Map_MapMinLatitude  = Maths_PIBY2;
float                   Map_MapMaxLatitude  = -Maths_PIBY2;
float                   Map_MapMinLongitude = Maths_TWOPI;
float                   Map_MapMaxLongitude = -Maths_TWOPI;
float                   Map_MapLatitude     = 0.0;
float                   Map_MapLongitude    = 0.0;
float                   Map_MapScaleFactor  = 0.0;

float                   Lambert_X;       /* standards parallel */
float                   Lambert_N;       /* convergence factor */
float                   Lambert_K;       /* constant term */
float                   msf;             /* metres scale factor */
float                   MapOffset;       /* y offset of screen in metres */

float                   MagneticVariation;

Map_CompassList         Map_Compasses;
TrackList               Tracks;
ArrowRecord             Arrow;

float                   OldLatitude;
float                   OldLongitude;
float                   OldYaw;
static unsigned int     OldPktNumber;
unsigned int            FileNumber;

static unsigned int     FontHeight;
static unsigned int     FontWidth;
static unsigned int     FontLineWidth;
static unsigned int     FontPoints;
static char             FontTable[128 * MAXPOINTS];
static unsigned int     CharLengths[128];

void Map_ShowTarget(void);
unsigned int NextTrackPoint(unsigned int);
unsigned int PrevTrackPoint(unsigned int);
int Min(int, int);
int Max(int, int);
void Circle5(int x1, int y1);

void MapChar(char Ch, int x0, int y0);
void Map_Chars(char str[], int x, int y);
void ReadMapFontFile(char FileName[]);
unsigned int CharSize(char v[]);

/* ------------------------------------------------------------- */
void MapChar(char Ch, int x0, int y0)
{
    unsigned int p;
	int          x, y;

    p = (int) Ch * MAXPOINTS;

    glRasterPos2i(x0, y0);
    Glibx_AntiAliasing(true);
	
    while (1)
	{
		glBegin(GL_LINE_STRIP);
		while (1)
		{
			x = (int) FontTable[p];
			p += 1;
			if (x < 0)
			{
				break;
			}
			y = (int) FontTable[p];
			p += 1;
			glVertex2i(x0 + x, y0 + y);
		}
		glEnd();
        if (x == EOC)
		{
		    break;
		}
	}
}

/* ------------------------------------------------- */
void Map_Chars(char str[], int x, int y)
{
    unsigned int p;
    char         Ch;

    p = 0;
    while (1)
    {
        Ch = str[p];
        if (Ch == 0)
        {
            break;
        }
        else
        {
            MapChar(Ch, x, y);
            x = x + FontWidth;
            p = p + 1;
        }
    }
}

/* ------------------------------------------------------------- */
void ReadMapFontFile(char FileName[])
{
    FILE         *f;
	unsigned int i;
    unsigned int h;
    unsigned int w;
    unsigned int lwidth;
	
    f = fopen(FileName, "r+b");
    if (f == NULL)
    {
        printf("Can't open font file %s\n", FileName);
        exit(EXIT_FAILURE);
    }

    fread((char *) &h, 1, 4, f);
    fread((char *) &w, 1, 4, f);
    fread((char *) &lwidth, 1, 4, f);
    fread((char *) &FontPoints, 1, 4, f);
    if (FontHeight == 0)
	{
	    FontHeight = h;
	}
    if (FontWidth == 0)
	{
	    FontWidth = w;
	}
    if (FontLineWidth == 0)
	{
	    FontLineWidth = lwidth;
	}
	
    memset(FontTable, 0, sizeof(FontTable));	
	for (i=0; i<128; i+=1)
	{
    	unsigned int p = i * MAXPOINTS;
		fread(&(FontTable[p]), 1, FontPoints, f);
		CharLengths[i] = CharSize(&FontTable[p]);
	}
    fclose(f);
}

/* ---------------------------------- */
unsigned int CharSize(char v[])
{
    unsigned int p = 0;
    int          x;
	
    while (1)
	{
		while (1)
		{
			x = (int) v[p];
			p += 1;
			if (x < 0)
			{
				break;
			}
			p += 1;
		}
        if (x == EOC)
		{
		    break;
		}
	}
	return p;
}

/* ------------------------------------------------------------- */
void Map_SetMapCentre(float Latitude, float Longitude)
{
    float Chi;
    float lat1;
    float lat2;
    float r;
    float r1;
    float r2;

    Map_MapLatitude  = Latitude;
    Map_MapLongitude = Longitude;
    Chi              = Maths_PIBY2 - fabs(Latitude);
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

    Chi   = Maths_PIBY2 - fabs(Latitude);
    Theta = Lambert_N * (Longitude - Map_MapLongitude);
    r     = Lambert_K * pow(tan(Chi / 2.0), Lambert_N);
    *x    = (r * sin(Theta) * msf / Map_MapScaleFactor) + MapCentreX;
    t     = (r * cos(Theta) * msf - MapOffset) / Map_MapScaleFactor;
    if (Map_MapLatitude > 0.0)
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
    if (Map_MapLatitude > 0.0)
    {
        dy = (MapCentreY - y) * Map_MapScaleFactor + MapOffset;
    }
    else
    {
        dy = (y - MapCentreY) * Map_MapScaleFactor + MapOffset;
    }
    r          = sqrt(dx * dx + dy * dy) / msf;
    Theta      = asin((x - MapCentreX) * Map_MapScaleFactor / (r * msf));
    *Longitude = Theta / Lambert_N + Map_MapLongitude;
    Chi        = 2.0 * atan(pow(r / Lambert_K, 1.0 / Lambert_N));
    *Latitude  = Maths_PIBY2 - Chi;
    if (Map_MapLatitude < 0.0)
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
long _TRUNC(float n)
{
    return (long) fabs(n);
}

/* --------------------------------------------- */
void Map_InitMinMaxLatLong(void)
{
    unsigned int i;
    float        Lat, Long;

    for (i = 1; i < NavLib_NumberOfBeacons; i = i + 1)
    {
        Lat  = NavLib_Beacons[i].BeaconLatitude;
        Long = NavLib_Beacons[i].BeaconLongitude;

        if (Lat < Map_MapMinLatitude)
        {
            Map_MapMinLatitude = Lat;
        }
        else if (Lat > Map_MapMaxLatitude)
        {
            Map_MapMaxLatitude = Lat;
        }

        if (Long < Map_MapMinLongitude)
        {
            Map_MapMinLongitude = Long;
        }
        else if (Long > Map_MapMaxLongitude)
        {
            Map_MapMaxLongitude = Long;
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
        Map_SetMapCentre(Map_AircraftLatitude, Map_AircraftLongitude);
        break;

    case IosDefn_MapCentre:
        // Get mouse xy
        Map_ScreenToGlobe((float) (Mouse_Mx), (float) (Mouse_My), &Lat, &Long);
        Map_SetMapCentre(Lat, Long);
        break;

    case IosDefn_MapCompass:
        if (Map_Compasses.Size < Map_MaxCompasses)
        {
            Map_ScreenToGlobe((float) (Mouse_Mx), (float) (Mouse_My), &Lat, &Long);
            Map_Compasses.CLat[Map_Compasses.Size]  = Lat;
            Map_Compasses.CLong[Map_Compasses.Size] = Long;
            Map_Compasses.Size                      = Map_Compasses.Size + 1;
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
        Map_Compasses.Size = 0;
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

    if (Tracks.Size < Map_MaxTracks)
    {
        Map_ScreenToGlobe((float) (mx1), (float) (my1), &Lat, &Long);
        Tracks.Lat1[Tracks.Size + 1]  = Lat; // skipping zeroth element...
        Tracks.Long1[Tracks.Size + 1] = Long;
        Map_ScreenToGlobe((float) (mx2), (float) (my2), &Lat, &Long);
        Tracks.Lat2[Tracks.Size + 1]  = Lat;
        Tracks.Long2[Tracks.Size + 1] = Long;
        Tracks.Size                  += 1;
        if (Tracks.Size >= Map_MaxTracks)
        {
            printf("Too many tracks (%d)\n", Map_MaxTracks);
            exit(-1);
        }
    }
}

/* --------------------------------------------- */
void Circle(float x, float y, int points, float size)
{
    int   i;
    float angle;

    glPushMatrix();
    glTranslatef(x, y, 0.0f);
    glBegin(GL_LINE_LOOP);
    for (i = 0; i < points; i++)
    {
        angle = ((float) Maths_TWOPI * i) / points;
        glVertex2f((float) cos(angle) * size, (float) sin(angle) * size);
    }
    glEnd();
    glPopMatrix();
}

/* --------------------------------------------- */
void Map_CompassRose(float Lat, float Long)
{
    unsigned int a;
    float        ax;
    float        r;
    float        x1, y1, x2, y2;
    float        dm;
    int          dx, dy;
    float        xc, yc;
    float        Lat1, Long1;

    Map_GlobeToScreen(Lat, Long, &xc, &yc);
    Map_ScreenToGlobe(xc, yc + 100.0, &Lat1, &Long1);
    dm = NavLib_Bearing(Lat, Long, Lat1, Long1) - Maths_Rads(MagneticVariation);
    if (dm > Maths_PI)
    {
        dm = dm - Maths_TWOPI;
    }
    dx = entier(70.0 * cos(dm));
    dy = entier(70.0 * sin(dm));

    Glibx_Colour(Glibx_GREY);
    Circle(entier(xc), entier(yc), 40, 100);

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
        x1 = xc + r * cos(ax);
        y1 = yc + r * sin(ax);
        x2 = xc + 100.0 * cos(ax);
        y2 = yc + 100.0 * sin(ax);
        Map_Draw(x1, y1, x2, y2);
    }

    //Glibx_SetFont(Glibx_GFONT10, 6);
    Glibx_Colour(Glibx_BLACK);
    Map_Chars("90", entier(xc) + dx - 3, entier(yc) + dy - 2);
    Map_Chars("270", entier(xc) - dx - 9, entier(yc) - dy - 4);
    Map_Chars("180", entier(xc) + dy - 9, entier(yc) - dx - 8);
}

/* --------------------------------------------- */
void DrawTrack(float Lat1, float Long1, float Lat2, float Long2)
{
    float x1, y1, x2, y2;

    Map_GlobeToScreen(Lat1, Long1, &x1, &y1);
    Map_GlobeToScreen(Lat2, Long2, &x2, &y2);
    Glibx_Colour(Glibx_GREY);
    Map_Draw(x1, y1, x2, y2);
}

/* --------------------------------------------- */
void Map_DrawTrackScreen(float x1, float y1, float x2, float y2)
{
    Glibx_Colour(Glibx_BLACK);
    Map_Draw(x1, y1, x2, y2);
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
void Map_Draw(float x1, float y1, float x2, float y2)  // returns bool
{
    glBegin(GL_LINES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glEnd();
}

/* --------------------------------------------- */
void DrawWayPoint(int x1, int y1)
{
    Glibx_Colour(Glibx_GREEN);
    Glibx_Draw(x1 - 5, y1, x1 + 5, y1);
    Glibx_Draw(x1, y1 - 5, x1, y1 + 5);
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

    Glibx_Colour(Glibx_RED);
    Circle5(x1, y1);
    
    q = Maths_Rads(r.Qdm + MagneticVariation);
    Maths_Normalise(&q);
    Len = (float)r.Length;
    dLat = (Len * cos(q)) / EarthRadius;
    dLong = (Len * sin(q)) / (cos(r.RunwayLatitude) * EarthRadius);
    Map_GlobeToScreen(r.RunwayLatitude + dLat, r.RunwayLongitude + dLong, &sx2, &sy2);
    sx1 = (float)x1;
    sy1 = (float)y1;
    dx = sx1 - sx2;
    dy = sy1 - sy2;

    if ( sqrt(dx * dx + dy * dy) > 5.0 ) {
        Glibx_Colour(Glibx_GREY);
        Glibx_Draw(sx1, sy1, sx2, sy2);
    }

}

/* --------------------------------------------- */
void Circle5(int x1, int y1)
{
    float        x, y, dx, dy;
    int          i;
    int          p = 0;

    static float circ[38] =
    {
        0.0,   5.0,  1.7,  4.7,  3.2,  3.8,  4.3,  2.5,  4.9,  0.9,  4.9, -0.9,
        4.3,  -2.5,  3.2, -3.8,  1.7, -4.7, -0.0, -5.0, -1.7, -4.7, -3.2, -3.8,
        -4.3, -2.5, -4.9, -0.9, -4.9,  0.9, -4.3,  2.5, -3.2,  3.8, -1.7,  4.7,
        0.0, 5.0
    };

    x = (float) x1;
    y = (float) y1;
    glBegin(GL_LINE_STRIP);

    for (i = 0; i <= 18; i += 1)
    {
        dx = circ[p];
        p  = p + 1;
        dy = circ[p];
        p  = p + 1;
        glVertex2f(x + dx, y + dy);
    }
    glEnd();
}

/* --------------------------------------------- */
void DrawNDBIcon(int x1, int y1)
{
    int        x, y, dx, dy;
    int        i;
    int        p = 0;

    static int dots7[40] =
    {
        5,   0,  3,  1,  1,  3,  0,  5,  0,  7,
        0,   9,  1, 11,  3, 13,  5, 14,  7, 14,
        9,  14, 11, 13, 13, 11, 14,  9, 14,  7,
        14,  5, 13,  3, 11,  1,  9,  0,  7, 0
    };
    static int dots5[32] =
    {
        5,   2,  3,  3,  2,  5,  2,  7,  2,  9,
        3,  11,  5, 12,  7, 12,  9, 12, 11, 11,
        12,  9, 12,  7, 12,  5, 11,  3,  9,  2,
        7, 2
    };
    static int dots3[32] =
    {
        5,   5,  4,  6, 4,  7, 4, 8,  5, 9,
        6,  10,  7, 10, 8, 10, 9, 9, 10, 8,
        10,  7, 10,  6, 9,  5, 8, 4,  7, 4,
        6, 4
    };

    x = x1;
    y = y1;
    Glibx_Colour(Glibx_BLUE);

    p = 0;
    glBegin(GL_POINTS);
    for (i = 0; i <= 19; i += 1)
    {
        dx = dots7[p];
        p  = p + 1;
        dy = dots7[p];
        p  = p + 1;
        glVertex2i(x + dx - 7, y + dy - 7);  /* -7 to realign NDB symbol */
    }
    glEnd();

    p = 0;
    glBegin(GL_POINTS);
    for (i = 0; i <= 15; i += 1)
    {
        dx = dots5[p];
        p  = p + 1;
        dy = dots5[p];
        p  = p + 1;
        glVertex2i(x + dx - 7, y + dy - 7);
    }
    glEnd();

    p = 0;
    glBegin(GL_POINTS);
    for (i = 0; i <= 15; i += 1)
    {
        dx = dots3[p];
        p  = p + 1;
        dy = dots3[p];
        p  = p + 1;
        glVertex2i(x + dx - 7, y + dy - 7);
    }
    glEnd();
}

/* --------------------------------------------- */
void DrawBeacon(int x, int y, NavLib_BeaconType t, char Ident[])
{
    char       str[5];

    static int VOR_symbol[14] =
    {
        -8, 0, -4, 7, 4, 7, 8, 0, 4, -7, -4, -7, -8, 0
    };
    static int DME_symbol[10] =
    {
        -8, -7, -8, 7, 8, 7, 8, -7, -8, -7
    };

    Glibx_Colour(Glibx_BLUE);
    //Glibx_SetFont(Glibx_GFONT10, 6);

    if (t & NavLib_NDB)
    {
        DrawNDBIcon(x, y);
        Map_Chars(Ident, x + 7, y + 5);
    }
    else if (t & NavLib_VOR)
    {
        if (t & NavLib_DME)
        {
            Glibx_DrawLines(5, x, y, DME_symbol);
        }
        Glibx_DrawLines(7, x, y, VOR_symbol);
        Glibx_Rectangle(x - 1, y - 1, 2, 2);
        Map_Chars(Ident, x - 12, y - 22);
    }
    else if (t & NavLib_ILS)
    {
        Glibx_Colour(Glibx_RED);
        Circle5(x, y);
        sprintf(str, "%s", Ident);
        Map_Chars(str, x - strlen(Ident) * FontWidth - 5, y);
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

    LatMin = (int) (Maths_Degrees(Map_MapMinLatitude)) * 2;
    if (LatMin < 0)
    {
        LatMin -= 2;
    }
    LatMax = (int) (Maths_Degrees(Map_MapMaxLatitude)) * 2;
    if (LatMax > 0)
    {
        LatMax += 2;
    }
    LongMin = (int) (Maths_Degrees(Map_MapMinLongitude)) * 2;
    if (LongMin < 0)
    {
        LongMin -= 2;
    }
    LongMax = (int) (Maths_Degrees(Map_MapMaxLongitude)) * 2;
    if (LongMax > 0)
    {
        LongMax += 2;
    }
    Glibx_LineWidth(1.0);
    //Glibx_SetFont(Glibx_GFONT10, 6);
    Glibx_Colour(Glibx_GREY);
    Glibx_ClipWindow(MapMinX, MapMinY, MapWidth, MapDepth);
	
    for (Long = LongMin; Long <= LongMax; Long += 1)
    {
        Map_GlobeToScreen(Maths_Rads((float) (LatMin) * 0.5), Maths_Rads((float) (Long) * 0.5), &sx1, &sy1);
        Map_GlobeToScreen(Maths_Rads((float) (LatMax) * 0.5), Maths_Rads((float) (Long) * 0.5), &sx2, &sy2);
        Map_Draw(sx1, sy1, sx2, sy2);

        if ((Long % 2) == 0)
        {
            float x0;
            float a, b;
            a  = (sy2 - sy1) / (sx2 - sx1);
            b  = sy1 - a * sx1;
            x0 = -b / a;

            sprintf(str, "%d%c", abs(Long) / 2, (Long >= 0) ? 'E' : 'W');
            if (sy1 < 0.0 && sy2 > 0.0 && x0 > 0.0 && x0 < (float) Glibx_SCREENWIDTH)
            {
                Map_Chars(str, (int) x0 + 3, 5);
            }
            else
            {
                Map_Chars(str, (int) sx1 + 3, (int) sy1 +4);
            }
            x0 = (sy2 - b) / a;
            if (sy2 > (float) Glibx_SCREENHEIGHT && sy1 < (float) Glibx_SCREENHEIGHT && x0 > 0.0 && x0 < (float) Glibx_SCREENWIDTH)
            {
                x0 = ((float) Glibx_SCREENHEIGHT - b) / a;
                Map_Chars(str, (int) x0 + 3, Glibx_SCREENHEIGHT - 20);
            }
            else
            {
                Map_Chars(str, (int) sx2 + 3, (int) sy2 - 10);
            }
        }
    }

    for (Lat = Min(LatMin, LatMax); Lat <= Max(LatMin, LatMax); Lat += 1)
    {
        for (Long = LongMin; Long <= LongMax - 1; Long += 1)
        {
            Map_GlobeToScreen(Maths_Rads((float) (Lat) * 0.5), Maths_Rads((float ) (Long) * 0.5), &sx1, &sy1);
            Map_GlobeToScreen(Maths_Rads((float) (Lat) * 0.5), Maths_Rads((float) (Long) * 0.5 + 0.5), &sx2, &sy2);
            Map_Draw(sx1, sy1, sx2, sy2);

            if (((Lat % 2) == 0) && (((LatMax > 0) && (Lat < LatMax)) || ((LatMin < 0) && (Lat == LatMin))))
            {
                float a  = (sy2 - sy1) / (sx2 - sx1);
                float b  = sy1 - a * sx1;
                float y0 = a * sx1 + b;

                sprintf(str, "%2d%c", abs(Lat) / 2, (Lat >= 0) ? 'N' : 'S');
                if (sx1 < (float) MapMinX && sx2 > (float) MapMinX && y0 > 0.0 && y0 < (float) Glibx_SCREENHEIGHT)
                {
                    Map_Chars(str, MapMinX + 3, (int) y0);
                }
                else if (Long == LongMin)
                {
                    Map_Chars(str, (int) sx1, (int) sy1 + 4);
                }

                y0 = a * sx2 + b;
                if (sx1 < (float) SCREENWIDTH && sx2 > (float) SCREENWIDTH && y0 > 0.0 && y0 < (float) Glibx_SCREENHEIGHT)
                {
                    Map_Chars(str, SCREENWIDTH - strlen(str) * 6 - 10, (int) y0);
                }
                else
                {
                    if (Long == (LongMax - 1))
                    {
                        Map_Chars(str, (int) sx2 - strlen(str) * 6, (int) sy2);
                    }
                }
            }
        }
    }

    for (i = 1; i <= NavLib_NumberOfBeacons; i++)
    {   /* Number of beacons should be unsigned. TO DO */
        Map_GlobeToScreen(NavLib_Beacons[i].BeaconLatitude, NavLib_Beacons[i].BeaconLongitude, &sx1, &sy1);
        if ((sx1 >= MapMinX) && (sx1 <= MapMaxX) && (sy1 >= MapMinY) && (sy1 <= MapMaxY))
        {
            DrawBeacon(entier(sx1), entier(sy1), NavLib_Beacons[i].Navaid, NavLib_Beacons[i].Ident);
        }
    }

    for (i = 1; i <= NavLib_NumberOfRunways; i++)
    {   /* Number of runways should be unsigned. TO DO */
        Map_GlobeToScreen(NavLib_Runways[i].RunwayLatitude, NavLib_Runways[i].RunwayLongitude, &sx1, &sy1);
        if ((sx1 >= MapMinX) && (sx1 <= MapMaxX) && (sy1 >= MapMinY) && (sy1 <= MapMaxY))
        {
            DrawRunway(entier(sx1), entier(sy1), NavLib_Runways[i]);
        }
    }

    for (i = 1; i <= NavLib_NumberOfWayPoints; i++)
    {
        Map_GlobeToScreen(NavLib_WayPoints[i].WayPointLatitude, NavLib_WayPoints[i].WayPointLongitude, &sx2, &sy2);
        DrawWayPoint(entier(sx2), entier(sy2));
        if (i >= 2)
        {
            glEnable(GL_LINE_STIPPLE);
            glLineStipple(4, 0xAAAA);
            Glibx_Draw(sx1, sy1, sx2, sy2);
            glDisable(GL_LINE_STIPPLE);
        }
        sx1 = sx2;
        sy1 = sy2;
    }


    if (Map_Compasses.Size > 0)
    {
        for (i = 0; i < Map_Compasses.Size; i++)
        {
            Map_CompassRose(Map_Compasses.CLat[i], Map_Compasses.CLong[i]);
        }
    }


    if (Tracks.Size > 0)
    {
        for (i = 1; i <= Tracks.Size; i++)
        {
            DrawTrack(Tracks.Lat1[i], Tracks.Long1[i], Tracks.Lat2[i], Tracks.Long2[i]);
        }
    }

    Map_ShowAircraftTrack(0);

    Map_ShowTarget();

    Glibx_ClipWindow(0, 0, SCREENWIDTH-1, SCREENDEPTH-1);
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

    Glibx_Colour(Glibx_BLACK);
    //Glibx_SetFont(Glibx_GFONT10, 9);
    sprintf(str, "% 02d %02d%c", Mins / 60, Mins % 60, (Lat >= 0.0) ? 'N' : 'S');
    Map_Chars(str, Mx + 16, My - 30);

    Mins = entier(60.0 * Maths_Degrees(Long));
    sprintf(str, (abs(Mins) >= 600) ? "%03d %02d%c" : " %02d %02d%c",
            abs(Mins) / 60, abs(Mins) % 60, (Mins < 0) ? 'W' : 'E');
    Map_Chars(str, Mx + 16, My - 46);
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
    b = entier(Maths_Degrees(NavLib_Bearing(Lat1, Long1, Lat2, Long2)) - MagneticVariation);

//    Map_DisplayCoordinates(x2, y2);

    Glibx_Colour(Glibx_BLACK);
    if (d > 999)
    {
        Map_Chars("--.-", x2 + 16, y2 - 62);
    }
    else
    {
        sprintf(str, " %2d.%d NM", d / 10, d % 10);
        Map_Chars(str, x2 + 16, y2 - 62);
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
    Map_Chars(str, x2 + 16, y2 - 78);
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
        glBegin(GL_POLYGON);
        glVertex3f(x, y, 0.0);
        glVertex3f(x1, y1, 0.0);
        glVertex3f(x2, y2, 0.0);
        glEnd();
    }
    else
    {
        Map_Draw(x, y, x1, y1);
        Map_Draw(x, y, x2, y2);
        Map_Draw(x1, y1, x2, y2);
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

            if (Map_AircraftTrackList->TrackLength >= Map_MaxTrackLength)
            {       /* track buffer is full - discard oldest point */
                Map_AircraftTrackList->FirstPoint = NextTrackPoint(Map_AircraftTrackList->FirstPoint);
                Map_AircraftTrackList->TrackLength--;
                if (Map_AircraftTrackList->DisplayedPoints > 0)
                {
                    Map_AircraftTrackList->DisplayedPoints--;
                }
            }

            Map_AircraftLatitude                                             = IosLink_AeroPkt.Latitude;
            Map_AircraftLongitude                                            = IosLink_AeroPkt.Longitude;
            Map_AircraftTrackList->xPoints[Map_AircraftTrackList->NextPoint] = IosLink_AeroPkt.Latitude;
            Map_AircraftTrackList->yPoints[Map_AircraftTrackList->NextPoint] = IosLink_AeroPkt.Longitude;
            Map_AircraftTrackList->NextPoint                                 = NextTrackPoint(Map_AircraftTrackList->NextPoint);
            Map_AircraftTrackList->TrackLength++;
            //printf("Map_UpdateTrackList() Trk Len: %d\n", Map_AircraftTrackList->TrackLength);
        }
        Map_TargetLatitude  = IosLink_AeroPkt.TLatitude;
        Map_TargetLongitude = IosLink_AeroPkt.TLongitude;
        Map_TargetHeading   = IosLink_AeroPkt.TYaw;
    }

    if (fabs(IosLink_AeroPkt.Yaw - OldYaw) > 0.02)
    {
        Map_AircraftHeading = IosLink_AeroPkt.Yaw;
        OldYaw              = IosLink_AeroPkt.Yaw;
    }

    Arrow.Hdg = Map_AircraftHeading;
}

/* ---------------------------------------------------- */
void Map_ShowAircraftTrack(int New) // bool param
{
    float sx1, sy1, sx2, sy2;


    Map_AircraftTrackList->DisplayedPoints    = 0;
    Map_AircraftTrackList->NextDisplayedPoint = Map_AircraftTrackList->FirstPoint;

    if (Map_AircraftTrackList->DisplayedPoints < Map_AircraftTrackList->TrackLength)
    {
        Glibx_Colour(Glibx_BLUE);
        do
        {
            if (Map_AircraftTrackList->NextDisplayedPoint != Map_AircraftTrackList->FirstPoint)
            {
                Map_GlobeToScreen(Map_AircraftTrackList->xPoints[PrevTrackPoint(Map_AircraftTrackList->NextDisplayedPoint)],
                                  Map_AircraftTrackList->yPoints[PrevTrackPoint(Map_AircraftTrackList->NextDisplayedPoint)],
                                  &sx1, &sy1);
                Map_GlobeToScreen(Map_AircraftTrackList->xPoints[Map_AircraftTrackList->NextDisplayedPoint],
                                  Map_AircraftTrackList->yPoints[Map_AircraftTrackList->NextDisplayedPoint],
                                  &sx2, &sy2);
                if (((sx1 - sx2) * (sx1 - sx2) + (sy1 - sy2) * (sy1 - sy2)) < 25.0)
                {
                    Map_Draw(sx1, sy1, sx2, sy2);
                }
            }
            Map_AircraftTrackList->NextDisplayedPoint = NextTrackPoint(Map_AircraftTrackList->NextDisplayedPoint);
            Map_AircraftTrackList->DisplayedPoints   += 1;
        } while (Map_AircraftTrackList->DisplayedPoints < Map_AircraftTrackList->TrackLength);
    }

    Glibx_Colour(Glibx_BLACK);
    Map_GlobeToScreen(Map_AircraftLatitude, Map_AircraftLongitude, &Arrow.x, &Arrow.y);
    ArrowHead(Arrow.x, Arrow.y, Map_AircraftHeading, false);
    Arrow.Hdg = Map_AircraftHeading;
}

/* ---------------------------------------------------- */
void Map_ShowTarget(void)
{
    float sx, sy;

    Map_GlobeToScreen(Map_TargetLatitude, Map_TargetLongitude, &sx, &sy);
    ArrowHead(sx, sy, Map_TargetHeading, true);
}

/* ---------------------------------------------------- */
unsigned int NextTrackPoint(unsigned int n)
{
    if (n >= Map_MaxTrackLength)
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
        return Map_MaxTrackLength;
    }
    else
    {
        return n - 1;
    }
}

/* --------------------------------------------- */
void Map_InitialiseTrackList(void)
{
    if (Map_AircraftTrackList != NULL)
    {
        free(Map_AircraftTrackList);
    }
    Map_AircraftTrackList = (Map_AircraftTrackRecord*) malloc(sizeof(Map_AircraftTrackRecord));
    if (!Map_AircraftTrackList)
    {
        printf("Memory error : Map_AircraftTrackList\n");
        exit(1);
    }

    Map_AircraftTrackList->TrackLength        = 0;
    Map_AircraftTrackList->DisplayedPoints    = 0;
    Map_AircraftTrackList->FirstPoint         = 1;
    Map_AircraftTrackList->NextPoint          = 1;
    Map_AircraftTrackList->NextDisplayedPoint = 1;

    OldLatitude  = 0.0;
    OldLongitude = 0.0;
    OldYaw       = 0.0;
    OldPktNumber = 0;
}

/* --------------------------------------------- */
void BEGIN_Map()
{
    Map_SetMapScaleFactor(Map_Default_Scale_Factor);
    Map_InitMinMaxLatLong();
    Map_SetMapCentre((Map_MapMinLatitude + Map_MapMaxLatitude) * 0.5,
                     (Map_MapMinLongitude + Map_MapMaxLongitude) * 0.5);
    Map_AircraftLatitude  = Maths_Rads(51.2);
    Map_AircraftLongitude = Maths_Rads(-0.3);
    Map_AircraftHeading   = Maths_Rads(90.0);
    Map_TargetLatitude    = 0.0;
    Map_TargetLongitude   = 0.0;
    Map_TargetHeading     = 0.0;
    Map_InitialiseTrackList();
	ReadMapFontFile("../files/mapfont12.fnt");
}
