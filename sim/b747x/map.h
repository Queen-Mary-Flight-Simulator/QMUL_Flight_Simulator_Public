/*
    map header file
*/

#ifndef Map_H
#define Map_H

#include <SIM/glibx.h>

#define SCREENWIDTH  Glibx_SCREENWIDTH
#define SCREENDEPTH  Glibx_SCREENHEIGHT
#define MapMinX      (SCREENWIDTH/2)
#define MapMaxX      SCREENWIDTH
#define MapMinY      0
#define MapMaxY      SCREENDEPTH
#define MapWidth     (SCREENWIDTH/2)
#define MapDepth     SCREENDEPTH
#define MapCentreX   (SCREENWIDTH * 3 / 4)   /* display area (pixels) */
#define MapCentreY   (SCREENDEPTH / 2)       /* display area (pixels) */

#define Map_MaxTrackLength 8000
#define Map_MaxTracks      20
#define Map_MaxCompasses   20

#define Map_ICON_ILS (char)128
#define Map_ICON_RUN (char)129
#define Map_ICON_NDB (char)130
#define Map_ICON_VOR (char)131
#define Map_ICON_DME (char)132

#define Map_Default_Scale_Factor 200.0

typedef struct {
    int    TrackLength;
    int    DisplayedPoints;
    int    FirstPoint;
    int    NextPoint;
    int    NextDisplayedPoint;
    float xPoints[Map_MaxTrackLength];
    float yPoints[Map_MaxTrackLength];

} Map_AircraftTrackRecord;

typedef struct {
    unsigned int Size;
    float CLat[Map_MaxCompasses];
    float CLong[Map_MaxCompasses];
} Map_CompassList;

extern Map_AircraftTrackRecord *Map_AircraftTrackList;
extern float Map_AircraftLatitude;
extern float Map_AircraftLongitude;
extern float Map_AircraftHeading;
extern Map_CompassList Map_Compasses;
extern float Map_MapLatitude;
extern float Map_MapLongitude;
extern float Map_MapScaleFactor;
extern float Map_MapMinLatitude;
extern float Map_MapMaxLatitude;
extern float Map_MapMinLongitude;
extern float Map_MapMaxLongitude;

extern void  Map_SetMapScaleFactor(float s);
extern float Map_GetMapScaleFactor(void);
extern void  Map_SetMapMagneticVariation(float d);
extern float Map_GetMagneticVariation(void);
extern void  Map_Draw(float x1, float y1, float x2, float y2);
extern void  Map_DrawMap(void);
extern void  Map_DisplayCoordinates(int mx, int my);
extern void  Map_DisplayDmeDistance(int, int, int, int);
extern void  Map_DrawTrackScreen(float, float, float, float);
extern void  Map_ChangeMap(unsigned int Cmd, int Mouse_Mx, int Mouse_My);
extern void  Map_ShowAircraftTrack(int New); /* param is bool */
extern void  Map_UpdateTrackList(void);
extern void  Map_InitialiseTrackList(void);
extern void  Map_InitMinMaxLatLong(void);
extern void  Map_GlobeToScreen(float Latitude, float Longitude, float *x, float *y);
extern void  Map_ScreenToGlobe(float x, float y, float *Latitude, float *Longitude);
extern void  Map_SetMapCentre(float Latitude, float Longitude);
extern void  Map_CompassRose(float Lat, float Long);
extern void  Map_AddTrack(int x1, int y1, int x2, int y2);
extern void  Map_Char(char Ch, int x, int y);
extern void  Map_Chars(char str[], int x, int y);
extern void  BEGIN_Map(void);

#endif
