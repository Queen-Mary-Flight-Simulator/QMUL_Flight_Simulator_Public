/*
    map header file
*/

#ifndef Map_H
#define Map_H

#include <SIM/glib.h>

#define SCREENWIDTH  Glib_SCREENWIDTH
#define SCREENDEPTH  Glib_SCREENHEIGHT
#define MapMinX      -20
#define MapMaxX      (SCREENWIDTH + 20)
#define MapMinY      -20
#define MapMaxY      (SCREENDEPTH + 20)
#define MapWidth     (SCREENWIDTH)
#define MapDepth     SCREENDEPTH
#define MapCentreX   (SCREENWIDTH / 2)   /* display area (pixels) */
#define MapCentreY   (SCREENDEPTH / 2)       /* display area (pixels) */

#define Map_Default_Scale_Factor 200.0

extern void  Map_SetMapScaleFactor(float s);
extern float Map_GetMapScaleFactor(void);
extern void  Map_SetMapMagneticVariation(float d);
extern float Map_GetMagneticVariation(void);
extern void  Map_DrawMap(void);
extern void  Map_DisplayCoordinates(int mx, int my);
extern void  Map_DisplayDmeDistance(int, int, int, int);
extern void  Map_ChangeMap(unsigned int Cmd, int Mouse_Mx, int Mouse_My);
extern void  Map_ShowAircraftTrack(void);
extern void  Map_UpdateTrackList(void);
extern void  Map_InitialiseTrackList(void);
extern void  Map_GlobeToScreen(float Latitude, float Longitude, float *x, float *y);
extern void  Map_ScreenToGlobe(float x, float y, float *Latitude, float *Longitude);
extern void  Map_SetMapCentre(float Latitude, float Longitude);
extern void  Map_CompassRose(int x, int y);
extern void  Map_AddTrack(int x1, int y1, int x2, int y2);
extern void  BEGIN_Map(void);

#endif
