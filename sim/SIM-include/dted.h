#pragma pack(push,2)

#ifndef DTED_H
#define DTED_H

#include <stdbool.h>

typedef struct
{
    int          bx1;        /* nearest 10m */
    int          by1;
    int          bx2;
    int          by2;
    int          xPosts;
    int          yPosts;
    double       RLAT;
    double       RLONG;
    double       RQDM;
    double       RunwayRotation;
    double       RunwayAltitude;
    double       RunwayX;
    double       RunwayY;
    double       RunwayZ;
 } DTED_Record;

double DTED_PostHeight(double latitude, double longitude, double AirfieldElevation);

extern void DTED_LoadDTED(char Filename[]);

extern bool DTED_Loaded();

extern double DTED_Incline();

extern void DTED_Exit();

extern void BEGIN_DTED();

#endif

#pragma pack(pop)
