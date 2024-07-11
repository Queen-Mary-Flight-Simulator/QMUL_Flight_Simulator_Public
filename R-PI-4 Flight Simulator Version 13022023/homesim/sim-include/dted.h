#include <stdbool.h>

#pragma pack(push,2)

#ifndef DTED_H
#define DTED_H

typedef struct
{
    float bx1;        /* nearest 10m */
    float by1;
    float bx2;
    float by2;
    int   xPosts;
    int   yPosts;
 } DTED_Record;

double DTED_PostHeight(double latitude, double longitude, double AirfieldElevation);

extern void DTED_LoadDTED(char Filename[]);

extern bool DTED_Loaded();

extern double DTED_Incline();

extern void DTED_Exit();

extern void BEGIN_DTED();

#endif

#pragma pack(pop)
