#pragma pack(push,2)

#ifndef SPLINELIB_H
#define SPLINELIB_H

#define MaxCurves 20
#define MaxPts    100

typedef struct 
{
    unsigned int np;
	float        xmin;
	float        xmax;
	float        z;
    float        xa[MaxPts];
    float        ya[MaxPts];
    float        y2[MaxPts];
} SplineLib_SData;

typedef struct 
{
    unsigned int    ncurves;
    SplineLib_SData curves[MaxCurves];
} SplineLib_SplineData;

extern float SplineLib_Lookup1(SplineLib_SplineData s, unsigned int n, float x);

extern float SplineLib_Lookup2(SplineLib_SplineData s, float var1, float var2);

extern void BEGIN_SplineLib();

#endif

#pragma pack(pop)
