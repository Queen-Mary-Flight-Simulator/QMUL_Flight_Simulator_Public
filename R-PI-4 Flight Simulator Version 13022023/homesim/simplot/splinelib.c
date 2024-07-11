/*
    Splines library
    DJA 20 November 2019
*/

#include <SIM/splinelib.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

float splint(float xa[], float ya[], float y2a[], int n, float x);
float Neville (int n, float x[], float y[], float t);

/* ------------------------------------------------------------------ */
float SplineLib_Lookup1(SplineLib_SplineData s, unsigned int n, float x)
{
    SplineLib_SData *p = &s.curves[n];
	
    if (x < p->xmin)
	{
	    return Neville(4, &p->xa[1], &p->ya[1], x);
	}
	else if  (x > p->xmax)
	{
	    return  Neville(4, &p->xa[p->np - 3], &p->ya[p->np - 3], x);
	}
	else
	{
	    return splint(p->xa, p->ya, p->y2, p->np, x);
    }
}

/* ------------------------------------------------------------------ */
float SplineLib_Lookup2(SplineLib_SplineData s, float var1, float var2)
{
    int   i;
    int   cn;
    float y1;
    float y2;
	float z1;
	float z2;
	
	cn = -1;
	for (i=0; i<s.ncurves-1; i+=1)
	{
	    if (var2 < s.curves[i+1].z)
		{
		    cn = i;
			break;
		}
	}
    if (cn < 0)
	{
	    cn = s.ncurves - 2;
	}
	
    z1 = s.curves[cn].z;
    z2 = s.curves[cn+1].z;
    y1 = SplineLib_Lookup1(s, cn, var1);
    y2 = SplineLib_Lookup1(s, cn + 1, var1);
    return y1 + (var2 - z1) * (y2 - y1) / (z2 - z1);
 }

/* ------------------------------------------------------------------ */
float splint(float xa[], float ya[], float y2a[], int n, float x)
{
    int   klo,khi,k;
    float h,b,a;

    klo=1;
    khi=n;
    while (khi-klo > 1) 
    {
        k=(khi+klo) / 2;
        if (xa[k] > x) 
            khi=k;
        else 
            klo=k;
    }
    h=xa[khi]-xa[klo];
    a=(xa[khi]-x)/h;
    b=(x-xa[klo])/h;
    return a*ya[klo]+b*ya[khi]+((a*a*a-a)*y2a[klo]+(b*b*b-b)*y2a[khi])*(h*h)/6.0;
}

/* -------------------------------------------------------------------- */
float Neville (int n, float x[], float y[], float t)
{
    unsigned int i;
	unsigned int j;
    float       f[100];
     
    for (i=0; i<n; i+=1 )
	{
        f[i] = y[i];
    }
	
    for (j=1; j<n; j+=1 )
	{
        for (i=n-1; i>=j; i-=1) 
		{
            f[i] = ((t - x[i-j]) * f[i] - (t - x[i]) * f[i-1]) / (x[i] - x[i-j]);
        }
    }
   
    return f[n-1];
}
