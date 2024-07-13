/* +------------------------------+---------------------------------+
   | Module      : maths.c        | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-06              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : simulator specific maths functions               |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */


#include <math.h>

#include <SIM/maths.h>

#define StepLength 0.02
#define ONERAD     (180.0L / M_PI)
#define TWOPI      (M_PI * 2.0L)

/* ------------------------------------------------------ */
float Maths_Limit(float x, float LowerBound, float UpperBound)
{
    if (x < LowerBound)
    {
        return LowerBound;
    }
    else if (x > UpperBound)
    {
        return UpperBound;
    }
	else
	{
	    return x;
	}
}

/* ------------------------------------------------------ */
float Maths_Normalise(float a)
{
    double x = (double) a;
    double r = fmod(x + M_PI, TWOPI);

    if (r < 0.0)
    {
        r += TWOPI;
    }
    return (float) (r - M_PI);
}

/* ------------------------------------------------------ */
float Maths_Rads(float x)
{
    return x / ONERAD;
}

/* ------------------------------------------------------ */
float Maths_Degrees(float x)
{
    return x * ONERAD;
}

/* ------------------------------------------------------ */
float Maths_Metres(float x)
{
    return x * 0.30479999;
}

/* ------------------------------------------------------ */
float Maths_Feet(float x)
{
    return x * 3.280840;
}

/* ------------------------------------------------------ */
float Maths_Integrate(float P, float V)
{
    return P + StepLength * V;
}

/* ------------------------------------------------------ */
double Maths_Double_Integrate(double P, float V)
{
    return P + (double) (StepLength * V);
}

/* ------------------------------------------------------ */
double Maths_Double_Normalise(double a)
{
    double x = fmod(a + M_PI, TWOPI);

    if (x < 0.0)
    {
        x += TWOPI;
    }
    return x - M_PI;
}

/* ------------------------------------------------------ */
void BEGIN_Maths()
{
}
