/* +------------------------------+---------------------------------+
   | Module      : maths.c        | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-02-03      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : simulator specific maths functions library       |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>

#include <SIM/maths.h>

#define StepLength    0.02
#define ONERAD        (180.0L / M_PI)

/* ------------------------------------------------------ */
void Maths_Limit(float *x, float LowerBound, float UpperBound)
{
    if (*x < LowerBound)
    {
        *x = LowerBound;
    }
    else if (*x > UpperBound)
    {
        *x = UpperBound;
    }
}

/* ------------------------------------------------------ */
void Maths_Normalise(float *a)
{
    double x = (double) *a;
    double r = fmod(x + Maths_PI, Maths_TWOPI);

    if (r < 0.0)
    {
        r += Maths_TWOPI;
    }
    *a = (float) (r - Maths_PI);
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
void Maths_Integrate(float *P, float V)
{
    *P = *P + StepLength * V;
}

/* ------------------------------------------------------ */
void Maths_Double_Integrate(double *P, float V)
{
    *P = *P + (double) (StepLength * V);
}

/* ------------------------------------------------------ */
void BEGIN_Maths()
{
}
