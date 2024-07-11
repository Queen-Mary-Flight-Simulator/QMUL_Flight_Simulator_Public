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
#define TWOPI         (M_PI * 2.0L)

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
double Maths_DoubleNormalise(double a)
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
