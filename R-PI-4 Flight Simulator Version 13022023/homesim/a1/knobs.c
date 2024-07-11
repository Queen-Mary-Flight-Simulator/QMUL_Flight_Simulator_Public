#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <SIM/maths.h>

#include "knobs.h"
#include "pfd.h"
#include "nfd.h"
#include "nav.h"
#include "compass.h"

unsigned int Knobs_BaroPressure;
unsigned int Knobs_Obs;
unsigned int Knobs_HI;

static unsigned int ticks;
static unsigned int obsinc;
static bool         oldleftb;

bool InsideCircle(int x, int y, int x0, int y0, int r);

void Knobs_Check(int x, int y, bool leftb, bool middleb, bool rightb)
{
	if (leftb & oldleftb)  /* check if left button held down */
	{
        ticks += 1;
		if (ticks >= 150)  /* speed up after 3s */
		{
		    obsinc = 5;
		}
	}
	else
	{
	    obsinc = 1;
		ticks = 0;
	}
	
	if (leftb) 
	{
	    if (InsideCircle(x, y, BaroKnobX, BaroKnobY, 32))
		{
    	    obsinc = 1;
	    	ticks = 0;

		    if (x < BaroKnobX)
			{
			    if (Knobs_BaroPressure > 9500)
				{
				    Knobs_BaroPressure -= 1;
				}
			}
			else
			{
			    if (Knobs_BaroPressure < 10500)
				{
				    Knobs_BaroPressure += 1;
				}
			}
	    }
		
	    if (InsideCircle(x, y, ObsKnobX, ObsKnobY, 32))
		{
		    if (x < ObsKnobX)
			{
			    if (Knobs_Obs <= 1)
				{
				    Knobs_Obs = 3600;
				}
				else
				{
				    Knobs_Obs -= obsinc;
				}
			}
			else
			{
			    if (Knobs_Obs >= 3600)
				{
				    Knobs_Obs = 1;
				}
				else
				{
				    Knobs_Obs += obsinc;
				}
			}
			Nav_HSI_Crs = Knobs_Obs / 10;
		}

	    if (InsideCircle(x, y, CompassKnobX, CompassKnobY, 32))
		{
		    if (x < CompassKnobX)
			{
			    if (Knobs_HI <= 1)
				{
				    Knobs_HI = 3600;
				}
				else
				{
				    Knobs_HI -= obsinc;
				}
			}
			else
			{
			    if (Knobs_HI >= 3600)
				{
				    Knobs_HI = 1;
				}
				else
				{
				    Knobs_HI += obsinc;
				}
			}
			Compass_Adjust = 2.0 * Maths_Rads((float) Knobs_HI);
		}
	}
	oldleftb = leftb;
}

bool InsideCircle(int x, int y, int x0, int y0, int r)
{
    return (x - x0) * (x - x0) + (y - y0) * (y - y0) < r * r;
}

void BEGIN_Knobs()
{
    Knobs_BaroPressure = 10130;
	Knobs_Obs          = 3600;
	Knobs_HI           = 3600;
	ticks              = 0;
	obsinc             = 1;
    oldleftb           = false;
}
