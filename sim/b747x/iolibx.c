#include <stdbool.h>
#include <SIM/iodefn.h>

#include "iolibx.h"

/* version for the plot and script modules using recorded pkts */

/* ---------------------------------------------------- */    
float IOLibx_GetEngineLever(unsigned int EngineNumber, IODefn_IODataPkt a)  /* EngineNumber 0..3 */
{
    return (float) a.AnalogueData[4] / 32767.0;
}

/* ---------------------------------------------------- */
float IOLibx_GetElevator(IODefn_IODataPkt a)
{
    return (float) a.AnalogueData[1] / 32767.0;
}

/* ---------------------------------------------------- */
float IOLibx_GetAileron(IODefn_IODataPkt a)
{
    return (float) a.AnalogueData[0] / 32767.0;
}

/* ---------------------------------------------------- */
float IOLibx_GetRudder(IODefn_IODataPkt a)
{
    return (float) a.AnalogueData[2] / 32767.0;
}

/* ---------------------------------------------------- */
float IOLibx_GetLeftBrake(IODefn_IODataPkt a)
{
    return (float) a.AnalogueData[6] / 32767.0;
}

/* ---------------------------------------------------- */
float IOLibx_GetRightBrake(IODefn_IODataPkt a)
{
    return (float) a.AnalogueData[5] / 32767.0;
}

/* ---------------------------------------------------- */
bool IOLibx_HoldButtonPressed(IODefn_IODataPkt a)
{
    return a.DigitalDataC & BIT4;
}

/* ---------------------------------------------------- */
void IOLibx_SetHoldButtonPressed(IODefn_IODataPkt a, bool s)
{
    if (s)
    { 
        a.DigitalDataC &= ~BIT4;
	}
	else
    { 
        a.DigitalDataC |= BIT4;
	}	
}

/* ---------------------------------------------------- */
void BEGIN_IOLibx()
{
}
