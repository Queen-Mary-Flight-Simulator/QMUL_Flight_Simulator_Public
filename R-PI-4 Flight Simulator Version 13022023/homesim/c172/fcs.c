/* +---------------------------+---------------------------------+
| Module      : fcs.c          | Version         : 1.2           |
| Last Edit   : 17-02-2022     | Reference Number: 02-01-11      |
|+-----------------------------+---------------------------------+
| Computer    : DELL1                                            |
| Directory   : /dja/aerosoft/cranfield/software/pfd/            |
| Compiler    : gcc 10.2.0                                       |
| OS          : Windows10                                        |
|+---------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
|+---------------------------------------------------------------+
| Description : Boeing 747-400 Flight Control System (FCS)       |
|                                                                |
|+---------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/maths.h>
#include <SIM/weather.h>

#include "aero.h"
#include "aerolink.h"
#include "model.h"
#include "fcs.h"
#include "iolib.h"

#define YAWDAMPER       true
#define TURNCOORDINATOR true
#define NumberOfEngines 4

#define ONERAD      (180.0 / M_PI)
#define DEG1        (1.0 / ONERAD)
#define DEG3        (3.0 / ONERAD)
#define DEG25       (25.0 / ONERAD)
#define DEG60       (60.0 / ONERAD)
#define DEG85       (85.0 / ONERAD)
#define DEG95       (95.0 / ONERAD)

bool                FCS_AP_SPD_Mode;
float               FCS_ThrottlePosition;
bool                FCS_AutoTrimming;
float               FCS_Kp;
float               FCS_Ki;
float               FCS_Kd;
int                 FCS_FD_VBar;
int                 FCS_FD_HBar;
bool                FCS_TOGAMode;

void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv);
float CheckFMS();
void CheckFCU();

/* --------------------------------------------------------- */
void FCS_FCS(float *Elevator, float *Aileron, float *Rudder)
{
}

/* --------------------------------------------------------- */
void FCS_ResetFCS()
{
}

/* --------------------------------------------------------- */
void FCS_Engage_ALT()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_ALT()
{
}

/* --------------------------------------------------------- */
bool FCS_ALTEngaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_Set_ALT(float Href)
{
}

/* --------------------------------------------------------- */
float FCS_ALT(float Href)  /* -ve up, metres */
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_HDG()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_HDG()
{
}

/* --------------------------------------------------------- */
bool FCS_HDG_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_Set_HDG(float Href)
{
}

/* --------------------------------------------------------- */
float FCS_HDG(float HdgRef)
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_SPD()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_SPD()
{
}

/* --------------------------------------------------------- */
bool FCS_SPD_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_Set_SPD(float Vref)
{
}

/* --------------------------------------------------------- */
float FCS_SPD(float Vref)
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_VSPD()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_VSPD()
{
}

/* --------------------------------------------------------- */
bool FCS_VSPD_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_Set_VSPD(float Vref)
{
}

/* --------------------------------------------------------- */
float FCS_VSPD(float Vref)
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_Pitch()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_Pitch()
{
}

/* --------------------------------------------------------- */
bool FCS_Pitch_Engaged()
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Set_Pitch(float Theta)
{
}

/* --------------------------------------------------------- */
float FCS_Pitch(float ThetaC)
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_FPA()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_FPA()
{
}

/* --------------------------------------------------------- */
bool FCS_FPA_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_Set_FPA(float Theta)
{
}

/* --------------------------------------------------------- */
float FCS_FPA(float GammaC)
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_BankAngle()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_BankAngle()
{
}

/* --------------------------------------------------------- */
bool FCS_BankAngle_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_Set_BankAngle(float Theta)
{
}

/* --------------------------------------------------------- */
float FCS_BankAngle(float RollC)
{
    return 0.0;
}

/* --------------------------------------------------------- */
void FCS_Engage_TurnCoordinator()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_TurnCoordinator()
{
}

/* --------------------------------------------------------- */
bool FCS_TurnCoordinator_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_TurnCoordinator(float *Rudder)
{
}

/* --------------------------------------------------------- */
void FCS_Engage_YawDamper()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_YawDamper()
{
}

/* --------------------------------------------------------- */
bool FCS_YawDamper_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_YawDamper(float *Rudder)
{
}

/* --------------------------------------------------------- */
void FCS_Engage_Autoland()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_Autoland()
{
}

/* --------------------------------------------------------- */
bool FCS_Autoland_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
void FCS_Autoland(float *Elevator, float *Aileron)
{
}

/* --------------------------------------------------------- */
void FCS_Engage_LOC()
{
}

/* --------------------------------------------------------- */
void FCS_Disengage_LOC()
{
}

/* --------------------------------------------------------- */
bool FCS_LOC_Engaged()
{
    return false;
}

/* --------------------------------------------------------- */
float FCS_LOC()
{
    return 0.0;
}

/* --------------------------------------------------------- */
void UpdateFlightDirector(float Elevator, float Aileron, int *fdh, int *fdv)
{
}

/* --------------------------------------------------------- */
void CheckFCU()
{
}

/* --------------------------------------------------------- */
float CheckFMS()
{
    return 0.0;
}

/* --------------------------------------------------------- */
void BEGIN_FCS()
{
}
