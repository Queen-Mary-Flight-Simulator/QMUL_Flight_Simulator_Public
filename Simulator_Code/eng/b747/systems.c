/* 
+------------------------------+---------------------------------+
| Module      : systems.c      | Version         : 1.1           |
| Last Edit   : 13-12-2016     | Reference Number: 02-01-16      |
|+-----------------------------+---------------------------------+
| Computer    : DELL2                                            |
| Directory   : /dja/aerosoft/cranfield/lfs/eicas/          |
| Compiler    : gcc 4.8.1                                        |
| OS          : Windows7                                         |
|+---------------------------------------------------------------+
| Authors     : D J Allerton                                     |
|             :                                                  |
|+---------------------------------------------------------------+
| Description : Boeing 747-400 Systems                           |
|                                                                |
|+---------------------------------------------------------------+
| Revisions   : none                                             |
|                                                                |
+----------------------------------------------------------------+ */

#include "systems.h"

bool         Systems_Failures[51];
bool         Systems_RemoteHold;
bool         Systems_Freezing;
unsigned int Systems_SysTicks;

bool         Systems_EngineFireSound;
bool         Systems_EngineFire[4];

/* nothing to do for the EFS-500 */

/* ----------------------------------------- */
void Systems_UpdateSystems()
{
    Systems_SysTicks += 1;

    if (Systems_SysTicks >= 50)
    { 
        Systems_SysTicks = 0; 
    }
}

/* ----------------------------------------- */
void BEGIN_Systems()
{
    unsigned int i;
    
    for (i = 0; i <= 50; i += 1)
    {
        Systems_Failures[i] = false;
    }
    Systems_RemoteHold = false;
    Systems_Freezing = false;
    Systems_SysTicks = 0;

    Systems_EngineFireSound = false;
	
}
