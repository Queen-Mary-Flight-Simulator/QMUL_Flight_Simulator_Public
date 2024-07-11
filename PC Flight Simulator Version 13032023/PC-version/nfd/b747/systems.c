/* +------------------------------+---------------------------------+
   | Module      : nfd.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-13      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Navigation Systems                               |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdbool.h>
#include <SIM/iodefn.h>
#include "systems.h"

bool Systems_Failures[50];
bool Systems_AdfFixedCard;
bool Systems_AdfDip;
bool Systems_HSI_Installed;
bool Systems_VOR_Installed;
bool Systems_Radio_Installed;
float Systems_SelectedAltitude;
bool Systems_MarkerTest;
IODefn_SwitchPosition Systems_MasterSwitch;
IODefn_SwitchPosition Systems_KeySwitch;
bool Systems_RemoteHold;

void BEGIN_Systems()
{
    unsigned int i;

    for (i = 0; i <= 49; i += 1) 
	{
      Systems_Failures[i] = false;
    }
    
    Systems_AdfFixedCard = true;
    Systems_AdfDip = false;
    Systems_HSI_Installed = true;
    Systems_VOR_Installed = false;
    Systems_Radio_Installed = true;
    Systems_MarkerTest = false;
    Systems_MasterSwitch = IODefn_On;
    Systems_KeySwitch = IODefn_On;
    Systems_RemoteHold = false;
}
