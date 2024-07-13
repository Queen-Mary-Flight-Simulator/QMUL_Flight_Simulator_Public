/* +------------------------------+---------------------------------+
   | Module      : iodefn.h       | Version         : 1.1           | 
   | Last Edit   : 19-11-07       | Reference Number: 01-01-01      |
   +------------------------------+---------------------------------+
   | Computer    : Gateway1                                         |
   | Directory   : /dja/io                                          |
   | Compiler    : gcc 3.2                                          |
   | OS          : RedHat 8.0                                       |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : input/output data packet definition              |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#ifndef IODefn_H
#define IODefn_H

#include "cbool.h"

#define IODefn_GearUp 0
#define IODefn_GearDown 1
#define IODefn_GearOff 2
typedef unsigned char IODefn_GearSelectorPosition;

#define IODefn_On 0
#define IODefn_Off 1
typedef unsigned char IODefn_SwitchPosition;

#define IODefn_TrimForwards 0
#define IODefn_TrimBackwards 1
#define IODefn_TrimLeft 2
#define IODefn_TrimRight 3
#define IODefn_TrimOff 4
typedef unsigned char IODefn_TrimSwitchPosition;

typedef struct {
    unsigned int PktNumber;
    float Elevator;
    float Aileron;
    float Rudder;
    float FlapSelector;
    float EngineLever;
    float ReverseLever;
    float LeftBrake;
    float RightBrake;

    IODefn_SwitchPosition LampsTestButton;
    IODefn_SwitchPosition WarningCancelButton;
    IODefn_GearSelectorPosition GearSelector;
    IODefn_TrimSwitchPosition RudderTrimSwitch;
    IODefn_SwitchPosition ThrottlePushButton;
    IODefn_SwitchPosition ThrottleSwitch;
    IODefn_SwitchPosition TriggerSwitch;
    IODefn_TrimSwitchPosition AileronTrimSwitch;
    IODefn_TrimSwitchPosition ElevatorTrimSwitch;
    boolean HoldButtonPressed;
    boolean RestoreButtonPressed;
    boolean FreezeButtonPressed;
    boolean ClockButtonPressed;
    IODefn_SwitchPosition AutomaticResetSwitch;
    IODefn_SwitchPosition RightBoostPumpSwitch;
    IODefn_SwitchPosition LeftBoostPumpSwitch;
    IODefn_SwitchPosition StarterSwitch;
    IODefn_SwitchPosition IgnitionSwitch;
    IODefn_SwitchPosition MasterSwitch;
    IODefn_SwitchPosition ColumnButton;
    IODefn_SwitchPosition ParkBrake;
    IODefn_SwitchPosition KeySwitch;

    unsigned char Filler[2];

    unsigned int TimeStamp;
} IODefn_IODataPkt;

#endif
