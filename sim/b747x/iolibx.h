#ifndef IOLibx_H
#define IOLibx_H

/* version for the IOS computer */

#include <stdbool.h>
#include <SIM/iodefn.h>

#define BIT0    0x01
#define BIT1    0x02
#define BIT2    0x04
#define BIT3    0x08
#define BIT4    0x10
#define BIT5    0x20
#define BIT6    0x40
#define BIT7    0x80

extern unsigned char IOLibx_DigitalDataOutA;
extern unsigned char IOLibx_DigitalDataOutB;
extern bool          IOLibx_SideStick;

/* analogue inputs */

extern float IOLibx_GetEngineLever(unsigned int LeverNumber, IODefn_IODataPkt a);

extern float IOLibx_GetElevator(IODefn_IODataPkt a);

extern float IOLibx_GetAileron(IODefn_IODataPkt a);

extern float IOLibx_GetRudder(IODefn_IODataPkt a);

extern float IOLibx_GetLeftBrake(IODefn_IODataPkt a);

extern float IOLibx_GetRightBrake(IODefn_IODataPkt a);

extern float IOLibx_GetSpeedBrake(IODefn_IODataPkt a);

extern float IOLibx_GetFlapSelector(IODefn_IODataPkt a);

/* digital input */

extern bool IOLibx_HoldButtonPressed(IODefn_IODataPkt a);

extern void IOLibx_SetHoldButtonPressed(IODefn_IODataPkt a, bool s);

extern void BEGIN_IOLibx();

#endif