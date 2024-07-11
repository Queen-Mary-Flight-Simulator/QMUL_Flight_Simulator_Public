/*
    ioslink.h
*/

#ifndef IosLink_H
#define IosLink_H

#include <stdbool.h>

#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>

#include "gui.h"

#define MapDisplay 1
#define ApproachDisplay 2
#define FlightDataDisplay 3
#define RawDataDisplay 4

extern IODefn_IODataPkt              IosLink_IOPkt1;
extern IODefn_IODataPkt              IosLink_IOPkt2;
extern AeroDefn_AeroDataPkt          IosLink_AeroPkt;
extern EngDefn_EngDataPkt            IosLink_EngPkt;
extern NavDefn_NavDataPkt            IosLink_NavPkt;
extern IosDefn_IosDataPkt            IosLink_IosPkt;
extern ProtoDefn_ProtoDataPkt        IosLink_ProtoPkt;
extern IosDefn_PlaybackDataPktRecord IosLink_IosPlotDataPkt;

extern char                IosLink_ScriptFilename[13];
extern unsigned int        IosLink_CmdPtr;
extern char                IosLink_SaveFileName[128];
extern unsigned int        IosLink_IOSMode;
extern bool                IosLink_Stopping;

void IosLink_Execute(GUI_MenuItem *m);

void IosLink_CheckPrint();

void IosLink_CheckSystemChange(void);

void IosLink_CheckWayPoints(void);

void IosLink_SendPosition(unsigned int, float, float);

void IosLink_SendWord(unsigned int x);

void IosLink_SendBoolean(unsigned int Cmd, bool x);

void IosLink_SendReal(unsigned int Cmd, float x);

void IosLink_SendRealArg(float x);

void IosLink_SendRealSIUnits(unsigned int Cmd, float x);

void IosLink_SendByte(unsigned char x);

void IosLink_SendInt(unsigned int Cmd, int x);

void IosLink_SendCard32(unsigned int Cmd, unsigned int x);

void IosLink_SendIntArg(int x);

void IosLink_SendFilename(unsigned int Cmd, char Str[]);

void IosLink_SendCmd(unsigned int);

void BEGIN_Link();


#endif

