#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <sys/time.h>

#include <SIM/iodefn.h>
#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/udplib.h>
#include <SIM/protodefn.h>
#include <SIM/iolib.h>

#include "iolink.h"

IODefn_IODataPkt       IOLink_IOPkt1;
IODefn_IODataPkt       IOLink_IOPkt2;
AeroDefn_AeroDataPkt   IOLink_AeroPkt;
EngDefn_EngDataPkt     IOLink_EngPkt;
NavDefn_NavDataPkt     IOLink_NavPkt;
IosDefn_IosDataPkt     IOLink_IosPkt;
ProtoDefn_ProtoDataPkt IOLink_ProtoPkt;

float                  IOLink_LastRestore;
bool                   IOLink_Restored;
bool                   IOLink_OctaveMode;

unsigned int           CmdPtr;
unsigned int           FrameNumber;

unsigned char GetByte();
float         GetReal();
unsigned int  GetWord();
int           GetInt();
unsigned int  GetCard32();
bool          GetBoolean();
void          GetFilename(char Str[]);

/* ---------------------------------------------------- */    
bool IOLink_RespondToIos(bool held)
{
    float          Vc;
    float          u;
    float          v;
    float          w;
	
    CmdPtr = 0;

    while (1)
    {
        switch (GetWord())
        {    
            case IosDefn_EndOfPkt:
            return false;
    
            case IosDefn_Exit:
                return true;
    
            case IosDefn_SetOctaveMode:
                IOLink_OctaveMode = GetBoolean();
                break;

	        case IosDefn_SetDate:
			    GetCard32();
			    break;
				
            case IosDefn_SetRunHoldFreeze:
                GetWord();
                break;
    
            case IosDefn_SetFlightControls:
                IOLib_Sidestick = GetBoolean();
                if (IOLib_Sidestick)
                {
                    IOLib_RepositionCLS(0.0, 0.0, 0.0, 0.0);
                }
                else
                {
                    u = IOLink_AeroPkt.U;
                    v = IOLink_AeroPkt.V;
                    w = IOLink_AeroPkt.W;
                    Vc = sqrt(u * u + v * v + w * w);
                    IOLib_RepositionCLS(Vc, IOLink_AeroPkt.ElevatorTrim, IOLink_AeroPkt.AileronTrim, IOLink_AeroPkt.RudderTrim);
                }
                break;
    
            case IosDefn_Restore:
                u = IOLink_IosPkt.RestoreVector.U;
                v = IOLink_IosPkt.RestoreVector.V;
                w = IOLink_IosPkt.RestoreVector.W;
                Vc = sqrt(u * u + v * v + w * w);
                if (!IOLib_Sidestick)
                {
                    IOLink_LastRestore = Vc;
                    IOLib_RepositionCLS(Vc, IOLink_AeroPkt.ElevatorTrim, IOLink_AeroPkt.AileronTrim, IOLink_AeroPkt.RudderTrim);
                }
				IOLink_Restored = true;
                break;

            case IosDefn_SetAircraftSpeed:
                break;
				
            default:
                return false;  /* unrecognised command */
        }
    }
}

/* ---------------------------------------------------- */    
bool GetBoolean()
{
    unsigned char x;

    x = GetByte();
    return (x != 0);
}

/* ---------------------------------------------------- */    
unsigned char GetByte()
{
    unsigned char x;

    x = IOLink_IosPkt.CmdBuff[CmdPtr];
    CmdPtr += 1;
    return x;
}

/* ---------------------------------------------------- */    
float GetReal()
{
    union 
    {
        float         r;
        unsigned char b[4];
    } x32;

    x32.b[0] = GetByte();
    x32.b[1] = GetByte();
    x32.b[2] = GetByte();
    x32.b[3] = GetByte();
    return x32.r;
}

/* ---------------------------------------------------- */    
unsigned int GetWord()
{
    union 
    {
        unsigned short int c;
        unsigned char      b[2];
    } x16;

    x16.b[0] = GetByte();
    x16.b[1] = GetByte();
    return (unsigned int) x16.c;
}

/* ---------------------------------------------------- */    
int GetInt()
{
    union 
    {
        short int     i;
        unsigned char b[2];
    } x16;

    x16.b[0] = GetByte();
    x16.b[1] = GetByte();
    return (int) x16.i;
}

/* ---------------------------------------------------- */    
unsigned int GetCard32()
{
    union 
    {
        unsigned int  i;
        unsigned char b[4];
    } x32;

    x32.b[0] = GetByte();
    x32.b[1] = GetByte();
    x32.b[2] = GetByte();
    x32.b[3] = GetByte();
    return x32.i;
}

/* ---------------------------------------------------- */    
void GetFilename(char Str[])
{
    unsigned int i;

    i = 0;
    do 
    {
        Str[i] = GetByte();
        i = i + 1;
    } while (!(Str[i - 1] == 0));
}

/* ---------------------------------------------------- */    
void IOLink_FormPacket()
{
    unsigned int i;

    IOLink_IOPkt1.PktNumber = FrameNumber;
    FrameNumber            += 1;

    for (i=0; i<=31; i+=1)
    {
        IOLink_IOPkt1.AnalogueData[i] = IOLib_AnalogueData[i];
    }
    IOLink_IOPkt1.DigitalDataA = IOLib_DigitalDataA;
    IOLink_IOPkt1.DigitalDataB = IOLib_DigitalDataB;
    IOLink_IOPkt1.DigitalDataC = IOLib_DigitalDataC;
    IOLink_IOPkt1.DigitalDataD = IOLib_DigitalDataD;
	IOLink_IOPkt1.Temperature  = IOLib_Temperature;
}

/* ---------------------------------------------------- */    
void BEGIN_IOLink()
{
    CmdPtr = 0;
    FrameNumber = 0;
    IOLink_OctaveMode = false;
	IOLink_Restored = false;
}
