/*
    ioslink.c
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <sys/time.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/navdefn.h>
#include <SIM/maths.h>
#include <SIM/pnglib.h>
#include <SIM/navlib.h>

#include "link.h"
#include "plot.h"
#include "gui.h"

union cmdpktarg
{ 
    short int          int16;
    unsigned short int card16;
    unsigned int       int32;
    float              real32;
    char               chars[12]; 
};

// Put into a math header, or ios_gui.h
#define entier(x)    ((x) >= 0.0 ? (int) ((x) + 0.5) : (int) ((x) - 0.5))

IODefn_IODataPkt              IosLink_IOPkt1;
IODefn_IODataPkt              IosLink_IOPkt2;
AeroDefn_AeroDataPkt          IosLink_AeroPkt;
EngDefn_EngDataPkt            IosLink_EngPkt;
NavDefn_NavDataPkt            IosLink_NavPkt;
IosDefn_IosDataPkt            IosLink_IosPkt;
IosDefn_PlaybackDataPktRecord IosLink_IosPlotDataPkt;
ProtoDefn_ProtoDataPkt        IosLink_ProtoPkt;

unsigned int                  IosLink_CmdPtr = 0;
char                          IosLink_ScriptFilename[13];
char                          IosLink_SaveFileName[128];
unsigned int                  IosLink_IOSMode = MapDisplay;
bool                          IosLink_Stopping;

static unsigned               PrintPending = 0;

float Convert(unsigned int, float);
void  WriteSaveFile(char[]);
bool  CheckFileExtension(char[], char[]);
void  AppendFileExtension(char[], char[]);

/* ----------------------------------------------------- */
void IosLink_Execute(GUI_MenuItem *m)
{
    int Cmd = m->Mval;
    
    switch (m->Info)
    {
		case Question:
			switch (Cmd)
			{
				case IosDefn_Exit:
					IosLink_Stopping = true;
					break;

				default:
					break;
			}
			break;

		case Numeric:
			switch (Cmd)
			{
				case IosDefn_SetPlotTime:
					Plot_PlotTime = (unsigned int) m->Data.Numeric.Val;
					break;

				case IosDefn_PlotGoto:
					Plot_SetPlotOrigin(entier(m->Data.Numeric.Val));
					break;

				default:
					break;
			}
			break;

		case Buttons:
			switch (Cmd)
			{
				case IosDefn_PlotRecord:
					Plot_SetRecordingMode(m->Data.Buttons.ActiveButton);
					Plot_PlaybackRecording = false;
					break;

				case IosDefn_PlaybackRecord:
					Plot_SetRecordingMode(m->Data.Buttons.ActiveButton);
					if (m->Data.Buttons.ActiveButton == 0)
						Plot_PlaybackRecording = false;
					else
						Plot_PlaybackRecording = true;
					break;

				case IosDefn_PlaybackReplay:
					Plot_SetPlayback(m->Data.Buttons.ActiveButton);
					break;

				default:
					break;
			}
			break;

		case FileName:
			switch (Cmd)
			{
				case IosDefn_PrintScreen:
					strcpy(IosLink_SaveFileName, m->Data.Fname);
					//printf("IosLink_SaveFileName %s\n", IosLink_SaveFileName);
					// Check file extension on IosLink_SaveFileName
					if (CheckFileExtension(IosLink_SaveFileName, ".png") == false)
					{
						AppendFileExtension(IosLink_SaveFileName, ".png");
					}
					PrintPending = 2; /* allow 2 frames to clear keyboard display from framestore*/
					break;

				case IosDefn_GNUplot:
					strcpy(IosLink_SaveFileName, m->Data.Fname);
					Plot_GNUplot(IosLink_SaveFileName);
					break;

				default:
					break;
			}
			break;

		case FileList:
			switch (Cmd)
			{
				case IosDefn_LoadPlaybackFile:
					Plot_LoadDataFile(m->Data.FileList.FileListName);
					break;

				default:
					break;
			}
			break;

		case Coordinates:
			break;

		case FlightData:
			break;

		case None:
			switch (Cmd)
			{
				case IosDefn_PlotNextPage:
					Plot_ShowNextPage();
					break;
					
				case IosDefn_PlotPreviousPage:
					Plot_ShowPreviousPage();
					break;
					
				case IosDefn_PlotNextMark:
					Plot_ShowNextMark();
					break;
					
				case IosDefn_PlotPreviousMark:
					Plot_ShowPreviousMark();
					break;
			}
			break;

		default:
			break;
    }
}

/* ----------------------------------------------------- */
void IosLink_CheckPrint()
{
    if (PrintPending > 0)
    {
        PrintPending = PrintPending - 1;
        if (PrintPending == 0)
        {
            PngLib_SavePngFile(IosLink_SaveFileName, 960, 0, 960, 1080); /* assumes display 1920 x 1080 */
        }
    }
}

/* ----------------------------------------------------- */
bool CheckFileExtension(char FileName[], char Ext[])
{
    if (strstr(FileName, Ext) == NULL)
        return false;
    else
        return true;
}

/* ----------------------------------------------------- */
void AppendFileExtension(char FileName[], char Ext[])
{
    strcat(FileName, Ext);
}

/* ----------------------------------------------------- */
void BEGIN_Link(void)
{
    IosLink_Stopping = false;
}
