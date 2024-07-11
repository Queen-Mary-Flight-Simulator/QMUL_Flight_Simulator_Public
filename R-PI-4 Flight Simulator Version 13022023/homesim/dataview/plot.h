/*
    plot.h
    
    Flight Data Plotting header file
*/

#ifndef Plot_H
#define Plot_H

#include <stdbool.h>
#include <SIM/iosdefn.h>

extern unsigned int Plot_PlotTime;
extern bool      Plot_PlaybackRecording;
extern unsigned int Plot_PlaybackRecordingTime;
extern bool      Plot_Playback;
extern unsigned int Plot_PlaybackTime;
extern IosDefn_PlaybackDataPktRecord *Plot_Handle;
extern unsigned int Plot_NextPkt;
  
void Plot_ShowPlot(void);

void Plot_ShowNextPage(void);

void Plot_ShowPreviousPage(void);

void Plot_AddMark(void);

void Plot_ShowNextMark(void);

void Plot_ShowPreviousMark(void);

void Plot_SetRecordingMode(unsigned int);

void Plot_SetPlotOrigin(unsigned int);

void Plot_SavePlotFile(void);

void Plot_LoadDataFile(char*);

void Plot_EndPlot(void);

void Plot_CreateFilename(char*, char*);

void Plot_GNUplot(char[]);

void Plot_SetPlayback(unsigned int);

void Plot_CheckPlayback(void);

void Plot_SaveData(void);

void Plot_CreatePlotWindow(void);

void BEGIN_Plot(void);

#endif

