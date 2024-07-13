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

#include "ioslink.h"
#include "map.h"
#include "menu.h"
#include "approach.h"
#include "plot.h"
#include "script.h"
#include "gui.h"

union cmdpktarg
{ short int          int16;
  unsigned short int card16;
  unsigned int       int32;
  float              real32;
  char               chars[12]; };

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

unsigned long                 OldPktNumber   = 10000000; /* mag var, vis etc */
unsigned long                 FrameNumber    = 0;
unsigned int                  IosLink_CmdPtr = 0;
char                          IosLink_ScriptFilename[13];
char                          IosLink_SaveFileName[128];
unsigned int                  IosLink_IOSMode = MapDisplay;
bool                          IosLink_ExitPending;

float   Convert(unsigned int, float);
void    WriteSaveFile(char[]);
static bool CheckFileExtension(char[], char[]);
void    AppendFileExtension(char[], char[]);
int     SendSaveFile(unsigned int Cmd, char Filename[]);

unsigned PrintPending = 0;

/* ----------------------------------------------------- */
void IosLink_CheckWayPoints(void)
{
    unsigned int i;

    if (NavLib_NextWayPoint > 0)
    {
        IosLink_SendInt(IosDefn_LoadFlightPlan, NavLib_NextWayPoint);
        for (i = 0; i <= 3; i = i + 1)
        {
            IosLink_SendByte(NavLib_WayPoints[NavLib_NextWayPoint].WayPointID[i]);
        }
        IosLink_SendRealArg(NavLib_WayPoints[NavLib_NextWayPoint].WayPointLatitude);
        IosLink_SendRealArg(NavLib_WayPoints[NavLib_NextWayPoint].WayPointLongitude);
        IosLink_SendIntArg(NavLib_WayPoints[NavLib_NextWayPoint].WayPointAltitude);
        IosLink_SendIntArg(NavLib_WayPoints[NavLib_NextWayPoint].WayPointSpeed);
        NavLib_NextWayPoint = NavLib_NextWayPoint + 1;
        if (NavLib_NextWayPoint > NavLib_NumberOfWayPoints)
        {
            NavLib_NextWayPoint = 0;
        }
    }
}

/* ----------------------------------------------------- */
void IosLink_Execute(unsigned int MainOption, unsigned int SubOption)
{
    Menu_MenuType dtype;
    int           Cmd = Menus[MainOption].State[SubOption].Mval;
    
	dtype = Menus[MainOption].State[SubOption].Info;

    switch (dtype)
    {
    case Question:
        switch (Cmd)
        {
        case IosDefn_Exit:
            IosLink_SendCmd(IosDefn_Exit);
			IosLink_ExitPending = true;
            break;

        case IosDefn_SetTargetConflict:
            IosLink_SendBoolean(Cmd, true);
            break;

        case IosDefn_ApproachReset:
            Approach_Reset();
            break;

        case IosDefn_MapReset:
            Map_InitialiseTrackList();
            Map_ChangeMap(IosDefn_MapReset, Gui_MouseX, Gui_MouseY);
            break;

        case IosDefn_PlotSave:
            Plot_SavePlotFile();
            break;

        case IosDefn_SwitchHUDOff:
        case IosDefn_SwitchTargetOff:
            IosLink_SendInt(Cmd, 0);
            break;

        case IosDefn_SetEngine1Fire:
        case IosDefn_SetEngine2Fire:
        case IosDefn_SetEngine3Fire:
        case IosDefn_SetEngine4Fire:
            IosLink_SendInt(Cmd, true);
            break;
			
        default:
            break;
        }
        break;

    case Numeric:
        switch (Cmd)
        {
        case IosDefn_MapScale:
            Map_SetMapScaleFactor(Menus[MainOption].State[SubOption].Data.Numeric.Val);
            Map_ChangeMap(Cmd, Gui_MouseX, Gui_MouseY);
            break;

        case IosDefn_SetMagneticVariation:
            Map_SetMapMagneticVariation(Menus[MainOption].State[SubOption].Data.Numeric.Val);
            IosLink_SendReal(Cmd, Menus[MainOption].State[SubOption].Data.Numeric.Val);
            break;

        case IosDefn_SetApproachRange:
            Approach_SetApproachRange(Menus[MainOption].State[SubOption].Data.Numeric.Val);
            break;

        case IosDefn_SetPlotTime:
            Plot_PlotTime = (unsigned int) Menus[MainOption].State[SubOption].Data.Numeric.Val;
            break;
        case IosDefn_SetPlaybackTime:
            Plot_PlaybackRecordingTime = (unsigned int) (Menus[MainOption].State[SubOption].Data.Numeric.Val * 50);
            break;

        case IosDefn_PlotGoto:
            Plot_SetPlotOrigin(entier(Menus[MainOption].State[SubOption].Data.Numeric.Val));
            break;

        default:
            IosLink_SendReal(Cmd, Menus[MainOption].State[SubOption].Data.Numeric.Val);
            break;
        }

        break;

    case Buttons:
        switch (Cmd)
        {
        case IosDefn_PlotRecord:
            Plot_SetRecordingMode(Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton);
            Plot_PlaybackRecording = false;
            break;

        case IosDefn_PlaybackRecord:
            Plot_SetRecordingMode(Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton);
            if (Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton == 0)
                Plot_PlaybackRecording = false;
            else
                Plot_PlaybackRecording = true;
            break;

        case IosDefn_PlaybackReplay:
            Plot_SetPlayback(Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton);
            //IosLink_SendBoolean(Cmd, (Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton != 1));
            if (Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton == 0)
                IosLink_SendBoolean(Cmd, false);
            else
                IosLink_SendBoolean(Cmd, true);
            break;

        case IosDefn_SetMorseMode:
        case IosDefn_SetRunHoldFreeze:
        case IosDefn_PlaybackCamera:
            IosLink_SendInt(Cmd, Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton);
            break;

        case IosDefn_SetFlightPlanMode:
            if (Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton != 0)
            {
                if (NavLib_NumberOfWayPoints > 0)
                {
                    NavLib_NextWayPoint = 1;
                    IosLink_SendInt(Cmd, NavLib_NumberOfWayPoints);
                }
            }
            else
            {
                IosLink_SendInt(Cmd, 0);
                NavLib_NextWayPoint = 0;
            }
            break;

        case IosDefn_SetMode:
            switch (Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton)
            {
            case 0:
                IosLink_IOSMode = MapDisplay;
                break;
            case 1:
                IosLink_IOSMode = ApproachDisplay;
                break;
            case 2:
                IosLink_IOSMode = FlightDataDisplay;
                break;
            case 3:
                IosLink_IOSMode = RawDataDisplay;
                break;
            }
            break;

        default:
            IosLink_SendBoolean(Cmd, (Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton != 0));
            break;
        }
        break;

    case FileName:
        switch (Cmd)
        {
        case IosDefn_Save:
            strcpy(IosLink_SaveFileName, Menus[MainOption].State[SubOption].Data.Fname);
            // Check file extension on IosLink_SaveFileName
            if (CheckFileExtension(IosLink_SaveFileName, ".sav") == false)
            {
                AppendFileExtension(IosLink_SaveFileName, ".sav");
            }
            WriteSaveFile(IosLink_SaveFileName);
            break;

        case IosDefn_PrintScreen:
            strcpy(IosLink_SaveFileName, Menus[MainOption].State[SubOption].Data.Fname);
            //printf("IosLink_SaveFileName %s\n", IosLink_SaveFileName);
            // Check file extension on IosLink_SaveFileName
            if (CheckFileExtension(IosLink_SaveFileName, ".png") == false)
            {
                AppendFileExtension(IosLink_SaveFileName, ".png");
            }
            PrintPending = 2; /* allow 2 frames to clear keyboard display from framestore*/
            break;

        case IosDefn_GNUplot:
            strcpy(IosLink_SaveFileName, Menus[MainOption].State[SubOption].Data.Fname);
            Plot_GNUplot(IosLink_SaveFileName);
            break;

        default:
            break;
        }
        break;

    case FileList:
        switch (Cmd)
        {
        case IosDefn_Restore:
            SendSaveFile(Cmd, Menus[MainOption].State[SubOption].Data.FileList.FileListName);
            break;

        case IosDefn_LoadPlaybackFile:
            Plot_LoadDataFile(Menus[MainOption].State[SubOption].Data.FileList.FileListName);
            break;

        case IosDefn_LoadTargetFile:
            AppendFileExtension(Menus[MainOption].State[SubOption].Data.FileList.FileListName, ".flt");
            IosLink_SendFilename(Cmd, Menus[MainOption].State[SubOption].Data.FileList.FileListName);
            break;

        case IosDefn_Visual:
        case IosDefn_Models:
        case IosDefn_LoadDTED:
            IosLink_SendFilename(Cmd, Menus[MainOption].State[SubOption].Data.FileList.FileListName);
            break;

        case IosDefn_Script:
            Script_ReadScriptFile(Menus[MainOption].State[SubOption].Data.FileList.FileListName);
            Plot_CreateFilename(".csv", IosLink_ScriptFilename);
            break;

        case IosDefn_LoadFlightPlan:
            NavLib_ReadFlightPlan(Menus[MainOption].State[SubOption].Data.FileList.FileListName);
            NavLib_NextWayPoint = 0;  // Disable tx until flightplan activated.
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
        case IosDefn_MapFind:
            Map_ChangeMap(IosDefn_MapFind, Gui_MouseX, Gui_MouseY);
            break;
//        case IosDefn_MapZoomOut:
//            Map_ChangeMap(IosDefn_MapZoomOut, Gui_MouseX, Gui_MouseY);
//            break;
//        case IosDefn_MapZoomIn:
//            Map_ChangeMap(IosDefn_MapZoomIn, Gui_MouseX, Gui_MouseY);
//            break;

        case IosDefn_ApproachMode:
            break;

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

        case IosDefn_PlotMark:
            Plot_AddMark();
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
            PngLib_SavePngFile(IosLink_SaveFileName);
        }
        ;
    }
}

/* ----------------------------------------------------- */
int SendSaveFile(unsigned int Cmd, char Filename[]) // returns bool
{
    FILE *f;

    if ((f = fopen(Filename, "rb")) == NULL)
    {
        printf("Error opening file.\n");
        return 0;
    }
    IosLink_SendCmd(Cmd);
    fread(&IosLink_IosPkt.RestoreVector, sizeof(IosDefn_RestoreVectorRecord), 1, f);
   
	fclose(f);

    return 1; // return bool
}

/* ----------------------------------------------------- */
static bool CheckFileExtension(char FileName[], char Ext[])
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

void WriteSaveFile(char Filename[])
{
    FILE                        *f;
    IosDefn_RestoreVectorRecord a;
    unsigned int                i, j;

    //CheckFileExtension(FileName, ".sav");
    if ((f = fopen(Filename, "wb")) == NULL)
    {
        printf("Error opening file. File not written\n");
        return;
    }

    a.Latitude      = IosLink_AeroPkt.Latitude;
    a.Longitude     = IosLink_AeroPkt.Longitude;
    a.Pz            = IosLink_AeroPkt.Wheelz;
    a.Pitch         = IosLink_AeroPkt.Pitch;
    a.Roll          = IosLink_AeroPkt.Roll;
    a.Yaw           = IosLink_AeroPkt.Yaw;
    a.U             = IosLink_AeroPkt.U;
    a.V             = IosLink_AeroPkt.V;
    a.W             = IosLink_AeroPkt.W;
    a.P             = IosLink_AeroPkt.P;
    a.Q             = IosLink_AeroPkt.Q;
    a.R             = IosLink_AeroPkt.R;
    a.CurrentRunway = IosLink_NavPkt.CurrentRunway;

    for (i = 0; i <= 0; i+=1) /* only 1 radio for the LFS */
    {
        a.SavedRadios[i].ComVHF1.Active = IosLink_NavPkt.SavedRadios[i].ComVHF1.Active;
        a.SavedRadios[i].ComVHF1.Stby   = IosLink_NavPkt.SavedRadios[i].ComVHF1.Stby;
        a.SavedRadios[i].ComVHF2.Active = IosLink_NavPkt.SavedRadios[i].ComVHF2.Active;
        a.SavedRadios[i].ComVHF2.Stby   = IosLink_NavPkt.SavedRadios[i].ComVHF2.Stby;
        a.SavedRadios[i].ComVHF3.Active = IosLink_NavPkt.SavedRadios[i].ComVHF3.Active;
        a.SavedRadios[i].ComVHF3.Stby   = IosLink_NavPkt.SavedRadios[i].ComVHF3.Stby;
        a.SavedRadios[i].ComHF1.Active  = IosLink_NavPkt.SavedRadios[i].ComHF1.Active;
        a.SavedRadios[i].ComHF1.Stby    = IosLink_NavPkt.SavedRadios[i].ComHF1.Stby;
        a.SavedRadios[i].ComHF2.Active  = IosLink_NavPkt.SavedRadios[i].ComHF2.Active;
        a.SavedRadios[i].ComHF2.Stby    = IosLink_NavPkt.SavedRadios[i].ComHF2.Stby;
        a.SavedRadios[i].ComAM.Active   = IosLink_NavPkt.SavedRadios[i].ComAM.Active;
        a.SavedRadios[i].ComAM.Stby     = IosLink_NavPkt.SavedRadios[i].ComAM.Stby;

        a.SavedRadios[i].NavADF.Active = IosLink_NavPkt.SavedRadios[i].NavADF.Active;
        a.SavedRadios[i].NavADF.Stby   = IosLink_NavPkt.SavedRadios[i].NavADF.Stby;
        a.SavedRadios[i].NavVOR.Active = IosLink_NavPkt.SavedRadios[i].NavVOR.Active;
        a.SavedRadios[i].NavVOR.Stby   = IosLink_NavPkt.SavedRadios[i].NavVOR.Stby;
        a.SavedRadios[i].NavILS.Active = IosLink_NavPkt.SavedRadios[i].NavILS.Active;
        a.SavedRadios[i].NavILS.Stby   = IosLink_NavPkt.SavedRadios[i].NavILS.Stby;
        a.SavedRadios[i].CrsKnob       = IosLink_NavPkt.SavedRadios[i].CrsKnob;
        a.SavedRadios[i].NavGuard      = IosLink_NavPkt.SavedRadios[i].NavGuard;
        a.SavedRadios[i].PowerSwitch   = IosLink_NavPkt.SavedRadios[i].PowerSwitch;

        for (j = 0; j < 14; j++)
        {
            a.SavedRadios[i].PushSwitches[j] = IosLink_NavPkt.SavedRadios[i].PushSwitches[j];
        }
    }

    a.FCU_FD           = IosLink_NavPkt.FCU_FD;
    a.FCU_LS           = IosLink_NavPkt.FCU_LS;
    a.FCU_LOC          = IosLink_NavPkt.FCU_LOC;
    a.FCU_AP1          = IosLink_NavPkt.FCU_AP1;
    a.FCU_AP2          = IosLink_NavPkt.FCU_AP2;
    a.FCU_ATHR         = IosLink_NavPkt.FCU_ATHR;
    a.FCU_EXPED        = IosLink_NavPkt.FCU_EXPED;
    a.FCU_APPR         = IosLink_NavPkt.FCU_APPR;
    a.FCU_SPD_MACH     = IosLink_NavPkt.FCU_SPD_MACH;
    a.FCU_HDG_TRK      = IosLink_NavPkt.FCU_HDG_TRK;

    //a.FCURange       = IosLink_NavPkt.FCURange;
    a.FCU_BaroPressure = IosLink_NavPkt.FCU_BaroPressure;
    a.FCU_HDG          = IosLink_NavPkt.FCU_HDG;
    a.FCU_ALT          = IosLink_NavPkt.FCU_ALT;
    a.FCU_SPD          = IosLink_NavPkt.FCU_SPD;
    a.FCU_VS           = IosLink_NavPkt.FCU_VS;
    a.FCU_BaroSTD      = IosLink_NavPkt.FCU_BaroSTD;
    a.FCU_BaroHg       = IosLink_NavPkt.FCU_BaroHg;
    a.FCU_HDG_Hold     = IosLink_NavPkt.FCU_HDG_Hold;
    a.FCU_ALT_Hold     = IosLink_NavPkt.FCU_ALT_Hold;
    a.FCU_SPD_Hold     = IosLink_NavPkt.FCU_SPD_Hold;
    a.FCU_VS_Hold      = IosLink_NavPkt.FCU_VS_Hold;
	a.FCU_Metric_ALT   = IosLink_NavPkt.FCU_Metric_ALT;

    a.NavAid1          = IosLink_NavPkt.NavAid1;
    a.NavAid2          = IosLink_NavPkt.NavAid2;
    a.Mode             = IosLink_NavPkt.Mode;
    a.DataMode         = IosLink_NavPkt.Data;

    fwrite(&a, sizeof(IosDefn_RestoreVectorRecord), 1, f);
    fclose(f);
}

/* --------------------------------------------- */

void IosLink_CheckSystemChange(void)
{
    float          x = 0.0;
    struct timeval tv;

    if (IosLink_AeroPkt.PktNumber < OldPktNumber) /* forced at start up */
    {
        gettimeofday(&tv, NULL);                   /* needed for RPi */
        IosLink_SendCard32(IosDefn_SetDate, tv.tv_sec);

        if (Menu_FindDefaultValue("Magnetic Var.", &x))
        {
            Map_SetMapMagneticVariation(x);
            IosLink_SendReal(IosDefn_SetMagneticVariation, x);
            //printf("IosLink_CheckSystemChange() : Send Mag Var : %f\n", x);
        }
        if (Menu_FindDefaultValue("Visibility", &x))
        {
            IosLink_SendReal(IosDefn_SetVisibility, x);
            //printf("IosLink_CheckSystemChange() : Send Vis     : %f\n", x);
        }
        if (Menu_FindDefaultValue("Vis rate", &x))
        {
            IosLink_SendReal(IosDefn_SetVisRate, x);
            //printf("IosLink_CheckSystemChange() : Send Vis     : %f\n", x);
        }
        if (Menu_FindDefaultValue("Time of Day", &x))
        {
            IosLink_SendReal(IosDefn_SetTimeOfDay, x);
            //printf("IosLink_CheckSystemChange() : Send Time    : %f\n", x);
        }
        OldPktNumber = IosLink_AeroPkt.PktNumber;
    }
}

/* --------------------------------------------- */

void IosLink_SendByte(unsigned char x)
{
    IosLink_IosPkt.CmdBuff[IosLink_CmdPtr] = x;
    IosLink_CmdPtr++;
}

/* --------------------------------------------- */

void IosLink_SendCmd(unsigned int x)
{
    union cmdpktarg t;

    t.card16 = (unsigned short) x;
    IosLink_SendByte(t.chars[0]);
    IosLink_SendByte(t.chars[1]);
}

/* --------------------------------------------- */

void IosLink_SendReal(unsigned int Cmd, float x)
{
    x = (float) Convert(Cmd, x);
    IosLink_SendCmd(Cmd);
    IosLink_SendRealArg(x);
}

/* --------------------------------------------- */

void IosLink_SendRealArg(float x)
{
    union cmdpktarg t;

    t.real32 = (float) x;
    IosLink_SendByte(t.chars[0]);
    IosLink_SendByte(t.chars[1]);
    IosLink_SendByte(t.chars[2]);
    IosLink_SendByte(t.chars[3]);
}

/* --------------------------------------------- */

void IosLink_SendRealSIUnits(unsigned int Cmd, float x)
{
    IosLink_SendCmd(Cmd);
    IosLink_SendRealArg(x);
}

/* --------------------------------------------- */

void IosLink_SendPosition(unsigned int Cmd, float x, float y)
{
    IosLink_SendCmd(Cmd);
    IosLink_SendRealArg(x);
    IosLink_SendRealArg(y);
}

/* --------------------------------------------- */

void IosLink_SendInt(unsigned int Cmd, int x)
{
    union cmdpktarg t;

    t.int16 = (short) x;
    IosLink_SendCmd(Cmd);
    IosLink_SendByte(t.chars[0]);
    IosLink_SendByte(t.chars[1]);
}

/* --------------------------------------------- */

void IosLink_SendCard32(unsigned int Cmd, unsigned int x)
{
    union cmdpktarg t;

    t.int32 = x;
    IosLink_SendCmd(Cmd);
    IosLink_SendByte(t.chars[0]);
    IosLink_SendByte(t.chars[1]);
    IosLink_SendByte(t.chars[2]);
    IosLink_SendByte(t.chars[3]);
}

/* --------------------------------------------- */

void IosLink_SendIntArg(int x)
{
    union cmdpktarg t;

    t.int16 = (short) x;
    IosLink_SendByte(t.chars[0]);
    IosLink_SendByte(t.chars[1]);
}

/* --------------------------------------------- */

void IosLink_SendBoolean(unsigned int Cmd, bool x)
{
    IosLink_SendCmd(Cmd);
    IosLink_SendByte(x);
    //printf("IosLink_SendBoolean %d %d\n", Cmd, x);
}

/* --------------------------------------------- */

void IosLink_SendFilename(unsigned int Cmd, char Str[])
{
    unsigned int i;

    //printf("IosLink_SendFilename %d %s\n", Cmd, Str);

    IosLink_SendCmd(Cmd);
    for (i = 0; i < strlen(Str); i++)
    {
        IosLink_SendByte(Str[i]);
    }
}

/* --------------------------------------------- */
float Convert(unsigned int Cmd, float x)
{
    switch (Cmd)
    {
    case IosDefn_SetAutopilotAltitude:
        return Maths_Metres(x); /* ( ft -> m */
        break;
    case IosDefn_SetAutopilotHeading:
        return Maths_Rads(x);   /* degrees -> radians */
        break;
    case IosDefn_SetAutopilotSpeed:
        return x * 0.5144; /* Kts -> m/s */
        break;
    case IosDefn_SetAutopilotVSpeed:
        return x * 0.00508; /* FPM -> m/s */
        break;
    case IosDefn_SetAircraftAltitude:
        return -Maths_Metres(x);
        break;
    case IosDefn_SetAircraftHeading:
        return Maths_Rads(x);
        break;
    case IosDefn_SetAircraftSpeed:
        return x * 0.5144; /* kts -> m/s */
        break;
    case IosDefn_SetCgPosition:
        return x / 100.0;
        break;
    case IosDefn_SetTurbulence:
    case IosDefn_SetGustIntermittency:
        return x;
        break;
    case IosDefn_SetWindSpeed:
        return x * 0.5144; /* kts -> m/s */
        break;
    case IosDefn_SetWindDir:
        return Maths_Rads(x);
        break;
    case IosDefn_SetQNH:
        return x;
        break;
    case IosDefn_SetMagneticVariation:
        return Maths_Rads(x);
        break;
    case IosDefn_SetCloudbase:
        return Maths_Metres(x);
        break;
    case IosDefn_SetVisibility:
        return x;
        break;
    case IosDefn_SetTimeOfDay:
        return x;
        break;
    case IosDefn_SetGroundTemperature:
        return x;
        break;
    case IosDefn_SetTargetDistance:
        return x;
        break;
    case IosDefn_SetTargetSpeed:
        return x * 0.5144; /*Kts --> m/s*/
        break;
    case IosDefn_SetTargetHeading:
        return Maths_Rads(x);
        break;
    case IosDefn_SetTargetTurnRate:
        return Maths_Rads(x);
        break;
    case IosDefn_SetTargetAltitude:
        return -Maths_Metres(x);
        break;
    case IosDefn_SetTargetClimbRate:
        return -Maths_Metres(x / 60.0); /* ft/min --> m/s */
        break;
    case IosDefn_WakeSetAltitude:
        return -Maths_Metres(x);
        break;
    case IosDefn_WakeSetHeading:
        return x;
        break;
    case IosDefn_WakeSetDistance:
        return x;
        break;
    case IosDefn_WakeSetVertSlope:
        return x;
        break;
    default:
        return x;
    }
}

void BEGIN_IOSLink(void)
{
    IosLink_ExitPending = false;
}

