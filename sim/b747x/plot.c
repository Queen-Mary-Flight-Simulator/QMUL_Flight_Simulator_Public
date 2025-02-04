#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h> // islower
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include <GL/gl.h>

#include <SIM/iosdefn.h>
#include <SIM/glibx.h>
#include <SIM/maths.h>

#include "map.h"
#include "plot.h"
#include "ioslink.h"
#include "iolibx.h"

#define xLHC        (Glibx_SCREENWIDTH / 2 + 80)
#define yLHC        20
#define Xaxis       (Glibx_SCREENWIDTH - 40 - xLHC)
#define MaxMarks    100
#define FontWidth   9  /* assumes font12.fnt */

char units[][9] = { "deg", "rad", "kt", "m/s", "ft", "deg/s", "rad/s", "fpm" };

typedef struct
{
    unsigned int PlotNumber;
    float        ScaleFactor;
    float        Minv;
    float        Maxv;
    float        Incv;
    char         Name[21];
    char         yLabel[21];
    int          Oldx[11];  /* 1 - 10 */
    int          Oldy[11];  /* 1 - 10 */
} PlotRecord;

typedef PlotRecord   PlotDataRecord[5]; // array 1..5 or 0..4

unsigned int                  Plot_PlotTime;
bool                          Plot_PlaybackRecording;
unsigned int                  Plot_PlaybackRecordingTime;
bool                          Plot_Playback;
unsigned int                  Plot_PlaybackTime;
unsigned int                  PlotMenu;

PlotDataRecord                PlotData;
unsigned int                  NumberOfVariables;
IosDefn_PlaybackDataPktRecord *Plot_Handle = NULL;
unsigned int                  MemSize;             /* size of data recording memory in bytes */
unsigned int                  MaxPlotRecords;      /* max number of records that can be stored */
bool                          RecordingMode;       /* data is written if TRUE */
bool                          NewMark;             /* current data value to be marked */
unsigned int                  MarksList[MaxMarks]; /* [1..MaxMarks] */
unsigned int                  Plot_NextPkt;        /* next available slot in memory */
unsigned int                  xTime;               /* time at origin (secs) */
unsigned int                  xLast;               /* last plotted x-position */
unsigned int                  NumberOfMarks;

void  ShowMarks(void);
void  DrawBox(unsigned int, unsigned int, unsigned int, unsigned int);
static float Plot_FindVariable(IosDefn_PlaybackDataPktRecord, unsigned int);
void  ReadBlk(IosDefn_PlaybackDataPktRecord *, IosDefn_PlaybackDataPktRecord *, unsigned int);

/* --------------------------------------------- */
void Plot_GNUplot(char Filename[])
{
    char                          PltFilename[20];
    char                          DataFilename[20];
    static FILE                   *PltFileStream;
    static FILE                   *DataFileStream;
    float                         dy;
    unsigned int                  i;
    unsigned int                  PlotTimeInc;
    IosDefn_PlaybackDataPktRecord Pkt;
    unsigned int                  PktNo;

    if (NumberOfVariables <= 0)
    {
        return;
    }

    dy = 1.0 / NumberOfVariables;

    strcpy(PltFilename, Filename);
    strcat(PltFilename, ".plt");
    strcpy(DataFilename, Filename);
    strcat(DataFilename, ".dat");

    if ((PltFileStream = fopen(PltFilename, "w")) == NULL)
    {
        printf("Error opening file %s\n", PltFilename);
        return;
    }

    if ((DataFileStream = fopen(DataFilename, "w")) == NULL)
    {
        printf("Error opening file %s\n", DataFilename);
        return;
    }

    if (Plot_PlotTime < 50)
    {
        PlotTimeInc = 1;
    }
    else if (Plot_PlotTime < 100)
    {
        PlotTimeInc = 5;
    }
    else if (Plot_PlotTime < 500)
    {
        PlotTimeInc = 10;
    }
    else
    {
        PlotTimeInc = 50;
    }

    fprintf(PltFileStream, "set terminal png truecolor font arial 8 size 600,%d\nset output \"%s.png\"\n",
            (int) (800.0 * (float) NumberOfVariables / 5.0), Filename);
    fprintf(PltFileStream, "set size 1,1\nset origin 0,0\nset lmargin 10\n");
    fprintf(PltFileStream, "set multiplot\nset grid\nset format y \"%%5g\"\n");
    fprintf(PltFileStream, "set size 1.0, %f\n", dy);

    fprintf(PltFileStream, "set xr[%f:%f]\n", (float) xTime, (float) xTime + (float) Plot_PlotTime);
    fprintf(PltFileStream, "set xtics %f\n", (float) PlotTimeInc);

    for (i = 1; i <= NumberOfVariables; i++)
    {
        fprintf(PltFileStream, "set origin 0, %f\n", (float) (i - 1) * dy);
        fprintf(PltFileStream, "set ylabel \"%s %s", PlotData[i].Name, PlotData[i].yLabel);
        fprintf(PltFileStream, "\"\n");
        fprintf(PltFileStream, "set yr[%f:%f]\n", PlotData[i].Minv, PlotData[i].Maxv);
        fprintf(PltFileStream, "set ytics %f\n", PlotData[i].Incv);
        fprintf(PltFileStream, "plot '%s.dat' using 1:%d notitle with lines\n", Filename, i + 1);
    }
    fprintf(PltFileStream, "unset multiplot\nreset\n");
    fclose(PltFileStream);

    for (PktNo = xTime * 50; PktNo <= (xTime + Plot_PlotTime) * 50; PktNo++)
    {
        if (PktNo > Plot_NextPkt)
        {
            break;
        }
        ReadBlk(&Pkt, Plot_Handle, PktNo);

        fprintf(DataFileStream, "%f", (float) PktNo / 50.0);
        for (i = 1; i <= NumberOfVariables; i++)
        {
            fprintf(DataFileStream, " %f",
                    Plot_FindVariable(Pkt, PlotData[i].PlotNumber) * PlotData[i].ScaleFactor);
            if (i >= NumberOfVariables)
            {
                fprintf(DataFileStream, "\n");
            }
        }
    }
    fclose(DataFileStream);
}

/* --------------------------------------------- */
void UpdatePlot(void)
{
    unsigned int                  i;
    int                           x0;
    int                           y0;
    float                         f;
    int                           yp;
    IosDefn_PlaybackDataPktRecord Pkt;
    float                         xmin;
    float                         xmax;
    int                           Height;
    int                           Width;
    unsigned int                  PktNo;

    xLast         = 0;
    NumberOfMarks = 0;

    if (NumberOfVariables <= 0)
    {
        return;
    }

    xmin   = (float) xTime * 50.0;
    xmax   = xmin + (float) Plot_PlotTime * 50.0;
    Height = (Glibx_SCREENHEIGHT - 40 - yLHC - (NumberOfVariables - 1) * 10) / NumberOfVariables;
    Width  = Xaxis;

    while (1)
    {
        PktNo = 50 * xTime + (unsigned int) ((float) (xLast) * (xmax - xmin) / (float) Xaxis);
        if ((xLast >= Xaxis) || (PktNo >= Plot_NextPkt))
        {
            break;
        }

        // if Plot_Handle !NULL - perhaps put at top of function.
        ReadBlk(&Pkt, Plot_Handle, PktNo);

        if (IOLibx_HoldButtonPressed(Pkt.IOPkt1))
        {
            NumberOfMarks            = NumberOfMarks + 1;
            MarksList[NumberOfMarks] = PktNo;
        }

        x0 = xLHC;
        y0 = yLHC;

        for (i = 1; i <= NumberOfVariables; i++)
        {
            //for ( i = 0; i < NumberOfVariables; i++ ) {
            Glibx_Colour(Glibx_BLACK);
            //WITH PlotData[i] {
            f  = Plot_FindVariable(Pkt, PlotData[i].PlotNumber) * PlotData[i].ScaleFactor;
            yp = y0 + intround(((f - PlotData[i].Minv) * (float) (Height)) / (PlotData[i].Maxv - PlotData[i].Minv));
            if (xLast > 0)
            {
                Glibx_ClipWindow(x0, y0, x0 + Width, y0 + Height);
                Glibx_Draw(x0 + PlotData[i].Oldx[i], PlotData[i].Oldy[i], x0 + (int) (xLast), yp);
            }
            PlotData[i].Oldx[i] = xLast;
            PlotData[i].Oldy[i] = yp;
            //}

            y0 = y0 + Height + 10;
        }
        xLast++;
    }

    Glibx_ClipWindow(0, 0, Glibx_SCREENWIDTH - 1, Glibx_SCREENHEIGHT - 1);
}

/* --------------------------------------------- */
void Axes(unsigned int n, PlotDataRecord p)
{
    float        x, y;
    unsigned int x0, y0;
    unsigned int x1, y1;
    unsigned int i;
    float        ymin, ymax, yinc;
    float        xmin, xmax, xinc;
    unsigned int Height;  /* height of each graph in pixels */
    unsigned int Width;   /* width of all graphs */
    char         str[21]; /* ARRAY [0..20] OF CHAR */
    unsigned int PlotTimeInc;

    if (n <= 0)
    {
        return;
    }

    if (Plot_PlotTime < 50)
    {
        PlotTimeInc = 1;
    }
    else if (Plot_PlotTime < 100)
    {
        PlotTimeInc = 5;
    }
    else if (Plot_PlotTime < 500)
    {
        PlotTimeInc = 10;
    }
    else
    {
        PlotTimeInc = 50;
    }

    Height = (Glibx_SCREENHEIGHT - 40 - yLHC - (n - 1) * 10) / n;
    Width  = Xaxis;

    xmin = (float) (xTime);
    xmax = xmin + (float) (Plot_PlotTime);
    xinc = (float) (PlotTimeInc);

    x0 = xLHC;
    y0 = yLHC;

    for (i = 1; i <= n; i++)
    {
        ymin = p[i].Minv;
        ymax = p[i].Maxv;
        yinc = p[i].Incv;

        Glibx_Colour(Glibx_GREY);
        x = xmin;
        do
        {
            x1 = x0 + (unsigned int) ((x - xmin) / (xmax - xmin) * (float) (Width));
            Glibx_Draw(x1, y0, x1, y0 + Height);
            x = x + xinc;
        } while (x < (xmax + 0.0001));

        y = ymin;
        do
        {
            y1 = y0 + (unsigned int) ((y - ymin) / (ymax - ymin) * (float) (Height));
            Glibx_Colour(Glibx_GREY);
            Glibx_Draw(x0, y1, x0 + Width, y1);
            sprintf(str, "%8.1f", y);
            Glibx_Colour(Glibx_BLUE);
            Map_Chars(str, x0 - strlen(str) * FontWidth, y1 - 8);
            y = y + yinc;
        } while (y < (ymax + 0.0001));

        Map_Chars(p[i].yLabel, x0 + 7, y1 - 20);
        Map_Chars(p[i].Name, Glibx_SCREENWIDTH - 120 + 4, y1 - 20); /* add the title */
        DrawBox(Glibx_SCREENWIDTH - 120, y1 - 22, strlen(p[i].Name) * FontWidth + 5, 18);

        y0 = y0 + Height + 10;
    }

    x = xmin;
    i = xTime;
    do
    {
        x1 = x0 + (unsigned int) ((x - xmin) / (xmax - xmin) * (float) (Width));
        sprintf(str, "%d", i);
        Map_Chars(str, x1 - strlen(str) * FontWidth / 2, yLHC - 16);
        i = i + PlotTimeInc;
        x = x + xinc;
    } while (x < (xmax + 0.0001));
}

/* ----------------------------------------------------- */
void ShowMarks(void)
{
    float        xmin, xmax;
    unsigned int i;
    int          xp;

    xmin = (float) (xTime) * 50.0;
    xmax = xmin + (float) (Plot_PlotTime) * 50.0;

    Glibx_Colour(Glibx_RED);
    for (i = 1; i <= NumberOfMarks; i++)
    {
        if (MarksList[i] > (50 * xTime))
        {
            xp = xLHC + (int) ((float) (MarksList[i] - 50 * xTime) * (float) Xaxis / (xmax - xmin));
            if ((xp >= xLHC) && (xp < (xLHC + Xaxis)))
            {
                Glibx_Draw(xp, yLHC, xp, yLHC + 5);
            }
        }
    }
}

/* ----------------------------------------------------- */
void DrawBox(unsigned int x1, unsigned int y1, unsigned int xs, unsigned int ys)
{
    unsigned int x2, y2;

    x2 = x1 + xs;
    y2 = y1 + ys;
    Glibx_Draw(x1, y1, x1, y2);
    Glibx_Draw(x1, y2, x2, y2);
    Glibx_Draw(x2, y2, x2, y1);
    Glibx_Draw(x2, y1, x1, y1);
}

/* --------------------------------------------- */
float FindScaleFactor(char *a)
{
    unsigned int i;
    int          s;

    for (i = 0; i <= 8; i++)
    {
        s = strcmp(a, units[i]);
        if (s == 0)
        {
            switch (i)
            {
            case 0:  return 57.2958;  break;
            case 1:  return 1.0;      break;
            case 2:  return 1.9440;   break;
            case 3:  return 1.0;      break;
            case 4:  return 3.2808;   break;
            case 5:  return 57.2958;  break;
            case 6:  return 1.0;      break;
            case 7:  return 196.8504; break;
            case 8:  return 1.1508;   break;
            default: return 1.0;      break;
            }
        }
    }
    return 1.0;
}

/* --------------------------------------------- */
static float Plot_FindVariable(IosDefn_PlaybackDataPktRecord Pkt, unsigned int n)
{
    switch (n)
    {
    case 0: return Pkt.AeroPkt.Rudder + Pkt.AeroPkt.RudderTrim;
    case 1: return Pkt.AeroPkt.Aileron + Pkt.AeroPkt.AileronTrim;
    case 2: return Pkt.AeroPkt.Elevator + Pkt.AeroPkt.ElevatorTrim;
    case 3: return Pkt.AeroPkt.Vc;
    case 4: return -Pkt.AeroPkt.Pz;
    case 5: return Pkt.AeroPkt.BetaDot;
    case 6: return Pkt.AeroPkt.Beta;
    case 7: return Pkt.AeroPkt.AlphaDot;
    case 8: return Pkt.AeroPkt.Alpha;
    case 9: return Pkt.AeroPkt.R;
    case 10: return Pkt.AeroPkt.P;
    case 11: return Pkt.AeroPkt.Q;
    case 12: return Pkt.AeroPkt.Yaw;
    case 13: return Pkt.AeroPkt.Roll;
    case 14: return Pkt.AeroPkt.Pitch;
    case 15: return Pkt.AeroPkt.Lift / (Pkt.AeroPkt.Mass * 9.81);
    case 16: return Pkt.NavPkt.ILS1.LocaliserError;
    case 17: return Pkt.NavPkt.ILS1.GlideSlopeError;
    case 18: return -Pkt.AeroPkt.Vd;
    case 19: return Pkt.AeroPkt.FlightData[0];
    case 20: return Pkt.AeroPkt.FlightData[1];
    case 21: return Pkt.AeroPkt.FlightData[2];
    }
    return 0.0; /* should never get here */
}

/* --------------------------------------------- */
unsigned int SetPlotValues(void)
{
    unsigned int i, j, k;

    if (PlotMenu == 0)
    {
        for (i = 1; i <= Menu_NumberOfMenus; i += 1)
        {
            if (strcmp(Menus[i].Title, "Flight Data") == 0)
            {
                PlotMenu = i;
                break;
            }
        }
    }

    j = 0;
    //for ( i = 1; i<= Menus[PlotMenu].NumberOfItems; i++ ) { // i=0; i<NumberofItems?
    for (i = 0; i < Menus[PlotMenu].NumberOfItems; i++)
    {
        if (Menus[PlotMenu].State[i].Data.FlightData.Plotting)
        {
            j = j + 1;
            if (j > 4)
            {             // Was 5 - only room for four on our screen
                j = 4;
            }
            PlotData[j].PlotNumber  = i;
            PlotData[j].ScaleFactor = FindScaleFactor(Menus[PlotMenu].State[i].Data.FlightData.UnitsList[Menus[PlotMenu].State[i].Data.FlightData.ActiveUnits]);
            PlotData[j].Minv        = Menus[PlotMenu].State[i].Data.FlightData.ymin;
            PlotData[j].Maxv        = Menus[PlotMenu].State[i].Data.FlightData.ymax;
            PlotData[j].Incv        = Menus[PlotMenu].State[i].Data.FlightData.yinc;
            for (k = 0; k <= strlen(Menus[PlotMenu].State[i].Mname); k++)
            {
                PlotData[j].Name[k] = Menus[PlotMenu].State[i].Mname[k];
            }
            for (k = 0; k <= strlen(Menus[PlotMenu].State[i].Data.FlightData.UnitsList[Menus[PlotMenu].State[i].Data.FlightData.ActiveUnits]); k++)
            {
                PlotData[j].yLabel[k] = Menus[PlotMenu].State[i].Data.FlightData.UnitsList[Menus[PlotMenu].State[i].Data.FlightData.ActiveUnits][k];
            }
        }
    }
    return j;
}

/* --------------------------------------------- */
void Plot_ShowPlot(void)
{
    Glibx_Colour(Glibx_GREY);
    //Glibx_SetFont(Glibx_GFONT10, 6);
    Glibx_LineWidth(1.0);

    NumberOfVariables = SetPlotValues();
//  printf("Plot_ShowPlot:NumberOfVariables: %d\n", NumberOfVariables);

    Axes(NumberOfVariables, PlotData);
    UpdatePlot();
    ShowMarks();
}

/* --------------------------------------------- */
void Plot_EndPlot(void)
{
    /* deallocate mem */
    free(Plot_Handle);
}

/* --------------------------------------------- */

void WriteBlk(IosDefn_PlaybackDataPktRecord *blk, IosDefn_PlaybackDataPktRecord *hld, unsigned int slot)
{
    // to, from, size
    memcpy(hld + slot, blk, sizeof(IosDefn_PlaybackDataPktRecord));
}

/* --------------------------------------------- */

void ReadBlk(IosDefn_PlaybackDataPktRecord *blk, IosDefn_PlaybackDataPktRecord *hld, unsigned int slot)
{
    // to, from, size
    memcpy(blk, hld + slot, sizeof(IosDefn_PlaybackDataPktRecord));
}

/* --------------------------------------------- */
/* invoked every iteration when all pkts captured */
void Plot_SaveData(void)
{
    if (RecordingMode)
    {
        if ((Plot_NextPkt < MaxPlotRecords) && (!IOLibx_HoldButtonPressed(IosLink_IOPkt1)))
        {
            IOLibx_SetHoldButtonPressed(IosLink_IosPlotDataPkt.IOPkt1, NewMark);
            NewMark                                               = false;
            WriteBlk(&IosLink_IosPlotDataPkt, Plot_Handle, Plot_NextPkt);
            Plot_NextPkt++;

            if (Plot_PlaybackRecording)
            {
                if (Plot_NextPkt > Plot_PlaybackRecordingTime)
                {
                    Plot_PlaybackRecording = false;
                    Plot_PlaybackTime      = 0;
                    RecordingMode          = false; /* stop recording */
                }
            }
        }
    }
}

/* --------------------------------------------- */
void Plot_ShowNextPage(void)
{
    xTime = xTime + Plot_PlotTime;
}

/* --------------------------------------------- */
void Plot_ShowPreviousPage(void)
{
    if (xTime <= 0)
    {
        return;
    }
    if (xTime < Plot_PlotTime)
    {
        xTime = 0;
    }
    else
    {
        xTime = xTime - Plot_PlotTime;
    }
}

/* --------------------------------------------- */
void Plot_AddMark(void)
{
    if (RecordingMode)
    {
        NewMark = true;
    }
}

/* --------------------------------------------- */
void Plot_ShowNextMark(void)
{
    IosDefn_PlaybackDataPktRecord a;
    unsigned int                  i;

    if (Plot_NextPkt > 0)
    {
        for (i = ((xTime + 1) * 50); i <= Plot_NextPkt - 1; i++)
        {
            ReadBlk(&a, Plot_Handle, i);
            if (IOLibx_HoldButtonPressed(a.IOPkt1))
            {
                xTime = i / 50;
                return;
            }
        }
    }
}

/* --------------------------------------------- */
void Plot_ShowPreviousMark(void)
{
    IosDefn_PlaybackDataPktRecord a;
    int                           i;

    if (xTime > 0)
    {
        for (i = (xTime * 50 - 1); i >= 0; i--)
        {
            ReadBlk(&a, Plot_Handle, i);
            if (IOLibx_HoldButtonPressed(a.IOPkt1))
            {
                xTime = i / 50;
                return;
            }
        }
    }
}

/* --------------------------------------------- */
void Plot_SetPlotOrigin(unsigned int n)
{
    xTime = n;
}

/* --------------------------------------------- */
void Plot_SetRecordingMode(unsigned int Button)
{
    switch (Button)
    {
    case 0: RecordingMode = false; /* Stop */
        break;
    case 1: RecordingMode = true;  /* Start */
        Plot_NextPkt      = 0;
        NumberOfMarks     = 0;
        break;
    case 2: RecordingMode = true; /* Continue */
        break;
    }
}

/* --------------------------------------------- */
void Plot_CreateFilename(char *Ext, char *Filename)
{
    char      tstr[50];
    struct tm *ptr;
    time_t    t;

    t   = time(NULL);
    ptr = localtime(&t);
    sprintf(tstr, "%2.2d%2.2d%2.2d%2.2d", ptr->tm_mday, (ptr->tm_mon) + 1, ptr->tm_hour, ptr->tm_min);
    strcat(tstr, Ext);
    strcpy(Filename, tstr);
    printf("Plot_CreateFilename : %s\n", Filename);
}

/* --------------------------------------------- */
void Plot_SavePlotFile(void)
{
    IosDefn_PlaybackDataPktRecord a;
    FILE                          *f;
    unsigned int                  i;
    unsigned int                  rv;
    char                          FileName[13];

    if (Plot_Handle == NULL)
    {
        return;
    }

    if (Plot_NextPkt > 0)
    {
        Plot_CreateFilename(".dat", FileName);
        if ((f = fopen(FileName, "wb")) == NULL)
        {
            printf("Plot_SavePlotFile : failed to create file.\n");
            return;
        }

        for (i = 0; i <= Plot_NextPkt - 1; i++)
        {
            ReadBlk(&a, Plot_Handle, i);
            // fwrite returns number of objects written
            rv = fwrite(&a, sizeof(IosDefn_PlaybackDataPktRecord), 1, f);
            if (rv != 1)
            {
                printf("Plot_SavePlotFile : Incomplete write on record : %d\n", i);
            }
        }
        fclose(f);
    }
}

/* --------------------------------------------- */
void Plot_SetPlayback(unsigned int Button)
{
    switch (Button)
    {
    case 0:
        Plot_Playback = false; /* Stop */
        printf("SetPlot_Playback : Stop\n");
        break;
    case 1:
        Plot_Playback     = true; /* Start */
        Plot_PlaybackTime = 0;
        printf("SetPlot_Playback : Start\n");
        break;
    case 2:
        Plot_Playback = true; /* Continue */
        printf("SetPlot_Playback : Continue\n");
        break;
    }
}

/* --------------------------------------------- */
void Plot_CheckPlayback(void)
{
    IosDefn_PlaybackDataPktRecord a;

    if (Plot_Playback)
    {
        Plot_PlaybackTime++;
        if (Plot_PlaybackTime >= Plot_NextPkt)
        {
            Plot_Playback     = false;
            Plot_PlaybackTime = 0;
            IosLink_SendBoolean(IosDefn_PlaybackReplay, false); /* turn recording off */
        }
        else
        {
            ReadBlk(&a, Plot_Handle, Plot_PlaybackTime);
            memcpy(&IosLink_IosPkt.PlaybackDataPkt, &a,
                   sizeof(IosDefn_PlaybackDataPktRecord)); // to, from, size
            IosLink_SendBoolean(IosDefn_PlaybackReplay, true);
        }
    }
}

/* --------------------------------------------- */
void Plot_LoadDataFile(char *FileName)
{
    IosDefn_PlaybackDataPktRecord a;
    FILE                          *f;
    unsigned int                  rv;
    unsigned int                  i = 0;
    Plot_NextPkt = 0;

    if ((f = fopen(FileName, "rb")) == NULL)
    {
        printf("Plot_LoadDataFile : failed to open file : %s\n", FileName);
        return;
    }

    while (1)
    {
        rv = fread(&a, sizeof(IosDefn_PlaybackDataPktRecord), 1, f);
        if (rv == 0) // no more records to read
        {
            break;
        }
        if (rv != 1)
        {
            printf("Plot_LoadDataFile : Incomplete read on record : %d. Subsequent data possibly corrupted.\n", i);
            break;
        }

        WriteBlk(&a, Plot_Handle, Plot_NextPkt);
        Plot_NextPkt++;
        i++;
    }
    fclose(f);
    xTime = 0;
}

/* --------------------------------------------- */
void BEGIN_Plot(void)
{
    Plot_PlotTime     = 10;
    Plot_NextPkt      = 0;
    xTime             = 0;
    NumberOfVariables = 0;
    xLast             = 0;
    RecordingMode     = false;
    NumberOfMarks     = 0;
    NewMark           = false;

    Plot_PlaybackRecording     = false;
    Plot_PlaybackRecordingTime = 30 * 50; /* 30 secs */
    Plot_Playback              = false;
    Plot_PlaybackTime          = 0;

    // Allocate 16 MB for plot data ~ 17000 slots ~ 5 mins recording
    MemSize        = 16777216;
    MaxPlotRecords = (unsigned int) (MemSize / sizeof(IosDefn_PlaybackDataPktRecord));
    Plot_Handle    = malloc(sizeof(IosDefn_PlaybackDataPktRecord) * MaxPlotRecords);
    if (!Plot_Handle)
    {
        printf("Memory error : Plot Storage Allocation Failed\n");
    }
}
