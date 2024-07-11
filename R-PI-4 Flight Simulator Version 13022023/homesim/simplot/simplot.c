#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include <SIM/aerodefn.h>
#include <SIM/engdefn.h>
#include <SIM/navdefn.h>
#include <SIM/iodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/protodefn.h>
#include <SIM/clocks.h>
#include <SIM/weather.h>

#include "aero.h"
#include "aerolink.h"
#include "englink.h"
#include "engines.h"
#include "fcs.h"
#include "maths.h"
#include "model.h"
#include "simulate.h"
#include "systems.h"
#include "stab.h"
#include "iolib.h"
#include "gear.h"

const char CR    = 13;
const char EOL   = 10;
const char TAB   = 9;
const char SPACE = ' ';

char *CmdWords[] =
    { "", "SET", "PLOT", "ENGAGE", "DISENGAGE", "TIME", "AUTOTRIM", "INPUT" } ;

char *PlotWords[] =
    { "",				"RUDDER",		"AILERON",		"ELEVATOR",			"TAS",			 /* 0-4 */
      "ALTITUDE",		"BETA_RATE",	"BETA",			"ALPHA_RATE",		"ALPHA",		 /* 5-9 */ 
      "PITCH_RATE",		"ROLL_RATE",	"YAW_RATE",		"PITCH_ACCN",		"ROLL_ACCN",	 /* 10-14 */
      "YAW_ACCN",		"PITCH",		"ROLL",			"YAW",				"LIFT",			 /* 15-19 */
      "THRUST",			"DRAG",			"SIDEFORCE",	"ENGINE_LEVER",		"GAMMA",		 /* 20-24 */
      "VERTICAL_SPEED",	"IAS",			"SPOILER",		"PARK_BRAKE",		"LEFT_BRAKE",	 /* 25-29 */
      "RIGHT_BRAKE",	"VAR1",			"VAR2",			"VAR3",             "EPR",   		 /* 30-34 */
      "DISTANCE",       "MACH",         "MASS" };                                            /* 35-39 */

char *SetWords[] =
    { "",				"TURBULENCE",	"WIND_SPEED",	"WIND_DIRECTION",	"QNH", 			 /* 0-4 */
      "MAG_VAR",		"OAT",			"RUDDER",		"AILERON",			"ELEVATOR", 	 /* 5-9 */ 
      "ENGINE_LEVER",	"ALTITUDE", 	"HEADING", 		"TAS",				"RATE_OF_CLIMB", /* 10-14 */
      "CG_POSITION",	"FLAPS",		"GEAR",			"AP_ALT",			"AP_HDG", 	     /* 15-19 */
      "AP_SPD",			"AP_VSPD",		"Kp",			"Ki",				"Kd",	         /* 20-24 */
      "IAS",			"SPOILER",		"PARK_BRAKE",	"LEFT_BRAKE",		"RIGHT_BRAKE",	 /* 25-29 */
      "LATITUDE",       "LONGITUDE",    "GROUND_LEVEL", "INCLINE",          "MASS" } ;       /* 30-34 */

char *InputWords[] =
    { "", "ELEVATOR", "AILERON", "RUDDER", "ENGINE_LEVER", "SPOILER", "PARK_BRAKE", "LEFT_BRAKE", "RIGHT_BRAKE" } ;

char *ShapeWords[] =
    { "", "STEP", "PULSE", "DOUBLET", "RAMP", "SINE" } ;

char *Units[] =
    { "",			"DEGS",			"RADS",			"DEG/S",		"RAD/S", 
      "DEG/S/S",	"RAD/S/S",		"FT",			"M",			"KM", 
      "NM",			"KTS",			"MPH",			"M/S",			"FPM", 
      "N",			"LBF",			"KG",			"LBS",			"KG/HR",
      "LBS/HR",		"LITRES/HR",	"%", 			"MB",			"INHG",
      "KG/M/M/M", 	"SECS", 		"MINS", 		"HOURS",		"RPM",
      "KM/HR",		"M/S/S",		"FT/S/S" };

float ConversionFactors[] =  /* conversion from Units[] above to SI units */
    { 1.0,			0.017453,		1.0,			0.017453,		1.0, 
      0.017453,		1.0,			0.3048,			1.0,			1000.0,
      1852.0,		0.5144,			0.4470,			1.0,			0.00508,
      1.0,			4.4482,			1.0,			0.453592,		1.0,
      2.2046,		1.3788,			0.01,			1.0,			33.8639,
      1.0,			1.0,			60.0,			3600.0,			0.1047, 
      0.277778,		1.0,			0.3048 };

char *APWords[] =
    { "", "ALT_HOLD", "HDG_HOLD", "SPD_HOLD", "VSPD_HOLD" };

#define MaxCmdWords        (sizeof(CmdWords) / sizeof(char *))
#define MaxPlotWords       (sizeof(PlotWords) / sizeof(char *))
#define MaxSetWords        (sizeof(SetWords) / sizeof(char *))
#define MaxInputWords      (sizeof(InputWords) / sizeof(char *))
#define MaxShapeWords      (sizeof(ShapeWords) / sizeof(char *))
#define MaxUnitWords       (sizeof(Units) / sizeof(char *))
#define MaxConversionWords (sizeof(ConversionFactors) / sizeof(float *))
#define MaxAPWords         (sizeof(APWords) / sizeof(char *))

#define MaxStringSize      80

#define k_ERROR            0
#define k_SET              1
#define k_PLOT             2
#define k_ENGAGE           3
#define k_DISENGAGE        4
#define k_TIME             5
#define k_AUTOTRIM         6
#define k_INPUT            7

#define k_NUMBER          200
#define k_NAME            300

#define k_EOF             9999

typedef struct
{
    unsigned int var;
    unsigned int units;
    float scalefactor;
    float ymin;
    float ymax;
} OutputListType;

bool          SymbolBackSpacing;
char          LastSymbolString[MaxStringSize + 1];
unsigned int  LastSymbol;
bool          File_EOF;
FILE          *InputFileStream;
FILE          *OutputFileStream;
FILE          *PltFileStream;
unsigned int   LineNumber;
OutputListType OutputList[50];
unsigned int   NumberOfOutputs = 0;
float          WindSpeed = 0.0;
float          WindDirection = 0.0;

/* prototypes */
void         CopyString(char a[], char b[]);
void         Complain(char ErrorMessage[], char Details[]);
unsigned int ReadSymbol(char a[]);
void         ReadName(char a[]);
void         ReadNumber(char a[]);
unsigned int NextSymbol(char Str[]);
void         FileBackSpace(char Ch);
unsigned int CheckUnits(char a[]);
float        ConvertNumber(char Str[]);
unsigned int FindSetVariable(char v[]);
unsigned int FindPlotVariable(char v[]);
unsigned int FindInputMode(char v[]);
unsigned int FindShape(char v[]);
unsigned int FindUnits(char v[]);
unsigned int FindAP(char v[]);
void         ReadStatement();
void         GetFileName(char infile[], char outfile[], char ext[]);
void         WritePltFile(char FileName[]);
bool         ReadSimplotFile(char FileName[]);
bool         Digit(char Ch);
char         Cap(char Ch);
bool         Alphabetic(char Ch);
char         Rdch();
void         PrintOutputs(int n);
void         PlotString(char str[], char underscore_str[]);

/* ----------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    BEGIN_Aero();
    BEGIN_AeroLink();
    BEGIN_EngLink();
    BEGIN_Clocks();
    BEGIN_Engines();
    BEGIN_FCS();
    BEGIN_Stab();
    BEGIN_IOLib();
    BEGIN_Maths();
    BEGIN_Model();
    BEGIN_Gear();
    BEGIN_Simulate();
    BEGIN_Systems();
    BEGIN_Weather();

    LineNumber = 1;

    if (argc > 1)
    {
        if (ReadSimplotFile(argv[1]) == false)
        {
            printf("Cannot find %s\n", argv[1]);
            return 0;
        }
    }

    WindSpeed = 0.0;
    WindDirection = 0.0;
    
    Simulate_Simulate(PrintOutputs);
    WritePltFile(argv[1]);
    return 0;
}

/* ----------------------------------------------------------------------------- */
void CopyString(char a[], char b[])
{
    strcpy(b, a);  /* copy string <a> to <b> */
}

/* ----------------------------------------------------------------------------- */
void Complain(char ErrorMessage[], char Details[])
{
    printf("Error: Line %d %s %s\n", LineNumber, ErrorMessage, Details);
    exit(0);
}

/* ----------------------------------------------------------------------------- */
unsigned int ReadSymbol(char Str[])
{
    unsigned int p;
    char Ch;

    if (SymbolBackSpacing)
    {
        CopyString(LastSymbolString, Str);
        SymbolBackSpacing = false;
        return LastSymbol;
    }

    p = 0;

    do
    {
        Ch = Rdch();
        if (Ch == (char) EOF)
        {
            Str[0] = '\0';
            return k_EOF;
        }
    } while (Ch == SPACE || Ch == TAB || Ch == EOL);

    switch (Ch)
    {
        case 'A': case 'B': case 'C': case 'D': case 'E':
        case 'F': case 'G': case 'H': case 'I': case 'J':
        case 'K': case 'L': case 'M': case 'N': case 'O':
        case 'P': case 'Q': case 'R': case 'S': case 'T':
        case 'U': case 'V': case 'W': case 'X': case 'Y':
        case 'Z':
            do
            {
                Str[p] = Ch;
                p     += 1;
                Ch     = Rdch();
            } while ((Ch >= 'A' && Ch <= 'Z') ||
                     (Ch == '_' && p > 0)      ||
                     (Ch == '/' && p > 0)     ||
                     (Ch >= '0' && Ch <= '9' && p > 0));

            Str[p] = '\0';
            FileBackSpace(Ch);
            CopyString(Str, LastSymbolString);
            LastSymbol = k_NAME;
            return LastSymbol;
            break;

        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
        case '-':
            do
            {
                Str[p] = Ch;
                p     += 1;
                Ch     = Rdch();
            } while ((Ch >= '0' && Ch <= '9') || Ch == '.');

            Str[p] = '\0';
            FileBackSpace(Ch);
            CopyString(Str, LastSymbolString);
            LastSymbol = k_NUMBER;
            return LastSymbol;
            break;

        case '%':
            CopyString("%", Str);    /* % is a valid unit! */
            CopyString(Str, LastSymbolString);
            LastSymbol = k_NAME;
            return LastSymbol;
            break;

        default:
            Complain("Unrecognised Keyword", Str);
            return k_ERROR;
            break;
    }
}

/* ----------------------------------------------------------------------------- */
void ReadName(char Str[])
{
    unsigned int p;

    p = ReadSymbol(Str);
    if (p != k_NAME && p != k_EOF)
    {
        Complain("name expected", Str);
    }
}

/* ----------------------------------------------------------------------------- */
void ReadNumber(char Str[])
{
    unsigned int p;

    p = ReadSymbol(Str);
    if (p != k_NUMBER && p != k_EOF)
    {
        Complain("number expected", Str);
    }
}

/* ----------------------------------------------------------------------------- */
unsigned int NextSymbol(char Str[])
{
    unsigned int Op;

    Op = ReadSymbol(Str);
    CopyString(Str, LastSymbolString);
    SymbolBackSpacing = true;
    return Op;
}

/* ----------------------------------------------------------------------------- */
void FileBackSpace(char Ch)
{
    if (Ch == EOL)
    {
        LineNumber = LineNumber - 1;
    }
    ungetc(Ch, InputFileStream);
}

/* ----------------------------------------------------------------------------- */
unsigned int CheckUnits(char Str[])
{
    unsigned int p;

    NextSymbol(Str);
    p = FindUnits(Str);
    if (p > 0)
    {
        ReadSymbol(Str);
        return p;
    }
    else
    {
        return 0;
    }
}


/* ----------------------------------------------------------------------------- */
float ConvertNumber(char Str[])
{
    double(x);

    x = atof(Str);
    return (float) x;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindSetVariable(char v[])
{
    unsigned int i;

    for (i = 1; i < MaxSetWords; i += 1)
    {
        if (strcmp(v, SetWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindPlotVariable(char v[])
{
    unsigned int i;

    for (i = 1; i < MaxPlotWords; i += 1)
    {
        if (strcmp(v, PlotWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindInputMode(char v[])
{
    unsigned int i;

    for (i = 1; i < MaxInputWords; i += 1)
    {
        if (strcmp(v, InputWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindShape(char v[])
{
    unsigned int i;

    for (i = 1; i < MaxShapeWords; i += 1)
    {
        if (strcmp(v, ShapeWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindUnits(char v[])
{
    unsigned int i;

    for (i = 1; i < MaxUnitWords; i += 1)
    {
        if (strcmp(v, Units[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindAP(char v[])
{
    unsigned int i;

    for (i = 1; i < MaxAPWords; i += 1)
    {
        if (strcmp(v, APWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
void ReadStatement()
{
    unsigned int Op;
    unsigned int p1, p2;
    char         Str[80];
    float        x1, x2, x3;
    unsigned int i;

    ReadName(Str);

    if (File_EOF)
    {
        return;
    }

    Op = 0;
    for (i=1; i<MaxCmdWords; i+=1)
    {
        if (strcmp(Str, CmdWords[i]) == 0)
        {
            Op = i;
            break;
        }
    }

    switch (Op)
    {
        case k_SET:
            ReadName(Str);
            p1 = FindSetVariable(Str);
            if (p1 > 0)
            {
                ReadNumber(Str);
                x1 = ConvertNumber(Str);
                p2 = CheckUnits(Str);
                if (p2 > 0)
                {
                    x1 = x1 * ConversionFactors[p2];
                }

                switch (p1)
                {
                    case 1:
                        Weather_Turbulence_Level = x1;
                        break;
                    case 2:
                        WindSpeed = x1;
                        Weather_SetWind(WindSpeed, WindDirection, false);
                        break;
                    case 3:
                        WindDirection = x1;
                        Weather_SetWind(WindSpeed, WindDirection, false);
                        break;
                    case 4:
                        Weather_RegionalQNH = (unsigned int) x1;
                        break;
                    case 5:
                        /* to be written MagneticVariation = x1; */
                        break;
                    case 6:
                        Weather_ISADeviation = x1;
                        break;
                    case 7:
                        Simulate_Rudder = x1 / Aero_RudderGain;
                        break;
                    case 8:
                        Simulate_Aileron = x1 / Aero_AileronGain;
                        break;
                    case 9:
                        Simulate_Elevator = x1 / Aero_ElevatorGain;
                        break;
                    case 10:
                        Simulate_EngineLever = x1;
                        break;
                    case 11:
                        Simulate_Altitude = -x1;
                        break;
                    case 12:
                        Simulate_Heading = x1;
                        break;
                    case 13:
                        Simulate_Speed = x1;
                        Simulate_IAS = false;
                        break;
                    case 14:
                        Simulate_VSpeed = x1;
                        break;
                    case 15:
                        Aero_CgPosition = x1;
                        break;
                    case 16:
                        Systems_FlapPosition = x1 / 30.0;  /* assume 0-30 */
                        break;
                    case 17:
                        Systems_GearPosition = x1;
                        break;
                    case 18:
                        Simulate_FCU_ALT = (unsigned short int) x1;
                        break;
                    case 19:
                        Simulate_FCU_HDG = (unsigned short int) x1;
                        break;
                    case 20:
                        Simulate_FCU_SPD = (unsigned short int) x1;
                        break;
                    case 21:
                        Simulate_FCU_VS = (short int) x1;
                        break;
                    case 22:
                        FCS_Kp = x1;
                        break;
                    case 23:
                        FCS_Ki = x1;
                        break;
                    case 24:
                        FCS_Kd = x1;
                        break;
                    case 25:
                        Simulate_Speed = x1;
                        Simulate_IAS = true;
                    case 26:
                        Simulate_Spoiler = x1;
                        break;
                    case 27:
                        Simulate_ParkBrake = x1;
                        break;
                    case 28:
                        Simulate_LeftBrake = x1;
                        break;
                    case 29:
                        Simulate_RightBrake = x1;
                        break;
                    case 30:
                        Simulate_Latitude = x1;
                        break;
                    case 31:
                        Simulate_Longitude = x1;
                        break;
                    case 34:
                        Simulate_Mass = x1;
printf("Mass set to %f\n", x1); // ***
                        break;
                }
            }
            else
            {
                Complain("Unknown Variable in Set Statement", Str);
            }
            break;

            case k_PLOT:
                ReadName(Str);
                p1 = FindPlotVariable(Str);
                if (p1 > 0)
                {
                    p2 = CheckUnits(Str);
                    if (p2 > 0)
                    {
                        x1 = ConversionFactors[p2];
                    }
                    else
                    {
                        x1 = 1.0;
                    }
                    ReadNumber(Str);
                    x2 = ConvertNumber(Str);
                    ReadNumber(Str);
                    x3 = ConvertNumber(Str);
                    NumberOfOutputs = NumberOfOutputs + 1;
                    OutputList[NumberOfOutputs].var = p1;
                    OutputList[NumberOfOutputs].units = p2;
                    OutputList[NumberOfOutputs].scalefactor = x1;
                    OutputList[NumberOfOutputs].ymin = x2;
                    OutputList[NumberOfOutputs].ymax = x3;
                }
                else
                {
                    Complain("Unknown Variable in Plot Statement", Str);
                }
                break;

            case k_TIME:
                ReadNumber(Str);
                x1 = ConvertNumber(Str);
                p2 = CheckUnits(Str);
                if (p2 > 0)
                {
                    x1 = x1 * ConversionFactors[p2];
                }
                Simulate_SimSteps = (unsigned int) x1 * 50;
                break;

            case k_ENGAGE:
                ReadName(Str);
                p1 = FindAP(Str);
                if (p1 > 0)
                {
                    switch (p1)
                    {
                        case 1:
                            Simulate_FCU_ALT_Hold = true;
                            break;
                        case 2:
                            Simulate_FCU_HDG_Hold = true;
                            break;
                        case 3:
                            Simulate_FCU_SPD_Hold = true;
                            break;
                        case 4:
                            Simulate_FCU_VS_Hold = true;
                            break;
                    }
                }
                else
                {
                    Complain("Unrecognised AP mode in Engage statement", Str);
                }
                break;

           case k_DISENGAGE:
               ReadName(Str);
               p1 = FindAP(Str);
               if (p1 > 0)
               {
                   switch (p1)
                   {
                       case 1:
                           Simulate_FCU_ALT_Hold = false;
                           break;
                        case 2:
                            Simulate_FCU_HDG_Hold = false;
                            break;
                        case 3:
                            Simulate_FCU_SPD_Hold = false;
                            break;
                        case 4:
                            Simulate_FCU_VS_Hold = false;
                            break;
                    }
               }
               else
               {
                   Complain("Unrecognised AP mode in Disengage statement", Str);
               }
               break;

            case k_AUTOTRIM:
                Simulate_AutoTrimMode = true;
                break;

            case k_INPUT:
                ReadName(Str);
                p1 = FindInputMode(Str);
                if (p1 > 0)
                {
                    ReadName(Str);
                    p2 = FindShape(Str);
                    if (p2 > 0)
                    {
                        ReadNumber(Str);
                        x1 = ConvertNumber(Str);
                        ReadNumber(Str);
                        x2 = ConvertNumber(Str);
                        ReadNumber(Str);
                        x3 = ConvertNumber(Str);
                        switch(p1)
                        {
                            case 1:
                                x3 = Maths_Rads(x3) / Aero_ElevatorGain;
                                break;
                            case 2:
                                x3 = Maths_Rads(x3) / Aero_AileronGain;
                                break;
                            case 3:
                                x3 = Maths_Rads(x3) / Aero_RudderGain;
                                break;
                                
                        }
                        Simulate_SetInputs(p1, p2, x1, x2, x3);
                    }
                    else
                    {
                        Complain("Unrecognised shape", Str);
                    }
                }
                else
                {
                    Complain("Unrecognised input mode", Str);
                }
                break;

            default:
                Complain("Unrecognised Keyword", Str);
                break;
    }
}

/* ----------------------------------------------------------------------------- */
void GetFileName(char infile[], char outfile[], char ext[])
{
    unsigned int i=0;
    unsigned int j=0;

    for (i=0; ; i++)
    {
        outfile[i] = infile[i];
        if (infile[i] == '.')
        {
            for (j=0; ; j++)
            {
                outfile[i+j+1] = ext[j];
                if (ext[j] == 0)
                {
                    return;
                }
            }
            return;
        }
    }
    Complain("Bad filename", infile);
}

/* ----------------------------------------------------------------------------- */
void PlotString(char str[], char underscore_str[])  /* only needed because GNUPLOT treats underscore as a subscript */
{
    unsigned int p = 0;
    
    while (1)
    {
        char ch = underscore_str[p];
        
        if (ch == '_')
        {
            ch = ' ';
        }
        str[p] = ch;
        if (ch == '\0')
        {
            break;
        }
        p += 1;
    }
}

/* ----------------------------------------------------------------------------- */
void WritePltFile(char FileName[])
{
    char         PngFileName[20];
    char         DatFileName[20];
    char         PlotName[50];
    
    unsigned int i;
    float        dy;

    dy = 1.0 / (float) NumberOfOutputs;

    GetFileName(FileName, DatFileName, "dat");
    GetFileName(FileName, PngFileName, "png");
    fprintf(PltFileStream, "set terminal png truecolor font arial 8 size 600,800\nset output \"%s\"\n", PngFileName);
    fprintf(PltFileStream, "set size 1,1\nset origin 0,0\nset lmargin 10\n");
    fprintf(PltFileStream, "set multiplot\nset grid\nset format y \"%%5g\"\n");
    fprintf(PltFileStream, "set size 1.0, %f\n", dy);
    fprintf(PltFileStream, "set style line 1 lw 1 linecolor rgb \"blue\"\n");

    for (i=1; i<=NumberOfOutputs; i++)
    {
        PlotString(PlotName, PlotWords[OutputList[i].var]);
        fprintf(PltFileStream, "set origin 0, %f\n", 1.0 - (float) i * dy);
        fprintf(PltFileStream, "set ylabel \"%s", PlotName);
        if (OutputList[i].units > 0)
        {
            fprintf(PltFileStream, " %s\"\n", Units[OutputList[i].units]);
        }
        else
        {
            fprintf(PltFileStream, "\"\n");
        }
        fprintf(PltFileStream, "set xr[0.0:%f]\n", (float) Simulate_SimSteps / 50.0);
        fprintf(PltFileStream, "set yr[%f:%f]\n", OutputList[i].ymin, OutputList[i].ymax);
        fprintf(PltFileStream, "plot '%s' using 1:%d title '%s' with lines\n", DatFileName, i+1, PlotName);
    }
    fprintf(PltFileStream, "unset multiplot\nreset\nset output\n");
    fclose(PltFileStream);
}

/* ----------------------------------------------------------------------------- */
bool ReadSimplotFile(char FileName[])
{
    char OutFileName[20];
    char PltFileName[20];

    File_EOF = false;

    GetFileName(FileName, OutFileName, "dat");
    OutputFileStream = fopen(OutFileName, "w");
    if (OutputFileStream == NULL)
    {
        Complain("Can't open", OutFileName);
    }

    GetFileName(FileName, PltFileName, "plt");
    PltFileStream = fopen(PltFileName, "w");
    if (PltFileStream == NULL)
    {
        Complain("Can't open", PltFileName);
    }

    InputFileStream = fopen(FileName, "r");
    if (InputFileStream == NULL)
    {
        return false;
    }
    else
    {
        do
        {
            ReadStatement();
        } while (!(File_EOF));

        fclose(InputFileStream);
        return true;
    }
}

/* ----------------------------------------------------------------------------- */
bool Digit(char Ch)
{
    return (Ch >= '0') && (Ch <= '9');
}

/* ----------------------------------------------------------------------------- */
char Cap(char Ch)
{
    if ((Ch >= 'a') && (Ch <= 'z'))
    {
        return Ch + 'A' - 'a';
    }
    else
    {
        return Ch;
    }
}

/* ----------------------------------------------------------------------------- */
bool Alphabetic(char Ch)
{
    return ((Ch >= 'A') && (Ch <= 'Z')) || (Ch == '_');
}

/* ----------------------------------------------------------------------------- */
char Rdch()
{
    char Ch;

    Ch = fgetc(InputFileStream);
    if (Ch == CR)      /* skip CR, return LF as EOL */
    {
        return Rdch();
    }

    if (Ch == EOL)
    {
        LineNumber = LineNumber + 1;
    }

    if (Ch == ';')
    {
        while (Ch != EOL && Ch != (char) EOF)
        {
            Ch = fgetc(InputFileStream);
        }
        if (Ch == EOL)
        {
            LineNumber = LineNumber + 1;
            return Rdch();
        }
    }
    File_EOF = Ch == (char) EOF;
    return Cap(Ch);
}

/* ----------------------------------------------------------------------------- */
void PrintOutputs(int n)
{
    unsigned int i;
    float        x = 0.0;

    if (n <= 0)
    {
        fclose(OutputFileStream);
        return;
    }

    fprintf(OutputFileStream, "%f, ", ((float) n) / 50.0);

    for (i=1; i<=NumberOfOutputs; i = i+1)
    {
        switch(OutputList[i].var)
        {
            case 1:
                x = Model_Rudder * Aero_RudderGain;
                break;
            case 2:
                x = Model_Aileron * Aero_AileronGain;
                break;
            case 3:
                x = Model_Elevator * Aero_ElevatorGain;
                break;
            case 4:
                x = Model_U;  /* *TAS */
                break;
            case 5:
                x = -Model_Pz;
                break;
            case 6:
                x = Model_BetaDot;
                break;
            case 7:
                x = Model_Beta;
                break;
            case 8:
                x = Model_AlphaDot;
                break;
            case 9:
                x = Model_Alpha;
                break;
            case 10:
                x = Model_Q;
                break;
            case 11:
                x = Model_P;
                break;
            case 12:
                x = Model_R;
                break;
            case 13:
                x = Model_QDot;
                break;
            case 14:
                x = Model_PDot;
                break;
            case 15:
                x = Model_RDot;
                break;
            case 16:
                x = Model_Pitch;
                break;
            case 17:
                x = Model_Roll;
                break;
            case 18:
                x = Model_Yaw;
                break;
            case 19:
                x = Model_Lift;
                break;
            case 20:
                x = Model_Thrust;
                break;
            case 21:
                x = Model_Drag;
                break;
            case 22:
                x = Model_SideForce;
                break;
            case 23:
                x = Engines_ThrottleLever[0];
                break;
            case 24:
                x = -Model_Vd / sqrt(Model_Vn * Model_Vn + Model_Ve * Model_Ve);
                break;
            case 25:
                x = -Model_Vd;
                break;
            case 26:
                x = Model_U * sqrt(Weather_DensityRatio);  /* IAS */
                break;
            case 27:
                x = 0.0; // *** Spoiler to be addedd
            case 28:
                x = (Model_LeftBrake + Model_RightBrake) / 2.0;
                break;
            case 29:
                x = Model_LeftBrake;
                break;
            case 30:
                x = Model_RightBrake;
                break;		   
            case 31:
                x = AeroLink_AeroPkt.FlightData[0];
                break;		   
            case 32:
                x = AeroLink_AeroPkt.FlightData[1];
                break;		   
            case 33:
                x = AeroLink_AeroPkt.FlightData[2];
                break;		   
            case 34:
                x = Engines_Engines[0].Epr;
                break;	
            case 35:
                x = Simulate_Distance;
                break;   
            case 36:
                x = Model_MachNumber;
                break;   
            case 37:
                x = Simulate_Mass;
                break;   
        }

        fprintf(OutputFileStream, "%f", x / OutputList[i].scalefactor);
        if (i < NumberOfOutputs)
        {
            fprintf(OutputFileStream, ", ");
        }
        else
        {
            fprintf(OutputFileStream, "\n");
        }
    }
}
