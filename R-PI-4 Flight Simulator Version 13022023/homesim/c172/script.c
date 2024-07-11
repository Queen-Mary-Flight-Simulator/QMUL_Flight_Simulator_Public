/* script.c
   modified 2/2/12 to remove fgetpos and fsetpos
   modified 29/01/21 for event version
   modified 21/03/22 char changed to int for most parsing
   DJA
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

#include <SIM/iosdefn.h>
#include <SIM/maths.h>
#include <SIM/navlib.h>

#include "ioslink.h"
#include "script.h"
#include "iolib.h"

#define ONERAD  (180.0L / M_PI)

const int CR    = 13;
const int EOL   = 10;
const int TAB   = 9;
const int SPACE = ' ';

bool      Script_Enabled;
bool      Script_Error;
char      Script_ErrorMessage[200];

typedef enum 
{
    Phase_None, Phase_Event, Phase_Action, Phase_Record, Phase_End
} ScriptPhase;

typedef enum 
{
    Waiting_To_Start, Waiting_To_Stop, Inactive
} ScriptState;

typedef struct
{
    unsigned int EventStart;
    unsigned int ActionStart;
    unsigned int RecordStart;
    unsigned int EndStart;
    ScriptState  Status;
} ScriptRecord;

typedef struct
{
    unsigned int Op;
    unsigned int Arg;
    float        Val;
} Instruction;

#define Off            0
#define PcTime         1
#define SimTime        2
#define ElapsedTime    3

typedef unsigned char   TimeStampMode;

#define Debugging         0
#define Null              ""
#define MaxStringSize     80

static char *KeyWords[] =
{ "",               "EVENT:",         "ACTION:",        "RECORD:",        "END:" };

static char *OperatorWords[] =
{ "",               "OR",         "AND" };

static char *ActionWords[] =
{ "",               "SET",            "FAIL",           "RESET",          "SAMPLE",
  "LOG",            "REPOSITION",     "ENGAGE",         "DISENGAGE",      "TIMESTAMP",
  "LOADFLTPLN",     "ACTIVATE",       "DEACTIVATE" };

static char *VarWords[] =
{ "",               "PITCH",          "ROLL",           "YAW",            "HEADING",
  "PITCH_RATE",     "ROLL_RATE",      "YAW_RATE",       "PITCH_ACCN",     "ROLL_ACCN",
  "YAW_ACCN",       "ALTITUDE",       "AIRSPEED",       "MACH_NUMBER",    "RATE_OF_CLIMB",
  "LATITUDE",       "LONGITUDE",      "ALPHA",          "BETA",           "ALPHA_RATE",
  "BETA_RATE",      "LIFT",           "THRUST",         "DRAG",           "SIDEFORCE",      
  "ELEVATOR",       "AILERON",        "RUDDER",         "ELEVATOR_TRIM",  "AILERON_TRIM",   
  "RUDDER_TRIM",    "LEFT_BRAKE",     "RIGHT_BRAKE",    "FLAPS",          "GEAR",           
  "MASS",           "ENGINE_LEVER1",  "ENGINE_LEVER2",  "ENGINE_LEVER3",  "ENGINE_LEVER4",  
  "ENGINE_LEVER5",  "ENGINE_LEVER6",  "ENGINE1_THRUST", "ENGINE1_RPM",    "ENGINE1_FF",     
  "ENGINE1_EGT",    "ENGINE1_BETA",   "ENGINE1_MP",     "ENGINE1_EPR",    "ENGINE2_THRUST", 
  "ENGINE2_RPM",    "ENGINE2_FF",     "ENGINE2_EGT",    "ENGINE2_BETA",   "ENGINE2_MP",     
  "ENGINE2_EPR",    "ENGINE3_THRUST", "ENGINE3_RPM",    "ENGINE3_FF",     "ENGINE3_EGT",    
  "ENGINE3_BETA",   "ENGINE3_MP",     "ENGINE3_EPR",    "ENGINE4_THRUST", "ENGINE4_RPM",    
  "ENGINE4_FF",     "ENGINE4_EGT",    "ENGINE4_BETA",   "ENGINE4_MP",     "ENGINE4_EPR",    
  "FUEL_LEFT",      "FUEL_RIGHT",     "RHO",            "OAT",            "BARO1",          
  "BARO2",          "MAGNETIC_VAR",   "NAV1_GS",        "NAV1_LOC",       "NAV1_DME",
  "NAV1_BEARING",   "NAV2_GS",        "NAV2_LOC",       "NAV2_DME",       "NAV2_BEARING",   
  "ADF1_DME",       "ADF1_BEARING",   "ADF2_DME",       "ADF2_BEARING",   "HSI_CRS",        
  "HSI_HDG",        "VOR_OBS",        "TIME" };

static char *SetWords[] =
{ "",               "TURBULENCE",     "WIND_SPEED",     "WIND_DIRECTION",   "QNH",
  "MAG_VAR",        "CLOUD_BASE",     "VISIBILITY",     "OAT",              "ALTITUDE",
  "HEADING",        "SPEED",          "CG_POSITION",    "RIGHT_FUEL",       "LEFT_FUEL",
  "TARGET_DIST",    "TARGET_SPEED",   "TARGET_HDG",     "TARGET_TURN_RATE", "TARGET_ALTITUDE", 
  "TARGET_ROC",     "AP_ALT",         "AP_HDG",         "AP_SPD" };

static char *FailWords[] =
{ "",               "FLAPS",          "GEAR",           "NAV1_LOC",       "NAV1_GS", 
  "NAV2_LOC",       "NAV2_GS",        "RMI1",           "RMI2",           "DME",
  "ENGINE1",        "ENGINE2",        "ENGINE3",        "ENGINE4",        "ASI",  
  "AI",             "VSI",            "ALTIMETER",      "TURN",           "HSICARD",  
  "RMICARD" };

static char *Units[] =
{ "",               "DEGS",           "RADS",           "DEG/S",          "RAD/S",
  "DEG/S/S",        "RAD/S/S",        "FT",             "M",              "KM",   
  "NM",             "KTS",            "MPH",            "M/S",            "FPM",
  "N",              "LBF",            "KG",             "LBS",            "KG/HR", 
  "LBS/HR",         "LITRES/HR",      "%",              "MB",             "INHG",
  "KG/M/M/M",       "SECS",           "MINS",           "HOURS",          "RPM", 
  "KM/HR",          "M/S/S",          "FT/S/S" };

static float ConversionFactor[] =
{ 1.0,              0.017453,         1.0,              0.017453,         1.0,       /* N.B. must be matched to Units */
  0.017453,         1.0,              0.3048,           1.0,              1000.0,   
  1852.0,           0.5144,           0.4470,           1.0,              0.00508,
  1.0,              4.4482,           1.0,              0.453592,         1.0,   
  2.2046,           1.3788,           0.01,             1.0,              33.8639,      
  1.0,              1.0,              60.0,             3600.0,           0.1047, 
  0.277778,         1.0,              0.3048 };

static char *APWords[] =
{ "",               "ALT_HOLD",      "HDG_HOLD",        "SPD_HOLD" };

#define MaxKeyWords       (sizeof(KeyWords) / sizeof(char *))
#define MaxOperatorWords  (sizeof(OperatorWords) / sizeof(char *))
#define MaxActionWords    (sizeof(ActionWords) / sizeof(char *))
#define MaxVarWords       (sizeof(VarWords) / sizeof(char *))
#define MaxSetWords       (sizeof(SetWords) / sizeof(char *))
#define MaxFailWords      (sizeof(FailWords) / sizeof(char *))
#define MaxUnits          (sizeof(Units) / sizeof(char *))
#define MaxConversions    (sizeof(ConversionFactor) / sizeof(char *))
#define MaxAPWords        (sizeof(APWords) / sizeof(char *))

#define k_ERROR           0
#define k_EVENT           1
#define k_ACTION          2
#define k_RECORD          3
#define k_END             4
#define k_OR              5
#define k_AND             6

#define k_SET             101
#define k_FAIL            102
#define k_RESET           103
#define k_SAMPLE          104
#define k_LOG             105
#define k_REPOSITION      106
#define k_ENGAGE          107
#define k_DISENGAGE       108
#define k_TIMESTAMP       109
#define k_LOADFLTPLN      110
#define k_ACTIVATE        111
#define k_DEACTIVATE      112

#define k_LBRACKET        201
#define k_RBRACKET        202
#define k_LESS            203
#define k_GREATER         204
#define k_PLUS            205
#define k_MINUS           206
#define k_MULT            207
#define k_DIV             208
#define k_NUMBER          209
#define k_VAR             210

#define k_EOF             9999

#define i_PUSHV           1
#define i_PUSHC           2
#define i_COMP            3
#define i_ADD             10
#define i_SUB             11
#define i_MULT            12
#define i_DIV             13
#define i_RECORD          14
#define i_SET             15
#define i_FAIL            16
#define i_RESET           17
#define i_SAMPLE          19
#define i_SWAP            20
#define i_NEG             21
#define i_LOG             22
#define i_TIMESTAMP       23
#define i_REPOSITION      24
#define i_ENGAGE          25
#define i_DISENGAGE       26
#define i_LOADFLTPLN      27
#define i_ACTIVATE        28
#define i_DEACTIVATE      29
#define i_RETURN          30

#define i_OR              51
#define i_AND             52

#define StackSize         200
#define CodeSize          2000
#define MaxMessages       100
#define OutFileBufSize    100000
#define MaxScripts        100

static unsigned int  Sp;
static float         Stack[StackSize];
static FILE          *FileStream;
static unsigned int  LineNumber;
static unsigned int  PC;
static Instruction   Instructions[CodeSize];
static float         SampleRate;
static unsigned int  ProgramSize;
static int           CurrentTick;
static int           NextTick;
static bool          Recording;
static float         RecordingList[MaxVarWords + 1];
static unsigned int  RecordingPtr;
static unsigned int  MessageNumber;
static char          Messages[MaxMessages + 1][80];
static TimeStampMode TimeStamping;
static int           PcTime0;
static int           ElapsedTimeCount;
static char          OutFileBuf[OutFileBufSize];
static unsigned int  OutFileBufPtr;
static bool          File_EOF;
static bool          SymbolBackSpacing;
static unsigned int  LastSymbol;
static char          LastSymbolString[MaxStringSize + 1];
static ScriptPhase   Phase = Phase_None;
static ScriptRecord  Script;

void          OutputWrch(char Ch);
void          OutputWriteString(char a[]);
void          OutputWriteReal(float x);
void          Complain(char ErrorMessage[], char Details[]);
unsigned int  ReadSymbol(char a[]);
unsigned int  CheckUnits(char a[]);
void          Script_ReadString(char a[], int Ch);
void          ReadName(char a[]);
void          ReadNumber(char a[]);
float         ConvertNumber(char Str[]);
unsigned int  FindVariable(char v[]);
unsigned int  FindSetVariable(char v[]);
unsigned int  FindFailure(char v[]);
unsigned int  FindUnits(char v[]);
void          ReadbooleanExpression();
void          ReadExpression();
unsigned int  ReadStatement();
unsigned int  NextMessage(char Str[]);
unsigned int  SetCommand(unsigned int p);
void          GenCode1(unsigned int OpCode);
void          GenCode2(unsigned int OpCode, unsigned int Arg);
void          GenCode3(unsigned int OpCode, unsigned int Arg, float Value);
bool          Digit(int Ch);
bool          Alphabetic(int Ch);
int           Script_Rdch();
void          StackInit();
void          StackPush(float Item);
float         StackPop();
float         StackTop();
void          StackPushboolean(bool Item);
bool          StackPopboolean();
float         Norm360(float x);
void          Record(float x);
void          WriteTimeStamp();
void          MonitorScript();
int           GetPcTime();
void          Initialise_Script();
float         ReadLatitude(char str[]);
float         ReadLongitude(char str[]);
void          CopyString(char a[], char b[]);
unsigned int  NextSymbol(char Str[]);
void          FileBackSpace(int Ch);
unsigned int  ReadAction(int Op);
void          ScriptUpdate(char Filename[]);
unsigned int  Lookup(char v[], char *name[], unsigned int maxwords);
int           CAP(int Ch);

/* ----------------------------------------------------------------------------- */
void CopyString(char a[], char b[])
{
    strcpy(b, a);  /* copy string <a> to <b> */
}

/* ----------------------------------------------------------------------------- */
float ReadLatitude(char str[])
{
    char         dir;
    unsigned int d;
    unsigned int m;
    unsigned int s;
    float        Lat;

    dir = str[0];
    if (dir != 'N' && dir != 'S')
    {
        Complain("Bad Lat ID", str);
    }

    if (!Digit(str[1]) || !Digit(str[2]))
    {
        Complain("Bad Lat degs", str);
    }
    d = (str[1] - '0') * 10 + str[2] - '0';
    if (d > 89)
    {
        Complain("Bad Lat degs", str);
    }

    if (!Digit(str[3]) || !Digit(str[4]))
    {
        Complain("Bad Lat mins", str);
    }
    m = (str[3] - '0') * 10 + str[4] - '0';
    if (m > 59)
    {
        Complain("Bad Lat mins", str);
    }
    if (str[5] != '.')
    {
        Complain("Bad Lat . missing", str);
    }

    if (!Digit(str[6]) || !Digit(str[7]))
    {
        Complain("Bad Lat secs", str);
    }
    s = (str[6] - '0') * 10 + str[7] - '0';
    if (s > 89)
    {
        Complain("Bad Lat secs", str);
    }

    Lat = (float) d + (float) m / 60.0 + (float) s / 6000.0;
    if (dir == 'S')
    {
        Lat = -Lat;
    }
    if (Lat < -90.0 || Lat > 90.0)
    {
        Complain("Bad Latitude", str);
    }
    return Lat;
}

/* ----------------------------------------------------------------------------- */
float ReadLongitude(char str[])
{
    char         dir;
    unsigned int d;
    unsigned int m;
    unsigned int s;
    float        Long;

    dir = str[9];
    if (dir != 'W' && dir != 'E')
    {
        Complain("Bad Long ID", str);
    }

    if (!Digit(str[10]) || !Digit(str[11]) || !Digit(str[12]))
    {
        Complain("Bad Long degs", str);
    }
    d = (str[10] - '0') * 100 + (str[11] - '0') * 10 + str[12] - '0';
    if (d > 179)
    {
        Complain("Bad Long degs", str);
    }

    if (!Digit(str[13]) || !Digit(str[14]))
    {
        Complain("Bad Long mins", str);
    }
    m = (str[13] - '0') * 10 + str[14] - '0';
    if (m > 59)
    {
        Complain("Bad Long mins", str);
    }

    if (str[15] != '.')
    {
        Complain("Bad Long . missing", str);
    }

    if (!Digit(str[16]) || !Digit(str[17]))
    {
        Complain("Bad Long secs", str);
    }
    s = (str[16] - '0') * 10 + str[17] - '0';
    if (s > 99)
    {
        Complain("Bad Long secs", str);
    }

    Long = (float) d + (float) m / 60.0 + (float) s / 6000.0;
    if (dir == 'W')
    {
        Long = -Long;
    }
    if (Long < -180 || Long > 180)
    {
        Complain("Bad Longitude", str);
    }
    return Long;
}

/* ----------------------------------------------------------------------------- */
void Script_SaveScript(char FileName[])
{
    FILE         *f;
    unsigned int i;
    
    if (OutFileBufPtr > 0)
    {
        f = fopen(FileName, "wb");
        if (f == NULL)
        {
            printf("Unable to open file %s", FileName);
            return;
        }

        for (i = 0; i < OutFileBufPtr; i += 1)
        {
            fputc(OutFileBuf[i], f);
        }

        fclose(f);
    }
}

/* ----------------------------------------------------------------------------- */
void OutputWrch(char Ch)
{
    if (OutFileBufPtr < OutFileBufSize)
    {
        if (Ch == EOL)
        {
            OutputWrch(CR);
        }
        OutFileBuf[OutFileBufPtr] = Ch;
        OutFileBufPtr             = OutFileBufPtr + 1;
    }
    else
    {
        Complain("outfile buffer overflow", "");
    }
}

/* ----------------------------------------------------------------------------- */
void OutputWriteString(char a[])
{
    unsigned int i;

    i = 0;
    while (a[i] != 0)
    {
        OutputWrch(a[i]);
        i += 1;
    }
}

/* ----------------------------------------------------------------------------- */
void OutputWriteReal(float x)
{
    char str[20];

    sprintf(str, "%7.2f", x);
    OutputWriteString(str);
}

/* ----------------------------------------------------------------------------- */
void Complain(char ErrorMessage[], char Details[])
{
    sprintf(Script_ErrorMessage, "Line %d <%s> %s\n", LineNumber, ErrorMessage, Details);
    Script_Enabled = false;
    Script_Error   = true;
}

/* ----------------------------------------------------------------------------- */
unsigned int ReadSymbol(char Str[])
{
    int          Ch;
    unsigned int i;

    if (SymbolBackSpacing)
    {
        CopyString(LastSymbolString, Str);
        SymbolBackSpacing = false;
        return LastSymbol;
    }

    Str[0] = 0;

    do
    {
        Ch = Script_Rdch();
        if (Ch == EOF)
        {
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
    
        FileBackSpace(Ch);
        ReadName(Str);
        for (i = 1; i < MaxKeyWords; i += 1)
        {
            if (strcmp(Str, KeyWords[i]) == 0)
            {
                LastSymbol = i;
                CopyString(Str, LastSymbolString);
                return LastSymbol;
            }
        }

        for (i = 1; i < MaxOperatorWords; i += 1)
        {
            if (strcmp(Str, OperatorWords[i]) == 0)
            {
                LastSymbol = i - 1 + k_OR;
                CopyString(Str, LastSymbolString);
                return LastSymbol;
            }
        }

        for (i = 1; i < MaxActionWords; i += 1)
        {
            if (strcmp(Str, ActionWords[i]) == 0 && Phase == Phase_Action)
            {
                LastSymbol = i + 100;
                CopyString(Str, LastSymbolString);
                return LastSymbol;
            }
        }
        CopyString(Str, LastSymbolString);
        LastSymbol = k_VAR;
        break;

    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        FileBackSpace(Ch);
        ReadNumber(Str);
        CopyString(Str, LastSymbolString);
        LastSymbol = k_NUMBER;
        break;

    case '(':
        LastSymbol = k_LBRACKET;
        break;

    case ')':
        LastSymbol = k_RBRACKET;
        break;

    case '<':
        LastSymbol = k_LESS;
        break;

    case '>':
        LastSymbol = k_GREATER;
        break;

    case '+':
        LastSymbol = k_PLUS;
        break;

    case '-':
        LastSymbol = k_MINUS;
        break;

    case '*':
        LastSymbol = k_MULT;
        break;

    case '/':
        LastSymbol = k_DIV;
        break;

    case '%':
        CopyString("%", Str);        /* % is a valid unit! */
        CopyString(Str, LastSymbolString);
        LastSymbol = k_VAR;
        break;

    default:
        Complain("Unrecognised Keyword %s\n", Str);
        return k_ERROR;
        break;
    }

    return LastSymbol;
}

/* ----------------------------------------------------------------------------- */
unsigned int NextSymbol(char Str[])
{
    unsigned int k = ReadSymbol(Str);
	
    LastSymbol = k;
    CopyString(Str, LastSymbolString);
    SymbolBackSpacing = true;
    return k;
}

/* ----------------------------------------------------------------------------- */
void FileBackSpace(int Ch)
{
    if (Ch == EOL)
    {
        LineNumber = LineNumber - 1;
    }
    ungetc(Ch, FileStream);
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
void Script_ReadString(char Str[], int tCh)   /* read a string until tCh encountered, stripping leading spaces */
{
    unsigned int p;
    int          Ch;

    p = 0;

    do        /* skip leading spaces and tabs */
    {
        Ch = Script_Rdch();
    } while (Ch == SPACE || Ch == TAB);

    do
    {
        Str[p] = Ch;
        p     += 1;
        Ch     = Script_Rdch();
    } while (Ch != tCh && Ch != EOL && Ch != EOF);

    Str[p] = 0;
}

/* ----------------------------------------------------------------------------- */
void ReadName(char Str[])
{
    unsigned int p;
    int          Ch;

    p = 0;

    do        /* skip leading spaces and tabs */
    {
        Ch = Script_Rdch();
    } while (Ch == SPACE || Ch == TAB);

    do
    {
        Str[p] = Ch;
        p     += 1;
        Ch     = Script_Rdch();
    } while (((Ch >= 'A') && (Ch <= 'Z')) ||
             ((Ch == '_') && (p > 0)) ||
             ((Ch == ':') && (p > 0)) ||
             ((Ch == '/') && (p > 0)) ||
             ((Ch >= '0') && (Ch <= '9') && (p > 0)));

    Str[p] = 0;
    FileBackSpace(Ch);
}

/* ----------------------------------------------------------------------------- */
void ReadNumber(char Str[])
{
    unsigned int p;
    int          Ch;

    p = 0;

    do        /* skip leading spaces and tabs */
    {
        Ch = Script_Rdch();
    } while (Ch == SPACE || Ch == TAB);

    do
    {
        Str[p] = Ch;
        p     += 1;
        Ch     = Script_Rdch();
    } while (((Ch >= '0') && (Ch <= '9')) || (Ch == '.'));

    Str[p] = 0;
    FileBackSpace(Ch);
}

/* ----------------------------------------------------------------------------- */
float ConvertNumber(char Str[])
{
    double x;

    x = atof(Str);
    return (float) x;
}

/* ----------------------------------------------------------------------------- */
unsigned int Lookup(char v[], char *name[], unsigned int maxwords)
{
    unsigned int i;

    for (i = 1; i < maxwords; i += 1)
    {
        if (strcmp(v, name[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindVariable(char v[])
{
    return Lookup(v, VarWords, MaxVarWords);
}

/* ----------------------------------------------------------------------------- */
unsigned int FindSetVariable(char v[])
{
    return Lookup(v, SetWords, MaxSetWords);
}

/* ----------------------------------------------------------------------------- */
unsigned int FindFailure(char v[])
{
    return Lookup(v, FailWords, MaxFailWords);
}

/* ----------------------------------------------------------------------------- */
unsigned int FindUnits(char v[])
{
    return Lookup(v, Units, MaxUnits);
}

/* ----------------------------------------------------------------------------- */
unsigned int FindAP(char v[])
{
    return Lookup(v, APWords, MaxAPWords);
}

/* ----------------------------------------------------------------------------- */
void ReadbooleanExpression()
{
    unsigned int Op;
    char         Str[20];
    bool         SwapArgs;

    if (NextSymbol(Str) == k_LBRACKET)
    {
        Op = ReadSymbol(Str);
        ReadbooleanExpression();
        Op = ReadSymbol(Str);
        if (Op != k_RBRACKET)
        {
            Complain(") missing", Null);
        }
    }
    else
    {
        ReadExpression();
        Op = ReadSymbol(Str);
        if (Op == k_LESS || Op == k_GREATER)
        {
            SwapArgs = Op == k_LESS;
            ReadExpression();
            if (SwapArgs)
            {
                GenCode1(i_SWAP);
            }
            GenCode1(i_COMP);
        }
        else
        {
            Complain("Relation operator expected in a bool expression", Str);
        }
    }

    Op = NextSymbol(Str);
    if (Op == k_OR)
    {
        Op = ReadSymbol(Str);
        ReadbooleanExpression();
        GenCode1(i_OR);
    }
    else if (Op == k_AND)
    {
        Op = ReadSymbol(Str);
        ReadbooleanExpression();
        GenCode1(i_AND);
    }
}

/* ----------------------------------------------------------------------------- */
void ReadExpression()
{
    unsigned int Op;
    unsigned int p;
    char         Str[80];
    float        x;

    Op = ReadSymbol(Str);

    if (Op == k_MINUS)
    {
        ReadExpression();
        GenCode1(i_NEG);
        return;
    }

    if (Op == k_LBRACKET)
    {
        ReadExpression();
        Op = ReadSymbol(Str);
        if (Op != k_RBRACKET)
        {
            Complain(") missing", Null);
        }
    }
    else if (Op == k_VAR)
    {
        p = FindVariable(Str);
        if (p > 0)
        {
            GenCode2(i_PUSHV, p);
        }
        else
        {
            Complain("Unknown variable in Expression", Str);
        }
    }
    else if (Op == k_NUMBER)
    {
        x = ConvertNumber(Str);
        p = CheckUnits(Str);
        if (p > 0)
        {
            x = x * ConversionFactor[p];
        }
        GenCode3(i_PUSHC, 0, x);
    }
    else
    {
        Complain("Unrecognised Variable", Str);
    }

    Op = NextSymbol(Str);
    if (Op == k_PLUS)
    {
        Op = ReadSymbol(Str);
        ReadExpression();
        GenCode1(i_ADD);
    }
    else if (Op == k_MINUS)
    {
        Op = ReadSymbol(Str);
        ReadExpression();
        GenCode1(i_SUB);
    }
    else if (Op == k_MULT)
    {
        Op = ReadSymbol(Str);
        ReadExpression();
        GenCode1(i_MULT);
    }
    else if (Op == k_DIV)
    {
        Op = ReadSymbol(Str);
        ReadExpression();
        GenCode1(i_DIV);
    }
}

/* ----------------------------------------------------------------------------- */
unsigned int ReadAction(int Op)
{
    int   p1;
    int   p2;
    char  Str[50];
    float x, x1, x2;
    
    switch (Op)
    {
        case k_SET:
            ReadName(Str);
            p1 = FindSetVariable(Str);
            if (p1 > 0)
            {
                ReadNumber(Str);
                x  = ConvertNumber(Str);
                p2 = CheckUnits(Str);
                if (p2 > 0)
                {
                    x = x * ConversionFactor[p2];
                }
                GenCode3(i_SET, SetCommand(p1), x);
                return k_SET;
            }
            else
            {
                Complain("Unknown Variable in Set Statement", Str);
            }
            return k_SET;
            break;

        case k_FAIL:
            ReadName(Str);
            p1 = FindFailure(Str);
            if (p1 > 0)
            {
                GenCode2(i_FAIL, p1);
                return k_FAIL;
            }
            else
            {
                Complain("Unrecognised Failure in Fail Statement", Str);
            }
            return k_FAIL;
            break;

        case k_RESET:
            ReadName(Str);
            p1 = FindFailure(Str);
            if (p1 > 0)
            {
                GenCode2(i_RESET, p1);
                return k_RESET;
            }
            else
            {
                Complain("Unrecognised Failure in Reset Statement", Str);
            }
            return k_RESET;
            break;

        case k_TIMESTAMP:
            ReadName(Str);
            if (strcmp(Str, "OFF") == 0)
            {
                TimeStamping = Off;
            }
            else if (strcmp(Str, "PCTIME") == 0)
            {
                TimeStamping = PcTime;
            }
            else if (strcmp(Str, "SIMTIME") == 0)
            {
                TimeStamping = SimTime;
            }
            else if (strcmp(Str, "ELAPSEDTIME") == 0)
            {
                TimeStamping = ElapsedTime;
            }
            else
            {
                Complain("Unknown timestamp mode", Str);
                TimeStamping = Off;
            }
            GenCode2(i_TIMESTAMP, TimeStamping);
            return k_TIMESTAMP;
            break;

        case k_SAMPLE:
            ReadNumber(Str);
            x  = ConvertNumber(Str);
            p2 = CheckUnits(Str);
            if (p2 > 0)
            {
                x = x * ConversionFactor[p2];
            }
            GenCode2(i_SAMPLE, x);
            return k_SAMPLE;
            break;

        case k_ENGAGE:
            ReadName(Str);
            p1 = FindAP(Str);
            if (p1 > 0)
            {
                GenCode2(i_ENGAGE, p1);
                return k_ENGAGE;
            }
            else
            {
                Complain("Unrecognised AP mode", Str);
            }
            break;

        case k_DISENGAGE:
            ReadName(Str);
            p1 = FindAP(Str);
            if (p1 > 0)
            {
                GenCode2(i_DISENGAGE, p1);
                return k_DISENGAGE;
            }
            else
            {
                Complain("Unrecognised AP mode", Str);
            }
            break;

        case k_LOG:
            Script_ReadString(Str, EOL);
            p1 = NextMessage(Str);
            GenCode2(i_LOG, p1);
            return k_LOG;
            break;

        case k_LOADFLTPLN:
            Script_ReadString(Str, EOL);
            p1 = NextMessage(Str);
            GenCode2(i_LOADFLTPLN, p1);
            return k_LOADFLTPLN;
            break;

        case k_REPOSITION:
            Script_ReadString(Str, EOL);
            x1 = ReadLatitude(Str);
            x2 = ReadLongitude(Str);
            GenCode3(i_PUSHC, 0, x1);
            GenCode3(i_PUSHC, 0, x2);
            GenCode1(i_REPOSITION);
            return k_REPOSITION;
            break;

        case k_ACTIVATE:
            GenCode1(i_ACTIVATE);
            return k_ACTIVATE;
            break;

        case k_DEACTIVATE:
            GenCode1(i_DEACTIVATE);
            return k_DEACTIVATE;
            break;

        default:
            break;
    }
    return k_ERROR;
}

/* ----------------------------------------------------------------------------- */
unsigned int ReadStatement()
{
    unsigned int Op;
    unsigned int p = 0;
    char         Str[80];

    Op = ReadSymbol(Str);
    switch (Op)
    {
        case k_EVENT:
            Phase = Phase_Event;
            Script.EventStart = PC;
            Script.Status = Inactive;
            
            Op = NextSymbol(Str);
            if (Op > k_END && Op != k_EOF)
            {
                ReadbooleanExpression();
            }
            GenCode1(i_RETURN);

            return k_EVENT;
            break;
            
        case k_ACTION:
            Script.ActionStart = PC;
            Phase = Phase_Action;

            while (1)
            {
                Op = NextSymbol(Str);
                if (Op <= k_END || Op == k_EOF)
                {
                    break;
                }
                Op = ReadSymbol(Str);
                ReadAction(Op);
                
            }
            GenCode1(i_RETURN);

            return k_ACTION;
            break;

        case k_END:
            Phase = Phase_End;
            Script.EndStart = PC;
            
            Op = NextSymbol(Str);
            if (Op > k_END && Op != k_EOF)
            {
                ReadbooleanExpression();
            }
            GenCode1(i_RETURN);

            return k_END;
            break;

        case k_RECORD:
            Phase = Phase_Record;
            Script.RecordStart = PC;

            while (1)
            {
                Op = NextSymbol(Str);
                if (Op <= k_END || Op == k_EOF)
                {
                    break;
                }
                ReadExpression();
                p = CheckUnits(Str);
                if (p > 0)
                {
                    if (ConversionFactor[p] != 1.0)
                    {
                        GenCode3(i_PUSHC, 0, ConversionFactor[p]);
                        GenCode1(i_DIV);
                    }
                }
                GenCode1(i_RECORD);
            }
            GenCode1(i_RETURN);
        
            return k_RECORD;
            break;

        case k_EOF:
            return k_EOF;
            break;

        default:
            Complain("Unrecognised Keyword", Str);
            return k_ERROR;
            break;
    }
    return Op;
}

/* ----------------------------------------------------------------------------- */
unsigned int NextMessage(char Str[])
{
    if (MessageNumber < MaxMessages)
    {
        MessageNumber = MessageNumber + 1;
        strcpy(Messages[MessageNumber], Str);
        return MessageNumber;
    }
    else
    {
        return 0;   /* message 0 = "" */
    }
}

/* ----------------------------------------------------------------------------- */
unsigned int SetCommand(unsigned int p)
{
    switch (p)
    {
    case 1:
        return IosDefn_SetTurbulence;
        break;
    case 2:
        return IosDefn_SetWindSpeed;
        break;
    case 3:
        return IosDefn_SetWindDir;
        break;
    case 4:
        return IosDefn_SetQNH;
        break;
    case 5:
        return IosDefn_SetMagneticVariation;
        break;
    case 6:
        return IosDefn_SetCloudbase;
        break;
    case 7:
        return IosDefn_SetVisibility;
        break;
    case 8:
        return IosDefn_SetGroundTemperature;
        break;
    case 9:
        return IosDefn_SetAircraftAltitude;
        break;
    case 10:
        return IosDefn_SetAircraftHeading;
        break;
    case 11:
        return IosDefn_SetAircraftSpeed;
        break;
    case 12:
        return IosDefn_SetCgPosition;
        break;
    case 13:
        return IosDefn_SetRightFuelQuantity;
        break;
    case 14:
        return IosDefn_SetLeftFuelQuantity;
        break;
    case 15:
        return IosDefn_SetTargetDistance;
        break;
    case 16:
        return IosDefn_SetTargetSpeed;
        break;
    case 17:
        return IosDefn_SetTargetHeading;
        break;
    case 18:
        return IosDefn_SetTargetTurnRate;
        break;
    case 19:
        return IosDefn_SetTargetAltitude;
        break;
    case 20:
        return IosDefn_SetTargetClimbRate;
        break;
    case 21:
        return IosDefn_SetAutopilotAltitude;
        break;
    case 22:
        return IosDefn_SetAutopilotHeading;
        break;
    case 23:
        return IosDefn_SetAutopilotSpeed;
        break;
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
void GenCode1(unsigned int OpCode)
{
    GenCode3(OpCode, 0, 0.0);
}

/* ----------------------------------------------------------------------------- */
void GenCode2(unsigned int OpCode, unsigned int Arg)
{
    GenCode3(OpCode, Arg, 0.0);
}

/* ----------------------------------------------------------------------------- */
void GenCode3(unsigned int OpCode, unsigned int Arg, float Value)
{
    if (PC >= CodeSize)
    {
        Complain("Program too large", "");
    }
    Instructions[PC].Op  = OpCode;
    Instructions[PC].Arg = Arg;
    Instructions[PC].Val = Value;
    PC                   = PC + 1;
}

/* ----------------------------------------------------------------------------- */
bool Script_ReadScriptFile(char FileName[])
{
    File_EOF           = false;
    LineNumber         = 1;
    PC                 = 1;
    Script_Enabled     = false;
    Script_Error       = false;

    Script.EventStart  = 0;
    Script.ActionStart = 0;
    Script.RecordStart = 0;
    Script.EndStart    = 0;
    Script.Status      = Inactive;

    FileStream = fopen(FileName, "rb");
    if (FileStream == NULL)
    {
        return false;
    }
    else
    {
        do
        {
            ReadStatement();
        } while (!(File_EOF || Script_Error));

        fclose(FileStream);

        Script.Status = Waiting_To_Start;
        
        if (Script_Error)
        {
            printf("Script_ReadScriptFile: %s\n", FileName);
            Script_DisAssembleScript();
        }
        Script_Enabled = true;
        ProgramSize          = PC;
        StackInit();
        PC = 1;
        Initialise_Script();
        return true;
    }
}

/* ----------------------------------------------------------------------------- */
bool Digit(int Ch)
{
    return (Ch >= '0') && (Ch <= '9');
}

/* ----------------------------------------------------------------------------- */
int CAP(int Ch)
{
    if ((Ch >= 'a') && (Ch <= 'z'))
    {
        return 'A' + Ch - 'a';
    }
    else
    {
        return Ch;
    }
}

/* ----------------------------------------------------------------------------- */
bool Alphabetic(int Ch)
{
    return ((Ch >= 'A') && (Ch <= 'Z')) || (Ch == '_');
}

/* ----------------------------------------------------------------------------- */
int Script_Rdch()
{
    int Ch;

    do
    {
        Ch = fgetc(FileStream);
    } while (Ch == CR);  /* ignore CR */

    if (Ch == EOL)
    {
        LineNumber = LineNumber + 1;
    }

    if (Ch == ';')
    {
        while (Ch != EOL && Ch != EOF)
        {
            Ch = fgetc(FileStream);
        }
        if (Ch == EOL)
        {
            LineNumber = LineNumber + 1;
            return Script_Rdch();
        }
    }
    File_EOF = (Ch == EOF);
    return CAP(Ch);
}

/* ----------------------------------------------------------------------------- */
void StackInit()
{
    Sp = 0;
}

/* ----------------------------------------------------------------------------- */
void StackPush(float Item)
{
    if (Sp >= StackSize)
    {
        Complain("Stack OverFlow", "");
    }
    else
    {
        Sp        = Sp + 1;
        Stack[Sp] = Item;
    }
}

/* ----------------------------------------------------------------------------- */
float StackPop()
{
    if (Sp <= 0)
    {
        Complain("Stack UnderFlow", "");
        return 0.0;
    }
    else
    {
        Sp = Sp - 1;
        return Stack[Sp + 1];
    }
}

/* ----------------------------------------------------------------------------- */
float StackTop()
{
    return Stack[Sp];
}

/* ----------------------------------------------------------------------------- */
void StackPushboolean(bool Item)
{
    if (Item)
    {
        StackPush(1.0);
    }
    else
    {
        StackPush(-1.0);
    }
}

/* ----------------------------------------------------------------------------- */
bool StackPopboolean()
{
    return StackPop() > 0.0;
}

/* ----------------------------------------------------------------------------- */
float Norm360(float x)
{
    int a;

    a = (unsigned int) (x * ONERAD + 0.5);
    while (a > 360)
    {
        a = a - 360;
    }
    while (a < 1)
    {
        a = a + 360;
    }
    return (float) (a) / ONERAD;
}

/* ----------------------------------------------------------------------------- */
void Script_DisAssembleScript()
{
    unsigned int i;
    unsigned int Op;
    unsigned int Arg;
    float        Val;

    for (i = 1; i < ProgramSize; i += 1)
    {
        printf("%3d: ", i);
        Op  = Instructions[i].Op;
        Arg = Instructions[i].Arg;
        Val = Instructions[i].Val;

        switch (Op)
        {
        case i_PUSHV:
            printf("PUSHV %d", Arg);
            break;
        case i_PUSHC:
            printf("PUSHC %f", Val);
            break;
        case i_COMP:
            printf("COMP");
            break;
        case i_OR:
            printf("OR");
            break;
        case i_AND:
            printf("AND");
            break;
        case i_ADD:
            printf("ADD");
            break;
        case i_SUB:
            printf("SUB");
            break;
        case i_MULT:
            printf("MULT");
            break;
        case i_DIV:
            printf("DIV");
            break;
        case i_RETURN:
            printf("RETURN");
            break;
        case i_RECORD:
            printf("RECORD ");
            break;
        case i_SET:
            printf("SET %d %f", Arg, Val);
            break;
        case i_FAIL:
            printf("FAIL %d", Arg);
            break;
        case i_RESET:
            printf("RESET %d", Arg);
            break;
        case i_ENGAGE:
            printf("ENGAGE ");
            switch (Arg)
            {
            case 1:
                printf("ALT_HOLD");
                break;
            case 2:
                printf("HDG_HOLD");
                break;
            case 3:
                printf("SPD_HOLD");
                break;
            }
            break;
        case i_DISENGAGE:
            printf("DISENGAGE ");
            switch (Arg)
            {
            case 1:
                printf("ALT_HOLD");
                break;
            case 2:
                printf("HDG_HOLD");
                break;
            case 3:
                printf("SPD_HOLD");
                break;
            }
            break;
        case i_ACTIVATE:
            printf("ACTIVATE FLTPLN");
            break;
        case i_DEACTIVATE:
            printf("DEACTIVATE FLTPLN");
            break;
        case i_SAMPLE:
            printf("SAMPLE %f", Val);
            break;
        case i_SWAP:
            printf("SWAP");
            break;
        case i_NEG:
            printf("NEG");
            break;
        case i_LOG:
            printf("LOG %s", Messages[Arg]);
            break;
        case i_LOADFLTPLN:
            printf("LOADFLTPLN %s", Messages[Arg]);
            break;
        case i_REPOSITION:
            printf("REPOSITION");
            break;
        case i_TIMESTAMP:
            printf("TIMESTAMP ");
            switch (Arg)
            {
            case 0:
                printf("OFF");
                break;
            case 1:
                printf("PCTIME");
                break;
            case 2:
                printf("SIMTIME");
                break;
            case 3:
                printf("ELAPSEDTIME");
                break;
            }
            break;
        }
        printf("\n");
    }
}

/* ----------------------------------------------------------------------------- */
void Script_ExecuteScript(char Filename[])
{
    ElapsedTimeCount = ElapsedTimeCount + 1;

    switch (Script.Status)
    {
        case Inactive:
            return;

        case Waiting_To_Start:
            PC = Script.EventStart;
            ScriptUpdate(Filename);
            if (StackPopboolean())
            {
                PC = Script.ActionStart;
                ScriptUpdate(Filename);
                Script.Status = Waiting_To_Stop;
            }
            return;

        case Waiting_To_Stop:
            PC = Script.EndStart;
            ScriptUpdate(Filename);
            MonitorScript();
            if (StackPopboolean())
            {
                Script.Status = Inactive;
                Script_Enabled = false;
                Script_SaveScript(Filename);
            }
            else
            {
                PC = Script.RecordStart;
                ScriptUpdate(Filename);
            }
            return;
    }
}

/* ----------------------------------------------------------------------------- */
void ScriptUpdate(char Filename[])
{
    unsigned int Op;
    unsigned int Arg;
    float        Val;
    float        x1, x2;
    bool         BoolArg;
    bool         b1, b2;
    
    while (1)
    {
        if (IOLib_GetHoldButton() || Script_Enabled == false)
        {
            return;
        }
        Op  = Instructions[PC].Op;
        Arg = Instructions[PC].Arg;
        Val = Instructions[PC].Val;

        if (Debugging)
        {
            printf("PC=%d", PC);
            printf(" SP=%d", Sp);
            printf(" Op=%d\n", Op);
        }

        switch (Op)
        {
        case i_PUSHV:
            switch (Arg)
            {
            case 1:
                StackPush(IosLink_AeroPkt.Pitch);
                break;
            case 2:
                StackPush(IosLink_AeroPkt.Roll);
                break;
            case 3:
                StackPush(IosLink_AeroPkt.Yaw);
                break;
            case 4:
                StackPush(Norm360(IosLink_AeroPkt.Yaw));
                break;
            case 5:
                StackPush(IosLink_AeroPkt.Q);
                break;
            case 6:
                StackPush(IosLink_AeroPkt.P);
                break;
            case 7:
                StackPush(IosLink_AeroPkt.R);
                break;
            case 8:
                StackPush(IosLink_AeroPkt.QDot);
                break;
            case 9:
                StackPush(IosLink_AeroPkt.PDot);
                break;
            case 10:
                StackPush(IosLink_AeroPkt.RDot);
                break;
            case 11:
                StackPush(-IosLink_AeroPkt.Pz);
                break;
            case 12:
                StackPush(IosLink_AeroPkt.Vc);
                break;
            case 13:
                StackPush(IosLink_AeroPkt.MachNumber);
                break;
            case 14:
                StackPush(-IosLink_AeroPkt.Vd);
                break;
            case 15:
                StackPush(IosLink_AeroPkt.Latitude);
                break;
            case 16:
                StackPush(IosLink_AeroPkt.Longitude);
                break;
            case 17:
                StackPush(IosLink_AeroPkt.Alpha);
                break;
            case 18:
                StackPush(IosLink_AeroPkt.Beta);
                break;
            case 19:
                StackPush(IosLink_AeroPkt.AlphaDot);
                break;
            case 20:
                StackPush(IosLink_AeroPkt.BetaDot);
                break;
            case 21:
                StackPush(IosLink_AeroPkt.Lift);
                break;
            case 22:
                StackPush(IosLink_AeroPkt.Thrust);
                break;
            case 23:
                StackPush(IosLink_AeroPkt.Drag);
                break;
            case 24:
                StackPush(IosLink_AeroPkt.SideForce);
                break;
            case 25:
                StackPush(IOLib_GetElevator());
                break;
            case 26:
                StackPush(IOLib_GetAileron());
                break;
            case 27:
                StackPush(IOLib_GetRudder());
                break;
            case 28:
                StackPush(IosLink_AeroPkt.ElevatorTrim);
                break;
            case 29:
                StackPush(0.0);
                break;
            case 30:
                StackPush(0.0);
                break;
            case 31:
                StackPush(IOLib_GetLeftBrake());
                break;
            case 32:
                StackPush(IOLib_GetRightBrake());
                break;
            case 33:
                StackPush(IosLink_AeroPkt.FlapPosition);
                break;
            case 34:
                StackPush(IosLink_AeroPkt.GearPosition);
                break;
            case 35:
                StackPush(IosLink_AeroPkt.Mass);
                break;
            case 36:
                StackPush(IosLink_EngPkt.EngineLevers[0]);
                break;
            case 37:
                StackPush(IosLink_EngPkt.EngineLevers[1]);
                break;
            case 38:
                StackPush(IosLink_EngPkt.EngineLevers[2]);
                break;
            case 39:
                StackPush(IosLink_EngPkt.EngineLevers[3]);
                break;
//            case 40:  /* changed for 4 lever aircraft */
//                StackPush(IosLink_EngPkt.EngineLevers[4]);
//                break;
//            case 41:
//                StackPush(IosLink_EngPkt.EngineLevers[5]);
//                break;
            case 42:
                StackPush(IosLink_EngPkt.Engines[0].Thrust);
                break;
            case 43:
                StackPush(IosLink_EngPkt.Engines[0].Rpm);
                break;
            case 44:
                StackPush(IosLink_EngPkt.Engines[0].FuelFlow);
                break;
            case 45:
                StackPush(IosLink_EngPkt.Engines[0].Egt);
                break;
            case 46:
                StackPush(IosLink_EngPkt.Engines[0].Beta);
                break;
            case 47:
                StackPush(IosLink_EngPkt.Engines[0].ManifoldPressure);
                break;
            case 48:
                StackPush(IosLink_EngPkt.Engines[0].Epr);
                break;
            case 49:
                StackPush(IosLink_EngPkt.Engines[1].Thrust);
                break;
            case 50:
                StackPush(IosLink_EngPkt.Engines[1].Rpm);
                break;
            case 51:
                StackPush(IosLink_EngPkt.Engines[1].FuelFlow);
                break;
            case 52:
                StackPush(IosLink_EngPkt.Engines[1].Egt);
                break;
            case 53:
                StackPush(IosLink_EngPkt.Engines[1].Beta);
                break;
            case 54:
                StackPush(IosLink_EngPkt.Engines[1].ManifoldPressure);
                break;
            case 55:
                StackPush(IosLink_EngPkt.Engines[1].Epr);
                break;
            case 56:
                StackPush(IosLink_EngPkt.Engines[2].Thrust);
                break;
            case 57:
                StackPush(IosLink_EngPkt.Engines[2].Rpm);
                break;
            case 58:
                StackPush(IosLink_EngPkt.Engines[2].FuelFlow);
                break;
            case 59:
                StackPush(IosLink_EngPkt.Engines[2].Egt);
                break;
            case 60:
                StackPush(IosLink_EngPkt.Engines[2].Beta);
                break;
            case 61:
                StackPush(IosLink_EngPkt.Engines[2].ManifoldPressure);
                break;
            case 62:
                StackPush(IosLink_EngPkt.Engines[2].Epr);
                break;
            case 63:
                StackPush(IosLink_EngPkt.Engines[3].Thrust);
                break;
            case 64:
                StackPush(IosLink_EngPkt.Engines[3].Rpm);
                break;
            case 65:
                StackPush(IosLink_EngPkt.Engines[3].FuelFlow);
                break;
            case 66:
                StackPush(IosLink_EngPkt.Engines[3].Egt);
                break;
            case 67:
                StackPush(IosLink_EngPkt.Engines[3].Beta);
                break;
            case 68:
                StackPush(IosLink_EngPkt.Engines[3].ManifoldPressure);
                break;
            case 69:
                StackPush(IosLink_EngPkt.Engines[3].Epr);
                break;
            case 70:
                StackPush(IosLink_EngPkt.FuelQuantityLeft);
                break;
            case 71:
                StackPush(IosLink_EngPkt.FuelQuantityRight);
                break;
            case 72:
                StackPush(IosLink_AeroPkt.Rho);
                break;
            case 73:
                StackPush(IosLink_AeroPkt.OAT);
                break;
            case 74:
                StackPush((float) (IosLink_NavPkt.BaroPressure1));
                break;
            case 75:
                StackPush((float) (IosLink_NavPkt.BaroPressure2));
                break;
            case 76:
                StackPush(IosLink_NavPkt.MagneticVariation);
                break;
            case 77:
                StackPush(IosLink_NavPkt.NAV1.GlideSlopeError);
                break;
            case 78:
                StackPush(IosLink_NavPkt.NAV1.LocaliserError);
                break;
            case 79:
                StackPush(IosLink_NavPkt.NAV1.SlantDistance);
                break;
            case 80:
                StackPush(IosLink_NavPkt.NAV1.BearingToStation);
                break;
            case 81:
                StackPush(IosLink_NavPkt.NAV2.GlideSlopeError);
                break;
            case 82:
                StackPush(IosLink_NavPkt.NAV2.LocaliserError);
                break;
            case 83:
                StackPush(IosLink_NavPkt.NAV2.SlantDistance);
                break;
            case 84:
                StackPush(IosLink_NavPkt.NAV2.BearingToStation);
                break;
            case 85:
                StackPush(IosLink_NavPkt.ADF1.SlantDistance);
                break;
            case 86:
                StackPush(IosLink_NavPkt.ADF1.BearingToStation);
                break;
            case 87:
                StackPush(IosLink_NavPkt.ADF2.SlantDistance);
                break;
            case 88:
                StackPush(IosLink_NavPkt.ADF2.BearingToStation);
                break;
            case 89:
                StackPush(Maths_Rads((float) IosLink_NavPkt.HSI_Crs));
                break;
            case 90:
                StackPush(Maths_Rads((float) IosLink_NavPkt.HSI_Hdg));
                break;
            case 91:
                StackPush(Maths_Rads((float) IosLink_NavPkt.VOR_Obs));
                break;
            case 92:
                StackPush((float) (ElapsedTimeCount) / 50.0);
                break;
            default:
                Complain("Execute PUSHV", "Invalid Arg");
                break;
            }
            break;

        case i_PUSHC:
            StackPush(Val);
            break;

        case i_COMP:
            x1 = StackPop();
            x2 = StackPop();
            StackPushboolean(x1 < x2);
            break;

        case i_OR:
            b1 = StackPopboolean();
            b2 = StackPopboolean();
            StackPushboolean(b1 || b2);
            break;

        case i_AND:
            b1 = StackPopboolean();
            b2 = StackPopboolean();
            StackPushboolean(b1 && b2);
            break;

        case i_ADD:
            StackPush(StackPop() + StackPop());
            break;

        case i_SUB:
            x1 = StackPop();
            x2 = StackPop();
            StackPush(x2 - x1);
            break;

        case i_MULT:
            StackPush(StackPop() * StackPop());
            break;

        case i_DIV:
            x1 = StackPop();
            x2 = StackPop();
            StackPush(x2 / x1);
            break;

        case i_SWAP:
            x1 = StackPop();
            x2 = StackPop();
            StackPush(x1);
            StackPush(x2);
            break;

        case i_NEG:
            StackPush(-StackPop());
            break;

        case i_RECORD:
            Record(StackPop());
            break;

        case i_RETURN:
            return;
            break;
            
        case i_SET:
            if (Arg == IosDefn_SetAircraftAltitude || Arg == IosDefn_SetCloudbase ||
                Arg == IosDefn_SetTargetAltitude || Arg == IosDefn_SetTargetClimbRate)
            {
                Val = -Val;
            }
            IosLink_SendRealSIUnits(Arg, Val);
            break;

        case i_FAIL:
        case i_RESET:
            BoolArg = Op == i_FAIL;
            switch (Arg)
            {
            case 1:
                IosLink_SendBoolean(IosDefn_FailFlaps, BoolArg);
                break;
            case 2:
                IosLink_SendBoolean(IosDefn_FailGear, BoolArg);
                break;
            case 3:
                IosLink_SendBoolean(IosDefn_FailNav1Localiser, BoolArg);
                break;
            case 4:
                IosLink_SendBoolean(IosDefn_FailNav1GlideSlope, BoolArg);
                break;
            case 5:
                IosLink_SendBoolean(IosDefn_FailNav2Localiser, BoolArg);
                break;
            case 6:
                IosLink_SendBoolean(IosDefn_FailNav2GlideSlope, BoolArg);
                break;
            case 7:
                IosLink_SendBoolean(IosDefn_FailRMI1, BoolArg);
                break;
            case 8:
                IosLink_SendBoolean(IosDefn_FailRMI2, BoolArg);
                break;
            case 9:
                IosLink_SendBoolean(IosDefn_FailDME, BoolArg);
                break;
            case 10:
                IosLink_SendBoolean(IosDefn_FailEngine1, BoolArg);
                break;
            case 11:
                IosLink_SendBoolean(IosDefn_FailEngine2, BoolArg);
                break;
            case 12:
                IosLink_SendBoolean(IosDefn_FailEngine3, BoolArg);
                break;
            case 13:
                IosLink_SendBoolean(IosDefn_FailEngine4, BoolArg);
                break;
            case 14:
                IosLink_SendBoolean(IosDefn_FailASI, BoolArg);
                break;
            case 15:
                IosLink_SendBoolean(IosDefn_FailAI, BoolArg);
                break;
            case 16:
                IosLink_SendBoolean(IosDefn_FailVSI, BoolArg);
                break;
            case 17:
                IosLink_SendBoolean(IosDefn_FailAltimeter, BoolArg);
                break;
            case 18:
                IosLink_SendBoolean(IosDefn_FailTurn, BoolArg);
                break;
            case 19:
                IosLink_SendBoolean(IosDefn_FailCompass, BoolArg);
                break;
            case 20:
                IosLink_SendBoolean(IosDefn_FailFD, BoolArg);
                break;
            }
            break;

        case i_ENGAGE:
        case i_DISENGAGE:
            BoolArg = Op == i_ENGAGE;
            switch (Arg)
            {
            case 1:
                IosLink_SendBoolean(IosDefn_AutopilotAltitudeOn, BoolArg);
                break;
            case 2:
                IosLink_SendBoolean(IosDefn_AutopilotHeadingOn, BoolArg);
                break;
            case 3:
                IosLink_SendBoolean(IosDefn_AutopilotSpeedOn, BoolArg);
                break;
            }
            break;

        case i_ACTIVATE:
            if (NavLib_NumberOfWayPoints > 0)
            {
                NavLib_NextWayPoint = 1;
                IosLink_SendInt(IosDefn_SetFlightPlanMode, NavLib_NumberOfWayPoints);
            }
            break;

        case i_DEACTIVATE:
            IosLink_SendInt(IosDefn_SetFlightPlanMode, 0);
            NavLib_NextWayPoint = 0;
            break;

        case i_SAMPLE:
            SampleRate = Val;
            if (SampleRate < 0.02)
            {
                SampleRate = 0.02;
            }
            break;

        case i_LOG:
            OutputWriteString(Messages[Arg]);
            OutputWrch(EOL);
            break;

        case i_LOADFLTPLN:
            NavLib_ReadFlightPlan(Messages[Arg]);
            NavLib_NextWayPoint = 0;      // Disable tx until flightplan activated.
            break;

        case i_REPOSITION:
            x2 = Maths_Rads(StackPop());
            x1 = Maths_Rads(StackPop());
            IosLink_SendPosition(IosDefn_RePositionAircraft, x1, x2);
            break;

        case i_TIMESTAMP:
            switch (Arg)
            {
            case 0:
                TimeStamping = Off;
                break;
            case 1:
                TimeStamping = PcTime;
                break;
            case 2:
                TimeStamping = SimTime;
                break;
            case 3:
                TimeStamping = ElapsedTime;
                break;
            }
            break;
        }
        
        PC += 1;
    }
}

/* ----------------------------------------------------------------------------- */
void Record(float x)
{
    if (Recording)
    {
        RecordingPtr                = RecordingPtr + 1;
        RecordingList[RecordingPtr] = x;
    }
}

/* ----------------------------------------------------------------------------- */
void WriteTimeStamp()
{
    unsigned int h, m, s, t;
    unsigned int ts = 0;
    char         str[20];
    
    if (TimeStamping == Off)
    {
        return;
    }
    else
    {
        switch (TimeStamping)
        {
            case PcTime:
                ts = PcTime0 + ElapsedTimeCount;
                break;
            case SimTime:
                ts = IosLink_AeroPkt.TimeStamp;
                break;
            case ElapsedTime:
                ts = ElapsedTimeCount;
                break;
        }
    }
    t  = ts % 50 * 2;
    ts = ts / 50;
    s  = ts % 60;
    ts = ts / 60;
    m  = ts % 60;
    h  = ts / 60;
    sprintf(str, "%02d:%02d:%02d.%02d, ", h, m, s, t);
    OutputWriteString(str);
}

/* ----------------------------------------------------------------------------- */
void MonitorScript()
{
    unsigned int i;

    CurrentTick = CurrentTick + 1;
    if (CurrentTick >= NextTick)
    {
        NextTick = NextTick + (unsigned int) (SampleRate * 50.0 + 0.1);
        if (RecordingPtr > 0)
        {
            WriteTimeStamp();
            for (i = 1; i <= RecordingPtr; i += 1)
            {
                OutputWriteReal(RecordingList[i]);
                if (i == RecordingPtr)
                {
                    OutputWrch(EOL);
                }
                else
                {
                    OutputWriteString(", ");
                }
            }
        }
        Recording    = true;
        RecordingPtr = 0;
    }
    else
    {
        Recording = false;
        return;
    }
}

/* ----------------------------------------------------------------------------- */
int GetPcTime()   /* ticks since midnight */
{
    time_t    now     = time(NULL);
    struct tm *now_tm = localtime(&now);

    return ((now_tm->tm_hour * 60 + now_tm->tm_min) * 60 + now_tm->tm_sec) * 50;
}

/* ----------------------------------------------------------------------------- */
void Initialise_Script()
{
    SampleRate       = 0.02; /* 50 Hz */
    CurrentTick      = 0;
    NextTick         = 0;
    Recording        = false;
    RecordingPtr     = 0;
    MessageNumber    = 0;
    OutFileBufPtr    = 0;
    TimeStamping     = Off;
    PcTime0          = GetPcTime();
    ElapsedTimeCount = 0;
}

/* ----------------------------------------------------------------------------- */
void BEGIN_Script()
{
}
