/* scriptx.c
   modified 2/2/12 to remove fgetpos and fsetpos
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
#include "iolibx.h"

const char CR    = 13;
const char EOL   = 10;
const char TAB   = 9;
const char SPACE = ' ';

bool       Script_ScriptEnabled;
bool       Script_ScriptError;
char       Script_ScriptErrorMessage[200];

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

#define MaxKeyWords       28
#define MaxVarWords       92
#define MaxSetWords       23
#define MaxFailWords      22
#define MaxUnits          32
#define MaxConversions    32
#define MaxAPWords        3

char *KeyWords[MaxKeyWords + 1] =
{ "",               "IF",       "THEN", "ELSE",       "END",    "WHILE",     "DO",        "UNLESS", "REPEAT", "UNTIL",
  "WAIT",       "OR",       "AND",  "RECORD",     "FOR",    "SET",       "FAIL",      "RESET",  "STOP",
  "SAMPLE",     "DELAY",    "LOG",  "REPOSITION", "ENGAGE", "DISENGAGE", "TIMESTAMP",
  "LOADFLTPLN", "ACTIVATE", "DEACTIVATE" };

char *VarWords[MaxVarWords + 1] =
{ "",                   "PITCH",         "ROLL",          "YAW",           "HEADING",
  "PITCH_RATE",     "ROLL_RATE",     "YAW_RATE",
  "PITCH_ACCN",     "ROLL_ACCN",     "YAW_ACCN",
  "ALTITUDE",       "AIRSPEED",      "MACH_NUMBER",   "RATE_OF_CLIMB",
  "LATITUDE",       "LONGITUDE",     "ALPHA",         "BETA",          "ALPHA_RATE","BETA_RATE",
  "LIFT",           "THRUST",        "DRAG",          "SIDEFORCE",
  "ELEVATOR",       "AILERON",       "RUDDER",
  "ELEVATOR_TRIM",  "AILERON_TRIM",  "RUDDER_TRIM",
  "LEFT_BRAKE",     "RIGHT_BRAKE",   "FLAPS",         "GEAR",          "MASS",
  "ENGINE_LEVER1",  "ENGINE_LEVER2", "ENGINE_LEVER3",
  "ENGINE_LEVER4",  "ENGINE_LEVER5", "ENGINE_LEVER6",
  "ENGINE1_THRUST", "ENGINE1_RPM",   "ENGINE1_FF",    "ENGINE1_EGT",
  "ENGINE1_BETA",   "ENGINE1_MP",    "ENGINE1_EPR",
  "ENGINE2_THRUST", "ENGINE2_RPM",   "ENGINE2_FF",    "ENGINE2_EGT",
  "ENGINE2_BETA",   "ENGINE2_MP",    "ENGINE2_EPR",
  "ENGINE3_THRUST", "ENGINE3_RPM",   "ENGINE3_FF",    "ENGINE3_EGT",
  "ENGINE3_BETA",   "ENGINE3_MP",    "ENGINE3_EPR",
  "ENGINE4_THRUST", "ENGINE4_RPM",   "ENGINE4_FF",    "ENGINE4_EGT",
  "ENGINE4_BETA",   "ENGINE4_MP",    "ENGINE4_EPR",
  "FUEL_LEFT",      "FUEL_RIGHT",    "RHO",           "OAT",
  "BARO1",          "BARO2",         "MAGNETIC_VAR",
  "NAV1_GS",        "NAV1_LOC",      "NAV1_DME",      "NAV1_BEARING",
  "NAV2_GS",        "NAV2_LOC",      "NAV2_DME",      "NAV2_BEARING",
  "ADF1_DME",       "ADF1_BEARING",  "ADF2_DME",      "ADF2_BEARING",
  "HSI_CRS",        "HSI_HDG",       "VOR_OBS",       "TIME" };

char *SetWords[MaxSetWords + 1] =
{ "",                     "TURBULENCE",      "WIND_SPEED",  "WIND_DIRECTION", "QNH",        "MAG_VAR",     "CLOUD_BASE",
  "VISIBILITY",       "OAT",             "ALTITUDE",    "HEADING",        "SPEED",      "CG_POSITION",
  "RIGHT_FUEL",       "LEFT_FUEL",       "TARGET_DIST", "TARGET_SPEED",   "TARGET_HDG",
  "TARGET_TURN_RATE", "TARGET_ALTITUDE", "TARGET_ROC",  "AP_ALT",         "AP_HDG",     "AP_SPD" };

char *FailWords[MaxFailWords + 1] =
{ "",         "FLAPS", "GEAR", "NAV1_LOC",  "NAV1_GS", "NAV2_LOC", "NAV2_GS",
  "RMI1", "RMI2",  "DME",  "ENGINE1",   "ENGINE2", "ENGINE3",  "ENGINE4",
  "ASI",  "AI",    "VSI",  "ALTIMETER", "TURN",    "HSICARD",  "RMICARD" };

char *Units[MaxUnits + 1] =
{ "",        "DEGS",  "RADS",  "DEG/S",    "RAD/S", "DEG/S/S", "RAD/S/S",
  "FT",  "M",     "KM",    "NM",       "KTS",   "MPH",     "M/S",      "FPM",
  "N",   "LBF",   "KG",    "LBS",      "KG/HR", "LBS/HR",  "LITRES/HR",
  "%",   "MB",    "INHG",  "KG/M/M/M", "SECS",  "MINS",    "HOURS",
  "RPM", "KM/HR", "M/S/S", "FT/S/S" };

float ConversionFactor[MaxConversions + 1] =
{ 1.0,        0.017453,     1.0, 0.017453,    1.0, 0.017453,    1.0,
  0.3048,      1.0,  1000.0,   1852.0, 0.5144,   0.4470,    1.0,0.00508,
  1.0,      4.4482,     1.0, 0.453592,    1.0,   2.2046, 1.3788,
  0.01,        1.0, 33.8639,      1.0,    1.0,     60.0, 3600.0,
  0.1047, 0.277778,     1.0, 0.3048 };

char *APWords[MaxAPWords + 1] =
{ "", "ALT_HOLD", "HDG_HOLD", "SPD_HOLD" };

#define k_ERROR           0
#define k_IF              1
#define k_THEN            2
#define k_ELSE            3
#define k_END             4
#define k_WHILE           5
#define k_DO              6
#define k_UNLESS          7
#define k_REPEAT          8
#define k_UNTIL           9
#define k_WAIT            10
#define k_OR              11
#define k_AND             12
#define k_RECORD          13
#define k_FOR             14
#define k_SET             15
#define k_FAIL            16
#define k_RESET           17
#define k_STOP            18
#define k_SAMPLE          19
#define k_DELAY           20
#define k_LOG             21
#define k_REPOSITION      22
#define k_ENGAGE          23
#define k_DISENGAGE       24
#define k_TIMESTAMP       25
#define k_LOADFLTPLN      26
#define k_ACTIVATE        27
#define k_DEACTIVATE      28
#define k_LBRACKET        29
#define k_RBRACKET        30
#define k_LESS            31
#define k_GREATER         32
#define k_PLUS            33
#define k_MINUS           34
#define k_MULT            35
#define k_DIV             36
#define k_NUMBER          37
#define k_VAR             38
#define k_EOF             9999

#define i_PUSHV           1
#define i_PUSHC           2
#define i_COMP            3
#define i_JT              4
#define i_JF              5
#define i_JUMP            6
#define i_LOOP            7
#define i_OR              8
#define i_AND             9
#define i_ADD             10
#define i_SUB             11
#define i_MULT            12
#define i_DIV             13
#define i_RECORD          14
#define i_SET             15
#define i_FAIL            16
#define i_RESET           17
#define i_STOP            18
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

#define StackSize         200
#define CodeSize          2000
#define MaxMessages       100
//#define OutFileBufSize   100000 // Mike Bromfield needed more room!
#define OutFileBufSize    4000000

unsigned int  Sp;
float         Stack[StackSize];
FILE          *FileStream;
unsigned int  LineNumber;
unsigned int  PC;
Instruction   Instructions[CodeSize];
float         SampleRate;
unsigned int  ProgramSize;
int           CurrentTick;
int           NextTick;
bool          Recording;
float         RecordingList[MaxVarWords + 1];
unsigned int  RecordingPtr;
unsigned int  MessageNumber;
char          Messages[MaxMessages + 1][80];
TimeStampMode TimeStamping;
int           PcTime0;
int           ElapsedTimeCount;
char          OutFileBuf[OutFileBufSize];
unsigned int  OutFileBufPtr;
bool          File_EOF;
bool          SymbolBackSpacing;
unsigned int  LastSymbol;
char          LastSymbolString[MaxStringSize + 1];

void          OutputWrch(char Ch);
void          OutputWriteString(char a[]);
void          OutputWriteReal(float x);
void          Complain(char ErrorMessage[], char Details[]);
unsigned int  ReadSymbol(char a[]);
unsigned int  CheckUnits(char a[]);
void          Script_ReadString(char a[], char Ch);
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
bool       Digit(char Ch);
bool       Alphabetic(char Ch);
char          Script_Rdch();
void          StackInit();
void          StackPush(float Item);
float         StackPop();
float         StackTop();
void          StackPushboolean(bool Item);
bool       StackPopboolean();
float         Norm360(float x);
void          Record(float x);
void          Wr2(unsigned int x);
void          WriteTimeStamp();
void          MonitorScript();
int           GetPcTime();
void          Initialise_Script();
float         ReadLatitude(char str[]);
float         ReadLongitude(char str[]);
void          CopyString(char a[], char b[]);
unsigned int  NextSymbol(char Str[]);
void          FileBackSpace(char Ch);

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

    if (OutFileBufPtr == 0)
    {
        return;
    }

    f = fopen(FileName, "w");
    if (f == NULL)
    {
        return;
    }

    for (i = 0; i < OutFileBufPtr; i = i + 1)
    {
        fputc(OutFileBuf[i], f);
    }

    fclose(f);
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
        i = i + 1;
    }
}

/* ----------------------------------------------------------------------------- */
void OutputWriteReal(float x)
{
    char str[20];

    sprintf(str, "%f", x);
    OutputWriteString(str);
}

/* ----------------------------------------------------------------------------- */
void Complain(char ErrorMessage[], char Details[])
{
    if (Script_ScriptError)
    {
        return;
    }

    sprintf(Script_ScriptErrorMessage, "Line %d <%s> %s\n", LineNumber, ErrorMessage, Details);
    Script_ScriptEnabled = false;
    Script_ScriptError   = true;
}

/* ----------------------------------------------------------------------------- */
unsigned int ReadSymbol(char Str[])
{
    char         Ch;
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
        for (i = 1; i <= MaxKeyWords; i += 1)
        {
            if (strcmp(Str, KeyWords[i]) == 0)
            {
                LastSymbol = i;
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
    unsigned int k;

    k          = ReadSymbol(Str);
    LastSymbol = k;
    CopyString(Str, LastSymbolString);
    SymbolBackSpacing = true;
    return k;
}

/* ----------------------------------------------------------------------------- */
void FileBackSpace(char Ch)
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
void Script_ReadString(char Str[], char tCh)   /* read a string until tCh encountered, stripping leading spaces */
{
    unsigned int p;
    char         Ch;

    p = 0;

    do        /* skip leading spaces and tabs */
    {
        Ch = Script_Rdch();
    } while (!(((Ch != SPACE) && (Ch != TAB)) || (Ch == EOF)));

    do
    {
        Str[p] = Ch;
        p     += 1;
        Ch     = Script_Rdch();
    } while (!(Ch == tCh || Ch == EOL || Ch == EOF));

    Str[p] = 0;
}

/* ----------------------------------------------------------------------------- */
void ReadName(char Str[])
{
    unsigned int p;
    char         Ch;

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
             ((Ch == '/') && (p > 0)) ||
             ((Ch >= '0') && (Ch <= '9') && (p > 0)));

    Str[p] = 0;
    FileBackSpace(Ch);
}

/* ----------------------------------------------------------------------------- */
void ReadNumber(char Str[])
{
    unsigned int p;
    char         Ch;

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
unsigned int FindVariable(char v[])
{
    unsigned int i;

    for (i = 1; i <= MaxVarWords; i += 1)
    {
        if (strcmp(v, VarWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindSetVariable(char v[])
{
    unsigned int i;

    for (i = 1; i <= MaxSetWords; i += 1)
    {
        if (strcmp(v, SetWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------------- */
unsigned int FindFailure(char v[])
{
    unsigned int i;

    for (i = 1; i <= MaxFailWords; i += 1)
    {
        if (strcmp(v, FailWords[i]) == 0)
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

    for (i = 1; i <= MaxUnits; i += 1)
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

    for (i = 1; i <= MaxAPWords; i += 1)
    {
        if (strcmp(v, APWords[i]) == 0)
        {
            return i;
        }
    }
    return 0;
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
unsigned int ReadStatement()
{
    unsigned int Op;
    unsigned int p1 = 0;
    unsigned int p2 = 0;
    char         Str[80];
    float        x;
    float        x1, x2;
    unsigned int L1 = 0;
    unsigned int L2 = 0;
    unsigned int L3 = 0;

    Op = ReadSymbol(Str);
    switch (Op)
    {
    case k_RECORD:
        ReadExpression();
        p2 = CheckUnits(Str);
        if (p2 > 0)
        {
            if (ConversionFactor[p2] != 1.0)
            {
                GenCode3(i_PUSHC, 0, ConversionFactor[p2]);
                GenCode1(i_DIV);
            }
        }
        GenCode1(i_RECORD);
        return k_RECORD;
        break;

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
        GenCode3(i_SAMPLE, p1, x);
        return k_SAMPLE;
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

    case k_IF:
        ReadbooleanExpression();
        Op = ReadSymbol(Str);
        if (Op != k_THEN)
        {
            Complain("THEN expected in IF Statement", Null);
        }
        else
        {
            L1 = PC;
            GenCode1(i_JF);
            do
            {
                Op = ReadStatement();
            } while (!(Op == k_END || Op == k_ELSE || Op == k_ERROR || Op == k_EOF));

            if (Op == k_ERROR)
            {
                Complain("ELSE expected in IF statement", Null);
            }

            if (Op == k_END)
            {
                Instructions[L1].Arg = PC;
            }
            else if (Op == k_ELSE)
            {
                L2 = PC;
                GenCode1(i_JUMP);
                L3 = PC;
                do
                {
                    Op = ReadStatement();
                } while (!(Op == k_END || Op == k_ERROR || Op == k_EOF));

                if (Op == k_ERROR)
                {
                    Complain("END expected in IF statement", Null);
                }
                Instructions[L1].Arg = L3;
                Instructions[L2].Arg = PC;
            }
            else
            {
                Complain("END expected in IF Statement", Null);
            }
        }
        break;

    case k_UNLESS:
        ReadbooleanExpression();
        if (ReadStatement() != k_DO)
        {
            Complain("DO Expected in UNLESS Statement", Null);
        }
        else
        {
            L1 = PC;
            GenCode1(i_JT);
            do
            {
                Op = ReadStatement();
            } while (!(Op == k_END || Op == k_ERROR || Op == k_EOF));
            if (Op != k_END)
            {
                Complain("END Expected in UNLESS Statement", Null);
            }
        }
        Instructions[L1].Arg = PC;
        break;

    case k_WHILE:
        L1 = PC;
        ReadbooleanExpression();
        if (ReadStatement() != k_DO)
        {
            Complain("DO Expected in WHILE Statement", Null);
        }
        else
        {
            L2 = PC;
            GenCode1(i_JF);
            do
            {
                Op = ReadStatement();
            } while (!(Op == k_END || Op == k_ERROR || Op == k_EOF));

            if (Op != k_END)
            {
                Complain("END Expected in WHILE Statement", Null);
            }
        }
        GenCode2(i_JUMP, L1);
        Instructions[L2].Arg = PC;
        break;

    case k_REPEAT:
        L1 = PC;
        do
        {
            Op = ReadStatement();
        } while (!(Op == k_UNTIL || Op == k_ERROR || Op == k_EOF));
        if (Op != k_UNTIL)
        {
            Complain("UNTIL Expected in REPEAT Statement", Null);
        }
        else
        {
            ReadbooleanExpression();
            GenCode2(i_JF, L1);
        }
        break;

    case k_FOR:
        ReadNumber(Str);
        x  = ConvertNumber(Str);
        p2 = CheckUnits(Str);
        if (p2 > 0)
        {
            x = x * ConversionFactor[p2];
        }
        GenCode3(i_PUSHC, 0, x);
        if (ReadStatement() != k_DO)
        {
            Complain("DO Expected in FOR Statement", Null);
        }
        else
        {
            L1 = PC;
            do
            {
                Op = ReadStatement();
            } while (!(Op == k_END || Op == k_ERROR || Op == k_EOF));
            GenCode3(i_PUSHC, 0, 0.02);
            GenCode1(i_SUB);
            GenCode2(i_LOOP, L1);
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

    case k_DELAY:
        ReadNumber(Str);
        x  = ConvertNumber(Str);
        p2 = CheckUnits(Str);
        if (p2 > 0)
        {
            x = x * ConversionFactor[p2];
        }
        GenCode3(i_PUSHC, 0, x);
        L1 = PC;
        GenCode3(i_PUSHC, 0, 0.02);
        GenCode1(i_SUB);
        GenCode2(i_LOOP, L1);
        break;

    case k_WAIT:
        L1 = PC;
        ReadbooleanExpression();
        GenCode2(i_JF, L1);
        break;

    case k_ELSE:
        return k_ELSE;
        break;

    case k_END:
        return k_END;
        break;

    case k_UNTIL:
        return k_UNTIL;
        break;

    case k_DO:
        return k_DO;
        break;

    case k_STOP:
        GenCode1(i_STOP);
        return k_STOP;
        break;

    case k_ACTIVATE:
        GenCode1(i_ACTIVATE);
        return k_ACTIVATE;
        break;

    case k_DEACTIVATE:
        GenCode1(i_DEACTIVATE);
        return k_DEACTIVATE;
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
    File_EOF             = false;
    LineNumber           = 1;
    PC                   = 1;
    Script_ScriptEnabled = false;
    Script_ScriptError   = false;

    FileStream = fopen(FileName, "r");
    if (FileStream == NULL)
    {
        return false;
    }
    else
    {
        do
        {
            ReadStatement();
        } while (!(File_EOF || Script_ScriptError));

        fclose(FileStream);

        if (Script_ScriptError)
        {
            printf("Script_ReadScriptFile: %s\n", FileName);
            Script_DisAssembleScript();
        }
        Script_ScriptEnabled = true;
        ProgramSize          = PC;
        StackInit();
        PC = 1;
        Initialise_Script();
        return true;
    }
}

/* ----------------------------------------------------------------------------- */
bool Digit(char Ch)
{
    return (Ch >= '0') && (Ch <= '9');
}

/* ----------------------------------------------------------------------------- */
char CAP(char Ch)
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
bool Alphabetic(char Ch)
{
    return ((Ch >= 'A') && (Ch <= 'Z')) || (Ch == '_');
}

/* ----------------------------------------------------------------------------- */
char Script_Rdch()
{
    char Ch;

    Ch = fgetc(FileStream);
    if (Ch == CR)      /* skip CR, return LF as EOL */
    {
        return Script_Rdch();
    }

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
    File_EOF = Ch == EOF;
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

    a = (unsigned int) (x * Maths_ONERAD + 0.5);
    while (a > 360)
    {
        a = a - 360;
    }
    while (a < 1)
    {
        a = a + 360;
    }
    return (float) (a) / Maths_ONERAD;
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
        case i_JT:
            printf("JT L%d", Arg);
            break;
        case i_JF:
            printf("JF L%d", Arg);
            break;
        case i_JUMP:
            printf("JUMP L%d", Arg);
            break;
        case i_LOOP:
            printf("LOOP L%d", Arg);
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
        case i_STOP:
            printf("STOP");
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
    unsigned int Op;
    unsigned int Arg;
    float        Val;
    float        x1, x2;
    unsigned int OldPC;
    bool         BoolArg;
    bool         b1, b2;

    ElapsedTimeCount = ElapsedTimeCount + 1;

    for (;; )
    {
        if (IOLibx_HoldButtonPressed(IosLink_IOPkt1) || Script_ScriptEnabled == false)
        {
            return;
        }

        OldPC = PC;

        if (PC >= ProgramSize)
        {
            Op = i_STOP;
        }
        else
        {
            Op  = Instructions[PC].Op;
            Arg = Instructions[PC].Arg;
            Val = Instructions[PC].Val;
        }

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
                StackPush(IOLibx_GetElevator(IosLink_IOPkt1));
                break;
            case 26:
                StackPush(IOLibx_GetAileron(IosLink_IOPkt1));
                break;
            case 27:
                StackPush(IOLibx_GetRudder(IosLink_IOPkt1));
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
                StackPush(IOLibx_GetLeftBrake(IosLink_IOPkt1));
                break;
            case 32:
                StackPush(IOLibx_GetRightBrake(IosLink_IOPkt1));
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
                StackPush((float) (ElapsedTimeCount) / 100.0);
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

        case i_JT:
            if (StackPopboolean())
            {
                PC = Arg;
            }
            else
            {
                PC = PC + 1;
            }
            break;

        case i_JF:
            if (StackPopboolean())
            {
                PC = PC + 1;
            }
            else
            {
                PC = Arg;
            }
            break;

        case i_JUMP:
            PC = Arg;
            break;

        case i_LOOP:
            if (StackTop() > 0.0)
            {
                PC = Arg;
            }
            else
            {
                StackPop();
                PC = PC + 1;
            }
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
                IosLink_SendBoolean(IosDefn_SetFlapsMode, BoolArg);
                break;
            case 2:
                IosLink_SendBoolean(IosDefn_SetGearMode, BoolArg);
                break;
            case 3:
                IosLink_SendBoolean(IosDefn_SetNav1LocaliserMode, BoolArg);
                break;
            case 4:
                IosLink_SendBoolean(IosDefn_SetNav1GlideSlopeMode, BoolArg);
                break;
            case 5:
                IosLink_SendBoolean(IosDefn_SetNav2LocaliserMode, BoolArg);
                break;
            case 6:
                IosLink_SendBoolean(IosDefn_SetNav2GlideSlopeMode, BoolArg);
                break;
            case 7:
                IosLink_SendBoolean(IosDefn_SetRMI1Mode, BoolArg);
                break;
            case 8:
                IosLink_SendBoolean(IosDefn_SetRMI2Mode, BoolArg);
                break;
            case 9:
                IosLink_SendBoolean(IosDefn_SetDMEMode, BoolArg);
                break;
            case 10:
                IosLink_SendBoolean(IosDefn_SetEngine1Mode, BoolArg);
                break;
            case 11:
                IosLink_SendBoolean(IosDefn_SetEngine2Mode, BoolArg);
                break;
            case 12:
                IosLink_SendBoolean(IosDefn_SetEngine3Mode, BoolArg);
                break;
            case 13:
                IosLink_SendBoolean(IosDefn_SetEngine4Mode, BoolArg);
                break;
            case 14:
                IosLink_SendBoolean(IosDefn_SetASIMode, BoolArg);
                break;
            case 15:
                IosLink_SendBoolean(IosDefn_SetAIMode, BoolArg);
                break;
            case 16:
                IosLink_SendBoolean(IosDefn_SetVSIMode, BoolArg);
                break;
            case 17:
                IosLink_SendBoolean(IosDefn_SetAltimeterMode, BoolArg);
                break;
            case 18:
                IosLink_SendBoolean(IosDefn_SetTurnMode, BoolArg);
                break;
            case 19:
                IosLink_SendBoolean(IosDefn_SetCompassMode, BoolArg);
                break;
            case 20:
                IosLink_SendBoolean(IosDefn_SetFDMode, BoolArg);
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

        case i_STOP:
            Script_ScriptEnabled = false;
            Script_SaveScript(Filename);
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
        if (Op != i_JUMP && Op != i_JT && Op != i_JF && Op != i_LOOP)
        {
            PC = PC + 1;
        }
        if (PC < OldPC)
        {
            MonitorScript();
            return;
        }
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
void Wr2(unsigned int x)
{
    OutputWrch(x / 10 + '0');
    OutputWrch(x % 10 + '0');
}

/* ----------------------------------------------------------------------------- */
void WriteTimeStamp()
{
    unsigned int h, m, s, t;
    unsigned int ts = 0;

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
    Wr2(h);
    OutputWrch(':');
    Wr2(m);
    OutputWrch(':');
    Wr2(s);
    OutputWrch('.');
    Wr2(t);
    OutputWrch(',');
    OutputWrch(SPACE);
}

/* ----------------------------------------------------------------------------- */
void MonitorScript()
{
    unsigned int i;

    CurrentTick = CurrentTick + 1;
    if (CurrentTick >= NextTick)
    {
        NextTick = CurrentTick + (unsigned int) (SampleRate * 50.0 + 0.1);
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
    SampleRate       = 1.0;
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
