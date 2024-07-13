#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include <SIM/iosdefn.h>
#include <SIM/glibx.h>

#include "ioslink.h"
#include "menu.h"
#include "scan.h"
#include "gui.h"
#include "map.h"
#include "ios.h"

#define k_ENTER     200
#define k_BS        201
#define k_CANCEL    202
#define k_CLR       203
#define k_CAPS      204

#define LF          10
#define CR          13
#define EOL         10

#define X0          (Glibx_SCREENWIDTH / 2)

typedef struct
{
    int  x;
    int  y;
    int  xs;
    int  ys;
    char str[8];
    int  val;
} KeyButton;


static KeyButton KeyPad[16] = {
    {   0,   0, 110,  50, "0",      '0'      },
    { 120,   0,  50,  50, ".",      '.'      },
    {   0,  60,  50,  50, "1",      '1'      },
    {  60,  60,  50,  50, "2",      '2'      },
    { 120,  60,  50,  50, "3",      '3'      },
    {   0, 120,  50,  50, "4",      '4'      },
    {  60, 120,  50,  50, "5",      '5'      },
    { 120, 120,  50,  50, "6",      '6'      },
    {   0, 180,  50,  50, "7",      '7'      },
    {  60, 180,  50,  50, "8",      '8'      },
    { 120, 180,  50,  50, "9",      '9'      },
    { 180,   0,  50, 110, "ENT",    k_ENTER  },
    { 180, 120,  50, 110, "BS",     k_BS     },
    {   0, 240,  50,  50, "-",      '-'      },
    {  60, 240,  50,  50, "CLR",    k_CLR    },
    { 120, 240, 110,  50, "CANCEL", k_CANCEL }
};

static KeyButton AlphaPad[53] = {
    {  35,   0,  50, 50, "\\",     '\\'     },
    {  95,   0,  50, 50, "Z",      'z'      },
    { 155,   0,  50, 50, "X",      'x'      },
    { 215,   0,  50, 50, "C",      'c'      },
    { 275,   0,  50, 50, "V",      'v'      },
    { 335,   0,  50, 50, "B",      'b'      },
    { 395,   0,  50, 50, "N",      'n'      },
    { 455,   0,  50, 50, "M",      'm'      },
    { 515,   0,  50, 50, ",",      ','      },
    { 575,   0,  50, 50, ".",      '.'      },
    { 635,   0,  50, 50, "/",      '/'      },
    { 695,   0, 100, 50, "ENT",    k_ENTER  },
    {   0,  60,  50, 50, "",       k_CAPS   },
    {  60,  60,  50, 50, "A",      'a'      },
    { 120,  60,  50, 50, "S",      's'      },
    { 180,  60,  50, 50, "D",      'd'      },
    { 240,  60,  50, 50, "F",      'f'      },
    { 300,  60,  50, 50, "G",      'g'      },
    { 360,  60,  50, 50, "H",      'h'      },
    { 420,  60,  50, 50, "J",      'j'      },
    { 480,  60,  50, 50, "K",      'k'      },
    { 540,  60,  50, 50, "L",      'l'      },
    { 600,  60,  50, 50, ";",      ';'      },
    { 660,  60,  50, 50, "\'",     '\''     },
    { 720,  60,  50, 50, "#",      '#'      },
    { 780,  60, 100, 50, "BS",     k_BS     },
    {  35, 120,  50, 50, "Q",      'q'      },
    {  95, 120,  50, 50, "W",      'w'      },
    { 155, 120,  50, 50, "E",      'e'      },
    { 215, 120,  50, 50, "R",      'r'      },
    { 275, 120,  50, 50, "T",      't'      },
    { 335, 120,  50, 50, "Y",      'y'      },
    { 395, 120,  50, 50, "U",      'u'      },
    { 455, 120,  50, 50, "I",      'i'      },
    { 515, 120,  50, 50, "O",      'o'      },
    { 575, 120,  50, 50, "P",      'p'      },
    { 635, 120,  50, 50, "[",      '['      },
    { 695, 120,  50, 50, "]",      ']'      },
    { 755, 120, 100, 50, "CLR",    k_CLR    },
    {   0, 180,  50, 50, "1",      '1'      },
    {  60, 180,  50, 50, "2",      '2'      },
    { 120, 180,  50, 50, "3",      '3'      },
    { 180, 180,  50, 50, "4",      '4'      },
    { 240, 180,  50, 50, "5",      '5'      },
    { 300, 180,  50, 50, "6",      '6'      },
    { 360, 180,  50, 50, "7",      '7'      },
    { 420, 180,  50, 50, "8",      '8'      },
    { 480, 180,  50, 50, "9",      '9'      },
    { 540, 180,  50, 50, "0",      '0'      },
    { 600, 180,  50, 50, "-",      '-'      },
    { 660, 180,  50, 50, "+",      '+'      },
    { 720, 180, 100, 50, "CANCEL", k_CANCEL }
};

int  CopyListOfFiles(char filename[]);
char ReadLine(FILE *f, char a[]);
char Rdchf(FILE *f);
void CheckNumeric(int mx, int my);
void CheckButtons(int mx, int my);
void CheckFlightData(int mx, int my);
void CheckCoordinates(int mx, int my);
void UpdateDialogue(int mx, int my);
void DrawFileBoxes();
void DrawButton(int n, char name[], int bcol, int fcol);
int  GetButton(int x, int y);
int  CheckFileList(int x, int y);
void LCDChars(char Str[], int x, int y);
void LCDChar(char Ch, int x, int y);
void LED(int x, int y);
void Segment(unsigned int n, int x, int y);
void DrawKeyPad(int x, int y);
void DrawAlphaPad(int x, int y);
void ShowDialogue();
void DrawNumericBox(int x, int y, char str[]);
void DrawAlphanumericBox(int x, int y, char str[]);
void DrawQuestionBox(int x, int y, char str[]);
void DrawButtonsBox(int x, int y, char str[]);
void DrawFlightDataBox(int x, int y, char str[]);
void DrawErrorBox(int x, int y);
int  GetKeyPad(int x, int y);
int  GetAlphaPad(int x, int y);
void CheckFlightDataNumeric(int mx, int my);

int           Gui_MouseX;
int           Gui_MouseY;
bool          Gui_MouseLeftButton;
bool          Gui_MouseMiddleButton;
bool          Gui_MouseRightButton;
float         Gui_MouseScrollFactor;

static int    QuestionX     = X0 + 100;
static int    QuestionY     = 500;
static int    ButtonsX      = X0 + 100;
static int    ButtonsY      = 400;
static int    FlightDataX   = X0 + 100;
static int    FlightDataY   = 200;
static int    NumericX      = X0 + 500;
static int    NumericY      = 400;
static int    AlphanumericX = X0 + 100;
static int    AlphanumericY = 350;

static int    PanningX;
static int    PanningY;
static bool   OldMouseLeftButton  = false;
static bool   OldMouseRightButton = false;
static int    MenuLevel           = 0;
static int    MainOption          = 0;
static int    SubOption           = 0;
static char   NumericBuff[50];
static char   AlphanumericBuff[50];
static char   MinBuff[50];
static char   MaxBuff[50];
static char   IncBuff[50];
static int    CurrentButton = 0;
static string ListOfFiles[50];
static int    nFiles        = 0;
static int    CurrentUnits  = 0;
static int    CurrentPlot   = 0;
static int    MinMaxIncBox  = 0;
static bool   MouseTracking = false;
static int    TrackX0, TrackY0;
static int    TrackState = 0;
static bool   MenuMoving = false;
static int    MenuX0, MenuY0;
static bool   ErrorPending = false;
static char   ErrorMessage[50];
static bool   caps = false;

/* ---------------------------------- */
void Gui_CheckMouse(void)
{
    bool leftkeypressed = Gui_MouseLeftButton && (!OldMouseLeftButton);

    if (leftkeypressed)
	{
	    if (Gui_MouseX > Glibx_SCREENWIDTH - 40 && Gui_MouseY > Glibx_SCREENHEIGHT - 40 &&
		    Gui_MouseX < Glibx_SCREENWIDTH - 20 && Gui_MouseY < Glibx_SCREENHEIGHT - 20)
		{
		    IOS_Mode = false;
			return;
		}
	}
	
    switch (MenuLevel)
    {
    case 0:
        IosLink_CheckPrint();     /* only print if menu level is 0 */

        if (leftkeypressed)
        {
            MenuLevel = 1;
        }
        break;

    case 1:
        if (leftkeypressed)
        {
            MainOption = GetButton(Gui_MouseX, Gui_MouseY);
            if ((MainOption >= 0) && (MainOption < Menu_NumberOfMenus))
            {
                MenuLevel = 2;
            }
            else
            {
                MenuLevel = 0;
            }
        }
        break;

    case 2:
        if (leftkeypressed)
        {
            SubOption = GetButton(Gui_MouseX, Gui_MouseY);
            if ((SubOption >= 0) && (SubOption < Menus[MainOption].NumberOfItems))
            {
                MenuLevel = 3;

                switch (Menus[MainOption].State[SubOption].Info)
                {
                case None:
                    IosLink_Execute(MainOption, SubOption);
                    MenuLevel = 0;
                    break;

                case Question:
                    break;

                case Numeric:
                    sprintf(NumericBuff, "%g", Menus[MainOption].State[SubOption].Data.Numeric.Val);
                    break;

                case Buttons:
                    CurrentButton = Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton;
                    break;

                case FileName:
                    AlphanumericBuff[0] = (char) 0;
                    break;

                case FileList:
                    switch (Menus[MainOption].State[SubOption].Mval)
                    {
                    case IosDefn_Restore:
                        nFiles = ScanDir(".", ".sav", ListOfFiles);
                        break;
                    case IosDefn_Script:
                        nFiles = ScanDir(".", ".scr", ListOfFiles);
                        break;
                    case IosDefn_LoadFlightPlan:
                        nFiles = ScanDir(".", ".pln", ListOfFiles);
                        break;
                    case IosDefn_LoadPlaybackFile:
                        nFiles = ScanDir(".", ".dat", ListOfFiles);
                        break;
                    case IosDefn_LoadTargetFile:
                        nFiles = CopyListOfFiles("../files/targets.lst"); /* diff. from LFS ! */
                        break;
                    case IosDefn_Models:
                        nFiles = CopyListOfFiles("../files/models.lst");
                        break;
                    case IosDefn_Visual:
                        nFiles = CopyListOfFiles("../files/visual.lst");
                        break;
                    case IosDefn_LoadDTED:
                        nFiles = CopyListOfFiles("../files/dteds.lst");
                        break;
                    }
                    break;

                case Coordinates:
                    break;

                case FlightData:
                    CurrentUnits = Menus[MainOption].State[SubOption].Data.FlightData.ActiveUnits;
                    CurrentPlot  = Menus[MainOption].State[SubOption].Data.FlightData.Plotting;
                    sprintf(MinBuff, "%g", Menus[MainOption].State[SubOption].Data.FlightData.ymin);
                    sprintf(MaxBuff, "%g", Menus[MainOption].State[SubOption].Data.FlightData.ymax);
                    sprintf(IncBuff, "%g", Menus[MainOption].State[SubOption].Data.FlightData.yinc);
                    MinMaxIncBox = 0;
                    break;
                }
            }
            else
            {
                MenuLevel = 0;
            }
        }
        break;

    case 3:
        if (MenuMoving)
        {
            UpdateDialogue(Gui_MouseX, Gui_MouseY);
            return;
        }
        if (Menus[MainOption].State[SubOption].Info == Coordinates)
        {
            if (MouseTracking == false)
            {
                MouseTracking       = true;
                Gui_MouseLeftButton = false;
            }
            UpdateDialogue(Gui_MouseX, Gui_MouseY);
        }
        else if (leftkeypressed)
        {
            UpdateDialogue(Gui_MouseX, Gui_MouseY);
        }
        break;
    }

    if (!MenuMoving)
    {
        OldMouseLeftButton  = Gui_MouseLeftButton;
        OldMouseRightButton = Gui_MouseRightButton;
    }
}

/* ---------------------------------- */
int CopyListOfFiles(char filename[])
{
    char tch;
    FILE *f;
    int  p = 0;

    f = fopen(filename, "r");
    if (f == NULL)
    {
        printf("Can't open file %s\n", filename);
        return 0;
    }

    while (1)
    {
        tch = ReadLine(f, ListOfFiles[p]);
        if (tch == EOF)
        {
            return p;
        }
        p = p + 1;
        if (p >= 25)
        {
            return 25;
        }
    }
    fclose(f);
}

/* ---------------------------------- */
char ReadLine(FILE *f, char a[])
{
    int  i = 0;
    char Ch;

    while (1)
    {
        Ch = Rdchf(f);
        if (Ch == EOL || Ch == EOF)
        {
            a[i] = '\0';
            return Ch;
        }
        else
        {
            a[i] = Ch;
            i    = i + 1;
        }
    }
}

/* ---------------------------------- */
char Rdchf(FILE *f)
{
    int Ch;

    Ch = fgetc(f);
    if (Ch == CR)
    {
        return Rdchf(f);
    }
    else
    {
        return Ch;
    }
}

/* ---------------------------------- */
void CheckFlightData(int mx, int my)
{
    int dy = Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits * 40;
    int i;
    int x, y;

    if (MinMaxIncBox > 0)
    {
        CheckFlightDataNumeric(mx, my);
        return;
    }

    if ((mx >= FlightDataX) && (mx <= (FlightDataX + 100)) && (my >= FlightDataY) && (my <= (FlightDataY + 50)))
    {
        Menus[MainOption].State[SubOption].Data.FlightData.ActiveUnits = CurrentUnits;
        Menus[MainOption].State[SubOption].Data.FlightData.Plotting    = CurrentPlot;
        Menus[MainOption].State[SubOption].Data.FlightData.ymin        = atof(MinBuff);
        Menus[MainOption].State[SubOption].Data.FlightData.ymax        = atof(MaxBuff);
        Menus[MainOption].State[SubOption].Data.FlightData.yinc        = atof(IncBuff);
        MenuLevel                                                      = 0;
        IosLink_Execute(MainOption, SubOption);
        return;
    }
    else if ((mx >= (FlightDataX + 110)) && (mx <= (FlightDataX + 210)) && (my >= FlightDataY) && (my <= (FlightDataY + 50)))
    {
        MenuLevel = 0; /* cancel */
        return;
    }

    if (Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits >= 3)
    {
        for (i = 0; i < Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits - 1; i = i + 1)
        {
            x = FlightDataX + 150;
            y = FlightDataY + 40 + dy - i * 40 - 5;
            if ((mx >= x) && (mx <= (x + 30 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
            {
                CurrentUnits = i;
                return;
            }
        }
    }

    x = FlightDataX + 70;
    y = FlightDataY + 75 + dy + 130 - 45;
    if ((mx >= x) && (mx <= (x + 130 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
    {
        MinMaxIncBox = 1;
        sprintf(NumericBuff, "%g", Menus[MainOption].State[SubOption].Data.FlightData.ymin);
        return;
    }
    else
    {
        y = y - 40;
        if ((mx >= x) && (mx <= (x + 130 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
        {
            MinMaxIncBox = 2;
            sprintf(NumericBuff, "%g", Menus[MainOption].State[SubOption].Data.FlightData.ymax);
            return;
        }
        else
        {
            y = y - 40;
            if ((mx >= x) && (mx <= (x + 130 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
            {
                MinMaxIncBox = 3;
                sprintf(NumericBuff, "%g", Menus[MainOption].State[SubOption].Data.FlightData.yinc);
                return;
            }
        }
    }

    x = FlightDataX + 150;
    y = FlightDataY + 40 + dy - (Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits - 1) * 40 - 5;
    if ((mx >= x) && (mx <= (x + 30 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
    {
        CurrentPlot = !CurrentPlot;
        return;
    }

    if ((mx >= FlightDataX) && (mx <= (FlightDataX + 210 - 1)) &&
        (my >= (FlightDataY + 60)) && (my <= (FlightDataY + 60 + 50 + dy + 120 - 1)))
    {
        if ((!MenuMoving) && (Gui_MouseLeftButton))
        {
            MenuX0     = mx;
            MenuY0     = my;
            MenuMoving = true;
        }
        else if ((MenuMoving) && (Gui_MouseLeftButton))
        {
            FlightDataX = FlightDataX + (mx - MenuX0);
            FlightDataY = FlightDataY + (my - MenuY0);
            MenuX0      = mx;
            MenuY0      = my;
        }
        else
        {
            MenuMoving         = false;
            OldMouseLeftButton = false;
        }
    }
}

/* ---------------------------------- */
void CheckButtons(int mx, int my)
{
    int dy = Menus[MainOption].State[SubOption].Data.Buttons.NumberOfButtons * 40;
    int i;
    int x, y;

    if ((mx >= ButtonsX) && (mx <= (ButtonsX + 100)) && (my >= ButtonsY) && (my <= (ButtonsY + 50)))
    {
        Menus[MainOption].State[SubOption].Data.Buttons.ActiveButton = CurrentButton;
        MenuLevel                                                    = 0;
        IosLink_Execute(MainOption, SubOption);
    }
    else if ((mx >= (ButtonsX + 110)) && (mx <= (ButtonsX + 210)) && (my >= ButtonsY) && (my <= (ButtonsY + 50)))
    {
        MenuLevel = 0; /* cancel */
    }

    for (i = 0; i < Menus[MainOption].State[SubOption].Data.Buttons.NumberOfButtons; i = i + 1)
    {
        x = ButtonsX + 150;
        y = ButtonsY + 40 + dy - i * 40 - 5;
        if ((mx >= x) && (mx <= (x + 30 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
        {
            CurrentButton = i;
            return;
        }
    }

    if ((mx >= ButtonsX) && (mx <= (ButtonsX + 210 - 1)) &&
        (my >= (ButtonsY + 60)) && (my <= (ButtonsY + 60 + 50 + dy - 1)))
    {
        if ((!MenuMoving) && (Gui_MouseLeftButton))
        {
            MenuX0     = mx;
            MenuY0     = my;
            MenuMoving = true;
        }
        else if ((MenuMoving) && (Gui_MouseLeftButton))
        {
            ButtonsX = ButtonsX + (mx - MenuX0);
            ButtonsY = ButtonsY + (my - MenuY0);
            MenuX0   = mx;
            MenuY0   = my;
        }
        else
        {
            MenuMoving         = false;
            OldMouseLeftButton = false;
        }
    }
}

/* ---------------------------------- */
void CheckNumeric(int mx, int my)
{
    int   s   = strlen(NumericBuff);
    int   key = GetKeyPad(mx, my);
    float x;
    int   x1, y1;

    if (ErrorPending)
    {
        x1 = NumericX + 65;
        y1 = NumericY + 130;
        if ((mx >= x1) && (mx <= (x1 + 100 - 1)) &&
            (my >= y1) && (my <= (y1 + 50 - 1)))
        {
            ErrorPending = false;
            return;
        }
        else
        {
            return;
        }
    }

    switch (key)
    {
    case '-':
        if (s == 0)
        {
            NumericBuff[s]     = key;
            NumericBuff[s + 1] = (char) 0;
        }
        break;

    case k_CLR:
        NumericBuff[0] = (char) 0;
        break;

    case k_ENTER:
        x = (float) atof(NumericBuff);
        if (x < Menus[MainOption].State[SubOption].Data.Numeric.MinVal)
        {
            ErrorPending = true;
            sprintf(ErrorMessage, "minimum value %g",
                    Menus[MainOption].State[SubOption].Data.Numeric.MinVal);
        }
        else if (x > Menus[MainOption].State[SubOption].Data.Numeric.MaxVal)
        {
            ErrorPending = true;
            sprintf(ErrorMessage, "maximum value %g",
                    Menus[MainOption].State[SubOption].Data.Numeric.MaxVal);
        }
        else
        {
            Menus[MainOption].State[SubOption].Data.Numeric.Val = x;
            MenuLevel                                           = 0;
            IosLink_Execute(MainOption, SubOption);
        }
        break;

    case k_CANCEL:
        MenuLevel = 0;
        break;

    case k_BS:
        if (s > 0)
        {
            s              = s - 1;
            NumericBuff[s] = (char) 0;
        }
        break;

    default:  /* 0..9 or . */
        if (key > 0)
        {
            if (s <= 7)
            {
                NumericBuff[s]     = key;
                NumericBuff[s + 1] = (char) 0;
            }
        }
        else
        {
            if ((mx >= NumericX) && (mx <= (NumericX + 230 - 1)) &&
                (my >= NumericY) && (my <= (NumericY + 120 - 1)))
            {
                if ((!MenuMoving) && (Gui_MouseLeftButton))
                {
                    MenuX0     = mx;
                    MenuY0     = my;
                    MenuMoving = true;
                }
                else if ((MenuMoving) && (Gui_MouseLeftButton))
                {
                    NumericX = NumericX + (mx - MenuX0);
                    NumericY = NumericY + (my - MenuY0);
                    MenuX0   = mx;
                    MenuY0   = my;
                }
                else
                {
                    MenuMoving         = false;
                    OldMouseLeftButton = false;
                }
            }
        }
        break;
    }
}

/* ---------------------------------- */
void CheckFlightDataNumeric(int mx, int my)
{
    int s   = strlen(NumericBuff);
    int key = GetKeyPad(mx, my);

    switch (key)
    {
    case 0:
        break;

    case '-':
        if (s == 0)
        {
            NumericBuff[s]     = key;
            NumericBuff[s + 1] = (char) 0;
        }
        break;

    case k_CLR:
        NumericBuff[0] = (char) 0;
        break;

    case k_ENTER:
        switch (MinMaxIncBox)
        {
        case 1:
            strcpy(MinBuff, NumericBuff);
            break;
        case 2:
            strcpy(MaxBuff, NumericBuff);
            break;
        case 3:
            strcpy(IncBuff, NumericBuff);
            break;
        }
        MinMaxIncBox = 0;
        break;

    case k_CANCEL:
        MinMaxIncBox = 0;
        break;

    case k_BS:
        if (s > 0)
        {
            s              = s - 1;
            NumericBuff[s] = (char) 0;
        }
        break;

    default:  /* 0..9 or . */
        if (s <= 7)
        {
            NumericBuff[s]     = key;
            NumericBuff[s + 1] = (char) 0;
        }
        break;
    }
}

/* ---------------------------------- */
void CheckAlphanumeric(int mx, int my)
{
    int s   = strlen(AlphanumericBuff);
    int key = GetAlphaPad(mx, my);

    switch (key)
    {
    case k_CLR:
        AlphanumericBuff[0] = (char) 0;
        break;

    case k_ENTER:
        MenuLevel = 0;
        strcpy(Menus[MainOption].State[SubOption].Data.Fname, AlphanumericBuff);
        IosLink_Execute(MainOption, SubOption);
        break;

    case k_CANCEL:
        MenuLevel = 0;
        break;

    case k_BS:
        if (s > 0)
        {
            s                   = s - 1;
            AlphanumericBuff[s] = (char) 0;
        }
        break;

    case k_CAPS:
        caps = !caps;
        break;

    default:  /* A..Z etc */
        if (key > 0)
        {
            if (s <= 60)
            {
                if ((key >= (int) 'a') && (key <= (int) 'z') && caps)
                {
                    key = key - (int) 'a' + (int) 'A';
                }
                AlphanumericBuff[s]     = key;
                AlphanumericBuff[s + 1] = (char) 0;
            }
        }
        else
        {
            if ((mx >= AlphanumericX) && (mx <= (AlphanumericX + 880 - 1)) &&
                (my >= AlphanumericY) && (my <= (AlphanumericY + 120 - 1)))
            {
                if ((!MenuMoving) && (Gui_MouseLeftButton))
                {
                    MenuX0     = mx;
                    MenuY0     = my;
                    MenuMoving = true;
                }
                else if ((MenuMoving) && (Gui_MouseLeftButton))
                {
                    AlphanumericX = AlphanumericX + (mx - MenuX0);
                    AlphanumericY = AlphanumericY + (my - MenuY0);
                    MenuX0        = mx;
                    MenuY0        = my;
                }
                else
                {
                    MenuMoving         = false;
                    OldMouseLeftButton = false;
                }
            }
        }
        break;
    }
}

/* ---------------------------------- */
void CheckCoordinates(int mx, int my)
{
    if (Gui_MouseLeftButton)
    {
        switch (Menus[MainOption].State[SubOption].Mval)
        {
        case IosDefn_MapCentre:
        case IosDefn_RePositionAircraft:
        case IosDefn_SetTargetPosition:
        case IosDefn_MapCompass:
            Map_ChangeMap(Menus[MainOption].State[SubOption].Mval, Gui_MouseX, Gui_MouseY);
            MouseTracking = false;
            MenuLevel     = 0;
            break;

        case IosDefn_MapTrack:
            if ((TrackState == 0) && (!OldMouseLeftButton))
            {
                TrackX0             = Gui_MouseX;
                TrackY0             = Gui_MouseY;
                TrackState          = 1;
                Gui_MouseLeftButton = false;      /* force continue after mouse click */
                OldMouseLeftButton  = false;
            }
            else if (TrackState == 1)
            {
                MouseTracking = false;
                MenuLevel     = 0;
                TrackState    = 0;
                Map_AddTrack(TrackX0, TrackY0, Gui_MouseX, Gui_MouseY);
            }
            break;
        }
    }
}

/* ---------------------------------- */
void CheckQuery(int mx, int my)
{
    if ((mx >= QuestionX) && (mx <= (QuestionX + 100)) && (my >= QuestionY) && (my <= (QuestionY + 50)))
    {
        MenuLevel = 0;
        IosLink_Execute(MainOption, SubOption);
    }
    else if ((mx >= (QuestionX + 110)) && (mx <= (QuestionX + 210)) && (my >= QuestionY) && (my <= (QuestionY + 50)))
    {
        MenuLevel = 0; /* cancel */
    }
    else if ((mx >= QuestionX) && (mx <= (QuestionX + 210 - 1)) &&
             (my >= (QuestionY + 60)) && (my <= (QuestionY + 60 + 50 - 1)))
    {
        if ((!MenuMoving) && (Gui_MouseLeftButton))
        {
            MenuX0     = mx;
            MenuY0     = my;
            MenuMoving = true;
        }
        else if ((MenuMoving) && (Gui_MouseLeftButton))
        {
            QuestionX = QuestionX + (mx - MenuX0);
            QuestionY = QuestionY + (my - MenuY0);
            MenuX0    = mx;
            MenuY0    = my;
        }
        else
        {
            MenuMoving         = false;
            OldMouseLeftButton = false;
        }
    }
}

/* ---------------------------------- */
void UpdateDialogue(int mx, int my)
{
    int f = 0;

    switch (Menus[MainOption].State[SubOption].Info)
    {
    case None:
        break;
    case Question:
        CheckQuery(mx, my);
        break;
    case Numeric:
        CheckNumeric(mx, my);
        break;
    case Buttons:
        CheckButtons(mx, my);
        break;
    case FileName:
        CheckAlphanumeric(mx, my);
        break;
    case FileList:
        f = CheckFileList(mx, my);
        if (f >= 0)
        {
            IosLink_Execute(MainOption, SubOption);
        }
        break;
    case Coordinates:
        CheckCoordinates(mx, my);
        break;
    case FlightData:
        CheckFlightData(mx, my);
        break;
    }
}

/* ---------------------------------- */
void Gui_Menu(void)
{
    int i;

    glLineWidth(2.0);
    Glibx_SetFont(Glibx_GFONT20, 8);

    switch (MenuLevel)
    {
    case 0:
        break;

    case 1:
        for (i = 0; i <= Menu_NumberOfMenus - 1; i = i + 1)
        {
            DrawButton(i, Menus[i].Title, Glibx_BLUE, Glibx_WHITE);
        }
        break;

    case 2:
        for (i = 0; i <= Menus[MainOption].NumberOfItems - 1; i = i + 1)
        {
            DrawButton(i, Menus[MainOption].State[i].Mname, Glibx_BLUE, Glibx_WHITE);
        }
        break;

    case 3:
        ShowDialogue();
        break;
    }
}

/* ---------------------------------- */
void ShowDialogue()
{
    float Lat, Long;

    Glibx_SetFont(Glibx_GFONT20, 8);
    switch (Menus[MainOption].State[SubOption].Info)
    {
    case None:
        break;
    case Question:
        DrawQuestionBox(QuestionX, QuestionY, Menus[MainOption].State[SubOption].Mprompt);
        break;
    case Numeric:
        DrawNumericBox(NumericX, NumericY, Menus[MainOption].State[SubOption].Mprompt);
        DrawKeyPad(NumericX, NumericY - 300);
        Glibx_Colour(Glibx_RED);
        LCDChars(NumericBuff, NumericX + 25, NumericY + 35);
        break;
    case Buttons:
        DrawButtonsBox(ButtonsX, ButtonsY, Menus[MainOption].State[SubOption].Mname);
        break;
    case FileName:
        DrawAlphanumericBox(AlphanumericX - 60, AlphanumericY, Menus[MainOption].State[SubOption].Mprompt);
        DrawAlphaPad(AlphanumericX - 60, AlphanumericY - 250);
        Glibx_Chars(AlphanumericBuff, AlphanumericX + 25, AlphanumericY + 35);
        break;
    case FileList:
        DrawFileBoxes();
        break;
    case Coordinates:
        switch (Menus[MainOption].State[SubOption].Mval)
        {
        case IosDefn_MapCentre:
        case IosDefn_RePositionAircraft:
        case IosDefn_SetTargetPosition:
            Map_DisplayCoordinates(Gui_MouseX, Gui_MouseY);
            break;
        case IosDefn_MapCompass:
            Map_ScreenToGlobe(Gui_MouseX, Gui_MouseY, &Lat, &Long);
            Map_CompassRose(Lat, Long);
            break;
        case IosDefn_MapTrack:
            Map_DisplayCoordinates(Gui_MouseX, Gui_MouseY);
            if (TrackState > 0)
            {
                Glibx_Colour(Glibx_BLACK);
                Glibx_Draw(TrackX0, TrackY0, Gui_MouseX, Gui_MouseY);
                Map_DisplayDmeDistance(TrackX0, TrackY0, Gui_MouseX, Gui_MouseY);
            }
        }
        break;
    case FlightData:
        DrawFlightDataBox(FlightDataX, FlightDataY, Menus[MainOption].State[SubOption].Mname);
        if (MinMaxIncBox > 0)
        {
//          DrawKeyPad(FlightDataX + 400, FlightDataY - 100);
            Glibx_Colour(Glibx_RED);
            LCDChars(NumericBuff, NumericX + 25, NumericY + 35);
        }
        break;
    }

    if (ErrorPending)
    {
        DrawErrorBox(NumericX, NumericY + 130);
    }
}

/* ---------------------------------- */
void DrawFileBoxes()
{
    int i;
	
    for (i = 0; i < nFiles; i = i + 1)
    {
        DrawButton(i, ListOfFiles[i], Glibx_BLUE, Glibx_WHITE);
    }
}

/* ---------------------------------- */
void DrawNumericBox(int x, int y, char str[])
{
    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(x - 10, y - 10, 250, 140);
    Glibx_Colour(Glibx_BLUE);
    Glibx_Rectangle(x, y, 230, 120);
    Glibx_Colour(Glibx_BLACK);
    Glibx_Rectangle(x + 15, y + 25, 200, 50);
    Glibx_Colour(Glibx_WHITE);
    Glibx_Chars(str, x + 10, y + 95);
}

/* ---------------------------------- */
void DrawAlphanumericBox(int x, int y, char str[])
{
    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(x - 10, y - 10, 900, 140);
    Glibx_Colour(Glibx_BLUE);
    Glibx_Rectangle(x, y, 880, 120);
    Glibx_Colour(Glibx_BLACK);
    Glibx_Rectangle(x + 15, y + 25, 850, 40);
    Glibx_Colour(Glibx_WHITE);
    Glibx_Chars(str, x + 10, y + 95);
}

/* ---------------------------------- */
void DrawErrorBox(int x, int y)
{
    int w;

    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(x - 10, y - 10, 250, 130);
    Glibx_Colour(Glibx_BLUE);
    Glibx_Rectangle(x, y + 60, 230, 50);
    Glibx_Colour(Glibx_WHITE);
    Glibx_Rectangle(x + 10, y + 70, 210, 30);
    Glibx_Colour(Glibx_RED);
    Glibx_Rectangle(x + 65, y, 100, 50);
    Glibx_Chars(ErrorMessage, x + 20, y + 80);
    Glibx_Colour(Glibx_WHITE);
    w = Glibx_StringSize("OK");
    Glibx_Chars("OK", x + 115 - w / 2, y + 25 - 8);
}

/* ---------------------------------- */
void DrawQuestionBox(int x, int y, char str[])
{
    int w;

    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(x - 10, y - 10, 230, 130);
    Glibx_Colour(Glibx_BLUE);
    Glibx_Rectangle(x, y + 60, 210, 50);
    Glibx_Rectangle(x, y, 100, 50);
    Glibx_Rectangle(x + 110, y, 100, 50);
    Glibx_Colour(Glibx_WHITE);
    w = Glibx_StringSize("OK");
    Glibx_Chars("OK", x + 50 - w / 2, y + 25 - 8);
    w = Glibx_StringSize("Cancel");
    Glibx_Chars("Cancel", x + 110 + 50 - w / 2, y + 25 - 8);
    Glibx_Chars(str, x + 10, y + 75);
}

/* ---------------------------------- */
void DrawButtonsBox(int x, int y, char str[])
{
    int dy = Menus[MainOption].State[SubOption].Data.Buttons.NumberOfButtons * 40;
    int i;
    int x1, y1;
    int w;

    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(x - 10, y - 10, 230, 130 + dy);
    Glibx_Colour(Glibx_BLUE);
    Glibx_Rectangle(x, y + 60, 210, 50 + dy);
    Glibx_Rectangle(x, y, 100, 50);
    Glibx_Rectangle(x + 110, y, 100, 50);
    Glibx_Colour(Glibx_WHITE);
    w = Glibx_StringSize("OK");
    Glibx_Chars("OK", x + 50 - w / 2, y + 25 - 8);
    w = Glibx_StringSize("Cancel");
    Glibx_Chars("Cancel", x + 110 + 50 - w / 2, y + 25 - 8);
    Glibx_Chars(str, x + 10, y + 75 + dy);

    for (i = 0; i < Menus[MainOption].State[SubOption].Data.Buttons.NumberOfButtons; i = i + 1)
    {
        Glibx_Colour(Glibx_WHITE);
        Glibx_Chars(Menus[MainOption].State[SubOption].Data.Buttons.ButtonsList[i], x + 30, y + 40 + dy - i * 40);
        Glibx_Colour(Glibx_BLACK);
        Glibx_Rectangle(x + 150, y + 40 + dy - i * 40 - 5, 30, 30);
    }

    Glibx_Colour(Glibx_WHITE);
    x1 = x + 150;
    y1 = y + 40 + dy - CurrentButton * 40 - 5;
    Glibx_Draw(x1 + 10, y1 + 5, x1 + 5, y1 + 10);
    Glibx_Draw(x1 + 10, y1 + 5, x1 + 25, y1 + 25);
}

/* ---------------------------------- */
void DrawFlightDataBox(int x, int y, char str[])
{
    int dy = Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits * 40;
    int i;
    int x1, y1;
    int w;

    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(x - 10, y - 10, 230, 130 + dy + 120);
    Glibx_Colour(Glibx_BLUE);
    Glibx_Rectangle(x, y + 60, 210, 50 + dy + 120);
    Glibx_Rectangle(x, y, 100, 50);
    Glibx_Rectangle(x + 110, y, 100, 50);
    Glibx_Colour(Glibx_WHITE);
    w = Glibx_StringSize("OK");
    Glibx_Chars("OK", x + 50 - w / 2, y + 25 - 8);
    w = Glibx_StringSize("Cancel");
    Glibx_Chars("Cancel", x + 110 + 50 - w / 2, y + 25 - 8);
    Glibx_Chars(str, x + 10, y + 75 + dy + 130);

    for (i = 0; i < Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits; i = i + 1)
    {
        Glibx_Colour(Glibx_WHITE);
        Glibx_Chars(Menus[MainOption].State[SubOption].Data.FlightData.UnitsList[i], x + 30, y + 40 + dy - i * 40);
        Glibx_Colour(Glibx_BLACK);
        Glibx_Rectangle(x + 150, y + 40 + dy - i * 40 - 5, 30, 30);
    }

    Glibx_Colour(Glibx_WHITE);
    Glibx_Chars("Min", x + 20, y + 75 + dy + 130 - 40);
    Glibx_Chars("Max", x + 20, y + 75 + dy + 130 - 80);
    Glibx_Chars("Inc", x + 20, y + 75 + dy + 130 - 120);
    Glibx_Colour(Glibx_BLACK);
    Glibx_Rectangle(x + 70, y + 75 + dy + 130 - 45, 130, 30);
    Glibx_Rectangle(x + 70, y + 75 + dy + 130 - 85, 130, 30);
    Glibx_Rectangle(x + 70, y + 75 + dy + 130 - 125, 130, 30);

    Glibx_Colour(Glibx_WHITE);
    Glibx_Chars(MinBuff, x + 80, y + 75 + dy + 130 - 45 + 5);
    Glibx_Chars(MaxBuff, x + 80, y + 75 + dy + 130 - 85 + 5);
    Glibx_Chars(IncBuff, x + 80, y + 75 + dy + 130 - 125 + 5);

    if (Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits >= 2)
    {
        Glibx_Colour(Glibx_WHITE);
        x1 = x + 150;
        y1 = y + 40 + dy - CurrentUnits * 40 - 5;
        Glibx_Draw(x1 + 10, y1 + 5, x1 + 5, y1 + 10);
        Glibx_Draw(x1 + 10, y1 + 5, x1 + 25, y1 + 25);
    }

    if (CurrentPlot)
    {
        Glibx_Colour(Glibx_WHITE);
        x1 = x + 150;
        y1 = y + 40 + dy - (Menus[MainOption].State[SubOption].Data.FlightData.NumberOfUnits - 1) * 40 - 5;
        Glibx_Draw(x1 + 10, y1 + 5, x1 + 5, y1 + 10);
        Glibx_Draw(x1 + 10, y1 + 5, x1 + 25, y1 + 25);
    }

    if (MinMaxIncBox > 0)
    {
        switch (MinMaxIncBox)
        {
        case 1:
            DrawNumericBox(NumericX, NumericY, "Min");
            DrawKeyPad(NumericX, NumericY - 300);
            break;
        case 2:
            DrawNumericBox(NumericX, NumericY, "Max");
            DrawKeyPad(NumericX, NumericY - 300);
            break;
        case 3:
            DrawNumericBox(NumericX, NumericY, "Inc");
            DrawKeyPad(NumericX, NumericY - 300);
            break;
        }
    }
}

/* ---------------------------------- */
int GetKeyPad(int x, int y)
{
    int i;
    int x1, y1, x2, y2;

    for (i = 0; i <= 15; i = i + 1)
    {
        x1 = NumericX + KeyPad[i].x;
        y1 = NumericY - 300 + KeyPad[i].y;
        x2 = x1 + KeyPad[i].xs - 1;
        y2 = y1 + KeyPad[i].ys - 1;
        if ((x >= x1) && (x <= x2) && (y >= y1) && (y <= y2))
        {
            return KeyPad[i].val;
        }
    }
    return 0;
}

/* ---------------------------------- */
int GetAlphaPad(int x, int y)
{
    int i;
    int x1, y1, x2, y2;

    for (i = 0; i <= 52; i = i + 1)
    {
        x1 = AlphanumericX - 60 + AlphaPad[i].x;
        y1 = AlphanumericY - 250 + AlphaPad[i].y;
        x2 = x1 + AlphaPad[i].xs - 1;
        y2 = y1 + AlphaPad[i].ys - 1;
        if ((x >= x1) && (x <= x2) && (y >= y1) && (y <= y2))
        {
            return AlphaPad[i].val;
        }
    }
    return 0;
}

/* ---------------------------------- */
void DrawKeyPad(int kpx, int kpy)
{
    int i;
    int xt, yt;
    int w;

    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(kpx - 10, kpy - 10, 250, 310);

    for (i = 0; i <= 15; i = i + 1)
    {
        w = Glibx_StringSize(KeyPad[i].str);
        Glibx_Colour(Glibx_BLUE);
        Glibx_Rectangle(kpx + KeyPad[i].x, kpy + KeyPad[i].y, KeyPad[i].xs, KeyPad[i].ys);
        xt = kpx + KeyPad[i].x + KeyPad[i].xs / 2 - w / 2;
        yt = kpy + KeyPad[i].y + KeyPad[i].ys / 2 - 8;
        Glibx_Colour(Glibx_WHITE);
        Glibx_Chars(KeyPad[i].str, xt, yt);
    }
}

/* ---------------------------------- */
void DrawAlphaPad(int apx, int apy)
{
    int i;
    int xt, yt;
    int w;

    Glibx_Colour(Glibx_GREY);
    Glibx_Rectangle(apx - 10, apy - 10, 900, 250);

    for (i = 0; i <= 52; i = i + 1)
    {
        w = Glibx_StringSize(AlphaPad[i].str);
        Glibx_Colour(Glibx_BLUE);
        Glibx_Rectangle(apx + AlphaPad[i].x, apy + AlphaPad[i].y, AlphaPad[i].xs, AlphaPad[i].ys);
        xt = apx + AlphaPad[i].x + AlphaPad[i].xs / 2 - w / 2;
        yt = apy + AlphaPad[i].y + AlphaPad[i].ys / 2 - 8;
        Glibx_Colour(Glibx_WHITE);
        Glibx_Chars(AlphaPad[i].str, xt, yt);
    }
    if (caps)
    {
        Glibx_Colour(Glibx_GREEN);
    }
    else
    {
        Glibx_Colour(Glibx_BLACK);
    }
    Glibx_Char('o', AlphanumericX + 35, AlphanumericY + 60 - 250 + 5);
    Glibx_Colour(Glibx_WHITE);
}

/* ---------------------------------- */
void DrawButton(int n, char name[], int bcol, int fcol)
{
    int x, y, w;

    x = X0 + 77 + (n % 5) * 170;
    y = 600 - (n / 5) * 120;
    w = Glibx_StringSize(name);
    Glibx_Colour(bcol);
    Glibx_Rectangle(x, y, 150, 100);
    Glibx_Colour(fcol);
    x = x + 75 - w / 2;
    y = y + 50 - 8;
    Glibx_Chars(name, x, y);
}

/* ---------------------------------- */
int GetButton(int x, int y)
{
    int i;
    int x1, y1;
    int x2, y2;

    for (i = 0; i <= 24; i++)
    {
        x1 = X0 + 77 + (i % 5) * 170;
        y1 = 600 - (i / 5) * 120;
        x2 = x1 + 150 - 1;
        y2 = y1 + 100 - 1;
        if ((x >= x1) && (x <= x2) && (y >= y1) && (y <= y2))
        {
            return i;
        }
    }
    return -1;
}

/* ---------------------------------- */
int CheckFileList(int x, int y)
{
    int i;
    int x1, y1;
    int x2, y2;

    for (i = 0; i < nFiles; i++)
    {
        x1 = X0 + 77 + (i % 5) * 170;
        y1 = 600 - (i / 5) * 120;
        x2 = x1 + 150 - 1;
        y2 = y1 + 100 - 1;
        if ((x >= x1) && (x <= x2) && (y >= y1) && (y <= y2))
        {
            strcpy(Menus[MainOption].State[SubOption].Data.FileList.FileListName, ListOfFiles[i]);
            nFiles    = 0;
            MenuLevel = 0;
            return i;
        }
    }

    nFiles    = 0;
    MenuLevel = 0;
    return -1;
}

/* ---------------------------------- */
void LCDChars(char Str[], int x, int y)
{
    unsigned int p;
    char         Ch;

    p = 0;
    while (1)
    {
        Ch = Str[p];
        if (Ch == 0)
        {
            break;
        }
        else
        {
            if (Ch == '.')
            {
                LCDChar('.', x - 3, y);
                x = x + 10;
            }
            else
            {
                LCDChar(Ch, x, y);
                x = x + 24;
            }
            p = p + 1;
        }
    }
}

/* ---------------------------------- */
void LCDChar(char Ch, int x, int y)
{
    unsigned int Segments[10][8] = {
        { 1, 2, 3, 4, 5, 6, 0, 0 }, /* 0 */
        { 2, 3, 0, 0, 0, 0, 0, 0 }, /* 1 */
        { 1, 6, 7, 3, 4, 0, 0, 0 }, /* 2 */
        { 1, 2, 3, 4, 7, 0, 0, 0 }, /* 3 */
        { 2, 3, 5, 7, 0, 0, 0, 0 }, /* 4 */
        { 4, 5, 7, 2, 1, 0, 0, 0 }, /* 5 */
        { 4, 5, 1, 2, 6, 7, 0, 0 }, /* 6 */
        { 2, 3, 4, 0, 0, 0, 0, 0 }, /* 7 */
        { 1, 2, 3, 4, 5, 6, 7, 0 }, /* 8 */
        { 1, 2, 3, 4, 5, 7, 0, 0 }
    };                              /* 9 */

    unsigned int p;
    unsigned int i;
    unsigned int s;

    if (Ch == '.')
    {
        LED(x, y);
        return;
    }

    if (Ch == '-')
    {
        Segment(7, x, y);
        return;
    }

    if (Ch >= '0' && Ch <= '9')
    {
        p = Ch - '0';
        for (i = 0; i <= 8; i += 1)
        {
            s = Segments[p][i];
            if (s == 0)
            {
                return;
            }
            else
            {
                Segment(s, x, y);
            }
        }
    }
}

/* ---------------------------------- */
void LED(int x, int y)
{
    Glibx_Draw(x + 2, y, x + 5, y);
    Glibx_Draw(x + 1, y + 1, x + 6, y + 1);
    Glibx_Draw(x, y + 2, x + 7, y + 2);
    Glibx_Draw(x, y + 3, x + 7, y + 3);
    Glibx_Draw(x, y + 4, x + 7, y + 4);
    Glibx_Draw(x, y + 5, x + 7, y + 5);
    Glibx_Draw(x + 1, y + 6, x + 6, y + 6);
    Glibx_Draw(x + 2, y + 7, x + 5, y + 7);
}

/* ---------------------------------- */
void Segment(unsigned int n, int x, int y)
{
    int i;

    if (n == 7)
    {
        Glibx_Draw(x + 1, y + 15, x + 14, y + 15);
        Glibx_Draw(x + 2, y + 16, x + 13, y + 16);
        Glibx_Draw(x + 2, y + 14, x + 13, y + 14);
    }
    else
    {
        for (i = 0; i <= 2; i += 1)
        {
            switch (n)
            {
            case 1:
                Glibx_Draw(x + i + 1, y + i, x + 14 - i, y + i);
                break;
            case 2:
                Glibx_Draw(x + 15 - i, y + 1 + i, x + 15 - i, y + 14 - i);
                break;
            case 3:
                Glibx_Draw(x + 15 - i, y + 16 + i, x + 15 - i, y + 29 - i);
                break;
            case 4:
                Glibx_Draw(x + i + 1, y + 30 - i, x + 14 - i, y + 30 - i);
                break;
            case 5:
                Glibx_Draw(x + i, y + 16 + i, x + i, y + 29 - i);
                break;
            case 6:
                Glibx_Draw(x + i, y + 1 + i, x + i, y + 14 - i);
                break;
            }
        }
    }
}

/* ---------------------------------- */
void Gui_Mouse_Button_Callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        OldMouseLeftButton  = Gui_MouseLeftButton;
        Gui_MouseLeftButton = (action == GLFW_PRESS);
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        Gui_MouseRightButton = (action == GLFW_PRESS);
        if (action == GLFW_PRESS)
        {
            PanningX            = Gui_MouseX;
            PanningY            = Gui_MouseY;
            OldMouseRightButton = true;
        }
    }
}

/* ---------------------------------- */
void Gui_Mouse_Cursor_Callback(GLFWwindow* window, double xpos, double ypos)
{
    float Lat, Long;

    Gui_MouseX = (int) xpos;
    Gui_MouseY = Glibx_SCREENHEIGHT - (int) ypos;

    if (Gui_MouseRightButton)
    {
        Map_ScreenToGlobe((float) MapCentreX - (float) (Gui_MouseX - PanningX), (float) MapCentreY - (float) (Gui_MouseY - PanningY), &Lat, &Long);
        PanningX = Gui_MouseX;
        PanningY = Gui_MouseY;
        Map_SetMapCentre(Lat, Long);
    }
}

/* ---------------------------------- */
void Gui_Mouse_Scroll_Callback(GLFWwindow* window, double xpos, double ypos)
{
    Gui_MouseScrollFactor -= (float) ypos * 10.0f;
    if (Gui_MouseScrollFactor < 1.0)
    {
        Gui_MouseScrollFactor = 1.0;
    }
    Map_SetMapScaleFactor(Gui_MouseScrollFactor);
}

/* --------------------------------------------------- */
void Gui_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right, float *scrollfactor)
{
    *x            = Gui_MouseX;
    *y            = Gui_MouseY;
    *left         = Gui_MouseLeftButton;
    *middle       = Gui_MouseMiddleButton;
    *right        = Gui_MouseRightButton;
	*scrollfactor = Gui_MouseScrollFactor;
}

/* ---------------------------------- */
void BEGIN_Gui(void)
{
    Gui_MouseLeftButton   = false;
    Gui_MouseMiddleButton = false;
    Gui_MouseRightButton  = false;
    Gui_MouseScrollFactor = Map_Default_Scale_Factor;
}
