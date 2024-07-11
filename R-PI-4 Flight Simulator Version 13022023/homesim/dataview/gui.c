#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include <GLFW/glfw3.h>

#include <SIM/iosdefn.h>
#include <SIM/glib.h>
#include <SIM/fileio.h>

#include "link.h"
#include "scan.h"
#include "gui.h"

#define k_ENTER     200
#define k_BS        201
#define k_CANCEL    202
#define k_CLR       203
#define k_CAPS      204

#define LF          10
#define CR          13
#define EOL         10

#define X0          (Glib_SCREENWIDTH / 2)

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

GUI_MenuRecord Menus[GUI_MaxNumberOfMenus];
int            GUI_NumberOfMenus;
FILE           *FileStream;

int            Gui_MouseX;
int            Gui_MouseY;
bool           Gui_MouseLeftButton;
bool           Gui_MouseMiddleButton;
bool           Gui_MouseRightButton;
float          Gui_MouseScrollFactor;

static int     QuestionX     = X0 + 100;
static int     QuestionY     = 500;
static int     ButtonsX      = X0 + 100;
static int     ButtonsY      = 400;
static int     FlightDataX   = X0 + 100;
static int     FlightDataY   = 200;
static int     NumericX      = X0 + 500;
static int     NumericY      = 400;
static int     AlphanumericX = X0 + 100;
static int     AlphanumericY = 350;

static bool    OldMouseLeftButton  = false;
static bool    OldMouseRightButton = false;
static int     MenuLevel           = 1;
static int     MainOption          = 0;
static int     SubOption           = 0;
static char    NumericBuff[50];
static char    AlphanumericBuff[50];
static char    MinBuff[50];
static char    MaxBuff[50];
static char    IncBuff[50];
static int     CurrentButton = 0;
static string  ListOfFiles[50];
static int     nFiles        = 0;
static int     CurrentUnits  = 0;
static int     CurrentPlot   = 0;
static int     MinMaxIncBox  = 0;
static bool    MouseTracking = false;
static bool    MenuMoving = false;
static int     MenuX0, MenuY0;
static bool    ErrorPending = false;
static char    ErrorMessage[50];
static bool    caps = false;

void ReadPrompt(char a[]);
int ReadButtons(char a[], GUI_ButtonsListType b);
int ReadUnits(char a[], GUI_UnitsListType b);
void ReadMenuTitles();
void printmenus();

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
void DrawKeyPad(int x, int y);
void DrawAlphaPad(int x, int y);
void ShowDialogue();
void DrawNumericBox(int x, int y, char str[]);
void DrawAlphanumericBox(int x, int y, char str[]);
void DrawQuestionBox(int x, int y, char str[]);
void DrawButtonsBox(int x, int y, char str[]);
void DrawFlightDataBox(int x, int y, char str[]);
void DrawErrorBox(int x, int y);
void LCDChars(char Str[], int x, int y);
int  GetKeyPad(int x, int y);
int  GetAlphaPad(int x, int y);
void CheckFlightDataNumeric(int mx, int my);
void splittext(char name[], char str1[], char str2[]);
void DrawShadow(int x, int y, int xs, int ys);
void ReadMenus(void);

/* ---------------------------------- */
void Gui_CheckMouse(void)
{
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    bool leftkeypressed = Gui_MouseLeftButton && (!OldMouseLeftButton);

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
				if ((MainOption >= 0) && (MainOption < GUI_NumberOfMenus))
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
				m = &Menus[MainOption].State[SubOption];  /* reset menu pointer */
				if ((SubOption >= 0) && (SubOption < Menus[MainOption].NumberOfItems))
				{
					MenuLevel = 3;

					switch (m->Info)
					{
					case None:
						IosLink_Execute(m);
						MenuLevel = 0;
						break;

					case Question:
						break;

					case Numeric:
						sprintf(NumericBuff, "%g", m->Data.Numeric.Val);
						break;

					case Buttons:
						CurrentButton = m->Data.Buttons.ActiveButton;
						break;

					case FileName:
						AlphanumericBuff[0] = (char) 0;
						break;

					case FileList:
						switch (m->Mval)
						{
						    case IosDefn_LoadPlaybackFile:
							    nFiles = ScanDir(".", ".dat", ListOfFiles);
							    break;
							
						    default:
    							break;
						}
						break;

					case Coordinates:
						break;

					case FlightData:
						CurrentUnits = m->Data.FlightData.ActiveUnits;
						CurrentPlot  = m->Data.FlightData.Plotting;
						sprintf(MinBuff, "%g", m->Data.FlightData.ymin);
						sprintf(MaxBuff, "%g", m->Data.FlightData.ymax);
						sprintf(IncBuff, "%g", m->Data.FlightData.yinc);
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
			if (m->Info == Coordinates)
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
    /* not required for Dataview */
	return 0;
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
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int dy = m->Data.FlightData.NumberOfUnits * 40;
    int i;
    int x, y;

    if (MinMaxIncBox > 0)
    {
        CheckFlightDataNumeric(mx, my);
        return;
    }

    if ((mx >= FlightDataX) && (mx <= (FlightDataX + 100)) && (my >= FlightDataY) && (my <= (FlightDataY + 50)))
    {
        m->Data.FlightData.ActiveUnits = CurrentUnits;
        m->Data.FlightData.Plotting    = CurrentPlot;
        m->Data.FlightData.ymin        = atof(MinBuff);
        m->Data.FlightData.ymax        = atof(MaxBuff);
        m->Data.FlightData.yinc        = atof(IncBuff);
        MenuLevel                      = 0;
        IosLink_Execute(m);
        return;
    }
    else if ((mx >= (FlightDataX + 110)) && (mx <= (FlightDataX + 210)) && (my >= FlightDataY) && (my <= (FlightDataY + 50)))
    {
        MenuLevel = 0; /* cancel */
        return;
    }

    if (m->Data.FlightData.NumberOfUnits >= 3)
    {
        for (i = 0; i < m->Data.FlightData.NumberOfUnits - 1; i = i + 1)
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
        sprintf(NumericBuff, "%g", m->Data.FlightData.ymin);
        return;
    }
    else
    {
        y = y - 40;
        if ((mx >= x) && (mx <= (x + 130 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
        {
            MinMaxIncBox = 2;
            sprintf(NumericBuff, "%g", m->Data.FlightData.ymax);
            return;
        }
        else
        {
            y = y - 40;
            if ((mx >= x) && (mx <= (x + 130 - 1)) && (my >= y) && (my <= (y + 30 - 1)))
            {
                MinMaxIncBox = 3;
                sprintf(NumericBuff, "%g", m->Data.FlightData.yinc);
                return;
            }
        }
    }

    x = FlightDataX + 150;
    y = FlightDataY + 40 + dy - (m->Data.FlightData.NumberOfUnits - 1) * 40 - 5;
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
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int          dy = m->Data.Buttons.NumberOfButtons * 40;
    int          i;
    int          x, y;

    if ((mx >= ButtonsX) && (mx <= (ButtonsX + 100)) && (my >= ButtonsY) && (my <= (ButtonsY + 50)))
    {
        m->Data.Buttons.ActiveButton = CurrentButton;
        MenuLevel = 0;
        IosLink_Execute(m);
    }
    else if ((mx >= (ButtonsX + 110)) && (mx <= (ButtonsX + 210)) && (my >= ButtonsY) && (my <= (ButtonsY + 50)))
    {
        MenuLevel = 0; /* cancel */
    }

    for (i = 0; i < m->Data.Buttons.NumberOfButtons; i = i + 1)
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
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int          s   = strlen(NumericBuff);
    int          key = GetKeyPad(mx, my);
    float        x;
    int          x1, y1;

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
			if (x < m->Data.Numeric.MinVal)
			{
				ErrorPending = true;
				sprintf(ErrorMessage, "minimum value %g", m->Data.Numeric.MinVal);
			}
			else if (x > m->Data.Numeric.MaxVal)
			{
				ErrorPending = true;
				sprintf(ErrorMessage, "maximum value %g", m->Data.Numeric.MaxVal);
			}
			else
			{
				m->Data.Numeric.Val = x;
				MenuLevel = 0;
				IosLink_Execute(m);
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
				if (s <= 6)
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
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int           s   = strlen(AlphanumericBuff);
    int           key = GetAlphaPad(mx, my);

    switch (key)
    {
		case k_CLR:
			AlphanumericBuff[0] = (char) 0;
			break;

		case k_ENTER:
			MenuLevel = 0;
			strcpy(m->Data.Fname, AlphanumericBuff);
			IosLink_Execute(m);
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
    /* not required for Dataview */
}

/* ---------------------------------- */
void CheckQuery(int mx, int my)
{
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];

    if ((mx >= QuestionX) && (mx <= (QuestionX + 100)) && (my >= QuestionY) && (my <= (QuestionY + 50)))
    {
        MenuLevel = 0;
        IosLink_Execute(m);
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
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int           f = 0;

    switch (m->Info)
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
				IosLink_Execute(m);
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
    int          i;

    Glib_LineWidth(2.0);
    Glib_SetFont(Glib_GFONT18, 8);

    switch (MenuLevel)
    {
		case 0:
			break;

		case 1:
			for (i = 0; i <= GUI_NumberOfMenus - 1; i = i + 1)
			{
				DrawButton(i, Menus[i].Title, Glib_BLUE, Glib_WHITE);
			}
			break;

		case 2:
			for (i = 0; i <= Menus[MainOption].NumberOfItems - 1; i = i + 1)
			{
				DrawButton(i, Menus[MainOption].State[i].Mname, Glib_BLUE, Glib_WHITE);
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
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];

    Glib_SetFont(Glib_GFONT18, 8);
    
	switch (m->Info)
    {
		case None:
			break;
			
		case Question:
			DrawQuestionBox(QuestionX, QuestionY, m->Mprompt);
			break;
			
		case Numeric:
			DrawNumericBox(NumericX, NumericY, m->Mprompt);
			DrawKeyPad(NumericX, NumericY - 300);
			Glib_Colour(Glib_RED);
			LCDChars(NumericBuff, NumericX + 25, NumericY + 35);
			break;
			
		case Buttons:
			DrawButtonsBox(ButtonsX, ButtonsY, m->Mname);
			break;
			
		case FileName:
			DrawAlphanumericBox(AlphanumericX - 60, AlphanumericY, m->Mprompt);
			DrawAlphaPad(AlphanumericX - 60, AlphanumericY - 250);
			Glib_Chars(AlphanumericBuff, AlphanumericX + 25, AlphanumericY + 35);
			break;
			
		case FileList:
			DrawFileBoxes();
			break;
			
		case Coordinates:
			switch (m->Mval)
			{
                /* not required for Dataview */
			}
			break;
			
		case FlightData:
			DrawFlightDataBox(FlightDataX, FlightDataY, m->Mname);
			if (MinMaxIncBox > 0)
			{
				Glib_Colour(Glib_RED);
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
        DrawButton(i, ListOfFiles[i], Glib_BLUE, Glib_WHITE);
    }
}

/* ---------------------------------- */
void DrawNumericBox(int x, int y, char str[])
{
    DrawShadow(x - 10, y - 10, 250, 140);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(x - 10, y - 10, 250, 140);
    Glib_Colour(Glib_BLUE);
    Glib_Rectangle(x, y, 230, 120);
    Glib_Colour(Glib_BLACK);
    Glib_Rectangle(x + 15, y + 25, 200, 50);
    Glib_Colour(Glib_WHITE);
    Glib_Chars(str, x + 10, y + 95);
}

/* ---------------------------------- */
void DrawAlphanumericBox(int x, int y, char str[])
{
    DrawShadow(x - 10, y - 10, 900, 140);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(x - 10, y - 10, 900, 140);
    Glib_Colour(Glib_BLUE);
    Glib_Rectangle(x, y, 880, 120);
    Glib_Colour(Glib_BLACK);
    Glib_Rectangle(x + 15, y + 25, 850, 40);
    Glib_Colour(Glib_WHITE);
    Glib_Chars(str, x + 10, y + 95);
}

/* ---------------------------------- */
void DrawErrorBox(int x, int y)
{
    int w;

    DrawShadow(x - 10, y - 10, 250, 130);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(x - 10, y - 10, 250, 130);
    Glib_Colour(Glib_BLUE);
    Glib_Rectangle(x, y + 60, 230, 50);
    Glib_Colour(Glib_WHITE);
    Glib_Rectangle(x + 10, y + 70, 210, 30);
    Glib_Colour(Glib_RED);
    Glib_Rectangle(x + 65, y, 100, 50);
    Glib_SetFont(Glib_GFONT18, 10);

    w = Glib_StringSize(ErrorMessage);
    Glib_Chars(ErrorMessage, x + 115 - w / 2, y + 80);
    Glib_Colour(Glib_WHITE);
    w = Glib_StringSize("OK");
    Glib_Chars("OK", x + 115 - w / 2, y + 25 - 8);
}

/* ---------------------------------- */
void DrawQuestionBox(int x, int y, char str[])
{
    int w;

    DrawShadow(x - 10, y - 10, 230, 130);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(x - 10, y - 10, 230, 130);
    Glib_Colour(Glib_BLUE);
    Glib_Rectangle(x, y + 60, 210, 50);
    Glib_Rectangle(x, y, 100, 50);
    Glib_Rectangle(x + 110, y, 100, 50);
    Glib_Colour(Glib_WHITE);
    w = Glib_StringSize("OK");
    Glib_Chars("OK", x + 50 - w / 2, y + 25 - 8);
    w = Glib_StringSize("Cancel");
    Glib_Chars("Cancel", x + 110 + 50 - w / 2, y + 25 - 8);
    Glib_Chars(str, x + 10, y + 75);
}

/* ---------------------------------- */
void DrawButtonsBox(int x, int y, char str[])
{
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int          dy = m->Data.Buttons.NumberOfButtons * 40;
    int          i;
    int          x1, y1;
    int          w;

    DrawShadow(x - 10, y - 10, 230, 130 + dy);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(x - 10, y - 10, 230, 130 + dy);
    Glib_Colour(Glib_BLUE);
    Glib_Rectangle(x, y + 60, 210, 50 + dy);
    Glib_Rectangle(x, y, 100, 50);
    Glib_Rectangle(x + 110, y, 100, 50);
    Glib_Colour(Glib_WHITE);
    w = Glib_StringSize("OK");
    Glib_Chars("OK", x + 50 - w / 2, y + 25 - 8);
    w = Glib_StringSize("Cancel");
    Glib_Chars("Cancel", x + 110 + 50 - w / 2, y + 25 - 8);
    Glib_Chars(str, x + 10, y + 75 + dy);

    for (i = 0; i < m->Data.Buttons.NumberOfButtons; i = i + 1)
    {
        Glib_Colour(Glib_WHITE);
        Glib_Chars(m->Data.Buttons.ButtonsList[i], x + 30, y + 40 + dy - i * 40);
        Glib_Colour(Glib_BLACK);
        Glib_Rectangle(x + 150, y + 40 + dy - i * 40 - 5, 30, 30);
    }

    Glib_Colour(Glib_GREEN);  /* was white */
    x1 = x + 150;
    y1 = y + 40 + dy - CurrentButton * 40 - 5;
	Glib_LineWidth(3.0);	/* will also force flush */
    Glib_Draw(x1 + 10, y1 + 5, x1 + 5, y1 + 10);
    Glib_Draw(x1 + 10, y1 + 5, x1 + 25, y1 + 25);
	Glib_LineWidth(1.0);
}

/* ---------------------------------- */
void DrawFlightDataBox(int x, int y, char str[])
{
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int          dy = m->Data.FlightData.NumberOfUnits * 40;
    int          i;
    int          x1, y1;
    int          w;

    DrawShadow(x - 10, y - 10, 230, 130 + dy + 120);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(x - 10, y - 10, 230, 130 + dy + 120);
    Glib_Colour(Glib_BLUE);
    Glib_Rectangle(x, y + 60, 210, 50 + dy + 120);
    Glib_Rectangle(x, y, 100, 50);
    Glib_Rectangle(x + 110, y, 100, 50);
    Glib_Colour(Glib_WHITE);
    w = Glib_StringSize("OK");
    Glib_Chars("OK", x + 50 - w / 2, y + 25 - 8);
    w = Glib_StringSize("Cancel");
    Glib_Chars("Cancel", x + 110 + 50 - w / 2, y + 25 - 8);
    Glib_Chars(str, x + 10, y + 75 + dy + 130);

    for (i = 0; i < m->Data.FlightData.NumberOfUnits; i = i + 1)
    {
        Glib_Colour(Glib_WHITE);
        Glib_Chars(m->Data.FlightData.UnitsList[i], x + 30, y + 40 + dy - i * 40);
        Glib_Colour(Glib_BLACK);
        Glib_Rectangle(x + 150, y + 40 + dy - i * 40 - 5, 30, 30);
    }

    Glib_Colour(Glib_WHITE);
    Glib_Chars("Min", x + 20, y + 75 + dy + 130 - 40);
    Glib_Chars("Max", x + 20, y + 75 + dy + 130 - 80);
    Glib_Chars("Inc", x + 20, y + 75 + dy + 130 - 120);
    Glib_Colour(Glib_BLACK);
    Glib_Rectangle(x + 70, y + 75 + dy + 130 - 45, 130, 30);
    Glib_Rectangle(x + 70, y + 75 + dy + 130 - 85, 130, 30);
    Glib_Rectangle(x + 70, y + 75 + dy + 130 - 125, 130, 30);

    Glib_Colour(Glib_WHITE);
    Glib_Chars(MinBuff, x + 80, y + 75 + dy + 130 - 45 + 5);
    Glib_Chars(MaxBuff, x + 80, y + 75 + dy + 130 - 85 + 5);
    Glib_Chars(IncBuff, x + 80, y + 75 + dy + 130 - 125 + 5);

    if (m->Data.FlightData.NumberOfUnits >= 2)
    {
        Glib_Colour(Glib_GREEN);
        x1 = x + 150;
        y1 = y + 40 + dy - CurrentUnits * 40 - 5;
		Glib_LineWidth(3.0);	/* will also force flush */	
        Glib_Draw(x1 + 10, y1 + 5, x1 + 5, y1 + 10);
        Glib_Draw(x1 + 10, y1 + 5, x1 + 25, y1 + 25);
		Glib_LineWidth(1.0);		
    }

    if (CurrentPlot)
    {
        Glib_Colour(Glib_GREEN);
        x1 = x + 150;
        y1 = y + 40 + dy - (m->Data.FlightData.NumberOfUnits - 1) * 40 - 5;
		Glib_LineWidth(3.0);	/* will also force flush */	
        Glib_Draw(x1 + 10, y1 + 5, x1 + 5, y1 + 10);
        Glib_Draw(x1 + 10, y1 + 5, x1 + 25, y1 + 25);
		Glib_LineWidth(1.0);		
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

    DrawShadow(kpx - 10, kpy - 10, 250, 310);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(kpx - 10, kpy - 10, 250, 310);

    for (i = 0; i <= 15; i = i + 1)
    {
        w = Glib_StringSize(KeyPad[i].str);
        Glib_Colour(Glib_BLUE);
        Glib_Rectangle(kpx + KeyPad[i].x, kpy + KeyPad[i].y, KeyPad[i].xs, KeyPad[i].ys);
        xt = kpx + KeyPad[i].x + KeyPad[i].xs / 2 - w / 2;
        yt = kpy + KeyPad[i].y + KeyPad[i].ys / 2 - 8;
        Glib_Colour(Glib_WHITE);
        Glib_Chars(KeyPad[i].str, xt, yt);
    }
}

/* ---------------------------------- */
void DrawAlphaPad(int apx, int apy)
{
    int i;
    int xt, yt;
    int w;

    DrawShadow(apx - 10, apy - 10, 900, 250);
    Glib_Colour(Glib_GREY);
    Glib_Rectangle(apx - 10, apy - 10, 900, 250);

    for (i = 0; i <= 52; i = i + 1)
    {
        w = Glib_StringSize(AlphaPad[i].str);
        Glib_Colour(Glib_BLUE);
        Glib_Rectangle(apx + AlphaPad[i].x, apy + AlphaPad[i].y, AlphaPad[i].xs, AlphaPad[i].ys);
        xt = apx + AlphaPad[i].x + AlphaPad[i].xs / 2 - w / 2;
        yt = apy + AlphaPad[i].y + AlphaPad[i].ys / 2 - 8;
        Glib_Colour(Glib_WHITE);
        Glib_Chars(AlphaPad[i].str, xt, yt);
    }
    if (caps)
    {
        Glib_Colour(Glib_GREEN);
    }
    else
    {
        Glib_Colour(Glib_BLACK);
    }
    Glib_Char('o', AlphanumericX + 35, AlphanumericY + 60 - 250 + 5);
    Glib_Colour(Glib_WHITE);
}

/* ---------------------------------- */
void splittext(char name[], char str1[], char str2[])
{
	int s = strlen(name);  /* length of string */
	int m = s / 2;         /* mid-point of string */
	int b = s;             /* break-point of string */
	int i;
	
	for(i=0; i<s; i+=1)
	{
	    char ch = name[i];
		if (ch == ' ')
		{
		    if (abs(i - m) < b)
			{
			    b = i;
			}
		}
	}
	
	for (i=0; i<=b; i+=1)
	{
	    str1[i] = name[i];
	}
	str1[b+1] = '\0';
	
	for (i=b+1; i<=s; i+=1)
	{
	    str2[i-b-1] = name[i];
	}
}

/* ---------------------------------- */
void DrawShadow(int x, int y, int xs, int ys)
{
    const int w = 10;

	return; // ***
	Glib_Colour(Glib_DARKGREY);
	Glib_Rectangle(x+w, y-w, xs, ys);
}

/* ---------------------------------- */
void DrawButton(int n, char name[], int bcol, int fcol)
{
    int x, y, w;
    char str1[100];
	char str2[100];
	bool singlestring = true;
	
    x = X0 + 77 + (n % 5) * 170;
    y = 600 - (n / 5) * 120;

    if (Glib_StringSize(name) > 150)
	{
	    splittext(name, str1, str2);
		singlestring = false;
	}
    w = Glib_StringSize(name);
	DrawShadow(x, y, 150, 100);
    Glib_Colour(bcol);
    Glib_Rectangle(x, y, 150, 100);
    Glib_Colour(fcol);
	if (singlestring)
	{
        Glib_Chars(name, x + 75 - w / 2, y + 50 - 8);
	}
	else
	{
	    int w1 = Glib_StringSize(str1);
		int w2 = Glib_StringSize(str2);
        Glib_Chars(str1, x + 75 - w1 / 2, y + 50 + 8);
        Glib_Chars(str2, x + 75 - w2 / 2, y + 50 - 19);
	}
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
    GUI_MenuItem *m = &Menus[MainOption].State[SubOption];
    int          i;
    int          x1, y1;
    int          x2, y2;

    for (i = 0; i < nFiles; i++)
    {
        x1 = X0 + 77 + (i % 5) * 170;
        y1 = 600 - (i / 5) * 120;
        x2 = x1 + 150 - 1;
        y2 = y1 + 100 - 1;
        if ((x >= x1) && (x <= x2) && (y >= y1) && (y <= y2))
        {
            strcpy(m->Data.FileList.FileListName, ListOfFiles[i]);
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
    Glib_SetFont(Glib_LFONT30, 10);
    Glib_Chars(Str, x, y);
}

/* ---------------------------------- */
void Gui_Mouse_Button_Callback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        OldMouseLeftButton  = Gui_MouseLeftButton;
        Gui_MouseLeftButton = (action == GLFW_PRESS);
    }
}

/* ---------------------------------- */
void Gui_Mouse_Cursor_Callback(GLFWwindow* window, double xpos, double ypos)
{
    Gui_MouseX = (int) xpos;
    Gui_MouseY = Glib_SCREENHEIGHT - (int) ypos;
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
void ReadPrompt(char a[])
{
    FileIO_ReadString(a, '<');
    FileIO_ReadString(a, '>');
}

/* ----------------------------------------------------- */
int ReadButtons(char a[], GUI_ButtonsListType b)
{
    int i, j, p;

    i = 0;
    j = 0;
    p = 0;
    while (1)
    {
        b[i][j] = a[p];
        if (a[p] == (char) 0)
        {
            return i + 1;
        }
        if (a[p] == ' ')
        {
            b[i][j] = (char) 0;
            i       = i + 1;
            j       = 0;
        }
        else
        {
            j = j + 1;
        }
        p = p + 1;
    }
}

/* ----------------------------------------------------- */
int ReadUnits(char a[], GUI_UnitsListType b)
{
    int i, j, p;

    i = 0;
    j = 0;
    p = 0;
    if (a[0] == (char) 0)
    {
        return 0;
    }
    while (1)
    {
        b[i][j] = a[p];
        if (a[p] == (char) 0)
        {
            return i + 1;
        }
        if (a[p] == ' ')
        {
            b[i][j] = (char) 0;
            i       = i + 1;
            j       = 0;
        }
        else
        {
            j = j + 1;
        }
        p = p + 1;
    }
}

/* ---------------------------------- */
void ReadMenuTitles()
{
    int tch;

    GUI_NumberOfMenus = 0;
    do
    {
        tch = FileIO_ReadString(Menus[GUI_NumberOfMenus].Title, ',');
        GUI_NumberOfMenus = GUI_NumberOfMenus + 1;
    } while (!(tch == EOL || tch == EOF));
}

/* ---------------------------------- */
void ReadMenus(void)
{
    int          ch;
    int          tch;
    char         PlotStr[]        = "Plot";
    int          MenuNum          = 0;
    int          GUI_MenuItemNum = 0;
    unsigned int j;
    char         a[51];

    FileStream = FileIO_Open("../files/fdr.dat");
    if (FileStream == NULL)
    {
        printf("Can't open menu file\n");
        exit(1);
    }

    FileIO_Select(FileStream);
	
    FileIO_SkipLine();
    FileIO_SkipLine();
    ReadMenuTitles();
    FileIO_SkipLine();
    //printf("NumberOfMenus=%d\n", GUI_NumberOfMenus);
	
    while (!FileIO_EOF())
    {
        //printf("MenuNum : %d GUI_MenuItemNum : %d  ", MenuNum, GUI_MenuItemNum);
        tch = FileIO_ReadString(Menus[MenuNum].State[GUI_MenuItemNum].Mname, '\\');
		//printf("tch: %c %d %d %s\n", tch, tch, (int) strlen(Menus[MenuNum].State[GUI_MenuItemNum].Mname),
        //                                     Menus[MenuNum].State[GUI_MenuItemNum].Mname);

        if (((tch == EOL) || (tch == EOF)) &&
            strlen(Menus[MenuNum].State[GUI_MenuItemNum].Mname) == 0)
        {
            Menus[MenuNum].NumberOfItems = GUI_MenuItemNum;
            //printf("NumberOfItems : %d\n", Menus[MenuNum].NumberOfItems);
            MenuNum          = MenuNum + 1;
            GUI_MenuItemNum = 0;
            //printf("\nNEW MENU\n");
            continue;
        }
        else if (((tch == EOL) || (tch == EOF)) &&
                 strlen(Menus[MenuNum].State[GUI_MenuItemNum].Mname) > 0)
        {
            //printf("\nBLAH : \n");
            continue;
        }
        else if (tch == '\\')
        {
            //printf("NEW MENU ITEM : %s\n", Menus[MenuNum].State[GUI_MenuItemNum].Mname );
            ch                                          = FileIO_Rdch();
            Menus[MenuNum].State[GUI_MenuItemNum].Mval = FileIO_ReadInt();
            //printf("%c\n", ch);

            switch (ch)
            {
				case 'n':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = Numeric;
					//printf("*");
					ReadPrompt(Menus[MenuNum].State[GUI_MenuItemNum].Mprompt);
					//printf("Numeric : %s\n", Menus[MenuNum].State[GUI_MenuItemNum].Mprompt);
					//printf("String a : %s\n", a);
					FileIO_ReadString(a, ' ');
					Menus[MenuNum].State[GUI_MenuItemNum].Data.Numeric.MinVal = atof(a);
					FileIO_ReadString(a, ' ');
					Menus[MenuNum].State[GUI_MenuItemNum].Data.Numeric.MaxVal = atof(a);
					FileIO_ReadString(a, EOL);
					Menus[MenuNum].State[GUI_MenuItemNum].Data.Numeric.Val = atof(a);
					//printf("MinVal : %f\n",Menus[MenuNum].State[GUI_MenuItemNum].Data.Numeric.MinVal);
					//printf("MaxVal : %f\n",Menus[MenuNum].State[GUI_MenuItemNum].Data.Numeric.MaxVal);
					//printf("Val : %f\n",Menus[MenuNum].State[GUI_MenuItemNum].Data.Numeric.Val);
					break;
					
				case 'q':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = Question;
					ReadPrompt(Menus[MenuNum].State[GUI_MenuItemNum].Mprompt);
					FileIO_SkipLine();
					break;
					
				case 'x':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = None;
					break;
					
				case 'o':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = Buttons;
					ReadPrompt(Menus[MenuNum].State[GUI_MenuItemNum].Mprompt);
					Menus[MenuNum].State[GUI_MenuItemNum].Data.Buttons.NumberOfButtons = ReadButtons(Menus[MenuNum].State[GUI_MenuItemNum].Mprompt, Menus[MenuNum].State[GUI_MenuItemNum].Data.Buttons.ButtonsList);
					Menus[MenuNum].State[GUI_MenuItemNum].Data.Buttons.ActiveButton    = 0;
					FileIO_SkipLine();
					/*
					   printf("Buttons : %s\n", Menus[MenuNum].State[GUI_MenuItemNum].Mprompt);
					   printf("# Buttons : %d\n", Menus[MenuNum].State[GUI_MenuItemNum].Data.Buttons.NumberOfButtons);
					   printf("Buttons : %s %s\n",
						   Menus[MenuNum].State[GUI_MenuItemNum].Data.Buttons.ButtonsList[0],
						   Menus[MenuNum].State[GUI_MenuItemNum].Data.Buttons.ButtonsList[1]);
					*/
					break;
					
				case 'f':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = FileName;
					ReadPrompt(Menus[MenuNum].State[GUI_MenuItemNum].Mprompt);
					FileIO_SkipLine();
					break;
					
				case 'l':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = FileList;
					break;
					
				case 'c':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = Coordinates;
					break;
					
				case 'd':
					Menus[MenuNum].State[GUI_MenuItemNum].Info = FlightData;
					ReadPrompt(a);
					Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.NumberOfUnits =
						ReadUnits(a, Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.UnitsList) + 1; /* allow extra plot button */
					//printf("# Units : %d\n", Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.NumberOfUnits);
					for (j = 0; j < strlen(PlotStr); j++)
					{
						Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.UnitsList[Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.NumberOfUnits - 1][j] = PlotStr[j];
					}

					//printf("Units: %s %s %s\n",
					//       Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.UnitsList[0],
					//       Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.UnitsList[1],
					//       Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.UnitsList[2]);

					Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.Plotting    = 0;
					Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.ActiveUnits = 0;
					FileIO_ReadString(a, ' ');
					Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.ymin = atof(a);
					FileIO_ReadString(a, ' ');
					Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.ymax = atof(a);
					FileIO_ReadString(a, EOL);
					Menus[MenuNum].State[GUI_MenuItemNum].Data.FlightData.yinc = atof(a);
					break;
					
				default:
					Menus[MenuNum].State[GUI_MenuItemNum].Info = None;
					FileIO_SkipLine();
					break;
            }
            /*
               printf("%s %c %d %d\n",
               Menus[MenuNum].State[GUI_MenuItemNum].Mname,
               ch,
               Menus[MenuNum].State[GUI_MenuItemNum].Mval,
               Menus[MenuNum].State[GUI_MenuItemNum].Info);
            */
            GUI_MenuItemNum += 1;
        }
        else
        {
            Menus[MenuNum].State[GUI_MenuItemNum].Info = None;
        }
    }

    FileIO_Close(FileStream);
    //printmenus();

    if (MenuNum != GUI_NumberOfMenus)
    {
        printf("Fatal: %d main menus %d menus read\n", GUI_NumberOfMenus, MenuNum);
        exit(1);
    }
    printf("%d menus read\n", GUI_NumberOfMenus);
}

/* ---------------------------------- */
//int GUI_FindDefaultValue(char Str[], double *Variable) // return bool
int GUI_FindDefaultValue(char Str[], float *Variable)
{
    int i, j;

    for (i = 0; i < GUI_NumberOfMenus; i++)
    {
        for (j = 0; j < Menus[i].NumberOfItems; j++)
        {
            if (strcmp(Str, Menus[i].State[j].Mname) == 0)
            {
                //*Variable = Menus[i].State[j].Mval;  // ?? M2 says Val , but this is Numerical dialog specific!
                *Variable = Menus[i].State[j].Data.Numeric.Val;
                return 1; // true
            }
        }
    }
    return 0; // false
}

/* ---------------------------------- */
void find_GUI_data(int mv, int *mdrop, int *mitem)
{
    int i, j;
    for (j = 0; j < 11; j++)
    {
        for (i = 0; i < Menus[j].NumberOfItems; i++)
        {
            if (mv == Menus[j].State[i].Mval)
            {
                *mdrop = j;
                *mitem = i;
                return;
            }
        }
    }
}

/* ---------------------------------- */
void printmenus()
{
    int i, j;

    for (i = 0; i <= 10; i++)
    {
        printf("Menu %d: title=%s items=%d\n", i, Menus[i].Title, Menus[i].NumberOfItems);
        for (j = 0; j <= Menus[i].NumberOfItems - 1; j++)
        {
            printf("%s %d %s\n", Menus[i].State[j].Mname,
                   Menus[i].State[j].Mval, Menus[i].State[j].Mprompt);
        }
    }
}

/* ---------------------------------- */
void BEGIN_Gui(void)
{
    Gui_MouseLeftButton   = false;
    Gui_MouseMiddleButton = false;
    Gui_MouseRightButton  = false;
	
	ReadMenus();
}
