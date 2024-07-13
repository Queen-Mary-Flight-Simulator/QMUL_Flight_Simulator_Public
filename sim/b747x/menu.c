#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SIM/fileio.h>
#include <SIM/glib.h>

#include "menu.h"

Menu_MenuRecord   Menus[Menu_MaxNumberOfMenus];
Menu_MenuModeType MenuMode;
int               Menu_NumberOfMenus;
FILE              *FileStream;

void ReadPrompt(char a[]);
int ReadButtons(char a[], Menu_ButtonsListType b);
int ReadUnits(char a[], Menu_UnitsListType b);
void ReadMenuTitles();
void printmenus();

/* ---------------------------------- */
void ReadPrompt(char a[])
{
    FileIO_ReadString(a, '<');
    FileIO_ReadString(a, '>');
}

/* ----------------------------------------------------- */
int ReadButtons(char a[], Menu_ButtonsListType b)
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
int ReadUnits(char a[], Menu_UnitsListType b)
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

    Menu_NumberOfMenus = 0;
    do
    {
        tch                = FileIO_ReadString(Menus[Menu_NumberOfMenus].Title, ',');
        Menu_NumberOfMenus = Menu_NumberOfMenus + 1;
    } while (!(tch == EOL || tch == EOF));
}

/* ---------------------------------- */
void Menu_ReadMenus(void)
{
    int          ch;
    int          tch;
    char         PlotStr[]        = "Plot";
    int          MenuNum          = 0;
    int          Menu_MenuItemNum = 0;
    unsigned int j;
    char         a[51];

    FileStream = FileIO_Open("../files/menu.dat");
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
    //printf("NumberOfMenus=%d\n", Menu_NumberOfMenus);
	
    while (!FileIO_EOF())
    {
        //printf("MenuNum : %d Menu_MenuItemNum : %d  ", MenuNum, Menu_MenuItemNum);
        tch = FileIO_ReadString(Menus[MenuNum].State[Menu_MenuItemNum].Mname, '\\');
		//printf("tch: %c %d %d %s\n", tch, tch, (int) strlen(Menus[MenuNum].State[Menu_MenuItemNum].Mname),
        //                                     Menus[MenuNum].State[Menu_MenuItemNum].Mname);

        if (((tch == EOL) || (tch == EOF)) &&
            strlen(Menus[MenuNum].State[Menu_MenuItemNum].Mname) == 0)
        {
            Menus[MenuNum].NumberOfItems = Menu_MenuItemNum;
            //printf("NumberOfItems : %d\n", Menus[MenuNum].NumberOfItems);
            MenuNum          = MenuNum + 1;
            Menu_MenuItemNum = 0;
            //printf("\nNEW MENU\n");
            continue;
        }
        else if (((tch == EOL) || (tch == EOF)) &&
                 strlen(Menus[MenuNum].State[Menu_MenuItemNum].Mname) > 0)
        {
            //printf("\nBLAH : \n");
            continue;
        }
        else if (tch == '\\')
        {
            //printf("NEW MENU ITEM : %s\n", Menus[MenuNum].State[Menu_MenuItemNum].Mname );
            ch                                          = FileIO_Rdch();
            Menus[MenuNum].State[Menu_MenuItemNum].Mval = FileIO_ReadInt();
            //printf("%c\n", ch);

            switch (ch)
            {
            case 'n':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = Numeric;
                //printf("*");
                ReadPrompt(Menus[MenuNum].State[Menu_MenuItemNum].Mprompt);
                //printf("Numeric : %s\n", Menus[MenuNum].State[Menu_MenuItemNum].Mprompt);
                //printf("String a : %s\n", a);
                FileIO_ReadString(a, ' ');
                Menus[MenuNum].State[Menu_MenuItemNum].Data.Numeric.MinVal = atof(a);
                FileIO_ReadString(a, ' ');
                Menus[MenuNum].State[Menu_MenuItemNum].Data.Numeric.MaxVal = atof(a);
                FileIO_ReadString(a, EOL);
                Menus[MenuNum].State[Menu_MenuItemNum].Data.Numeric.Val = atof(a);
                //printf("MinVal : %f\n",Menus[MenuNum].State[Menu_MenuItemNum].Data.Numeric.MinVal);
                //printf("MaxVal : %f\n",Menus[MenuNum].State[Menu_MenuItemNum].Data.Numeric.MaxVal);
                //printf("Val : %f\n",Menus[MenuNum].State[Menu_MenuItemNum].Data.Numeric.Val);
                break;
            case 'q':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = Question;
                ReadPrompt(Menus[MenuNum].State[Menu_MenuItemNum].Mprompt);
                FileIO_SkipLine();
                break;
            case 'x':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = None;
                break;
            case 'o':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = Buttons;
                ReadPrompt(Menus[MenuNum].State[Menu_MenuItemNum].Mprompt);
                Menus[MenuNum].State[Menu_MenuItemNum].Data.Buttons.NumberOfButtons = ReadButtons(Menus[MenuNum].State[Menu_MenuItemNum].Mprompt, Menus[MenuNum].State[Menu_MenuItemNum].Data.Buttons.ButtonsList);
                Menus[MenuNum].State[Menu_MenuItemNum].Data.Buttons.ActiveButton    = 0;
                FileIO_SkipLine();
                /*
                   printf("Buttons : %s\n", Menus[MenuNum].State[Menu_MenuItemNum].Mprompt);
                   printf("# Buttons : %d\n", Menus[MenuNum].State[Menu_MenuItemNum].Data.Buttons.NumberOfButtons);
                   printf("Buttons : %s %s\n",
                       Menus[MenuNum].State[Menu_MenuItemNum].Data.Buttons.ButtonsList[0],
                       Menus[MenuNum].State[Menu_MenuItemNum].Data.Buttons.ButtonsList[1]);
                */
                break;
            case 'f':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = FileName;
                ReadPrompt(Menus[MenuNum].State[Menu_MenuItemNum].Mprompt);
                FileIO_SkipLine();
                break;
            case 'l':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = FileList;
                break;
            case 'c':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = Coordinates;
                break;
            case 'd':
                Menus[MenuNum].State[Menu_MenuItemNum].Info = FlightData;
                ReadPrompt(a);
                Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.NumberOfUnits =
                    ReadUnits(a, Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.UnitsList) + 1; /* allow extra plot button */
                //printf("# Units : %d\n", Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.NumberOfUnits);
                for (j = 0; j < strlen(PlotStr); j++)
                {
                    Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.UnitsList[Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.NumberOfUnits - 1][j] = PlotStr[j];
                }

                //printf("Units: %s %s %s\n",
                //       Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.UnitsList[0],
                //       Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.UnitsList[1],
                //       Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.UnitsList[2]);

                Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.Plotting    = 0;
                Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.ActiveUnits = 0;
                FileIO_ReadString(a, ' ');
                Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.ymin = atof(a);
                FileIO_ReadString(a, ' ');
                Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.ymax = atof(a);
                FileIO_ReadString(a, EOL);
                Menus[MenuNum].State[Menu_MenuItemNum].Data.FlightData.yinc = atof(a);
                break;
            default:
                Menus[MenuNum].State[Menu_MenuItemNum].Info = None;
                FileIO_SkipLine();
            }
            /*
               printf("%s %c %d %d\n",
               Menus[MenuNum].State[Menu_MenuItemNum].Mname,
               ch,
               Menus[MenuNum].State[Menu_MenuItemNum].Mval,
               Menus[MenuNum].State[Menu_MenuItemNum].Info);
            */
            Menu_MenuItemNum++;
        }
        else
        {
            Menus[MenuNum].State[Menu_MenuItemNum].Info = None;
        }
    }

    FileIO_Close(FileStream);
    //printmenus();

    if (MenuNum != Menu_NumberOfMenus)
    {
        printf("Fatal: %d main menus %d menus read\n", Menu_NumberOfMenus, MenuNum);
        exit(1);
    }
    printf("%d menus read\n", Menu_NumberOfMenus);
}

/* ---------------------------------- */
//int Menu_FindDefaultValue(char Str[], double *Variable) // return bool
int Menu_FindDefaultValue(char Str[], float *Variable)
{
    int i, j;

    for (i = 0; i < Menu_NumberOfMenus; i++)
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
void find_menu_data(int mv, int *mdrop, int *mitem)
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

void BEGIN_Menu(void)
{
}

