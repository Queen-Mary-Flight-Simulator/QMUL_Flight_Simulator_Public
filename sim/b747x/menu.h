/*
    Conversion of menu.def
*/

#ifndef Menu_H
#define Menu_H

#include <stdio.h>

#define Menu_MaxNumberOfMenus     25  /* number of menus supported */
#define Menu_MenuSize             25  /* number of entries in a menu */
#define Menu_MnameStringSize      20  /* width (chars) of each menu item */
#define Menu_MpromptStringSize    30  /* width (chars) of each menu prompt */
#define Menu_ButtonsListSize      10  /* number of buttons */
#define Menu_ButtonsStringSize    16  /* width (chars) of button items */
#define Menu_FileListSize         20  /* number of files in a file list */
#define Menu_FileListStringSize   128  /* width (chars) of a complete file path */
#define Menu_FilenameStringSize   128  /* width (chars) of a complete file path */
#define Menu_UnitsListSize        5   /* number of units in a flight data menu */
#define Menu_UnitsStringSize      10  /* width (chars) of a units item */

typedef enum {
    Menu_Map, Menu_Approach, Menu_Data
} Menu_MenuModeType;

typedef enum {
    None, Question, Numeric, Buttons, FileName, FileList, Coordinates, FlightData
} Menu_MenuType;

/*
typedef enum {
    On, Off
} MenuHighlight;
*/

typedef char                Menu_MnameString[Menu_MnameStringSize];
typedef char                Menu_MpromptString[Menu_MpromptStringSize];
typedef char                Menu_FileListString[Menu_FileListStringSize];
typedef Menu_FileListString Menu_FileListType[Menu_FileListSize];
typedef char                Menu_FnameString[Menu_FilenameStringSize];
typedef char                Menu_ButtonsString[Menu_ButtonsStringSize];
typedef Menu_ButtonsString  Menu_ButtonsListType[Menu_ButtonsListSize];
typedef char                Menu_UnitsString[Menu_UnitsStringSize];
typedef Menu_UnitsString    Menu_UnitsListType[Menu_UnitsListSize];

typedef struct {
    int                  NumberOfButtons;
    Menu_ButtonsListType ButtonsList;
    int                  ActiveButton;
} Buttons_t;

typedef struct {
    float MinVal;
    float MaxVal;
    float Val;
} Numeric_t;

typedef struct {
    int                 NumberOfFiles;
    Menu_FileListString FileListName;
} FileList_t;

typedef struct {
    int                NumberOfUnits;
    int                ActiveUnits;
    Menu_UnitsListType UnitsList;
    int                Plotting; /* bool */
    float              ymin;
    float              ymax;
    float              yinc;
} FlightData_t;

typedef union {
    Buttons_t         Buttons;
    Numeric_t         Numeric;
    FileList_t        FileList;
    Menu_FnameString  Fname;
    FlightData_t      FlightData;
} Data_t;

typedef struct {
    Menu_MnameString   Mname;
    int                Mval;
    Menu_MpromptString Mprompt;
    Menu_MenuType      Info;
    Data_t             Data;
} Menu_MenuItem;

typedef struct {
    Menu_MenuItem State[Menu_MenuSize];
    int           NumberOfItems;
    char          Title[32];
} Menu_MenuRecord;

extern int Menu_NumberOfMenus;

extern Menu_MenuRecord Menus[Menu_MaxNumberOfMenus];
extern void Menu_ReadMenus(void);
extern int  Menu_FindDefaultValue(char Str[], float *Variable);
extern void BEGIN_Menu(void);

#endif
