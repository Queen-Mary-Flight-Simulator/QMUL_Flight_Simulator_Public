#ifndef GUI_H
#define GUI_H

#include <GLFW/glfw3.h>

#define GUI_MaxNumberOfMenus     25  /* number of menus supported */
#define GUI_MenuSize             25  /* number of entries in a menu */
#define GUI_MnameStringSize      20  /* width (chars) of each menu item */
#define GUI_MpromptStringSize    30  /* width (chars) of each menu prompt */
#define GUI_ButtonsListSize      10  /* number of buttons */
#define GUI_ButtonsStringSize    16  /* width (chars) of button items */
#define GUI_FileListSize         20  /* number of files in a file list */
#define GUI_FileListStringSize   128  /* width (chars) of a complete file path */
#define GUI_FilenameStringSize   128  /* width (chars) of a complete file path */
#define GUI_UnitsListSize        5   /* number of units in a flight data menu */
#define GUI_UnitsStringSize      10  /* width (chars) of a units item */

typedef enum 
{
    None, Question, Numeric, Buttons, FileName, FileList, Coordinates, FlightData
} GUI_MenuType;

typedef char                GUI_MnameString[GUI_MnameStringSize];
typedef char                GUI_MpromptString[GUI_MpromptStringSize];
typedef char                GUI_FileListString[GUI_FileListStringSize];
typedef GUI_FileListString  GUI_FileListType[GUI_FileListSize];
typedef char                GUI_FnameString[GUI_FilenameStringSize];
typedef char                GUI_ButtonsString[GUI_ButtonsStringSize];
typedef GUI_ButtonsString   GUI_ButtonsListType[GUI_ButtonsListSize];
typedef char                GUI_UnitsString[GUI_UnitsStringSize];
typedef GUI_UnitsString     GUI_UnitsListType[GUI_UnitsListSize];

typedef struct 
{
    int                 NumberOfButtons;
    GUI_ButtonsListType ButtonsList;
    int                 ActiveButton;
} Buttons_t;

typedef struct 
{
    float MinVal;
    float MaxVal;
    float Val;
} Numeric_t;

typedef struct 
{
    int                 NumberOfFiles;
    GUI_FileListString  FileListName;
} FileList_t;

typedef struct
{
    int                 NumberOfUnits;
    int                 ActiveUnits;
    GUI_UnitsListType   UnitsList;
    int                 Plotting; /* bool */
    float               ymin;
    float               ymax;
    float               yinc;
} FlightData_t;

typedef union 
{
    Buttons_t           Buttons;
    Numeric_t           Numeric;
    FileList_t          FileList;
    GUI_FnameString     Fname;
    FlightData_t        FlightData;
} Data_t;

typedef struct {
    GUI_MnameString     Mname;
    int                 Mval;
    GUI_MpromptString   Mprompt;
    GUI_MenuType        Info;
    Data_t              Data;
} GUI_MenuItem;

typedef struct {
    GUI_MenuItem        State[GUI_MenuSize];
    int                 NumberOfItems;
    char                Title[32];
} GUI_MenuRecord;

extern int GUI_NumberOfMenus;

extern int Gui_MouseX;
extern int Gui_MouseY;
extern bool Gui_MouseLeftButton;
extern bool Gui_MouseRightButton;
extern float Gui_Mouse_Scroll;

void Gui_Mouse_Button_Callback(GLFWwindow* window, int button, int action, int mods);
void Gui_Mouse_Cursor_Callback(GLFWwindow* window, double xpos, double ypos);
void Gui_Mouse_Scroll_Callback(GLFWwindow* window, double xpos, double ypos);
void Gui_CheckMouse(void);
void Gui_GetMouse(int *x, int *y, bool *left, bool *middle, bool *right, float *scrollfactor);
void Gui_Menu(void);

extern GUI_MenuRecord Menus[GUI_MaxNumberOfMenus];
extern int  GUI_FindDefaultValue(char Str[], float *Variable);
extern void BEGIN_Menu(void);

void BEGIN_Gui(void);

#endif
