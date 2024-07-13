#ifndef Script_H
#define Script_H

#include <stdbool.h>

extern bool Script_ScriptEnabled;
extern bool Script_ScriptError;
extern char Script_ScriptErrorMessage[200];
extern bool Script_ReadScriptFile(char FileName[]);
extern void Script_ExecuteScript(char FileName[]);
extern void Script_SaveScript(char FileName[]);
extern void Script_DisAssembleScript();
extern void BEGIN_Script();
#endif

