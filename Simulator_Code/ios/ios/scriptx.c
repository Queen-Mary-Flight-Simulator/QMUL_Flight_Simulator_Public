#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "script.h"

int main(int argc, char *argv[])
{
    BEGIN_Script();

    if (argc > 1)
    {
        if (Script_ReadScriptFile(argv[1]) == false)
        {
            printf("Cannot find %s\n", argv[1]);
            return 0;
        }
    }

    if (argc > 2)
    {
        if ((strcmp(argv[2], "DIS") == 0) || (strcmp(argv[2], "dis") == 0))
        {
            Script_DisAssembleScript();
        }
    }

    if (Script_ScriptError == true)
    {
        printf("%s\n", Script_ScriptErrorMessage);
    }

    return 0;
}
