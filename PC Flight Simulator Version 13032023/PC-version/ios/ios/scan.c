/* ScanDir for Windows
   based on Microsoft TechNet article http://technet.microsoft.com/en-us/library/bb497011.aspx
   DJA 6 Sept 2008 */

#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include "scan.h"

static int CheckFileExtension(char[], char[]);

/* --------------------------- */
int ScanDir(char *dir, char ext[], string filenames[])
{
    DIR           *dp;
    struct dirent *dir_entry;
    int           nfiles = 0;

    if ((dp = opendir(dir)) == NULL)
    {
        printf("cannot open directory: %s\n", dir);
        return 0;
    }

    while ((dir_entry = readdir(dp)) != NULL)
    {
        if (CheckFileExtension(dir_entry->d_name, ext))
        {
            if (nfiles < 25)
            {
                strcpy(filenames[nfiles], dir_entry->d_name);
                nfiles = nfiles + 1;
            }
        }
    }

    closedir(dp);
    return nfiles;
}

/* --------------------------- */
static int CheckFileExtension(char FileName[], char Ext[])
{
    if (strstr(FileName, Ext) == NULL)
        return 0;
    else
        return 1;
}

void BEGIN_Scan()
{
}
