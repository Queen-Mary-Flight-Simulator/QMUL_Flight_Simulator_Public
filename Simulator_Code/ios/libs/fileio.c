/* +------------------------------+---------------------------------+
   | Module      : fileio.c       | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-03              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : file I/O functions                               |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <SIM/fileio.h>

#define CR 13

static unsigned int LineNumber;
static FILE         *FileStream;
static int          LastCh;

/* --------------------------------------------------------- */
int FileIO_Rdch()
{
    int ch;

    do
    {
		ch = fgetc(FileStream);
	} while (ch == CR);  /* ignore CR */
	
	LastCh = ch;
    if (ch == EOL)
    {
        LineNumber += 1;
    }

    return ch;
}

/* --------------------------------------------------------- */
void FileIO_SkipLine()
{
    int ch;

    do
    {
        ch = FileIO_Rdch();
        if (ch == EOF)
        {
            return;
        }
    } while (ch != EOL);
}

/* --------------------------------------------------------- */
void FileIO_BackSpace(int ch)
{
    if (ch == EOL)
    {
        LineNumber = LineNumber - 1;
    }
    ungetc(ch, FileStream);
}

/* --------------------------------------------------------- */
bool FileIO_EOF()
{
    return (LastCh == EOF);
}

/* --------------------------------------------------------- */
FILE *FileIO_Open(char *str)
{
    FILE *f = fopen(str, "r");
	return f;
}

/* --------------------------------------------------------- */
void FileIO_Close(FILE *f)
{
    fclose(f);
}

/* --------------------------------------------------------- */
void FileIO_Select(FILE *f)
{
    FileStream = f;
}

/* --------------------------------------------------------- */
int FileIO_LineNumber()
{
    return LineNumber;
}

/* --------------------------------------------------------- */
int FileIO_ReadString(char *str, char tch)  /* returns terminating character, tch not included in string */
{
    unsigned int p = 0;
    int          ch;

    do
    {
        ch = FileIO_Rdch();
    } while (ch == ' ');

    while (1)
    {
        if (ch == tch || ch == EOL || ch == EOF)
		{
		    break;
		}
        str[p] = (char) ch;
        p     += 1;
        ch     = FileIO_Rdch();
    }
    str[p] = '\0';
    return ch;
}

/* --------------------------------------------------------- */
int FileIO_ReadInt()
{
    int n = 0;
    int ch;

    do
    {
        ch = FileIO_Rdch();
    } while (ch == ' ');

    while (ch >= '0' && ch <= '9')
    {
        n  = n * 10 + ch - (int) '0';
        ch = FileIO_Rdch();
    }
    return n;
}

/* --------------------------------------------------------- */
float FileIO_ReadFloat()
{
    char str[100];

    FileIO_ReadString(str, ',');
    return (float) atof(str);
}

/* --------------------------------------------------------------------- */
void BEGIN_FileIO(void)
{
    LineNumber = 0;
	FileStream = NULL;
	LastCh = 0;
}
