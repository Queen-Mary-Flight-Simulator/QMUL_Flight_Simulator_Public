#ifndef FileIO_H
#define FileIO_H

#include <stdlib.h>
#include <stdbool.h>

#define EOL  10

extern int   FileIO_Rdch();
extern void  FileIO_SkipLine();
extern void  FileIO_BackSpace(int ch);
extern bool  FileIO_EOF();
extern FILE  *FileIO_Open(char *str);
extern void  FileIO_Close(FILE *f);
extern void  FileIO_Select(FILE *f);
extern int   FileIO_LineNumber();
extern float FileIO_ReadFloat();
extern int   FileIO_ReadInt();
extern int   FileIO_ReadString(char *str, char tch);
extern void  BEGIN_FileIO();

#endif
