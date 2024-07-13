#ifndef Glibx_H
#define Glibx_H

#include <stdbool.h>
#include <GL/gl.h>

#define Glibx_SCREENWIDTH  1920
#define Glibx_SCREENHEIGHT 1080

#define Glibx_BLACK 0
#define Glibx_WHITE 1
#define Glibx_BLUE 2
#define Glibx_GREY 3
#define Glibx_RED 4
#define Glibx_GREEN 5
#define Glibx_MAGENTA 6
#define Glibx_BROWN 7
#define Glibx_CYAN 8
#define Glibx_YELLOW 9

#define Glibx_GFONT10 1
#define Glibx_GFONT20 2

extern void Glibx_Window(float x1, float y1, float x2, float y2);
extern void Glibx_Char(char Ch, int x, int y);
extern void Glibx_Draw(int x1, int y1, int x2, int y2);
extern void Glibx_ClipWindow(int x, int y, int xs, int ys);
extern void Glibx_DrawLines(unsigned int n, int x, int y, int v[]);
extern void Glibx_Dot(int x, int y);
extern void Glibx_Colour(unsigned int col);
extern void Glibx_LineWidth(float w);
extern void Glibx_Rectangle(int x1, int y1, int xside, int yside);
extern void Glibx_Triangle(int x1, int y1, int x2, int y2, int x3, int y3);
extern void Glibx_SetFont(unsigned int font, int spacing);
extern void Glibx_Chars(char str[], int x, int y);
extern void Glibx_AntiAliasing(bool Mode);
extern int  Glibx_StringSize(char str[]);
extern void Glibx_Flush();
extern void Glibx_Init();
extern void Glibx_Close();
extern void BEGIN_Glibx();

#endif
