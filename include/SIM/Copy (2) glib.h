#ifndef Glib_H
#define Glib_H

#include <stdbool.h>

#include <GL/gl.h>

#define Glib_SCREENWIDTH 1920
#define Glib_SCREENHEIGHT 1080

#define Glib_BLACK 0
#define Glib_WHITE 1
#define Glib_BLUE 2
#define Glib_GREY 3
#define Glib_RED 4
#define Glib_GREEN 5
#define Glib_MAGENTA 6
#define Glib_BROWN 7
#define Glib_CYAN 8
#define Glib_YELLOW 9
#define Glib_ORANGE 10

#define Glib_GFONT8 1
#define Glib_GFONT12 2
#define Glib_GFONT16 3
#define Glib_GFONT24 4

#define Glib_ANCHOR_CENTRE   0
#define Glib_ANCHOR_LOWLEFT  1

extern void Glib_HBox(char str[], int x, int y);
extern void Glib_VBox(char str[], int x, int y);
extern void Glib_Char(char, int, int);
extern void Glib_Draw(int x1, int y1, int x2, int y2);
extern void Glib_Line_Strip(int x, int y, unsigned int n, int v[]);
extern void Glib_Line_Loop(int x, int y, unsigned int n, int v[]);
extern void Glib_Colour(unsigned int col);
extern void Glib_LineWidth(float w);
extern void Glib_Rectangle(int x1, int y1, int xside, int yside);
extern void Glib_Triangle(int x1, int y1, int x2, int y2, int x3, int y3);
extern void Glib_SetFont(unsigned int font, int spacing);
extern void Glib_Chars(char str[], int x, int y);
extern void Glib_AntiAliasing(bool Mode);
extern void Glib_ClipWindow(int x, int y, int xs, int ys);
extern void Glib_RemoveClipWindow(void);
extern void Glib_Flush();
extern void Glib_Init();
extern void Glib_LoadIdentity();
extern void Glib_Rotate(float a);
extern void Glib_Translate(float x, float y);

extern void BEGIN_Glib();

#endif
