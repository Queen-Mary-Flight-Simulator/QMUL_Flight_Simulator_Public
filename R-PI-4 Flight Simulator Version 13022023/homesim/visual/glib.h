#ifndef Glib_H
#define Glib_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define Glib_SCREENWIDTH 1024
#define Glib_SCREENHEIGHT 768
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

#define Glib_gFont8 1
#define Glib_gFont12 2
#define Glib_gFont16 3
#define Glib_gFont24 4

typedef void (*Glib_PtrProc) ();
typedef void (*Glib_PtrProc3) (char, int, int);
extern Glib_PtrProc3 Glib_Char;
extern void Glib_Draw(int x1, int y1, int x2, int y2);
extern void Glib_DrawPolygon(unsigned int n, int x, int y, int v[]);
extern void Glib_DrawLines(unsigned int n, int x, int y, int v[]);
extern void Glib_Colour(unsigned int col);
extern void Glib_LineWidth(float w);
extern void Glib_Rectangle(int x1, int y1, int xside, int yside);
extern void Glib_SetFont(unsigned int font, int spacing);
extern void Glib_Chars(char str[], int x, int y);
extern void Glib_AntiAliasing(bool Mode);
extern void Glib_ClipWindow(int x, int y, int xs, int ys);
extern void BEGIN_Glib();

#ifdef __cplusplus
}
#endif

#endif
