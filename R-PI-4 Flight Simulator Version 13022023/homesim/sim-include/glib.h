#ifndef Glib_H
#define Glib_H

#include <stdbool.h>

#include <GL/gl.h>

#include <cglm/cglm.h>  /* inline version */

#define Glib_SCREENWIDTH  1920
#define Glib_SCREENHEIGHT 1080
//#define Glib_SCREENWIDTH  1024
//#define Glib_SCREENHEIGHT 768

#define Glib_BLACK      0
#define Glib_WHITE      1
#define Glib_BLUE       2
#define Glib_GREY       3
#define Glib_RED        4
#define Glib_GREEN      5
#define Glib_MAGENTA    6
#define Glib_BROWN      7
#define Glib_CYAN       8
#define Glib_YELLOW     9
#define Glib_ORANGE    10
#define Glib_LIGHTBLUE 11
#define Glib_LIGHTGREY 12
#define Glib_DARKGREY  13

#define Glib_GFONT10    1  /* graphics fonts */
#define Glib_GFONT18    2
#define Glib_EFONT8     4  /* EFIS fonts */
#define Glib_EFONT12    5
#define Glib_EFONT16    6
#define Glib_EFONT24    7
#define Glib_LFONT20    8  /* LCD FCU font */
#define Glib_LFONT30    9  /* LCD Radio font */

#define Glib_ANCHOR_CENTRE  0
#define Glib_ANCHOR_LOWLEFT 1

extern void Glib_Char(char, int, int);
extern void Glib_Chars(char str[], int x, int y);
extern int  Glib_StringSize(char str[]);
extern void Glib_Draw(int x1, int y1, int x2, int y2);
extern void Glib_Dot(int x, int y);
extern void Glib_Rectangle(int x1, int y1, int xside, int yside);
extern void Glib_Triangle(int x1, int y1, int x2, int y2, int x3, int y3);
extern void Glib_DrawTexture(int x0, int y0, int sx0, int sy0, float tx1, float ty1, float tx2, float ty2, float Alpha);
extern void Glib_DrawTextureRotated(int x0, int y0, int sx0, int sy0, float tx1, float ty1, float tx2, float ty2, float R, float Alpha);
extern void Glib_Line_Strip(int x, int y, unsigned int n, int v[]);
extern void Glib_Line_Loop(int x, int y, unsigned int n, int v[]);
extern void Glib_DrawLines(int x, int y, int n, int v[]);
extern void Glib_Colour(unsigned int col);
extern void Glib_LineWidth(float w);
extern void Glib_SetFont(unsigned int font, int spacing);
extern void Glib_AntiAliasing(bool Mode);
extern void Glib_ClipWindow(int x, int y, int xs, int ys);
extern void Glib_RemoveClipWindow(void);
extern void Glib_Flush();
extern void Glib_Init();
extern void Glib_LoadIdentity();
extern void Glib_Rotate(float a);
extern void Glib_Translate(float x, float y);
extern void Glib_Close(void);
extern void Glib_LoadFont(char filename[], unsigned int fontnumber, unsigned int height);
extern void Glib_LoadTexture(char filename[], unsigned int texturenumber);
extern void Glib_SetTexture(unsigned int t);
extern bool Glib_Errors(); 
extern void Glib_Info();
extern void Glib_PushMatrix();
extern void Glib_PopMatrix();
extern void Glib_SetNormal();
extern void Glib_DrawSegment(GLfloat px, GLfloat py, GLfloat pz, GLfloat nx, GLfloat ny, GLfloat nz, GLfloat u, GLfloat v);
extern void Glib_RotatePitch3D(float pitch);
extern void Glib_RotateRoll3D(float roll);
extern void Glib_RotateYaw3D(float yaw);
extern void Glib_Lighting(vec3 pos, vec3 intensity, vec3 Ka, vec3 Kd, vec3 Ks, float shine);
extern void Glib_Print(int n); // ***

extern void BEGIN_Glib();

#endif
