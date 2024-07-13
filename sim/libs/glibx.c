#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <FTGL/ftgl.h>

#include <SIM/Glibx.h>

#define MAXFONTS      20

typedef GLfloat RGBValues[3];

static RGBValues ColourTable[10] = 
{
  {0.00, 0.00, 0.00},
  {0.83, 0.83, 0.83},
  {0.0,  0.25, 0.65}, 
  {0.50, 0.50, 0.50},
  {0.83, 0.00, 0.00},
  {0.00, 0.83, 0.00},
  {0.83, 0.00, 0.83},
  {0.44, 0.27, 0.06},
  {0.00, 0.83, 0.83},
  {0.83, 0.83, 0.00} 
};

FTGLfont*           Fonts[MAXFONTS + 1]; // font descriptor
static unsigned int CurrentFont = 0;
static float        wx1, wy1, wx2, wy2;

/* ------------------------------------------------------------- */
void Glibx_ReadFontFile(int FontNo, char FontName[], int FontHeight, int FontWidth)
{
    Fonts[FontNo] = ftglCreatePixmapFont(FontName);
    if (!Fonts[FontNo])
	{
	    printf("Can't open font file %s\n", FontName);
		exit(-1);
	}
    ftglSetFontFaceSize(Fonts[FontNo], FontHeight, FontWidth);	
}

/* ------------------------------------------------------------- */
void Glibx_RemoveFont(unsigned int font)
{
    ftglDestroyFont(Fonts[font]);
}

/* ------------------------------------------------------------- */
void Glibx_SetFont(unsigned int font, int spacing)
{
    CurrentFont = font;
}

/* ------------------------------------------------------------- */
void Glibx_Chars(char str[], int x, int y)
{
    glRasterPos2i(x, y);
    ftglRenderFont(Fonts[CurrentFont], str, FTGL_RENDER_ALL);
}

/* -------------------------------------- */
void Glibx_Char(char ch, int x, int y)
{
    char str[2];
	
	str[0] = ch;
	str[1] = '\0';
	Glibx_Chars(str, x, y);
}

/* -------------------------------------- */
int Glibx_StringSize(char str[])
{
    int   len;
    float box[6];
  
    len = (int) strlen(str);
    ftglGetFontBBox(Fonts[CurrentFont], str, len, box);
    return (int) (box[3] - box[0]);
}

/* -------------------------------------- */
void Glibx_Colour(unsigned int col)
{
    glColor3fv(ColourTable[col]);
}

/* -------------------------------------- */
void Glibx_Draw(int x1, int y1, int x2, int y2)
{
    glBegin(GL_LINES);
    // Portrait mode
    //glVertex2i(-y1, x1);
    //glVertex2i(-y2, x2);
    // Landscape Display
    glVertex2i(x1, y1);
    glVertex2i(x2, y2);
    glEnd();
}

/* -------------------------------------- */
void Glibx_Window(float x1, float y1, float x2, float y2)
{
    wx1 = x1;
    wy1 = y1;
    wx2 = x2;
    wy2 = y2;
}

/* -------------------------------------- */
void Glibx_DrawPolygon(unsigned int n, int x, int y, int v[])
{
    int          dx;
    int          dy;
    unsigned int i;
    unsigned int p;
  
    p = 0;
    if (n >= 3) 
	{
        glBegin(GL_POLYGON);
        for (i = 0; i <= n - 1; i += 1) 
		{
            dx = v[p];
            p = p + 1;
            dy = v[p];
            p = p + 1;
            //glVertex2i(-(y + dy), x + dx);
            glVertex2i(x + dx, y + dy);
        }
        glEnd();
    }
}

/* -------------------------------------- */
void Glibx_DrawLines(unsigned int n, int x, int y, int v[])
{
    int          dx;
    int          dy;
    unsigned int i;
    unsigned int p;

    p = 0;
  
    if (n > 1) 
    {
        glBegin(GL_LINE_STRIP);
        for (i = 0; i <= n - 1; i += 1) 
		{
            dx = v[p];
            p = p + 1;
            dy = v[p];
            p = p + 1;
            //glVertex2i(-(y + dy), x + dx);
            glVertex2i(x + dx, y + dy);
        }
        glEnd();
    }
}

/* -------------------------------------- */
void Glibx_LineWidth(float w)
{
    glLineWidth((GLfloat) w);
    glPointSize((GLfloat) w);
}

/* -------------------------------------- */
void Glibx_Rectangle(int x1, int y1, int xside, int yside)
{
    //glRecti(-y1 - yside, x1, -y1, x1 + xside);
    glRecti(x1, y1, x1 + xside, y1 + yside);
}

/* -------------------------------------- */
void Glibx_AntiAliasing(bool Mode)
{
    if (Mode)
    {
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    }
    else
    {
        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_BLEND);
    }
}

/* -------------------------------------- */
void Glibx_ClipWindow(int x, int y, int xs, int ys)
{
    glEnable(GL_SCISSOR_TEST);
    glScissor(x, y, xs, ys);
}

/* -------------------------------------- */
void BEGIN_Glibx()
{
    wx1 = 0.0;
    wy1 = 0.0;
    wx2 = (float) (Glibx_SCREENWIDTH - 1);
    wy2 = (float) (Glibx_SCREENHEIGHT - 1);

    Glibx_ReadFontFile(Glibx_GFONT10, "c:/windows/fonts/arial.ttf", 10, 10);
    Glibx_ReadFontFile(Glibx_GFONT20, "c:/windows/fonts/arial.ttf", 20, 20);
}
