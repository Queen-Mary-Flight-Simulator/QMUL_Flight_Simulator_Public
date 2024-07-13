#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <FreeImage.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <cglm/cglm.h>  /* inline version */

#include <SIM/glib.h>
#include <SIM/textureid.h>
#include <SIM/shader.h>

#include <ft2build.h>
#include <freetype/freetype.h>

#define DOTS            0
#define VECTORS         1
#define TRIANGLES       2
#define TEXTURES        3
#define STRINGS         4

#define MAXVECTORS      15000
#define MAXTRIANGLES    200
#define MAXDOTS         200
#define MAXTEXTURES     10000
#define MAXSTRINGS      10000
#define MAXSEGMENTS      6000

#define MAXFONTFILES    20
#define MAXTEXTUREFILES 100
#define MAXWIDTH        1024
#define MAXCHARS        100

#define MAX(a,b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a > _b ? _a : _b; })
     
#define ONERAD          (180.0f / M_PI)

#define MAXSTACK        10   /* max nesting of stack */
#define PORTRAIT        false

/**
 * The atlas struct holds a texture that contains the visible ASCII
 * characters of a font rendered with a defined character height.
 * It also contains an array of all the information necessary to
 * generate the vertex and texture coordinates for each character.
 * After the atlas is constructed, no further FreeType functions are used.
 */
struct atlas 
{
    GLuint tex;		// texture object

    unsigned int w;	// width of texture in pixels
    unsigned int h;	// height of texture in pixels

    struct 
    {
        float ax;	// advance.x
        float ay;	// advance.y

        float bw;	// bitmap.width;
        float bh;	// bitmap.height;

        float bl;	// bitmap_left;
        float bt;	// bitmap_top;

        float tx;	// x offset of glyph in texture coordinates
        float ty;	// y offset of glyph in texture coordinates
    } c[128];		// character information
};

struct LightInfo
{
    vec3 Position;  /* light position in eye coords */
	vec3 Intensity; /* A,D,S intensity */
};

struct MaterialInfo
{
    vec3 Ka;          /* ambient reflectivity */
    vec3 Kd;          /* diffuse reflectivity */
    vec3 Ks;          /* specular reflectivity */
	float Shininess;  /* specular shininess factor */
};

typedef GLfloat   RGBValues[3];

const RGBValues ColourTable[] = 
{
    { 0.00, 0.00, 0.00 }, /* BLACK */
    { 0.83, 0.83, 0.83 }, /* WHITE */
    { 0.00, 0.25, 0.65 }, /* BLUE */
    { 0.51, 0.51, 0.51 }, /* GREY */
    { 0.83, 0.00, 0.00 }, /* RED */
    { 0.00, 0.83, 0.00 }, /* GREEN */
    { 0.83, 0.00, 0.83 }, /* MAGENTA */
    { 0.44, 0.27, 0.06 }, /* BROWN */
    { 0.00, 0.83, 0.83 }, /* CYAN */
    { 0.83, 0.83, 0.00 }, /* YELLOW */
    { 0.82, 0.52, 0.18 }, /* ORANGE */
    { 0.00, 0.60, 1.00 }, /* LIGHT BLUE */
    { 0.61, 0.61, 0.61 }, /* LIGHT GREY */
    { 0.41, 0.41, 0.41 }, /* DARK GREY */
    { 0.00, 0.00, 0.00 }, /* SPARE */
    { 0.00, 0.00, 0.00 }  /* SPARE */
};

const char errNames[9][36] = 
{
    "Unknown OpenGL error",
    "GL_INVALID_ENUM", "GL_INVALID_VALUE", "GL_INVALID_OPERATION",
    "GL_INVALID_FRAMEBUFFER_OPERATION", "GL_OUT_OF_MEMORY",
    "GL_STACK_UNDERFLOW", "GL_STACK_OVERFLOW", "GL_CONTEXT_LOST" 
};

static GLuint       VectorProgram;
static GLuint       vaoHandles[3];
static GLuint       vboHandles[2];

static GLuint       positionBufferHandle;
static GLuint       colorBufferHandle;

static GLfloat      DotPosition[MAXDOTS * 2];
static GLfloat      DotColor[MAXDOTS * 3];
static GLfloat      VectorPosition[MAXVECTORS * 4];
static GLfloat      VectorColor[MAXVECTORS * 6];
static GLfloat      TrianglePosition[MAXTRIANGLES * 6];
static GLfloat      TriangleColor[MAXTRIANGLES * 9];
static GLfloat      Strings[MAXSTRINGS * 42];
static GLfloat      TextureVertices[MAXTEXTURES * 24];
static GLfloat      SegmentVertices[MAXSEGMENTS * 8];

static unsigned int ndots;
static unsigned int nvectors;
static unsigned int ntriangles;
static unsigned int ntextures;
static unsigned int nstrings;
static unsigned int nsegments;

static unsigned int maxdots = 0; // *** 
static unsigned int maxvectors = 0; // ***
static unsigned int maxtriangles = 0; // ***
static unsigned int maxtextures = 0; // ***
static unsigned int maxstrings = 0; // ***
static unsigned int maxsegments = 0; // ***

static unsigned int nflushes = 0; // ***

static int          stackp = 0;
static mat4         stack[MAXSTACK+1];
static mat4         Transform;
static mat3         Normal;            /* used for spheres */
static mat4         ProjectionMatrix;  /* used for spheres */
static mat4         MVP;               /* used for spheres */

static GLuint       StringProgram;
static GLint        uniform_tex;

static GLuint       StringVAO;
static GLuint       StringVBO;

static unsigned int CurrentColour;
static struct atlas *CurrentFont = NULL;
static unsigned int oldfont = 0;
static unsigned int CurrentTexture = 0;
static unsigned int oldtexture = 0;
static unsigned int TextureVAO;
static unsigned int TextureVBO;
static GLuint       TextureProgram;
static GLint        uniform_Texture_Sampler;

static GLuint       SphereProgram;
static GLuint       SphereVAO;
static GLuint       SphereVBO;

static GLuint       vtloc;      /* vector transformation matrix location */
static GLuint       ttloc;      /* texture transformation matrix location */
static GLuint       stloc;      /* text transformation matrix location */

static GLint uniformMatrixModelViewloc; /* sphere uniforms */
static GLint uniformMatrixNormalloc;
static GLint uniformMatrixProjectionloc;
static GLint uniformMatrixMVPloc;
static GLint uniformLightPositionloc;
static GLint uniformLightIntensityloc;
static GLint uniformMaterialKdloc;
static GLint uniformMaterialKsloc;
static GLint uniformMaterialKaloc;
static GLint uniformMaterialShininessloc;
static GLint uniformTex1loc;

static vec3 uniformLightPosition = { 596.0f, 1200.0f, 500.0f };
static vec3 uniformLightIntensity = { 1.0f, 1.0f, 1.0f };
static vec3 uniformMaterialKd = { 0.8f, 0.8f, 0.8f };
static vec3 uniformMaterialKs = { 0.5f, 0.5f, 0.5f };
static vec3 uniformMaterialKa = { 0.2f, 0.2f, 0.2f };
static GLfloat uniformMaterialShininess = 3.0f;
 
static struct atlas *FontTable[MAXFONTFILES+1];
static GLuint       TextureTable[MAXTEXTUREFILES+1];

void CreateAtlas(struct atlas *a, FT_Face face, int height);
void DeleteAtlas(struct atlas *a);
void Init_Vectors();
void Init_Textures(); 
void Init_Strings(); 
void Init_Segments(); 
void initLights();
void printmatrix(mat4 m);
GLuint GetUniformLocation(GLuint program, const char shader[], const char varname[]);

/* ***************** */
void Glib_Print(int n)
{
    printf("%d, %d, %d, %d, %d, %d, %d, %d\n", n, maxdots, maxvectors, maxtriangles, maxtextures, maxstrings, maxsegments, nflushes); // ***
    maxdots = 0; // *** 
    maxvectors = 0; // ***
    maxtriangles = 0; // ***
    maxtextures = 0; // ***
    maxstrings = 0; // ***
    maxsegments = 0; // ***
    nflushes = 0;
}

/* ------------------------------------------------------------- */
GLuint GetUniformLocation(GLuint program, const char shader[], const char varname[])
{
    GLuint loc = glGetUniformLocation(program, varname);
    if (loc < 0)
    {
        printf("%s shader: cannot find %s\n", shader, varname);
        exit(-1);  /* fatal error */
    }
	return loc;
}

/* ------------------------------------------------------------- */
void Glib_LoadTexture(char tname[], unsigned int texturenumber) 
{
    unsigned int      width;
    unsigned int      height;
    FIBITMAP          *bitmap;
    BYTE              *data;
    FREE_IMAGE_FORMAT fif;
    
    glGenTextures(1, &TextureTable[texturenumber]);
    glBindTexture(GL_TEXTURE_2D, TextureTable[texturenumber]);

    // Get the format of the image file
    fif = FreeImage_GetFileType(tname, 0);

    // Load the data in bitmap if possible
    if (fif != FIF_UNKNOWN && FreeImage_FIFSupportsReading(fif)) 
    {
        bitmap = FreeImage_Load(fif, tname, 0);
    }
    else 
    {
        printf("Unable to load the image file %s\n", tname);
        exit(-1);
    }

    width = FreeImage_GetWidth(bitmap);
    height = FreeImage_GetHeight(bitmap);
    data = (BYTE*) FreeImage_GetBits(bitmap);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*) data);
    glGenerateMipmap(GL_TEXTURE_2D);

    FreeImage_Unload(bitmap);
}

/* --------------------------------------------- */
void CreateAtlas(struct atlas *a, FT_Face face, int height) 
{
    unsigned int roww = 0;
    unsigned int rowh = 0;
    int          ox = 0;
    int          oy = 0;
    int          i;
    
    FT_Set_Pixel_Sizes(face, 0, height);
    FT_GlyphSlot g = face->glyph;

    a->w = 0;
    a->h = 0;

    memset(a->c, 0, sizeof(a->c));

    /* Find minimum size for a texture holding all visible ASCII characters */
    for (i = 32; i < 128; i+=1) 
    {
        if (FT_Load_Char(face, i, FT_LOAD_RENDER)) 
        {
            fprintf(stderr, "Loading character %c failed!\n", i);
            continue;
        }
        if (roww + g->bitmap.width + 1 >= MAXWIDTH) 
        {
            a->w = MAX(a->w, roww);
            a->h += rowh;
            roww = 0;
            rowh = 0;
        }
        roww += g->bitmap.width + 1;
        rowh = MAX(rowh, g->bitmap.rows);
    }

    a->w = MAX(a->w, roww);
    a->h += rowh;

    /* Create a texture that will be used to hold all ASCII glyphs */
    glGenTextures(1, &a->tex);
    glBindTexture(GL_TEXTURE_2D, a->tex);
    glUniform1i(uniform_tex, 0);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, a->w, a->h, 0, GL_ALPHA, GL_UNSIGNED_BYTE, 0);
    glGenerateMipmap(GL_TEXTURE_2D);

    /* We require 1 byte alignment when uploading texture data */
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    /* Clamping to edges is important to prevent artifacts when scaling */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    /* Linear filtering usually looks best for text */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    /* Paste all glyph bitmaps into the texture, remembering the offset */

    rowh = 0;

    for (i = 32; i < 128; i+=1) 
    {
        if (FT_Load_Char(face, i, FT_LOAD_RENDER)) 
        {
            fprintf(stderr, "Loading character %c failed!\n", i);
            continue;
        }

        if (ox + g->bitmap.width + 1 >= MAXWIDTH) 
        {
            oy += rowh;
            rowh = 0;
            ox = 0;
        }

        glTexSubImage2D(GL_TEXTURE_2D, 0, ox, oy, g->bitmap.width, g->bitmap.rows, GL_ALPHA, GL_UNSIGNED_BYTE, g->bitmap.buffer);
        a->c[i].ax = g->advance.x >> 6;
        a->c[i].ay = g->advance.y >> 6;

        a->c[i].bw = g->bitmap.width;
        a->c[i].bh = g->bitmap.rows;

        a->c[i].bl = g->bitmap_left;
        a->c[i].bt = g->bitmap_top;

        a->c[i].tx = ox / (float)a->w;
        a->c[i].ty = oy / (float)a->h;

        rowh = MAX(rowh, g->bitmap.rows);
        ox += g->bitmap.width + 1;
    }
    //printf("Generated a %d x %d (%d kb) texture atlas\n", a->w, a->h, a->w * a->h / 1024);
}

/* --------------------------------------------- */
void DeleteAtlas(struct atlas *a) 
{
    glDeleteTextures(1, &a->tex);
    free(a);
}

/*---------------------------------------------------------------------------*/
void Glib_DrawTexture(int x0, int y0, int sx0, int sy0, float tx1, float ty1, float tx2, float ty2, float Alpha)
{
    float x1 = (float) x0; /* EFIS landscape coords */
    float y1 = (float) y0;
    float x2 = (float) (x0 + sx0);
    float y2 = (float) (y0 + sy0);

    TextureVertices[ntextures * 24 + 0]  = x1;  /* bottom left */
    TextureVertices[ntextures * 24 + 1]  = y1;
    TextureVertices[ntextures * 24 + 2]  = tx1;
    TextureVertices[ntextures * 24 + 3]  = ty1;
    TextureVertices[ntextures * 24 + 4]  = x2;  /* bottom right */
    TextureVertices[ntextures * 24 + 5]  = y1;
    TextureVertices[ntextures * 24 + 6]  = tx2;
    TextureVertices[ntextures * 24 + 7]  = ty1;
    TextureVertices[ntextures * 24 + 8]  = x2;  /* top right */
    TextureVertices[ntextures * 24 + 9]  = y2;
    TextureVertices[ntextures * 24 + 10] = tx2;
    TextureVertices[ntextures * 24 + 11] = ty2;

    TextureVertices[ntextures * 24 + 12] = x2;  /* top right */
    TextureVertices[ntextures * 24 + 13] = y2;
    TextureVertices[ntextures * 24 + 14] = tx2;
    TextureVertices[ntextures * 24 + 15] = ty2;
    TextureVertices[ntextures * 24 + 16] = x1;  /* top left */
    TextureVertices[ntextures * 24 + 17] = y2;
    TextureVertices[ntextures * 24 + 18] = tx1;
    TextureVertices[ntextures * 24 + 19] = ty2;
    TextureVertices[ntextures * 24 + 20] = x1;  /* bottom left */
    TextureVertices[ntextures * 24 + 21] = y1;
    TextureVertices[ntextures * 24 + 22] = tx1;
    TextureVertices[ntextures * 24 + 23] = ty1;

    ntextures += 1;
    if (ntextures >= MAXTEXTURES)
    {
        printf("Textures overflow (%d)\n", MAXTEXTURES);
        exit(-1);
    }
}

/*---------------------------------------------------------------------------*/
void Glib_DrawTextureRotated(int x0, int y0, int sx0, int sy0, float tx1, float ty1, float tx2, float ty2, float R, float Alpha)
{
    float x  = (float) x0; /* EFIS landscape coords */
    float y  = (float) y0;
    float sx = (float) sx0;
    float sy = (float) sy0;

    Glib_PushMatrix();
    Glib_Translate(x, y);
    Glib_Rotate(R);

    TextureVertices[ntextures * 24 + 0]  = -sx / 2.0f;  /* bottom left */
    TextureVertices[ntextures * 24 + 1]  = -sy / 2.0f;
    TextureVertices[ntextures * 24 + 2]  = tx1;
    TextureVertices[ntextures * 24 + 3]  = ty1;
    TextureVertices[ntextures * 24 + 4]  = sx / 2.0f;   /* bottom right */
    TextureVertices[ntextures * 24 + 5]  = -sy / 2.0f;
    TextureVertices[ntextures * 24 + 6]  = tx2;
    TextureVertices[ntextures * 24 + 7]  = ty1;
    TextureVertices[ntextures * 24 + 8]  = sx / 2.0f;   /* top right */
    TextureVertices[ntextures * 24 + 9]  = sy / 2.0f;
    TextureVertices[ntextures * 24 + 10] = tx2;
    TextureVertices[ntextures * 24 + 11] = ty2;

    TextureVertices[ntextures * 24 + 12] = sx / 2.0f;   /* top right */
    TextureVertices[ntextures * 24 + 13] = sy / 2.0f;
    TextureVertices[ntextures * 24 + 14] = tx2;
    TextureVertices[ntextures * 24 + 15] = ty2;
    TextureVertices[ntextures * 24 + 16] = -sx / 2.0f;  /* top left */
    TextureVertices[ntextures * 24 + 17] = sy / 2.0f;
    TextureVertices[ntextures * 24 + 18] = tx1;
    TextureVertices[ntextures * 24 + 19] = ty2;
    TextureVertices[ntextures * 24 + 20] = -sx / 2.0f;  /* bottom left */
    TextureVertices[ntextures * 24 + 21] = -sy / 2.0f;
    TextureVertices[ntextures * 24 + 22] = tx1;
    TextureVertices[ntextures * 24 + 23] = ty1;
    
    ntextures += 1;
    if (ntextures >= MAXTEXTURES)
    {
        printf("Textures overflow (%d)\n", MAXTEXTURES);
        exit(-1);
    }
    
    Glib_PopMatrix();
}

/*---------------------------------------------------------------------------*/
void Glib_Close() 
{
    unsigned int i;
    
    glDeleteProgram(VectorProgram);
    glDeleteProgram(StringProgram);
    glDeleteProgram(TextureProgram);
    
    for (i=1; i<=MAXFONTFILES; i+=1)
    {
        if (FontTable[i] != NULL)
        {
            DeleteAtlas(FontTable[i]);
        }
    }
}

/* ------------------------------------------------- */
void Glib_SetTexture(unsigned int t)
{
    if (TextureTable[t] == 0)
    {
        printf("Missing texture %d\n", t);
        exit(-1);
    }
    
    if (t != oldtexture)
    {
        Glib_Flush();
        CurrentTexture = TextureTable[t];
        oldtexture = t;
    }
}

/* ------------------------------------------------- */
void Glib_SetFont(unsigned int font, int spacing)  /* N.B. spacing ignored with tt fonts */
{
    if (FontTable[font] == NULL)
    {
        printf("Missing font %d\n", font);
        exit(-1);
    }
    
    if (font != oldfont)
    {
        Glib_Flush();
        CurrentFont = FontTable[font];
        oldfont = font;
    }
}

/* ------------------------------------------------- */
void Glib_Chars(char *text, int x1, int y1) 
{
    int i;
    struct atlas *a = CurrentFont;
    
    float x = (float) x1;
    float y = (float) y1;

    float r = ColourTable[CurrentColour][0];
    float g = ColourTable[CurrentColour][1];
    float b = ColourTable[CurrentColour][2];

    /* Loop through all characters */
    for (i=0; i<strlen(text); i+=1) 
    {
        int ch = (int) text[i];
        
        /* Calculate the vertex and texture coordinates */
        float x2 = x + a->c[ch].bl;
        float y2 = -y - a->c[ch].bt;
        float w = a->c[ch].bw;
        float h = a->c[ch].bh;

        /* Advance the cursor to the start of the next character */
        x += a->c[ch].ax;
        y += a->c[ch].ay;

        /* Skip glyphs that have no pixels */
        if (!w || !h)
        {
            continue;
        }
        
        Strings[nstrings * 42 + 0]  = x2;
        Strings[nstrings * 42 + 1]  = -y2;
        Strings[nstrings * 42 + 2]  = a->c[ch].tx;
        Strings[nstrings * 42 + 3]  = a->c[ch].ty;
        Strings[nstrings * 42 + 4]  = r;
        Strings[nstrings * 42 + 5]  = g;
        Strings[nstrings * 42 + 6]  = b;
        Strings[nstrings * 42 + 7]  = x2 + w;
        Strings[nstrings * 42 + 8]  = -y2;
        Strings[nstrings * 42 + 9]  = a->c[ch].tx + a->c[ch].bw / a->w;
        Strings[nstrings * 42 + 10] = a->c[ch].ty;
        Strings[nstrings * 42 + 11] = r;
        Strings[nstrings * 42 + 12] = g;
        Strings[nstrings * 42 + 13] = b;
        Strings[nstrings * 42 + 14] = x2;
        Strings[nstrings * 42 + 15] = -y2 - h;
        Strings[nstrings * 42 + 16] = a->c[ch].tx;
        Strings[nstrings * 42 + 17] = a->c[ch].ty + a->c[ch].bh / a->h;
        Strings[nstrings * 42 + 18] = r;
        Strings[nstrings * 42 + 19] = g;
        Strings[nstrings * 42 + 20] = b;
        Strings[nstrings * 42 + 21] = x2 + w;
        Strings[nstrings * 42 + 22] = -y2;
        Strings[nstrings * 42 + 23] = a->c[ch].tx + a->c[ch].bw / a->w;
        Strings[nstrings * 42 + 24] = a->c[ch].ty;
        Strings[nstrings * 42 + 25] = r;
        Strings[nstrings * 42 + 26] = g;
        Strings[nstrings * 42 + 27] = b;
        Strings[nstrings * 42 + 28] = x2;
        Strings[nstrings * 42 + 29] = -y2 - h;
        Strings[nstrings * 42 + 30] = a->c[ch].tx;
        Strings[nstrings * 42 + 31] = a->c[ch].ty + a->c[ch].bh / a->h;
        Strings[nstrings * 42 + 32] = r;
        Strings[nstrings * 42 + 33] = g;
        Strings[nstrings * 42 + 34] = b;
        Strings[nstrings * 42 + 35] = x2 + w;
        Strings[nstrings * 42 + 36] = -y2 - h;
        Strings[nstrings * 42 + 37] = a->c[ch].tx + a->c[ch].bw / a->w;
        Strings[nstrings * 42 + 38] = a->c[ch].ty + a->c[ch].bh / a->h;
        Strings[nstrings * 42 + 39] = r;
        Strings[nstrings * 42 + 40] = g;
        Strings[nstrings * 42 + 41] = b;
        
        nstrings += 1;

        if (nstrings >= MAXSTRINGS)
        {
            printf("Strings overflow (%d)\n", MAXSTRINGS);
            exit(-1);
        }
    }
}

/* -------------------------------------- */
void Glib_Char(char ch, int x, int y)
{
    char str[2];
    
    str[0] = ch;
    str[1] = '\0';
    Glib_Chars(str, x, y);
}

/* -------------------------------------- */
int Glib_StringSize(char str[])
{
    int i;
    struct atlas *a = CurrentFont;
    float x = 0.0;

    /* Loop through all characters */
    for (i=0; i<strlen(str); i+=1) 
    {
        int ch = (int) str[i];
        
        x += a->c[ch].ax;
    }
    return (int) x;
}

/* ------------------------------------------------- */
void Glib_Colour(unsigned int col)
{
    CurrentColour = col;
}

/* ------------------------------------------------- */
void Glib_DrawSegment(GLfloat px, GLfloat py, GLfloat pz, GLfloat nx, GLfloat ny, GLfloat nz, GLfloat u, GLfloat v)
{
//printf("DrawSegment: nsegments=%d px=%f py=%f pz=%f nx=%f ny=%f nz=%f u=%f, v=%f\n", nsegments, px, py, pz, nx, ny, nz, u, v); // ***
//fflush(stdout); // ***
    SegmentVertices[nsegments * 8 + 0] = px;
    SegmentVertices[nsegments * 8 + 1] = py;
    SegmentVertices[nsegments * 8 + 2] = pz;
    SegmentVertices[nsegments * 8 + 3] = nx;
    SegmentVertices[nsegments * 8 + 4] = ny;
    SegmentVertices[nsegments * 8 + 5] = nz;
    SegmentVertices[nsegments * 8 + 6] = u;
    SegmentVertices[nsegments * 8 + 7] = v;
    
    nsegments += 1;
    
    if (nsegments >= MAXSEGMENTS)
    {
        printf("Segments overflow (%d)\n", MAXSEGMENTS);
        exit(-1);
    }
}

/* ------------------------------------------------- */
void Glib_Draw(int x1, int y1, int x2, int y2)
{
    float r = ColourTable[CurrentColour][0];
    float g = ColourTable[CurrentColour][1];
    float b = ColourTable[CurrentColour][2];

    VectorPosition[nvectors * 4 + 0] = (float) x1;
    VectorPosition[nvectors * 4 + 1] = (float) y1;
    VectorPosition[nvectors * 4 + 2] = (float) x2;
    VectorPosition[nvectors * 4 + 3] = (float) y2;

    VectorColor[nvectors * 6 + 0] = r;
    VectorColor[nvectors * 6 + 1] = g;
    VectorColor[nvectors * 6 + 2] = b;
    VectorColor[nvectors * 6 + 3] = r;
    VectorColor[nvectors * 6 + 4] = g;
    VectorColor[nvectors * 6 + 5] = b;
    
    nvectors += 1;
    
    if (nvectors >= MAXVECTORS)
    {
        printf("Vectors overflow (%d)\n", MAXVECTORS);
        exit(-1);
    }
}

/* ------------------------------------------------- */
void Glib_Triangle(int x1, int y1, int x2, int y2, int x3, int y3)
{
    float r = ColourTable[CurrentColour][0];
    float g = ColourTable[CurrentColour][1];
    float b = ColourTable[CurrentColour][2];

    TrianglePosition[ntriangles * 6 + 0]  = (float) x1;
    TrianglePosition[ntriangles * 6 + 1]  = (float) y1;
    TrianglePosition[ntriangles * 6 + 2]  = (float) x2;
    TrianglePosition[ntriangles * 6 + 3]  = (float) y2;
    TrianglePosition[ntriangles * 6 + 4]  = (float) x3;
    TrianglePosition[ntriangles * 6 + 5]  = (float) y3;

    TriangleColor[ntriangles * 9 + 0]  = r;
    TriangleColor[ntriangles * 9 + 1]  = g;
    TriangleColor[ntriangles * 9 + 2]  = b;
    TriangleColor[ntriangles * 9 + 3]  = r;
    TriangleColor[ntriangles * 9 + 4]  = g;
    TriangleColor[ntriangles * 9 + 5]  = b;
    TriangleColor[ntriangles * 9 + 6]  = r;
    TriangleColor[ntriangles * 9 + 7]  = g;
    TriangleColor[ntriangles * 9 + 8]  = b;

    ntriangles += 1;
    if (ntriangles >= MAXTRIANGLES)
    {
        printf("Triangles overflow (%d)\n", MAXTRIANGLES);
        exit(-1);
    }
}

/* ------------------------------------------------- */
void Glib_Rectangle(int sx1, int sy1, int xs, int ys)
{
    int sx2 = sx1 + xs;
    int sy2 = sy1 + ys;

    Glib_Triangle(sx1, sy1, sx1, sy2, sx2, sy2);
    Glib_Triangle(sx1, sy1, sx2, sy2, sx2, sy1);
}

/* -------------------------------------- */
void Glib_Dot(int x, int y)
{
    float r = ColourTable[CurrentColour][0];
    float g = ColourTable[CurrentColour][1];
    float b = ColourTable[CurrentColour][2];
    
    DotPosition[ndots * 2 + 0]  = (float) x;
    DotPosition[ndots * 2 + 1]  = (float) y;

    DotColor[ndots * 3 + 0]  = r;
    DotColor[ndots * 3 + 1]  = g;
    DotColor[ndots * 3 + 2]  = b;

    ndots += 1;
    if (ndots >= MAXDOTS)
    {
        printf("Dots overflow (%d)\n", MAXDOTS);
        exit(-1);
    }
}

/* ------------------------------------------------- */
void Glib_Line_Loop(int x, int y, unsigned int n, int v[])
{
    int          x0 = x + v[0];
    int          y0 = y + v[1];
    int          x1 = x0;
    int          y1 = y0;
    unsigned int i;
    unsigned int p = 2;

    if (n >= 2)
    {
        for (i=1; i<=n; i+=1)
        {
            int x2 = x + v[p];
            int y2 = y + v[p+1];
            Glib_Draw(x1, y1, x2, y2);
            p += 2;
            x1 = x2;
            y1 = y2;
        }
        Glib_Draw(x0, y0, x1, y1);
    }
}

/* ------------------------------------------------- */
void Glib_Line_Strip(int x, int y, unsigned int n, int v[])
{
    int          x1 = x + v[0];
    int          y1 = y + v[1];
    unsigned int i;
    unsigned int p = 2;

    if (n > 1)
    {
        for (i=1; i<=n; i+=1)
        {
            int x2 = x + v[p];
            int y2 = y + v[p+1];
            Glib_Draw(x1, y1, x2, y2);
            p += 2;
            x1 = x2;
            y1 = y2;
        }
    }
}

/* -------------------------------------- */
void Glib_DrawLines(int x, int y, int n, int v[])
{
	unsigned int i;
	unsigned int p = 0;
	
	for (i=1; i<=n; i+=1) 
	{
    	int x1 = x + v[p];
	    int y1 = y + v[p+1];
		int x2 = x + v[p+2];
		int y2 = y + v[p+3];
		p += 4;
		Glib_Draw(x1, y1, x2, y2);
	}
}

/* ------------------------------------------------- */
void Glib_LineWidth(float w)
{
    Glib_Flush();
    glLineWidth((GLfloat) w);
    //glPointSize((GLfloat) w);
}

/* ------------------------------------------------- */
void Glib_AntiAliasing(bool Mode)
{
    Glib_Flush();
    if (Mode)
    {
        glEnable(GL_LINE_SMOOTH);
    }
    else
    {
        glDisable(GL_LINE_SMOOTH);
    }
}

/* -------------------------------------- */
void Glib_ClipWindow(int x, int y, int xs, int ys)
{
    Glib_Flush();
    glEnable(GL_SCISSOR_TEST);
	if (PORTRAIT)
	{
	    glScissor(Glib_SCREENWIDTH - 1 - y - ys, x, ys, xs);
	}
	else
	{
        glScissor(x, y, xs, ys);
    }
}

/* ------------------------------------------------- */
void Glib_RemoveClipWindow(void)
{
    Glib_Flush();
    glDisable(GL_SCISSOR_TEST);
}

/* -------------------------------------- */
void Glib_Flush()
{
if (nvectors>0 || ntriangles>0 || ntextures>0 || nstrings>0)
    nflushes += 1;
//printf("%f, %d, %d, %d, %d\n", glfwGetTime(), nvectors, ntriangles, ntextures, nstrings); // **  
	if (ndots > 0)
	{
		glUseProgram(VectorProgram);
		glBindVertexArray(vaoHandles[DOTS]);
		glUniformMatrix4fv(vtloc, 1, GL_FALSE, (float *) Transform);
		glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
		glBufferData(GL_ARRAY_BUFFER, ndots * 2 * sizeof(float), DotPosition, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
		glBufferData(GL_ARRAY_BUFFER, ndots * 3 * sizeof(float), DotColor, GL_STATIC_DRAW);
		glDrawArrays(GL_POINTS, 0, ndots * 3);
        if (ndots > maxdots) // ***
		    maxdots = ndots;
		ndots = 0;
        glBindVertexArray(0);
	}

	if (nvectors > 0)
	{
		glUseProgram(VectorProgram);
		glBindVertexArray(vaoHandles[VECTORS]);
		glUniformMatrix4fv(vtloc, 1, GL_FALSE, (float *) Transform);
		glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
		glBufferData(GL_ARRAY_BUFFER, nvectors * 4 * sizeof(float), VectorPosition, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
		glBufferData(GL_ARRAY_BUFFER, nvectors * 6 * sizeof(float), VectorColor, GL_STATIC_DRAW);
		glDrawArrays(GL_LINES, 0, nvectors * 2);
        if (nvectors > maxvectors) // ***
		    maxvectors = nvectors;
		nvectors = 0;
        glBindVertexArray(0);
	}

	if (ntriangles > 0)
	{
		glUseProgram(VectorProgram);
		glBindVertexArray(vaoHandles[TRIANGLES]);
		glUniformMatrix4fv(vtloc, 1, GL_FALSE, (float *) Transform);
		glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
		glBufferData(GL_ARRAY_BUFFER, ntriangles * 6 * sizeof(float), TrianglePosition, GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
		glBufferData(GL_ARRAY_BUFFER, ntriangles * 9 * sizeof(float), TriangleColor, GL_STATIC_DRAW);
		glDrawArrays(GL_TRIANGLES, 0, ntriangles * 3);
        if (ntriangles > maxtriangles) // ***
		    maxtriangles = ntriangles;
		ntriangles = 0;
        glBindVertexArray(0);
	}

	if (ntextures > 0)
	{
		glUseProgram(TextureProgram);
		glBindVertexArray(TextureVAO);
		glUniformMatrix4fv(ttloc, 1, GL_FALSE, (float *) Transform);
		glBindTexture(GL_TEXTURE_2D, CurrentTexture);
		glUniform1i(uniform_Texture_Sampler, 0);
		glBindBuffer(GL_ARRAY_BUFFER, TextureVBO);
		glBufferData(GL_ARRAY_BUFFER, ntextures * 24 * sizeof(float), TextureVertices, GL_STATIC_DRAW);
		glDrawArrays(GL_TRIANGLES, 0, ntextures * 6);
        if (ntextures > maxtextures) // ***
		    maxtextures = ntextures;
		ntextures = 0;
        glBindVertexArray(0);
	}

	if (nstrings > 0)
	{
		glUseProgram(StringProgram);
		glBindVertexArray(StringVAO);
		glUniformMatrix4fv(stloc, 1, GL_FALSE, (float *) Transform);
		glBindTexture(GL_TEXTURE_2D, CurrentFont->tex);
		glUniform1i(uniform_tex, 0);
		glBindBuffer(GL_ARRAY_BUFFER, StringVBO);
		glBufferData(GL_ARRAY_BUFFER, nstrings * 42 * sizeof(float), Strings, GL_STATIC_DRAW);
		glDrawArrays(GL_TRIANGLES, 0, nstrings * 6);
        if (nstrings > maxstrings) // ***
		    maxstrings = nstrings;
		nstrings = 0;
        glBindVertexArray(0);
    }		

	if (nsegments > 0)
	{
	    glm_mat4_mul(ProjectionMatrix, Transform, MVP);
		
		glUseProgram(SphereProgram);
		glBindVertexArray(SphereVAO);
		glUniformMatrix4fv(uniformMatrixModelViewloc, 1, GL_FALSE, (float *) Transform);
		glUniformMatrix3fv(uniformMatrixNormalloc, 1, GL_FALSE, (float *) Normal);
		glUniformMatrix4fv(uniformMatrixProjectionloc, 1, GL_FALSE, (float *) ProjectionMatrix);
		glUniformMatrix4fv(uniformMatrixMVPloc, 1, GL_FALSE, (float *) MVP);
		
        glUniform1i(uniformTex1loc, 0);
        glUniform3fv(uniformLightPositionloc, 1, (float *) uniformLightPosition);
        glUniform3fv(uniformLightIntensityloc, 1, (float *) uniformLightIntensity);
        glUniform3fv(uniformMaterialKdloc, 1, (float *) uniformMaterialKd);
        glUniform3fv(uniformMaterialKsloc, 1, (float *) uniformMaterialKs);
        glUniform3fv(uniformMaterialKaloc, 1, (float *) uniformMaterialKa);
        glUniform1f(uniformMaterialShininessloc, uniformMaterialShininess);

		glBindTexture(GL_TEXTURE_2D, CurrentTexture);
		
		glBindBuffer(GL_ARRAY_BUFFER, SphereVBO);
		glBufferData(GL_ARRAY_BUFFER, nsegments * 8 * sizeof(float), SegmentVertices, GL_STATIC_DRAW);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, nsegments);
        if (nsegments > maxsegments) // ***
		    maxsegments = nsegments;
		nsegments = 0;
        glBindVertexArray(0);
	}
}

/* ------------------------------------------------------------- */
void Init_Textures() 
{
    glGenVertexArrays(1, &TextureVAO);
    glBindVertexArray(TextureVAO);

    glGenBuffers(1, &TextureVBO);

    glBindBuffer(GL_ARRAY_BUFFER, TextureVBO);
    glBufferData(GL_ARRAY_BUFFER, MAXTEXTURES * 24 * sizeof(float), TextureVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) (0 * sizeof(float)));
    glEnableVertexAttribArray(0);
    
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) (2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    TextureProgram = LoadShaders("../glsl/texture-v.glsl", "../glsl/texture-f.glsl");
    if (TextureProgram == 0)
    {
        printf("Shader compilation failed\n");
        exit(-1);
    }

    glUseProgram(TextureProgram);
    ttloc = GetUniformLocation(TextureProgram, "Texture", "TransformationMatrix");
    uniform_Texture_Sampler = GetUniformLocation(TextureProgram, "Texture", "texture_sampler");
}

/* ------------------------------------------------------------- */
void Init_Segments() 
{
    glGenVertexArrays(1, &SphereVAO);
    glBindVertexArray(SphereVAO);

    glGenBuffers(1, &SphereVBO);

    glBindBuffer(GL_ARRAY_BUFFER, SphereVBO);
    glBufferData(GL_ARRAY_BUFFER, MAXSEGMENTS * 8 * sizeof(float), SegmentVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *) (0 * sizeof(float)));
    glEnableVertexAttribArray(0);
    
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *) (3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *) (6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    SphereProgram = LoadShaders("../glsl/sphere-v.glsl", "../glsl/sphere-f.glsl");
    if (SphereProgram == 0)
    {
        printf("Sphere shader: compilation failed\n");
        exit(-1);
    }

    glm_ortho(0.0, (float) Glib_SCREENWIDTH, 0.0, (float) Glib_SCREENHEIGHT, -1.0, 200.0, ProjectionMatrix);  /* form projection matrix */
	glActiveTexture(GL_TEXTURE0);
	
    glUseProgram(SphereProgram);

    uniformMatrixModelViewloc   = GetUniformLocation(SphereProgram, "Sphere", "ModelViewMatrix");
    uniformMatrixNormalloc      = GetUniformLocation(SphereProgram, "Sphere", "NormalMatrix");
	uniformMatrixProjectionloc  = GetUniformLocation(SphereProgram, "Sphere", "ProjectionMatrix");
	uniformMatrixMVPloc         = GetUniformLocation(SphereProgram, "Sphere", "MVP");
	
    uniformLightPositionloc     = GetUniformLocation(SphereProgram, "Sphere", "Light.Position");
    uniformLightIntensityloc    = GetUniformLocation(SphereProgram, "Sphere", "Light.Intensity");
    uniformMaterialKaloc        = GetUniformLocation(SphereProgram, "Sphere", "Material.Ka");
    uniformMaterialKdloc        = GetUniformLocation(SphereProgram, "Sphere", "Material.Kd");
    uniformMaterialKsloc        = GetUniformLocation(SphereProgram, "Sphere", "Material.Ks");
    uniformMaterialShininessloc = GetUniformLocation(SphereProgram, "Sphere", "Material.Shininess");
    uniformTex1loc              = GetUniformLocation(SphereProgram, "Sphere", "Tex1");
}

/* -------------------------------------- */
void Glib_Lighting(vec3 pos, vec3 intensity, vec3 Ka, vec3 Kd, vec3 Ks, float shine)
{
    uniformLightPosition[0]  = pos[0];
    uniformLightPosition[1]  = pos[1];
    uniformLightPosition[2]  = pos[2];
    uniformLightIntensity[0] = intensity[0];
    uniformLightIntensity[1] = intensity[1];
    uniformLightIntensity[2] = intensity[2];
    uniformMaterialKa[0]     = Ka[0];
    uniformMaterialKa[1]     = Ka[1];
    uniformMaterialKa[2]     = Ka[2];
    uniformMaterialKd[0]     = Kd[0];
    uniformMaterialKd[1]     = Kd[1];
    uniformMaterialKd[2]     = Kd[2];
    uniformMaterialKs[0]     = Ks[0];
    uniformMaterialKs[1]     = Ks[1];
    uniformMaterialKs[2]     = Ks[2];
    uniformMaterialShininess = shine;
}

/* -------------------------------------- */
void Init_Vectors()
{
    glGenVertexArrays(4, &vaoHandles[0]);

    glGenBuffers(2, vboHandles);
    positionBufferHandle = vboHandles[0];
    colorBufferHandle = vboHandles[1];

    /* set up dots */
    glBindVertexArray(vaoHandles[DOTS]);

    glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, MAXDOTS * 2 * sizeof(float), DotPosition, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glEnableVertexAttribArray(0);  // Vertex position
    glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, MAXDOTS * 3 * sizeof(float), DotColor, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glEnableVertexAttribArray(1);  // Vertex color

    /* set up vectors */
    glBindVertexArray(vaoHandles[VECTORS]);

    glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, MAXVECTORS * 4 * sizeof(float), VectorPosition, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glEnableVertexAttribArray(0);  // Vertex position
    glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, MAXVECTORS * 6 * sizeof(float), VectorColor, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glEnableVertexAttribArray(1);  // Vertex color

    /* set up triangles */
    glBindVertexArray(vaoHandles[TRIANGLES]);

    glBindBuffer(GL_ARRAY_BUFFER, positionBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, MAXTRIANGLES * 6 * sizeof(float), TrianglePosition, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glEnableVertexAttribArray(0);  // Vertex position
    glBindBuffer(GL_ARRAY_BUFFER, colorBufferHandle);
    glBufferData(GL_ARRAY_BUFFER, MAXTRIANGLES * 9 * sizeof(float), TriangleColor, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);
    glEnableVertexAttribArray(1);  // Vertex color

    VectorProgram = LoadShaders("../glsl/vector-v.glsl", "../glsl/vector-f.glsl");
    if (VectorProgram == 0)
    {
        printf("Vector shader: compilation failed %s %s\n", "vector-v.glsl", "vector-f.glsl");
        exit(-1);
    }
    
    glUseProgram(VectorProgram);
    vtloc = GetUniformLocation(VectorProgram, "Vector", "TransformationMatrix");
}

/* --------------------------------------------- */
void Init_Strings() 
{
    glGenVertexArrays(1, &StringVAO);
    glBindVertexArray(StringVAO);
    
    // Create the vertex buffer object
    glGenBuffers(1, &StringVBO);

    glBindBuffer(GL_ARRAY_BUFFER, StringVBO);
    glBufferData(GL_ARRAY_BUFFER, MAXSTRINGS * 42 * sizeof(float), Strings, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*) (0 * sizeof(float)));
    glEnableVertexAttribArray(0);
    
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*) (2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*) (4 * sizeof(float)));
    glEnableVertexAttribArray(2);
            
    StringProgram = LoadShaders("../glsl/string-v.glsl", "../glsl/string-f.glsl");
    if (StringProgram == 0)
    {
        fprintf(stderr, "String shader compilation failed %s %s\n", "string-v.glsl", "string-f.glsl");
        exit(-1);
    }
    
    glUseProgram(StringProgram);
    stloc = GetUniformLocation(StringProgram, "String", "TransformationMatrix");
    uniform_tex = GetUniformLocation(StringProgram, "String", "tex");
}

/* --------------------------------------------- */
void Glib_Init()
{
    Init_Vectors();
    Init_Textures();
    Init_Strings();
    Init_Segments();
}

/* --------------------------------------------- */
void Glib_LoadFont(char filename[], unsigned int fontnumber, unsigned int height) 
{
    struct atlas *a;
    FT_Face      face;
    FT_Library   ft;
    
    if (FT_Init_FreeType(&ft)) 
    {
        fprintf(stderr, "Could not initialise freetype library\n");
        exit(-1);
    }

    if (FT_New_Face(ft, filename, 0, &face)) 
    {
        fprintf(stderr, "Could not open font file %s\n", filename);
        exit(-1);
    }

    if (fontnumber < 1 || fontnumber > MAXFONTFILES)
    {
        printf("Font %d out of range\n", fontnumber);
        exit(-1); 
    }
    
    if (FontTable[fontnumber] != NULL)
    {
        printf("Font %d clash\n", fontnumber);
        exit(-1);
    }
    
    a = (struct atlas *) malloc(sizeof(struct atlas));
    if (a == NULL)
    {
        fprintf(stderr, "No space for font atlas %s\n", filename);
        exit(-1);
    }
    
    CreateAtlas(a, face, height);
    FontTable[fontnumber] = a;

    //printf("Font %d Created: %s h=%d\n", fontnumber, filename, height);
}

/* ------------------------------------------------- */
void Glib_PushMatrix()
{
    if (stackp >= MAXSTACK)
	{
	    printf("PushMatrix overflow\n");
		exit(-1);
	}

	/*  no need to flush as Transform is not changed */
    glm_mat4_copy(Transform, stack[stackp]);
	stackp += 1;
}

/* ------------------------------------------------- */
void Glib_PopMatrix()
{
    if (stackp <= 0)
	{
	    printf("PopMatrix underflow\n");
		exit(-1);
	}

    Glib_Flush();
	stackp -= 1; 
    glm_mat4_copy(stack[stackp], Transform);
}

/* ------------------------------------------------- */
void Glib_LoadIdentity()
{
    Glib_Flush();
    glm_mat4_identity(Transform);
	#if (PORTRAIT)
	    Glib_Translate(1024.0, 0.0);
	    Glib_Rotate(90.0);
    #endif
}

/* ------------------------------------------------- */
void Glib_Translate(float x, float y) /* n.b. translation is 2D only */
{
    vec4 t = { x, y, 0.0, 0.0 };
	
	Glib_Flush();
    glm_translate(Transform, t);
} 

/* ------------------------------------------------- */
void Glib_Rotate(float a)   /* n.b. rotation is 2D only, about the z axis */
{
    Glib_Flush();
    glm_rotate(Transform, a / ONERAD, (vec4) { 0.0, 0.0, 1.0, 0.0 } );	
}

/* ------------------------------------------------- */
void Glib_SetNormal()   /* compute mat3 Normal from mat4 Transform */
{
    mat4 t;
	
    glm_mat4_copy(Transform, t);
	glm_mat4_inv(t, t);
	glm_mat4_transpose(t);
    glm_mat4_pick3(t, Normal);
}

/* ------------------------------------------------- */
void Glib_RotatePitch3D(float pitch)
{
    Glib_Flush();
    glm_rotate(Transform, pitch / ONERAD, (vec4) { 0.0f, 0.0f, 1.0f, 0.0f } );	
}

/* ------------------------------------------------- */
void Glib_RotateRoll3D(float roll)
{
    Glib_Flush();
    glm_rotate(Transform, roll / ONERAD, (vec4) { 1.0f, 0.0f, 0.0f, 0.0 } );	
}

/* ------------------------------------------------- */
void Glib_RotateYaw3D(float yaw)
{
    Glib_Flush();
    glm_rotate(Transform, yaw / ONERAD, (vec4) { 0.0f, 1.0f, 0.0f, 0.0 } );	
}

/* ------------------------------------------------- */
void printmatrix(mat4 m)
{
    glm_mat4_print(m, stdout); 
}

/* ------------------------------------------------- */
bool Glib_Errors() 
{
    int numErrors = 0;
    GLenum err;

    while ((err = glGetError()) != GL_NO_ERROR) 
    {
        numErrors += 1;
        int errNum = 0;
        switch (err) 
        {
            case GL_INVALID_ENUM:
                errNum = 1;
                break;
            case GL_INVALID_VALUE:
                errNum = 2;
                break;
            case GL_INVALID_OPERATION:
                errNum = 3;
                break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:
                errNum = 4;
                break;
            case GL_OUT_OF_MEMORY:
                errNum = 5;
                break;
            case GL_STACK_UNDERFLOW:
                errNum = 6;
                break;
            case GL_STACK_OVERFLOW:
                errNum = 7;
                break;
            case GL_CONTEXT_LOST:
                errNum = 8;
                break;
        }
        printf("OpenGL ERROR: %s.\n", errNames[errNum]);
    }
    return (numErrors != 0);
}

/* ------------------------------------------------- */
void Glib_Info()
{
   int major;
   int minor;
   int rev;
   
    glfwGetVersion(&major, &minor, &rev);
    printf("GLFW %d.%d revision %d\n", major, minor, rev);
    
    printf("Renderer: %s\n", glGetString(GL_RENDERER));
    printf("OpenGL version supported %s\n", glGetString(GL_VERSION));
    printf("Supported GLSL version is %s.\n", (char *) glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("Using GLEW version %s.\n", glewGetString(GLEW_VERSION));
}

/* ------------------------------------------------- */
void BEGIN_Glib()
{
    int i;
    
    for (i=0; i<=MAXFONTFILES; i+=1)
    {
        FontTable[i] = NULL;
    }
    
    for (i=0; i<=MAXTEXTUREFILES; i+=1)
    {
        TextureTable[i] = 0;
    }
    
    nvectors = 0;
    ntriangles = 0;
    ndots = 0;
    ntextures = 0;
    nstrings = 0;
    nsegments = 0;
	
	stackp = 0;
}
