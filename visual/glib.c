#include <GL/gl.h>
#include "glib.h"

Glib_PtrProc3 Glib_Char;

typedef GLfloat RGBValues[3];

static RGBValues ColourTable[10] = {
  {0.00, 0.00, 0.00},
  {0.83, 0.83, 0.83},
  {0.0,  0.25, 0.65}, 
  {0.31, 0.31, 0.31},
  {0.83, 0.00, 0.00},
  {0.00, 0.83, 0.00},
  {0.83, 0.00, 0.83},
  {0.44, 0.27, 0.06},
  {0.00, 0.83, 0.83},
  {0.83, 0.83, 0.00} };

static unsigned int gfont8Offsets[128] = {
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    4,   14,   18,   36, 
    42,   60,   82,   90,  108,  130,  138,  172,  194,  194, 
   194,  194,  194,  194,  194,  194,  208,  230,  250,  264, 
   278,  290,  310,  322,  334,  348,  362,  368,  378,  386, 
   404,  418,  436,  456,  480,  488,  500,  506,  516,  526, 
   536,  544,  544,  544,  544,  544,  544,  544,  544,  544, 
   544,  544,  544,  544,  544,  544,  544,  544,  544,  544, 
   544,  544,  544,  544,  544,  544,  544,  544,  544,  544, 
   544,  544,  544,  544,  544,  544,  544,  544};

static unsigned int gfont8Size[128] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   2,   5,   2,   9,   3, 
    9,  11,   4,   9,  11,   4,  17,  11,   0,   0, 
    0,   0,   0,   0,   0,   7,  11,  10,   7,   7, 
    6,  10,   6,   6,   7,   7,   3,   5,   4,   9, 
    7,   9,  10,  12,   4,   6,   3,   5,   5,   5, 
    4,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0};

static unsigned char gfont8Tab[544] = {
 0, 4, 4, 4, 4, 1, 5, 1, 5, 0, 4, 0, 4, 1, 0, 0, 4, 8, 0, 1,
 0, 7, 1, 8, 3, 8, 4, 7, 4, 1, 3, 0, 1, 0, 0, 1, 0, 6, 2, 8,
 2, 0, 0, 5, 0, 7, 1, 8, 3, 8, 4, 7, 4, 5, 0, 2, 0, 0, 4, 0,
 0, 7, 1, 8, 3, 8, 4, 7, 4, 5, 1, 4, 4, 3, 4, 1, 3, 0, 1, 0,
 0, 1, 3, 0, 3, 8, 0, 3, 4, 3, 4, 8, 0, 8, 0, 5, 3, 5, 4, 4,
 4, 1, 3, 0, 1, 0, 0, 1, 4, 7, 3, 8, 1, 8, 0, 7, 0, 1, 1, 0,
 3, 0, 4, 1, 4, 3, 3, 4, 0, 4, 0, 8, 4, 8, 4, 5, 0, 0, 3, 4,
 1, 4, 0, 3, 0, 1, 1, 0, 3, 0, 4, 1, 4, 3, 3, 4, 1, 4, 0, 5,
 0, 7, 1, 8, 3, 8, 4, 7, 4, 5, 3, 4, 0, 1, 1, 0, 3, 0, 4, 1,
 4, 7, 3, 8, 1, 8, 0, 7, 0, 5, 1, 4, 4, 4, 0, 0, 0, 4, 2, 8,
 4, 4, 4, 0, 4, 4, 0, 4, 2, 4, 4, 3, 4, 1, 3, 0, 0, 0, 0, 8,
 3, 8, 4, 7, 4, 5, 2, 4, 0, 4, 4, 5, 4, 7, 3, 8, 1, 8, 0, 7,
 0, 1, 1, 0, 3, 0, 4, 1, 4, 3, 0, 8, 0, 0, 3, 0, 4, 1, 4, 7,
 3, 8, 0, 8, 4, 0, 0, 0, 0, 8, 4, 8, 0, 8, 0, 4, 4, 4, 0, 0,
 0, 8, 4, 8, 0, 8, 0, 4, 4, 4, 4, 7, 3, 8, 1, 8, 0, 7, 0, 1,
 1, 0, 3, 0, 4, 1, 4, 4, 2, 4, 0, 0, 0, 8, 0, 4, 4, 4, 4, 8,
 4, 0, 0, 0, 4, 0, 2, 0, 2, 8, 4, 8, 0, 8, 0, 8, 4, 8, 2, 8,
 2, 0, 1, 0, 0, 1, 0, 3, 0, 0, 0, 8, 0, 4, 2, 4, 4, 8, 2, 4,
 4, 0, 0, 8, 0, 0, 4, 0, 0, 0, 0, 8, 2, 4, 4, 8, 4, 0, 0, 0,
 0, 8, 4, 0, 4, 8, 0, 1, 0, 7, 1, 8, 3, 8, 4, 7, 4, 1, 3, 0,
 1, 0, 0, 1, 0, 0, 0, 8, 3, 8, 4, 7, 4, 5, 3, 4, 0, 4, 4, 0,
 1, 0, 0, 1, 0, 7, 1, 8, 3, 8, 4, 7, 4, 0, 1, 3, 0, 0, 0, 8,
 3, 8, 4, 7, 4, 5, 3, 4, 0, 4, 3, 4, 4, 3, 4, 0, 4, 7, 3, 8,
 1, 8, 0, 7, 0, 5, 1, 4, 3, 4, 4, 3, 4, 1, 3, 0, 1, 0, 0, 1,
 0, 8, 4, 8, 2, 8, 2, 0, 0, 8, 0, 1, 1, 0, 3, 0, 4, 1, 4, 8,
 0, 8, 2, 0, 4, 8, 0, 8, 0, 0, 2, 4, 4, 0, 4, 8, 0, 8, 4, 0,
 2, 4, 4, 8, 0, 0, 0, 8, 2, 4, 4, 8, 2, 4, 2, 0, 0, 8, 4, 8,
 0, 0, 4, 0};

static unsigned int gfont12Offsets[128] = {
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,    0,   10,   10,   28, 
    34,   50,   76,   84,  102,  126,  132,  164,  186,  186, 
   186,  186,  186,  186,  186,  186,  200,  220,  236,  250, 
   264,  276,  296,  308,  320,  334,  346,  352,  362,  370, 
   388,  402,  424,  442,  466,  474,  480,  486,  496,  506, 
   516,  524,  524,  524,  524,  524,  524,  524,  524,  524, 
   524,  524,  524,  524,  524,  524,  524,  524,  524,  524, 
   524,  524,  524,  524,  524,  524,  524,  524,  524,  524, 
   524,  524,  524,  524,  524,  524,  524,  524};

static unsigned int gfont12Size[128] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   5,   0,   9,   3, 
    8,  13,   4,   9,  12,   3,  16,  11,   0,   0, 
    0,   0,   0,   0,   0,   7,  10,   8,   7,   7, 
    6,  10,   6,   6,   7,   6,   3,   5,   4,   9, 
    7,  11,   9,  12,   4,   3,   3,   5,   5,   5, 
    4,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0};

static unsigned char gfont12Tab[524] = {
 3, 0, 5, 0, 5, 2, 3, 2, 3, 0, 0, 2, 0,10, 2,12, 4,12, 6,10,
 6, 2, 4, 0, 2, 0, 0, 2, 0, 8, 3,12, 3, 0, 0,10, 2,12, 4,12,
 6,10, 6, 7, 0, 2, 0, 0, 6, 0, 0,10, 2,12, 4,12, 6,10, 6, 8,
 4, 6, 3, 6, 4, 6, 6, 4, 6, 2, 4, 0, 2, 0, 0, 2, 4, 0, 4,12,
 0, 5, 6, 5, 0, 2, 2, 0, 4, 0, 6, 2, 6, 6, 4, 8, 0, 8, 0,12,
 6,12, 6,10, 4,12, 2,12, 0,10, 0, 6, 0, 2, 2, 0, 4, 0, 6, 2,
 6, 4, 4, 6, 0, 6, 0,12, 6,12, 0, 0, 1, 6, 0, 8, 0,10, 2,12,
 4,12, 6,10, 6, 8, 4, 6, 1, 6, 0, 4, 0, 2, 2, 0, 4, 0, 6, 2,
 6, 4, 4, 6, 0, 2, 2, 0, 4, 0, 6, 2, 6,10, 4,12, 2,12, 0,10,
 0, 8, 2, 6, 6, 6, 0, 0, 0, 8, 3,12, 6, 8, 6, 0, 6, 5, 0, 5,
 0, 0, 0,11, 4,11, 6, 9, 6, 7, 4, 5, 6, 4, 6, 2, 4, 0, 0, 0,
 6,10, 4,12, 2,12, 0,10, 0, 2, 2, 0, 4, 0, 6, 2, 0, 0, 4, 0,
 6, 2, 6,10, 4,12, 0,12, 0, 0, 6,11, 0,11, 0, 6, 6, 6, 0, 6,
 0, 0, 6, 0, 6,11, 0,11, 0, 6, 6, 6, 0, 6, 0, 0, 6,10, 4,12,
 2,12, 0,10, 0, 2, 2, 0, 4, 0, 6, 2, 6, 6, 2, 6, 0, 0, 0,11,
 0, 6, 6, 6, 6,11, 6, 0, 2,11, 6,11, 4,11, 4, 0, 6, 0, 2, 0,
 0, 2, 1, 0, 3, 0, 4, 2, 4,11, 2,11, 6,11, 0,11, 0, 0, 0, 5,
 6, 9, 0, 5, 6, 0, 0,11, 0, 0, 6, 0, 0, 0, 0,12, 3, 7, 6,12,
 6, 0, 0, 0, 0,11, 6, 0, 6,11, 0, 2, 0,10, 2,12, 4,12, 6,10,
 6, 2, 4, 0, 2, 0, 0, 2, 0, 0, 0,11, 4,11, 6, 9, 6, 6, 4, 5,
 0, 5, 5, 1, 4, 0, 2, 0, 0, 2, 0,10, 2,12, 4,12, 6,10, 6, 2,
 5, 1, 3, 3, 0, 0, 0,11, 4,11, 6, 9, 6, 6, 4, 5, 0, 5, 4, 5,
 6, 0, 6, 9, 4,11, 2,11, 0, 9, 0, 8, 2, 6, 4, 6, 6, 4, 6, 2,
 4, 0, 2, 0, 0, 2, 0,11, 6,11, 3,11, 3, 0, 0,11, 3, 0, 6,11,
 0,11, 3, 0, 6,11, 0,11, 0, 0, 3, 7, 6, 0, 6,11, 0,11, 6, 0,
 3, 5, 6,11, 0, 0, 0,11, 3, 7, 6,11, 3, 7, 3, 0, 0,11, 6,11,
 0, 0, 6, 0};

static unsigned int gfont16Offsets[128] = {
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,   18,   18,   18,   18, 
    18,   18,   18,   18,   18,   18,   18,   18,   18,   18, 
    18,   18,   18,   18,   18,   18,   18,   18,   18,   18, 
    18,   18,   18,   18,   18,   18,   18,   28,   28,   46, 
    52,   70,   96,  104,  124,  148,  156,  188,  212,  212, 
   212,  212,  212,  212,  212,  212,  230,  252,  272,  286, 
   300,  312,  334,  346,  358,  372,  384,  390,  400,  408, 
   426,  440,  464,  482,  506,  514,  526,  532,  542,  552, 
   562,  570,  570,  570,  570,  570,  570,  570,  570,  570, 
   570,  570,  570,  570,  570,  570,  570,  570,  570,  570, 
   570,  570,  570,  570,  570,  570,  570,  570,  570,  570, 
   570,  570,  570,  570,  570,  570,  570,  570};

static unsigned int gfont16Size[128] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   9,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   5,   0,   9,   3, 
    9,  13,   4,  10,  12,   4,  16,  12,   0,   0, 
    0,   0,   0,   0,   0,   9,  11,  10,   7,   7, 
    6,  11,   6,   6,   7,   6,   3,   5,   4,   9, 
    7,  12,   9,  12,   4,   6,   3,   5,   5,   5, 
    4,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0};

static unsigned char gfont16Tab[570] = {
 0, 2, 0, 6, 2, 8, 6, 8, 8, 6, 8, 2, 6, 0, 2, 0, 0, 2, 4, 0,
 6, 0, 6, 2, 4, 2, 4, 0, 0, 2, 0,13, 2,15, 5,15, 7,13, 7, 2,
 5, 0, 2, 0, 0, 2, 4, 0, 4,15, 0, 9, 7, 0, 0, 0, 0, 4, 7,10,
 7,13, 6,15, 2,15, 0,13, 0,11, 0,10, 0,13, 2,15, 5,15, 7,13,
 7,10, 4, 8, 7, 6, 7, 2, 5, 0, 2, 0, 0, 2, 0, 4, 5, 0, 5,15,
 0, 5, 7, 5, 7,15, 0,15, 0,10, 5,10, 7, 8, 7, 2, 5, 0, 2, 0,
 0, 2, 0, 4, 7,11, 7,13, 5,15, 2,15, 0,13, 0, 2, 2, 0, 5, 0,
 7, 2, 7, 6, 5, 8, 0, 8, 0,15, 7,15, 7,11, 0, 0, 2, 8, 0,10,
 0,13, 2,15, 5,15, 7,13, 7,10, 5, 8, 7, 6, 7, 2, 5, 0, 2, 0,
 0, 2, 0, 6, 2, 8, 5, 8, 0, 4, 0, 2, 2, 0, 5, 0, 7, 2, 7,13,
 5,15, 2,15, 0,13, 0,10, 2, 8, 7, 8, 6, 0, 6, 7, 6,12, 3,15,
 0,12, 0, 7, 0, 0, 0, 7, 6, 7, 4, 8, 7,10, 7,13, 5,15, 0,15,
 0, 0, 5, 0, 7, 2, 7, 6, 4, 8, 0, 8, 7,11, 7,13, 5,15, 2,15,
 0,13, 0, 2, 2, 0, 5, 0, 7, 2, 7, 4, 0,15, 0, 0, 5, 0, 7, 2,
 7,13, 5,15, 0,15, 7, 8, 0, 8, 0,15, 7,15, 0,15, 0, 0, 7, 0,
 7,15, 0,15, 0, 8, 7, 8, 0, 8, 0, 0, 7,11, 7,13, 5,15, 2,15,
 0,13, 0, 2, 2, 0, 5, 0, 7, 2, 7, 8, 2, 8, 7, 0, 7,15, 7, 8,
 0, 8, 0,15, 0, 0, 1,15, 5,15, 3,15, 3, 0, 1, 0, 5, 0, 0, 4,
 0, 2, 2, 0, 4, 2, 4,15, 1,15, 7,15, 7,13, 0, 7, 0,15, 0, 0,
 0, 7, 7, 0, 0,15, 0, 0, 7, 0, 0, 0, 0,15, 3, 8, 6,15, 6, 0,
 0, 0, 0,15, 7, 0, 7,15, 0, 2, 0,13, 2,15, 5,15, 7,13, 7, 2,
 5, 0, 2, 0, 0, 2, 0, 0, 0,15, 5,15, 7,13, 7,10, 5, 8, 0, 8,
 7, 2, 7,13, 5,15, 2,15, 0,13, 0, 2, 2, 0, 5, 0, 7, 2, 6, 1,
 7, 0, 3, 4, 0, 0, 0,15, 5,15, 7,13, 7,10, 5, 8, 0, 8, 5, 8,
 7, 0, 7,11, 7,13, 5,15, 2,15, 0,13, 0,11, 7, 4, 7, 2, 5, 0,
 2, 0, 0, 2, 0, 4, 3, 0, 3,15, 6,15, 0,15, 0,15, 0, 2, 2, 0,
 5, 0, 7, 2, 7,15, 0,15, 3, 0, 6,15, 0,15, 0, 0, 3, 7, 6, 0,
 6,15, 0,15, 6, 0, 3, 7, 6,15, 0, 0, 0,15, 3, 7, 6,15, 3, 7,
 3, 0, 0,15, 7,15, 0, 0, 7, 0};

static unsigned int gfont24Offsets[128] = {
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
     0,    0,    0,    0,    0,    0,   18,   18,   18,   18, 
    18,   18,   18,   18,   18,   18,   18,   18,   18,   18, 
    18,   18,   18,   18,   18,   18,   18,   18,   18,   18, 
    18,   18,   18,   18,   18,   18,   18,   18,   18,   44, 
    50,   70,  108,  116,  140,  176,  182,  230,  266,  266, 
   266,  266,  266,  266,  266,  266,  266,  266,  266,  288, 
   288,  288,  288,  288,  288,  288,  288,  288,  288,  288, 
   288,  288,  288,  288,  324,  332,  332,  332,  332,  332, 
   332,  332,  332,  332,  332,  332,  332,  332,  332,  332, 
   332,  332,  332,  332,  332,  332,  332,  332,  332,  332, 
   332,  332,  332,  332,  332,  332,  332,  332,  332,  332, 
   332,  332,  332,  332,  332,  332,  332,  332};

static unsigned int gfont24Size[128] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   9,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,  13,   3, 
   10,  19,   4,  12,  18,   3,  24,  18,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,  11,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,  18,   4,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 
    0,   0,   0,   0,   0,   0,   0,   0};

static unsigned char gfont24Tab[332] = {
 2, 0, 0, 2, 0, 4, 2, 6, 4, 6, 6, 4, 6, 2, 4, 0, 2, 0, 0,20,
 0, 3, 1, 1, 3, 0, 6, 0, 8, 1, 9, 3, 9,20, 8,22, 6,23, 3,23,
 1,22, 0,20, 5, 0, 5,23, 0,18, 0,20, 1,22, 3,23, 6,23, 8,22,
 9,20, 9,14, 0, 3, 0, 0, 9, 0, 0,20, 1,22, 3,23, 6,23, 8,22,
 9,20, 9,14, 8,12, 6,11, 4,11, 6,11, 8,10, 9, 8, 9, 3, 8, 1,
 6, 0, 3, 0, 1, 1, 0, 3, 6, 0, 6,23, 0, 8, 9, 8, 9,23, 0,23,
 0,16, 6,16, 8,15, 9,13, 9, 3, 8, 1, 6, 0, 3, 0, 1, 1, 0, 3,
 9,20, 8,22, 6,23, 3,23, 1,22, 0,20, 0, 3, 1, 1, 3, 0, 6, 0,
 8, 1, 9, 3, 9, 9, 8,11, 6,12, 3,12, 1,11, 0, 9, 0,23, 9,23,
 0, 0, 3,12, 1,14, 0,16, 0,20, 1,22, 3,23, 6,23, 8,22, 9,20,
 9,16, 8,14, 6,12, 3,12, 1,11, 0, 9, 0, 3, 1, 1, 3, 0, 6, 0,
 8, 1, 9, 3, 9, 9, 8,11, 6,12, 0, 3, 1, 1, 3, 0, 6, 0, 8, 1,
 9, 3, 9,20, 8,22, 6,23, 3,23, 1,22, 0,20, 0,16, 1,14, 3,12,
 6,12, 8,14, 9,16, 0, 0, 0,23, 3,23, 6,23, 8,22, 9,20, 9, 3,
 8, 1, 6, 0, 3, 0, 0, 0, 9,20, 8,22, 6,23, 3,23, 1,22, 0,20,
 0,15, 1,13, 3,12, 6,12, 8,11, 9, 9, 9, 3, 8, 1, 6, 0, 3, 0,
 1, 1, 0, 3, 5, 0, 5,23, 9,23, 0,23};

static int FontSpacing;
static void gfont8Char(char Ch, int x, int y);
static void gfont12Char(char Ch, int x, int y);
static void gfont16Char(char Ch, int x, int y);
static void gfont24Char(char Ch, int x, int y);

static void gfont8Char(char Ch, int x, int y)
{
  unsigned int offset;
  unsigned int size;
  unsigned int i;

  offset = (unsigned int) gfont8Offsets[Ch];
  size = (unsigned int) gfont8Size[Ch];
  glBegin(GL_LINE_STRIP);
  {
      for (i = 1; i <= size; i += 1) {
        glVertex2i(x + (int) gfont8Tab[offset], y + (int) gfont8Tab[offset + 1]);
        offset = offset + 2;
      }
  }
  glEnd();
}

static void gfont12Char(char Ch, int x, int y)
{
  unsigned int offset;
  unsigned int size;
  unsigned int i;

  offset = gfont12Offsets[Ch];
  size = gfont12Size[Ch];
  glBegin(GL_LINE_STRIP);
  {
      for (i = 1; i <= size; i += 1) {
        glVertex2i(x + (int) gfont12Tab[offset], y + (int) gfont12Tab[offset + 1]);
        offset = offset + 2;
      }
  }
  glEnd();
}

static void gfont16Char(char Ch, int x, int y)
{
  unsigned int offset;
  unsigned int size;
  unsigned int i;

  offset = gfont16Offsets[Ch];
  size = gfont16Size[Ch];
  glBegin(GL_LINE_STRIP);
  {
      for (i = 1; i <= size; i += 1) {
        glVertex2i(x + (int) gfont16Tab[offset], y + (int) gfont16Tab[offset + 1]);
        offset = offset + 2;
      }
  }
  glEnd();
}

static void gfont24Char(char Ch, int x, int y)
{
  unsigned int offset;
  unsigned int size;
  unsigned int i;

  offset = gfont24Offsets[Ch];
  size = gfont24Size[Ch];
  glBegin(GL_LINE_STRIP);
  {
      for (i = 1; i <= size; i += 1) {
        glVertex2i(x + (int) gfont24Tab[offset], y + (int) gfont24Tab[offset + 1]);
        offset = offset + 2;
      }
  }
  glEnd();
}

void Glib_SetFont(unsigned int font, int spacing)
{
  switch (font) {
  case 1:;
    Glib_Char = gfont8Char;
    break;
  case 2:;
    Glib_Char = gfont12Char;
    break;
  case 3:;
    Glib_Char = gfont16Char;
    break;
  case 4:;
    Glib_Char = gfont24Char;
    break;
  }
  FontSpacing = spacing;
}

void Glib_Chars(char str[], int x, int y)
{
  unsigned int p;
  char Ch;

  p = 0;
  for (;;) {
    Ch = str[p];
    if (Ch == 0) {
      break;
    } else {
      Glib_Char(Ch, x, y);
      x = x + FontSpacing;
      p = p + 1;
    }
  }
}

void Glib_Colour(unsigned int col)
{
  glColor3fv(ColourTable[col]);
}

void Glib_Draw(int x1, int y1, int x2, int y2)
{
  glBegin(GL_LINES);
  glVertex2i(x1, y1);
  glVertex2i(x2, y2);
  glEnd();
}

void Glib_DrawPolygon(unsigned int n, int x, int y, int v[])
{
  int dx;
  int dy;
  unsigned int i;
  unsigned int p;
  
  p = 0;
  if (n >= 3) {
    glBegin(GL_POLYGON);
    {
        for (i = 0; i <= n - 1; i += 1) {
          dx = v[p];
          p = p + 1;
          dy = v[p];
          p = p + 1;
          glVertex2i(x + dx, y + dy);
        }
    }
    glEnd();
  }
}

void Glib_DrawLines(unsigned int n, int x, int y, int v[])
{
  int dx;
  int dy;
  unsigned int i;
  unsigned int p;

  p = 0;
  
  if (n > 1) {
    glBegin(GL_LINE_STRIP);
    {
        for (i = 0; i <= n - 1; i += 1) {
          dx = v[p];
          p = p + 1;
          dy = v[p];
          p = p + 1;
          glVertex2i(x + dx, y + dy);
        }
    }
    glEnd();
  }
}

void Glib_LineWidth(float w)
{
  glLineWidth((GLfloat) w);
  glPointSize((GLfloat) w);
}

void Glib_Rectangle(int x1, int y1, int xside, int yside)
{
  glRecti(x1, y1, x1 + xside, y1 + yside);
}

void Glib_AntiAliasing(boolean Mode)
{
  if (Mode) {
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  } else {
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);
  }
}

void Glib_ClipWindow(int x, int y, int xs, int ys)
{
  glEnable(GL_SCISSOR_TEST);
  glScissor(x, y, xs, ys);
}

void BEGIN_Glib()
{
    Glib_Char = gfont16Char;
    FontSpacing = 10;
}
