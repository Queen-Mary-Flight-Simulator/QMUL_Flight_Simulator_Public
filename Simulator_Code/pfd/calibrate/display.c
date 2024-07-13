#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <GL/gl.h>

#include <SIM/iodefn.h>
#include <SIM/glib.h>

#include "display.h"

#define ONERAD (180.0 / M_PI)
#define intround(x) ((x) >= 0.0 ? (int)((x)+0.5) : (int)((x)-0.5))

int           AData[32];
unsigned char DataA;
unsigned char DataB;
unsigned char DataC;
unsigned char DataD;

int           OldAData[32];
int           Noise[32];
int           OldNoise[32];
int           Count100;
unsigned int  Wheel;

void circle(int ox, int oy, int r);
void LineRectangle(int x, int y, int xs, int ys);
void Mark(int x, int y, int inc, int a1, int a2, int r1, int r2);
void Mark2(int x, int y, int inc, int skipangle, int a1, int a2, int r1, int r2);
void Background();
void UpdateAnalogueDisplay(unsigned int n, int adcvalue, unsigned int noise);
void UpdateDigitalDisplay(unsigned int n, bool newvalue);
void UpdateWheel(int x, int y);

/* -----------------------------------------------*/
void circle(int ox, int oy, int r)
{
    int a;
    float x1, y1;
    float x2, y2;
    
    x1 = (float) ox + (float) r;
    y1 = (float) oy;

    for (a=0; a<=360; a+=2)
    {
        x2 = (float) ox + (float) r * cos((float) a / 57.29577951);
        y2 = (float) oy + (float) r * sin((float) a / 57.29577951);
        if (a > 0)
        {
            Glib_Draw(x1, y1, x2, y2);
        }
        x1 = x2;
        y1 = y2;
    }
}

/* -----------------------------------------------*/
void LineRectangle(int x, int y, int xs, int ys)
{
    Glib_Draw(x, y, x, y + ys);
    Glib_Draw(x, y + ys, x + xs, y + ys);
    Glib_Draw(x + xs, y + ys, x + xs, y);
    Glib_Draw(x + xs, y, x, y);
}

/* -----------------------------------------------*/
void Mark(int x, int y, int inc, int a1, int a2, int r1, int r2)
{
    int   x1, y1, x2, y2;
    float xx;

    a1 = a1 * 10;
    a2 = a2 * 10;
    while (a1 <= a2) 
    {
        xx = ((float) (a1) * 0.1) / ONERAD;
        x1 = intround(cos(xx) * (float) r1);
        y1 = intround(sin(xx) * (float) r1);
        x2 = intround(cos(xx) * (float) r2);
        y2 = intround(sin(xx) * (float) r2);
        Glib_Draw(x + x1, y + y1, x + x2, y + y2);
        a1 = a1 + inc;
    }
}

/* -----------------------------------------------*/
void Mark2(int x, int y, int inc, int skipangle, int a1, int a2, int r1, int r2)
{
    int   a, x1, y1, x2, y2;
    float xx;

    a1 = a1 * 10;
    a2 = a2 * 10;
    a = a1;
    while (a <= a2) 
	{
        xx = ((float) (a) * 0.1) / ONERAD;
        //xx = Maths_Normalise(xx);
        x1 = intround(cos(xx) * (float) r1);
        y1 = intround(sin(xx) * (float) r1);
        x2 = intround(cos(xx) * (float) r2);
        y2 = intround(sin(xx) * (float) r2);
        if ((a - a1) % skipangle != 0) 
		{
            Glib_Draw(x + x1, y + y1, x + x2, y + y2);
        }
        a = a + inc;
    }
}

/* -----------------------------------------------*/
void Background()
{
    int  i;
	
    Glib_AntiAliasing(true);
    Glib_LineWidth(1.0);

    for (i = 0; i < 32; i += 1)
    {
        int x = 0 + (i % 8) * 128 + 64;
        int y = 768 - (i /8) * 150 - 100;
        
        Glib_LineWidth(1.0);
        Glib_Colour(Glib_WHITE);
        circle(x, y, 60);

        Mark(x, y, 1280, -38, 218, 42, 60);
        Mark2(x, y, 128, 1280, -38, 218, 52, 60);
	}
}

/* -----------------------------------------------*/
void UpdateAnalogueDisplay(unsigned int n, int adcvalue, unsigned int noise)
{
    int   x1, y1, x2, y2;
    float a;
    char  str[10];
    int   d;
	
    Glib_LineWidth(2.0);
    x1 = 0 + (n % 8) * 128 + 64;
    y1 = 768 - (n /8) * 150 - 100;
    a = (218.0 - (float) (adcvalue) / 16.0) / ONERAD;
    x2 = x1 + intround(cos(a) * 60.0);
    y2 = y1 + intround(sin(a) * 60.0);
    Glib_Colour(Glib_WHITE);
    Glib_Draw(x1, y1, x2, y2);
    
    Glib_Colour(Glib_GREEN);
    str[0] = 'P';
    str[1] = (n + 1) / 10 + '0';
    str[2] = (n + 1) % 10 + '0';
    str[3] = '\0'; 
    Glib_Chars(str, x1 - 9, y1 - 30);

    sprintf(str, "%-4d/", adcvalue);
	d = Glib_StringSize(str);
    Glib_Chars(str, x1 - 35, y1 - 45);
    Glib_Colour(Glib_YELLOW);
    sprintf(str, "%-3d", noise);
    Glib_Chars(str, x1 - 35 + d, y1 - 45);
}

/* -----------------------------------------------*/
void UpdateDigitalDisplay(unsigned int n, bool newvalue)
{
    unsigned int x, y;

    x = 125 + n % 16 * 50 + 5;
    y = 55 - n / 16 * 40 + 5;
    if (newvalue) 
    {
        Glib_Colour(Glib_RED);
    } 
    else 
    {
        Glib_Colour(Glib_BLUE);
    }
    Glib_LineWidth(2.0);
    Glib_Rectangle(x, y, 38, 25);
    Glib_Colour(Glib_WHITE);
    LineRectangle(x, y, 38, 25);
    //Glib_LineWidth(1.0);
    Glib_Char(n / 8 + 'A', x + 11, y + 7);
    Glib_Char(n % 8 + '0', x + 21, y + 7);
}

/* -----------------------------------------------*/
void UpdateWheel(int x, int y)
{
    int i;
    
    Wheel +=10;
    if (Wheel >= 180)
    {
        Wheel = 0;
    }
    
    Glib_Colour(Glib_RED);
    for (i=0; i<=90; i+=1)
    {
        float a = (float) (i + Wheel) / ONERAD;
        float dx = cos(a) * 10.0;
        float dy = sin(a) * 10.0;
        
        Glib_Colour(Glib_RED);
        Glib_Draw(x, y, x + dx, y + dy);
        Glib_Draw(x, y, x - dx, y - dy);
        Glib_Colour(Glib_GREEN);
        Glib_Draw(x, y, x - dy, y + dx);
        Glib_Draw(x, y, x + dy, y - dx);
    }
}

/* -----------------------------------------------*/
void Display_Update(IODefn_IODataPkt IOPkt1)
{
    unsigned int p = 1;
    unsigned int i;
    bool         DigitalData;
	
    Background();

    for (i = 0; i < 32; i += 1) 
    {
        AData[i] = (unsigned int) IOPkt1.AnalogueData[i];
    }

    DataA = IOPkt1.DigitalDataA;
    DataB = IOPkt1.DigitalDataB;
    DataC = IOPkt1.DigitalDataC;
    DataD = IOPkt1.DigitalDataD;

    for (i = 0; i < 32; i += 1) 
    {
        Noise[i] = Noise[i] + abs((AData[i] - OldAData[i]) / 16);
        if (Noise[i] > 999) 
        {
            Noise[i] = 999;
        }

        UpdateAnalogueDisplay(i, AData[i], OldNoise[i]);
        OldAData[i] = AData[i];
    }

    for (i = 0; i <= 31; i += 1) 
    {
        if (i < 8) 
        {
            DigitalData = p & DataA;
        } 
        else if (i < 16) 
        {
            DigitalData = p & DataB;
        } 
        else if (i < 24) 
        {
            DigitalData = p & DataC;
        } 
        else 
        {
            DigitalData = p & DataD;
        }
        p = p * 2;
        if (p >= 256) 
        {
            p = 1;
        }
        UpdateDigitalDisplay(i, DigitalData);
    }

    UpdateWheel(40, 40);

    Count100 = Count100 + 1;
    if (Count100 >= 100) 
    {
        for (i = 0; i < 32; i += 1) 
        {
            OldNoise[i] = Noise[i];
            Noise[i] = 0;
        }
        Count100 = 0;
    }
}

/* -----------------------------------------------*/
void BEGIN_Display()
{
    unsigned int i;

    for (i = 0; i < 32; i += 1) 
    {
        AData[i] = 0;
        OldAData[i] = 0;
        Noise[i] = 0;
        OldNoise[i] = 0;
    }

    Count100 = 0;
    Wheel = 0;
}
