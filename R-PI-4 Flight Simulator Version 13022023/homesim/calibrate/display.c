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

int           OldAData[MaxChannels];
int           Noise[MaxChannels];
int           OldNoise[MaxChannels];
int           Count100;
float         Wheel;

void Background();
void UpdateAnalogueDisplay(unsigned int n, int adcvalue, unsigned int noise);
void UpdateDigitalDisplay(unsigned int n, bool newvalue);
void UpdateWheel(int x, int y);
void LineRectangle(int x, int y, int xs, int ys);

/* -----------------------------------------------*/
void Background()
{
    int  i;

    Glib_SetTexture(1);
    
    for (i = 0; i <= MaxChannels - 1; i += 1)
    {
        int x = 200 + (i % 4) * 220 - 110;
        int y = 650 - (i / 4) * 220 - 110;
        
        Glib_DrawTexture(x, y, 220, 220, 0.2-0.11, 0.2-0.11, 0.2+0.11, 0.2+0.11, 1.0); 
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
    x1 = 200 + (n % 4) * 220;
    y1 = 650 - (n / 4) * 220;
    a = ((float) (adcvalue) / 200.0) / ONERAD;  /* 30000 = 150 degrees */
    x2 = x1 + intround(sin(a) * 100.0);
    y2 = y1 + intround(cos(a) * 100.0);
    Glib_Colour(Glib_WHITE);
    Glib_Draw(x1, y1, x2, y2);
    
    Glib_Colour(Glib_GREEN);
    str[0] = 'P';
    str[1] = (n + 1) / 10 + '0';
    str[2] = (n + 1) % 10 + '0';
    str[3] = '\0'; 
    Glib_Chars(str, x1 - 14, y1 - 30);

    sprintf(str, "%-6d/", adcvalue);
	d = Glib_StringSize(str);
    Glib_Chars(str, x1 - 40, y1 - 50);
    Glib_Colour(Glib_YELLOW);
    sprintf(str, "%-3d", noise);
    Glib_Chars(str, x1 - 40 + d, y1 - 50);
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
    Wheel +=10.0;
    if (Wheel >= 180.0)
    {
        Wheel = 0.0;
    }
    
	Glib_SetTexture(1);
	Glib_DrawTextureRotated(x, y, 50, 50, 0.5-0.1, 0.2-0.1, 0.5+0.1, 0.2+0.1, Wheel, 0.1);
}

/* -----------------------------------------------*/
void Display_Update(IODefn_IODataPkt IOPkt1)
{
    unsigned int p = 1;
    unsigned int i;
    bool         DigitalData;
	
    Background();

    for (i = 0; i <= MaxChannels - 1; i += 1) 
    {
        int a = IOPkt1.AnalogueData[i] & 0xff;
        int b = (IOPkt1.AnalogueData[i] >> 8) & 0xff;
        int x = (b << 8) | a;
        if ((x >> 15) != 0)
        {
            x |= 0xffff0000;
        }
        AData[i] = x;
    }

    DataA = IOPkt1.DigitalDataA;
    DataB = IOPkt1.DigitalDataB;
    DataC = IOPkt1.DigitalDataC;
    DataD = IOPkt1.DigitalDataD;

    for (i = 0; i <= MaxChannels - 1; i += 1) 
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

    UpdateWheel(60, 50);

    Count100 = Count100 + 1;
    if (Count100 >= 100) 
    {
        for (i = 0; i <= MaxChannels - 1; i += 1) 
        {
            OldNoise[i] = Noise[i];
            Noise[i] = 0;
        }
        Count100 = 0;
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
void BEGIN_Display()
{
    unsigned int i;

    for (i = 0; i <= MaxChannels - 1; i += 1) 
    {
        AData[i] = 0;
        OldAData[i] = 0;
        Noise[i] = 0;
        OldNoise[i] = 0;
    }

    Count100 = 0;
    Wheel = 0;
}
