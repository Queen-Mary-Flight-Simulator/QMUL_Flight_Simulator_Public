/* +------------------------------+---------------------------------+
   | Module      : diagnostics.c  | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 02-01-08      |
   +------------------------------+---------------------------------+
   | Computer    : DELL1                                            |
   | Directory   : /dja/aerosoft/cranfield/software/pfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Real-time performance display (optional)         |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdlib.h>
#include <stdio.h>

#include <GLFW/glfw3.h>

#include <SIM/glib.h>

/* frametime in usec
   20 ms = 20000 us = 1000 pix */

static double Timer1 = 0.0;
static double Timer2 = 0.0;
static double Timer3 = 0.0;
static double oldtimer = 0.0;

/* --------------------------------------------- */   
void Diagnostics_SetTimer1()
{
    Timer1 = glfwGetTime();
}

/* --------------------------------------------- */   
void Diagnostics_SetTimer2()
{
    Timer2 = glfwGetTime();
//    AeroLink_AddFlightData(1, (Timer2 - Timer1) * 1000000.0); // *************
}

/* --------------------------------------------- */   
void Diagnostics_SetTimer3()
{
    Timer3 = glfwGetTime();
}

/* --------------------------------------------- */   
void Diagnostics_Diagnostics(int x, int y, int width)
{
    int i;
    int ftime;
    char str[20];
	
    Glib_LoadIdentity();  /* reset to absolute coords */

    ftime = (int) ((Timer3 - Timer1) * 1000000.0); /* frame time in usec */
    Glib_Colour(Glib_GREEN);
    Glib_Rectangle(x, y, ftime * width / 20000, 8);
	
    ftime = (int) ((Timer3 - Timer2) * 1000000.0); /* frame time in usec */
    Glib_Colour(Glib_YELLOW);
    Glib_Rectangle(x, y+8, ftime * width / 20000, 8);
    Glib_Flush();
	
    Glib_Colour(Glib_GREY);
    Glib_Draw(x, y, x + width, y);
    for (i=0; i<=20000; i+=1000)
    {
        Glib_Draw(x + i * width / 20000, y, x + i * width / 20000, y + 20);
    }

	sprintf(str, "%6.2f FPS", 1.0 / (Timer1 - oldtimer));
    Glib_SetFont(1, 0);
	Glib_Chars(str, x + width + 10, y + 5);
    oldtimer = Timer1;
}

/* --------------------------------------------- */   
void BEGIN_Diagnostics()
{
}
