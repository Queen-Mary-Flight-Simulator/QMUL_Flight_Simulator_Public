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

#include <GL/gl.h>

#include <stdbool.h>

#include <GLFW/glfw3.h>

#include <SIM/maths.h>
#include <SIM/glib.h>
#include "aerolink.h"

/* frametime in usec
   20 ms = 20000 us = 1000 pix */

double Timer1 = 0.0;
double Timer2 = 0.0;

/* --------------------------------------------- */   
void Diagnostics_SetTimer1()
{
    Timer1 = glfwGetTime();
}

/* --------------------------------------------- */   
void Diagnostics_SetTimer2()
{
    Timer2 = glfwGetTime();
    AeroLink_AddFlightData(1, (Timer2 - Timer1) * 1000000.0); // *************
}

/* --------------------------------------------- */   
void Diagnostics_Diagnostics(int x, int y, int width)
{
    int i;
    int ftime;
    
    ftime = (int) ((Timer2 - Timer1) * 1000000.0); /* frame time in usec */
    
	Glib_Colour(Glib_YELLOW);
    Glib_LineWidth(10.0);
    Glib_Draw(x, y+6, x+ftime * width / 20000, y+6); 

    Glib_Colour(Glib_GREY);
    Glib_LineWidth(1.0);
    Glib_Draw(x, y, x+width, y);
    for (i=0; i<=20000; i+=1000)
    {
        Glib_Draw(x + i * width / 20000, y, x + i * width / 20000, y+20);
    }
}

void BEGIN_Diagnostics()
{
}
