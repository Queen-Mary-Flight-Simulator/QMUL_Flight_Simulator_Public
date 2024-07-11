/* +------------------------------+---------------------------------+
   | Module      : panel.c        | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-09      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Panel Management                                 |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <SIM/glib.h>

#include "radio.h"
#include "fcu.h"
#include "gui.h"
#include "panellib.h"
#include "panel.h"
#include "ios.h"
#include "nfd.h"

static unsigned int PanelMode;

/* ---------------------------------------- */
void Panel_CheckPanel()
{
    int  mx;
    int  my;
    bool leftb;
    bool middleb;
    bool rightb;
    float scrollfactor;
    
    Gui_GetMouse(&mx, &my, &leftb, &middleb, &rightb, &scrollfactor);

	if (leftb)
	{
		int  x;
		
		for (x = 17; x <= 377; x += 120)
		{
			if (mx >= (NFD_OFFSET_X + x) && mx <= (NFD_OFFSET_X + x + 102) && my >= (NFD_OFFSET_Y + 8) && my <= (NFD_OFFSET_Y + 35))
			{
				PanelMode = x / 120;
				//if (PanelMode == 2)
				//{
				//    PanelMode = 1; /* force singe radio mode (compatability with LFS) */
				//}
			}
		}
	}
    
    PanelLib_DisplayMode(PanelMode);
    
    /* translate offset will be set to the left hand corner of RMP box or FCU box */

    switch (PanelMode)
    {
        case 0:
            break;
        case 1:
            Radio_UpdateRMP(0, leftb, middleb, rightb, mx, my);
            break;
        case 2:
            Radio_UpdateRMP(1, leftb, middleb, rightb, mx, my);
            break;
        case 3:
            FCU_UpdateFCU(leftb, middleb, rightb, mx, my);
            break;
    }
}

/* ---------------------------------------- */
void BEGIN_Panel()
{
    PanelMode = 0;
}
