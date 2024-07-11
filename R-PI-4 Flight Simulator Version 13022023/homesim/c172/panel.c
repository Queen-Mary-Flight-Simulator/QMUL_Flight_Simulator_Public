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
#include <GL/gl.h>
#include <SIM/glib.h>

#include "radio.h"
#include "radio.h"
#include "knobs.h"
#include "ios.h"
#include "gui.h"

void Panel_CheckPanel()
{
    int  mx;
    int  my;
    bool leftb;
    bool middleb;
    bool rightb;
    float scrollfactor;
    
    Gui_GetMouse(&mx, &my, &leftb, &middleb, &rightb, &scrollfactor);
 
    Knobs_Check(mx, my, leftb, middleb, rightb);
    Radio_CheckRadio(mx, my, leftb, middleb, rightb);
}

void BEGIN_Panel()
{
}
