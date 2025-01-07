/*
    Offline RMP test
    to build: gcc rmptest.c -o rmptest radio.o rs232lib.o -lncurses -lpthread
    DJA 16 January 2024
*/
 

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <ncurses.h>

#include "radio.h"

#define SwAM      0x01
#define SwHF2     0x02
#define SwSEL     0x04
#define SwHF1     0x08
#define SwVHF3    0x10
#define SwVHF2    0x20
#define SwVHF1    0x40

#define SwPWR     0x01
#define SwBFO     0x02
#define SwADF     0x04
#define SwMLS     0x08
#define SwILS     0x10
#define SwVOR     0x20
#define SwNAV     0x40

struct timeval tv;

/* -------------------------------- */
int main(int argc, char *argv[])
{
    unsigned int t1, t2;
    int FrameNumber = 0;
	
    printf("RMP Test Display Program\n");

    BEGIN_Radio();
	
    t1 = 0;

    initscr();
	cbreak();
	noecho();
    
    while (1)
	{
        FrameNumber += 1;
		
        if ((FrameNumber % 5) == 0)  /* only update the RMP at 10 Hz (every 5th frame) */
	    {
            int x = 0; /* dummy args for LFS */
            Radio_UpdateRMP(x, x, x, x, x, x);
        }
		
	    mvprintw(0,  0, "Frame:  %d", FrameNumber);
        mvprintw(1,  0, "Description    Active  Standby");
        mvprintw(2,  0, "VHF1           %3d.%3d %3d.%3d\n", Radio_Radios[0].ComVHF1.Active / 1000, Radio_Radios[0].ComVHF1.Active % 1000, Radio_Radios[0].ComVHF1.Stby / 1000, Radio_Radios[0].ComVHF1.Stby % 1000);
        mvprintw(3,  0, "VHF2           %3d.%3d %3d.%3d\n", Radio_Radios[0].ComVHF2.Active / 1000, Radio_Radios[0].ComVHF2.Active % 1000, Radio_Radios[0].ComVHF2.Stby / 1000, Radio_Radios[0].ComVHF2.Stby % 1000);
        mvprintw(4,  0, "VHF3           %3d.%3d %3d.%3d\n", Radio_Radios[0].ComVHF3.Active / 1000, Radio_Radios[0].ComVHF3.Active % 1000, Radio_Radios[0].ComVHF3.Stby / 1000, Radio_Radios[0].ComVHF3.Stby % 1000);
        mvprintw(5,  0, "HF1             %6d  %6d\n", Radio_Radios[0].ComHF1.Active, Radio_Radios[0].ComHF1.Stby);
        mvprintw(6,  0, "HF2             %6d  %6d\n", Radio_Radios[0].ComHF2.Active, Radio_Radios[0].ComHF2.Stby);
        mvprintw(7,  0, "AM              %6d  %6d\n", Radio_Radios[0].ComAM.Active, Radio_Radios[0].ComAM.Stby);

        mvprintw(9,  0, "VOR             %3d.%2d  %3d.%2d\n", Radio_Radios[0].NavVOR.Active / 100, Radio_Radios[0].NavVOR.Active % 100, Radio_Radios[0].NavVOR.Stby / 100, Radio_Radios[0].NavVOR.Stby % 100);
        mvprintw(10, 0, "ILS             %3d.%2d  %3d.%2d\n", Radio_Radios[0].NavILS.Active / 100, Radio_Radios[0].NavILS.Active % 100, Radio_Radios[0].NavILS.Stby / 100, Radio_Radios[0].NavILS.Stby % 100);
        mvprintw(11, 0, "ADF             %6d  %6d\n", Radio_Radios[0].NavADF.Active, Radio_Radios[0].NavADF.Stby);

		mvprintw(13, 0, "CRS: %3d\n", (unsigned int) Radio_Radios[0].CrsKnob);

		mvprintw(15,  0, "AM:   %s", (Radio_Radios[0].Mode >> 8) & SwAM ? "ON" : "OFF");
		mvprintw(16,  0, "HF2:  %s", (Radio_Radios[0].Mode >> 8) & SwHF2 ? "ON" : "OFF");
		mvprintw(17,  0, "SEL:  %s", (Radio_Radios[0].Mode >> 8) & SwSEL ? "ON" : "OFF");
		mvprintw(18,  0, "HF1:  %s", (Radio_Radios[0].Mode >> 8) & SwHF1 ? "ON" : "OFF");
		mvprintw(19,  0, "VHF3: %s", (Radio_Radios[0].Mode >> 8) & SwVHF3 ? "ON" : "OFF");
		mvprintw(20,  0, "VHF2: %s", (Radio_Radios[0].Mode >> 8) & SwVHF2 ? "ON" : "OFF");
		mvprintw(21,  0, "VHF1: %s", (Radio_Radios[0].Mode >> 8) & SwVHF1 ? "ON" : "OFF");

		mvprintw(22,  0, "PWR:  %s", Radio_Radios[0].Mode & SwPWR ? "ON" : "OFF");
		mvprintw(23,  0, "BFO:  %s", Radio_Radios[0].Mode & SwBFO ? "ON" : "OFF");
		mvprintw(24,  0, "ADF:  %s", Radio_Radios[0].Mode & SwADF ? "ON" : "OFF");
		mvprintw(25,  0, "MLS:  %s", Radio_Radios[0].Mode & SwMLS ? "ON" : "OFF");
		mvprintw(26,  0, "ILS:  %s", Radio_Radios[0].Mode & SwILS ? "ON" : "OFF");
		mvprintw(27,  0, "VOR:  %s", Radio_Radios[0].Mode & SwVOR ? "ON" : "OFF");
		mvprintw(28,  0, "NAV:  %s", Radio_Radios[0].Mode & SwNAV ? "ON" : "OFF");

		mvprintw(30,  0, "Swap: %s", Radio_Radios[0].Mode & 0x80 ? "ON" : "OFF");

        while (1) 
        {
            gettimeofday(&tv, NULL);
            t2 = tv.tv_usec / 20000L;
            if (t1 != t2) 
            {
                t1 = t2;
                break;
            }
			refresh();
        }
    }
}
