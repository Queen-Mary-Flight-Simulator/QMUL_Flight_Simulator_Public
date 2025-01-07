/*
    Offline FCU test
    to build: gcc fcutest.c -o fcutest fcu.o rs232lib.o -lncurses -lpthread
    DJA 16 January 2024
*/
 
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <ncurses.h>

#include <SIM/navdefn.h>

#include "fcu.h"

struct timeval tv;

char* NavSwitchPosition(NavDefn_FCUNav s);
char* KnobPosition(NavDefn_FCUKnob k);
char* ModeSelector(NavDefn_FCUMode m);

/* -------------------------------- */
char* ModeSelector(NavDefn_FCUMode m)
{
    switch (m)
	{
	    case NavDefn_ModeILS:
		    return "ILS";
		case NavDefn_ModeVOR:
            return "VOR";
		case NavDefn_ModeNAV:
		    return "NAV";
		case NavDefn_ModeARC:
		    return "ARC";
		case NavDefn_ModePLAN:
		    return "PLAN";
		default:
		    return "Unknown";
	}
}

/* -------------------------------- */
char* NavSwitchPosition(NavDefn_FCUNav s)
{
    switch (s)
	{
	    case NavDefn_NavADF:
		    return "ADF";
		case NavDefn_NavOFF:
            return "OFF";
		case NavDefn_NavVOR:
		    return "VOR";
		default:
		    return "Unknown";
	}
}

/* -------------------------------- */
char* KnobPosition(NavDefn_FCUKnob k)
{
    switch (k)
	{
	    case NavDefn_Pushed:
		    return "Pushed";
		case NavDefn_Middle:
            return "Middle";
		case NavDefn_Pulled:
		    return "Pulled";
		default:
		    return "Unknown";
	}
}

/* -------------------------------- */
int main(int argc, char *argv[])
{
    unsigned int t1, t2;
    int FrameNumber = 0;
	
    printf("FCU Test Display Program\n");

    BEGIN_FCU();

    t1 = 0;

    initscr();
	cbreak();
	noecho();
	
    while (1)
	{
        FrameNumber += 1;
    
        if ((FrameNumber % 50) == 0)
        {
		    mvprintw(0,  0, "Frame:  %d", FrameNumber);
			mvprintw(1,  0, "FD:     %s", FCU_FD ? "ON" : "OFF");
			mvprintw(2,  0, "LS:     %s", FCU_LS ? "ON" : "OFF");
			mvprintw(3,  0, "LOC:    %s", FCU_LOC ? "ON" : "OFF");
			mvprintw(4,  0, "AP1:    %s", FCU_AP1 ? "ON" : "OFF");
			mvprintw(5,  0, "AP2:    %s", FCU_AP2 ? "ON" : "OFF");
			mvprintw(6,  0, "ATHR:   %s", FCU_ATHR ? "ON" : "OFF");
			mvprintw(7,  0, "EXPED:  %s", FCU_EXPED ? "ON" : "OFF");
			mvprintw(8,  0, "APPR:   %s", FCU_APPR ? "ON" : "OFF");
			mvprintw(9,  0, "MACH:   %s", FCU_SPD_MACH_Button ? "MACH" : "KTS");
			mvprintw(10, 0, "HDG:    %s", FCU_HDG_TRK_Button ? "HDG/VS" : "TRK/FPA");
			mvprintw(11, 0, "Metric: %s", FCU_Metric_Button ? "HDG/VS" : "TRK/FPA");
			mvprintw(12, 0, "Baro:   %s", FCU_BaroHg ? "InHg" : "hPa");
			mvprintw(13, 0, "STD:    %s", FCU_BaroSTD ? "STD" : "QNH");

			mvprintw(15, 0, "Baro:      %d", FCU_BaroPressure);
			mvprintw(16, 0, "HDG:       %d", FCU_HDG);
			mvprintw(17, 0, "TRK:       %d", FCU_TRK);
			mvprintw(18, 0, "ALT:       %d", FCU_ALT);
			mvprintw(19, 0, "SPD:       %d", FCU_SPD);
			mvprintw(20, 0, "VS:        %d", FCU_VS);
			mvprintw(21, 0, "FPA:       %d", FCU_FPA);
			mvprintw(22, 0, "ALT range: %d", FCU_FPA);

			mvprintw(24, 0, "HDG knob: %s", KnobPosition(FCU_HDGKnob));
			mvprintw(25, 0, "ALT knob: %s", KnobPosition(FCU_ALTKnob));
			mvprintw(26, 0, "SPD knob: %s", KnobPosition(FCU_SPDKnob));
			mvprintw(27, 0, "VS knob:  %s", KnobPosition(FCU_VSKnob));

			mvprintw(29, 0, "NAV1: %s", NavSwitchPosition(FCU_NavSwitch1));
			mvprintw(30, 0, "NAV2: %s", NavSwitchPosition(FCU_NavSwitch2));

			mvprintw(32, 0, "Mode: %s", ModeSelector(FCU_ModeSelector));
			mvprintw(33, 0, "Range: %d", FCU_RangeSelector);

			mvprintw(35,  0, "CSTR:  %s", FCU_CSTR_Button ? "ON" : "OFF");
			mvprintw(36,  0, "WPT:   %s", FCU_WPT_Button ? "ON" : "OFF");
			mvprintw(37,  0, "VOR D: %s", FCU_VORD_Button ? "ON" : "OFF");
			mvprintw(38,  0, "NDB:   %s", FCU_NDB_Button ? "ON" : "OFF");
			mvprintw(39,  0, "ARPT:  %s", FCU_ARPT_Button ? "ON" : "OFF");
			
            while (1) 
            {
                gettimeofday(&tv, NULL);
                t2 = tv.tv_usec / 20000L;
                if (t1 != t2) 
                {
                    t1 = t2;
                    break;
                }
            }
			refresh();
        }
	}
}
