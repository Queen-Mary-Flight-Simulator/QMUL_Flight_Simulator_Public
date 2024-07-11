#ifndef FCU_H
#define FCU_H

#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>

#define FCU_DME	0
#define FCU_ILS	1
#define FCU_VOR	2
#define FCU_TAC	3
#define FCU_NDB	4

typedef unsigned char FCU_BeaconMode;
typedef unsigned int  FCU_BeaconType;

extern bool         FCU_FD;              /* FD left or FD right button pressed */
extern bool         FCU_LS;              /* LS left or LS right button pressed */
extern bool         FCU_LOC;             /* LOC button pressed */
extern bool         FCU_AP1;             /* AP1 button pressed */
extern bool         FCU_AP2;             /* AP2 button pressed */
extern bool         FCU_ATHR;            /* ATHR button pressed */
extern bool         FCU_EXPED;           /* EXPED button button pressed */
extern bool         FCU_APPR;            /* APPR button pressed */
extern bool         FCU_SPD_MACH_Button; /* SPD or MACH button */
extern unsigned int FCU_BaroPressure;    /* left or right baro pressure */
extern bool         FCU_BaroHg;          /* left or right baro Hg selection */
extern bool         FCU_BaroSTD;         /* STD or QNH */
extern bool         FCU_HDG_TRK_Button;  /* HDG or TRK button */
extern unsigned int FCU_MACH;
extern unsigned int FCU_HDG;
extern unsigned int FCU_TRK;
extern unsigned int FCU_ALT;
extern unsigned int FCU_SPD;
extern int          FCU_VS;
extern int          FCU_FPA;
extern bool         FCU_Metric_Button;

extern NavDefn_FCUKnob FCU_BaroKnob;     /* baro pressure selected */
extern NavDefn_FCUKnob FCU_HDGKnob;      /* HDG select */
extern NavDefn_FCUKnob FCU_ALTKnob;      /* ALT select */
extern NavDefn_FCUKnob FCU_SPDKnob;      /* SPD select */
extern NavDefn_FCUKnob FCU_VSKnob;       /* VS select */

extern unsigned int FCU_ALT_Range;

extern NavDefn_FCUNav  FCU_NavSwitch1;
extern NavDefn_FCUNav  FCU_NavSwitch2;
extern NavDefn_FCUMode FCU_ModeSelector;
extern NavDefn_FCUData FCU_DataMode;
extern unsigned int    FCU_RangeSelector;

extern bool            FCU_CSTR_Button;
extern bool            FCU_WPT_Button;
extern bool            FCU_VORD_Button;
extern bool            FCU_NDB_Button;
extern bool            FCU_ARPT_Button;

extern void FCU_InitialiseFCU();
extern void FCU_UpdateFCU(int leftb, int middleb, int rightb, int x, int y);
extern void FCU_RestoreFCU(IosDefn_RestoreVectorRecord v);
extern void FCU_StopFCU();
extern void BEGIN_FCU();

#endif
