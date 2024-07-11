#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include <SIM/glib.h>

#include "pfd.h"
#include "aerolink.h"
#include "aero.h"
#include "fcs.h"
#include "model.h"

#define FMA_LeftX   0
#define FMA_LeftY   0
#define FMA_CentreX 0
#define FMA_CentreY 0
#define FMA_RightX  0
#define FMA_RightY  0

int AutothrottleModeTimeout;
int RollModeTimeout;
int PitchModeTimeout;
int StatusTimeout;

int AutothrottleMode;
int oldAutothrottleMode;
int RollMode;
int RollArmMode;
int oldRollMode;
int PitchMode;
int PitchArmMode;
int oldPitchMode;
int Status;
int oldStatus;

char *ATWords[] =
    { "",               "THR",          "THR REF",    	"HOLD",   			"IDLE",          /* 0-4 */
      "SPD" };                                                                               /* 5-9 */

char *RollWords[] =
    { "",				"HDG HOLD",	  	"HDG SEL",	    "LNAV",             "LOC",           /* 0-4 */
      "ROLLOUT",        "TO/GA",        "ATT" };                                             /* 5-9 */

char *PitchWords[] =
    { "",				"TO/GA",        "ALT",   	    "V/S",		    	"VNAV PTH",	   	 /* 0-4 */
      "VNAV SPD",		"VNA ALT",    	"G/S",          "FLARE",	    	"FLCH SPD" };	 /* 5-9 */

char *StatusWords[] =
    { "",				"FD",		    "CMD",           "LAND 3",           "LAND 2",	     /* 0-4 */
      "NO AUTOLAND" };
    
char *RollArmWords[] =
    { "",				"LNAV",	        "LOC",		    "ROLLOUT" };					     /* 0-4 */

char *PitchArmWords[] =
    { "",				"G/S",		    "FLARE",		"VNAV" };		                    /* 0-4 */

void UpdateStatus();
void UpdateAutothrottleMode();
void UpdateRollMode();
void UpdatePitchMode();
void rect(int x, int y, int xs, int ys);

/* ------------------------------------------------- */
void FMA_FMA(int x, int y)
{
    Glib_LoadIdentity();
    Glib_Translate((float) x, (float) y);

    Glib_Colour(Glib_GREY);
    Glib_Rectangle(0, 0, 321, 53);
    Glib_Colour(Glib_WHITE);
    Glib_Rectangle(106, 0, 3, 53);
    Glib_Rectangle(213, 0, 3, 53);
    
    UpdateStatus();
    UpdateAutothrottleMode();
    UpdateRollMode();
    UpdatePitchMode();

    Glib_Colour(Glib_GREEN);
    Glib_SetFont(Glib_EFONT16, 0);
   	Glib_LineWidth(2.0);
    
    if (PitchMode > 0)
    {
        int p = Glib_StringSize(PitchWords[PitchMode]);
        Glib_Chars(PitchWords[PitchMode], 267 - p / 2, 28);
    
        if (PitchModeTimeout > 0)
        {
            PitchModeTimeout -= 1;
            rect(267 - p / 2 - 2, 24, p + 4, 24);
        }
    }
    
    if (RollMode > 0)
    {
        int p = Glib_StringSize(RollWords[RollMode]);
        
        Glib_Chars(RollWords[RollMode], 160 - p / 2, 28);
    
        if (RollModeTimeout > 0)
        {
            RollModeTimeout -= 1;
            rect(160 - p / 2 - 2, 24, p + 4, 24);
        }
    }
    
    if (AutothrottleMode > 0)
    {
        int p = Glib_StringSize(ATWords[AutothrottleMode]);
        
        Glib_Chars(ATWords[AutothrottleMode], 53 - p / 2, 28);
    
        if (AutothrottleModeTimeout > 0)
        {
            AutothrottleModeTimeout -= 1;
            rect(53 - p / 2 - 2, 24, p + 4, 24);
        }
    }
    
    if (Status > 0)
    {
        int p = Glib_StringSize(StatusWords[Status]);
        
        Glib_Chars(StatusWords[Status], 160 - p / 2, -60);
    
        if (StatusTimeout > 0)
        {
            StatusTimeout -= 1;
            rect(160 - p / 2 - 2, -64, p + 4, 24);
        }
    }

    Glib_Colour(Glib_WHITE);
    Glib_SetFont(Glib_EFONT12, 0);

    if (PitchArmMode > 0)
    {
        int p = Glib_StringSize(PitchArmWords[PitchArmMode]);
        Glib_Chars(PitchArmWords[PitchArmMode], 267 - p / 2, 10);
    }
}

/* ------------------------------------------------- */
void UpdatePitchMode()
{
    float h = AeroLink_NavPkt.GroundLevel + Aero_CGHeight - Model_Pz;

    PitchArmMode = 0;
    if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && AeroLink_NavPkt.FCU_APPR && !Model_OnTheGround && h < 15.24)  /* below 50 ft RA */
    {
        PitchMode = 8; /* FLARE */
    }
    else if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && AeroLink_NavPkt.FCU_APPR && !Model_OnTheGround)
    {
        PitchMode = 7;  /* G/S */
        if (h < 457.0 && h > 15.24)
        {
            PitchArmMode = 2;  /* FLARE */
        }
    }
    else if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && AeroLink_NavPkt.FCU_ALT)
    {
        PitchMode = 2;  /* ALT */
    }
    else if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && AeroLink_NavPkt.FCU_VS)
    {
        PitchMode = 3;  /* V/S */
    }
    else if (FCS_TOGAMode && Model_OnTheGround)
    {
        PitchMode = 1;      /* TO/GA */
		PitchArmMode = 3;   /* VNAV */
    }
    else
    {
        PitchMode = 0;
    }
    
    if (PitchMode != oldPitchMode)
    {
        PitchModeTimeout = 500;
        oldPitchMode = PitchMode;
    }
}

/* ------------------------------------------------- */
void UpdateRollMode()
{
    if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && (AeroLink_NavPkt.FCU_HDGKnob == NavDefn_Pulled))
    {
        RollMode = 2;  /* HDG SEL */
    }
    else if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && (AeroLink_NavPkt.WayPoint.BeaconStatus))
    {
        RollMode = 3; /* LNAV */
    }
    else if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && (AeroLink_NavPkt.FCU_LOC || AeroLink_NavPkt.FCU_APPR))
    {
        if (Model_OnTheGround)
		{
		    RollMode = 3;     /* LNAV */
			RollArmMode = 0;
		}
		RollMode = 4;     /* LOC */
		RollArmMode = 3;  /* ROLLOUT */
    }
    else if ((AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2) && Model_OnTheGround)
    {
        RollMode = 5;  /* ROLLOUT */
    }
    else if (FCS_TOGAMode && Model_OnTheGround)
    {
        RollMode = 6;     /* TO/GA */
		RollArmMode = 1;  /* LNAV */
    }
    else
    {
        RollMode = 0;
    }

    if (RollMode != oldRollMode)
    {
        RollModeTimeout = 500;
        oldRollMode = RollMode;
    }
}

/* ------------------------------------------------- */
void UpdateAutothrottleMode()
{
    float h = AeroLink_NavPkt.GroundLevel + Aero_CGHeight - Model_Pz;

    if (AeroLink_NavPkt.FCU_ATHR && (AeroLink_NavPkt.FCU_SPDKnob == NavDefn_Pulled))
    {
        if (h > 15.24)  /* above 50 ft */
        {
            AutothrottleMode = 5;  /* SPD */
        }
        else
        {
            AutothrottleMode = 0;
        }
    }
    else if (AeroLink_NavPkt.FCU_ATHR && FCS_ThrottlePosition < 0.21)
    {
        AutothrottleMode = 4;  /* IDLE */
    }
    else if (AeroLink_NavPkt.FCU_ATHR && FCS_TOGAMode)
    {
        if (Model_OnTheGround && Model_U < 33.44)  /* < 65 Kt */
        {
            AutothrottleMode = 2;  /* THR REF */
        }
        else if (h < 122.0) /* 400 ft */
        {
            AutothrottleMode = 3;  /* HOLD */
        }
        else
        {
            AutothrottleMode = 2;  /* THR REF */
        }
    }
    else if (AeroLink_NavPkt.FCU_ATHR && (AeroLink_NavPkt.FCU_SPDKnob == NavDefn_Pulled))
    {
        AutothrottleMode = 3;  /* HOLD */
    }
    else
    {
        AutothrottleMode = 0;
    }

    if (AutothrottleMode != oldAutothrottleMode)
    {
        AutothrottleModeTimeout = 500;
        oldAutothrottleMode = AutothrottleMode;
    }
}

/* ------------------------------------------------- */
void UpdateStatus()
{
    if (AeroLink_NavPkt.FCU_APPR)
    {
        if (AeroLink_NavPkt.FCU_AP1 && AeroLink_NavPkt.FCU_AP2)
        {
            Status = 3;  /* LAND 3 */
        }
        else if (AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2)
        {
            Status = 4;  /* LAND 2 */
        }
        else
        {
            Status = 5;  /* NO AUTOLAND */
        }
    }
    else if (AeroLink_NavPkt.FCU_FD && !AeroLink_NavPkt.FCU_AP1 && !AeroLink_NavPkt.FCU_AP2)
    {
        Status = 1;  /* FD */
    }
    else if (AeroLink_NavPkt.FCU_AP1 || AeroLink_NavPkt.FCU_AP2)
    {
        Status = 2;  /* CMD */
    }
    else
    {
        Status = 0;
    }
    
    if (Status != oldStatus)
    {
        StatusTimeout = 500;
        oldStatus = Status;
    }
}

/* ------------------------------------------------- */
void rect(int x, int y, int xs, int ys)
{
    Glib_Draw(x, y, x, y + ys);
    Glib_Draw(x, y + ys, x + xs, y + ys);
    Glib_Draw(x + xs, y + ys, x + xs, y);
    Glib_Draw(x + xs, y, x, y);
}

/* ------------------------------------------------- */
void BEGIN_FMA()
{
    AutothrottleModeTimeout = 0;
    RollModeTimeout = 0;
    PitchModeTimeout = 0;
    StatusTimeout = 0;
    
    AutothrottleMode = 0;
    oldAutothrottleMode = 0;
    RollMode = 0;
    RollArmMode = 0;
    oldRollMode = 0;
    PitchMode = 0;
    PitchArmMode = 0;
    oldPitchMode = 0;
    Status = 0;
    oldStatus = 0;
}
