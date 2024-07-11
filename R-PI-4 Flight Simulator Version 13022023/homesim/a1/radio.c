#include <GL/gl.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <SIM/aerodefn.h>
#include <SIM/iosdefn.h>
#include <SIM/navdefn.h>
#include <SIM/glib.h>
#include <SIM/navlib.h>
#include <SIM/clocks.h>
#include <SIM/maths.h>

#include "navlink.h"
#include "nfd.h"
#include "nav.h"
#include "radio.h"

NavDefn_RadioPanel Radio_Radios[2];

typedef struct
{
    int xc;
    int yc;
} Vertex;

#define VhfBeacon NavLib_DME | NavLib_ILS | NavLib_VOR
#define AdfBeacon NavLib_NDB

#define SwNone    0
#define SwNAV     1
#define SwVOR     2
#define SwILS     3
#define SwMLS     4
#define SwADF     5
#define SwBFO     6
#define SwHF1     7
#define SwHF2     8
#define SwAM      9
#define SwVHF1    10
#define SwVHF2    11
#define SwVHF3    12
#define SwSEL     13

#define COM_OffKnobX     (NFD_RadioX + 40)
#define COM_OffKnobY     (NFD_RadioY + 277)

#define COM_KnobX        (NFD_RadioX + 130)
#define COM_KnobY        (NFD_RadioY + 277)

#define COM_LeftPanelX   (NFD_RadioX - 20)
#define COM_LeftPanelY   (NFD_RadioY + 317)
#define COM_RightPanelX  (COM_LeftPanelX + 80)
#define COM_RightPanelY  (COM_LeftPanelY)
#define COM_ChangeoverX  (NFD_RadioX + 69)
#define COM_ChangeoverY  (NFD_RadioY + 294)

#define NAV_OffKnobX     (NFD_RadioX + 210)
#define NAV_OffKnobY     (NFD_RadioY + 277)

#define NAV_KnobX        (NFD_RadioX + 310)
#define NAV_KnobY        (NFD_RadioY + 277)

#define NAV_LeftPanelX   (NFD_RadioX + 165)
#define NAV_LeftPanelY   (NFD_RadioY + 317)
#define NAV_RightPanelX  (NAV_LeftPanelX + 80)
#define NAV_RightPanelY  (NAV_LeftPanelY)
#define NAV_ChangeoverX  (NFD_RadioX + 244)
#define NAV_ChangeoverY  (NFD_RadioY + 294)

#define ADF_OffKnobX     (NFD_RadioX + 265)
#define ADF_OffKnobY     (NFD_RadioY + 187)

#define ADF_KnobX        (NFD_RadioX + 320)
#define ADF_KnobY        (NFD_RadioY + 192)

#define ADF_LeftPanelX   (NFD_RadioX + 25)
#define ADF_LeftPanelY   (NFD_RadioY + 212)
#define ADF_RightPanelX  (ADF_LeftPanelX + 110)
#define ADF_RightPanelY  (ADF_LeftPanelY)
#define ADF_ChangeoverX  (NFD_RadioX + 125)
#define ADF_ChangeoverY  (NFD_RadioY + 190)

#define XPDR_SelectorX   (NFD_RadioX + 55)
#define XPDR_SelectorY   (NFD_RadioY + 117)
#define XPDR_KnobX       (NFD_RadioX + 310)
#define XPDR_KnobY       (NFD_RadioY + 127)

#define XPDR_LeftPanelX   (NFD_RadioX + 145)
#define XPDR_LeftPanelY   (NFD_RadioY + 124)
#define XPDR_RightPanelX  (XPDR_LeftPanelX + 60)
#define XPDR_RightPanelY  (XPDR_LeftPanelY)

#define DME_OffSwitchX    (NFD_RadioX + 250)
#define DME_OffSwitchY    (NFD_RadioY + 10)
#define DME_PanelX        (NFD_RadioX + 37)
#define DME_PanelY        (NFD_RadioY + 42)

#define NumberOfPowerSwitches 6

static bool         PowerSwitch[NumberOfPowerSwitches];

static unsigned int DigitsMin;
static unsigned int DigitsMax;
static unsigned int DigitsInc;
static unsigned int PanelDigits;

static bool         oldleftb;
static unsigned int NavMode;
static int          COM_OffKnobAngle;
static int          NAV_OffKnobAngle;
static int          COM_InnerKnobAngle;
static int          COM_OuterKnobAngle;
static int          NAV_InnerKnobAngle;
static int          NAV_OuterKnobAngle;
static int          ADF_OffKnobAngle;
static int          ADF_InnerKnobAngle;
static int          ADF_OuterKnobAngle;
static int          XPDR_SelectorAngle;
static int          XPDR_InnerKnobAngle;
static int          XPDR_OuterKnobAngle;
static char         COM_LeftPanelStr[20];
static char         COM_RightPanelStr[20];
static char         NAV_LeftPanelStr[20];
static char         NAV_RightPanelStr[20];
static char         ADF_LeftPanelStr[20];
static char         ADF_RightPanelStr[20];
static char         XPDR_LeftPanelStr[20];
static char         XPDR_RightPanelStr[20];
static unsigned int ticks;
static unsigned int ticklimit;
static int          XPDR_Code;

static unsigned int OldVor1;
static unsigned int OldAdf1;

static bool InsideCircle(int x, int y, int x0, int y0, int r);
static bool InsideBox(int x, int y, int x0, int y0, int xs, int ys);
static void FormFreqString(unsigned int f, unsigned int mode, char v[]);
static void SetParameters();
static void SetFreq(unsigned int f, unsigned int mode);
static unsigned int GetStbyFreq(unsigned int mode);
static unsigned int GetActiveFreq(unsigned int mode);
static void ChangeoverSwitch();
static void DisplayPanel(char str[], unsigned int x, unsigned int y);
static void DME_OffButton(bool onstate);
static void Display_DME();
static void Display_XPDR(int fl);

/* ---------------------------------------------- */
void Radio_UpdateRadio()
{
    unsigned int v1;
    unsigned int a1;

    v1 = Radio_Radios[0].NavVOR.Active;
    if (v1 != OldVor1)
    {
        Nav_VOR1.SelectedBeacon = NavLib_LookupChannel(NavLink_AeroPkt.Latitude, NavLink_AeroPkt.Longitude, v1, VhfBeacon);
        OldVor1                 = v1;
    }
    a1 = Radio_Radios[0].NavADF.Active;
    if (a1 != OldAdf1)
    {
        Nav_ADF1.SelectedBeacon = NavLib_LookupChannel(NavLink_AeroPkt.Latitude, NavLink_AeroPkt.Longitude, a1, AdfBeacon);
        OldAdf1                 = a1;
    }
}

/* ---------------------------------------------- */
static bool InsideCircle(int x, int y, int x0, int y0, int r)
{
    return (x - x0) * (x - x0) + (y - y0) * (y - y0) < r * r;
}

/* ---------------------------------------------- */
static bool InsideBox(int x, int y, int x0, int y0, int xs, int ys)
{
    return x >= x0 && x <= x0 + xs && y >= y0 && y <= y0 + ys;
}

/* ---------------------------------------------- */
void Radio_CheckRadio(int x, int y, int leftb, int middleb, int rightb)
{
	if (leftb & oldleftb)  // check if left button held down
	{
        ticks += 1;
		if (ticks >= 150)  // speed up after 3s
		{
		    ticklimit = 2;  // set to fast mode - update every 2nd frame
		}
	}
	else
	{
	    ticklimit = 20;  // set to slow mode - update every 10th frame
		ticks = 0;
	}
	
    NavMode = SwVHF1;
    SetParameters();
    PanelDigits = GetActiveFreq(NavMode);
    FormFreqString(PanelDigits, NavMode, COM_LeftPanelStr);
    PanelDigits = GetStbyFreq(NavMode);
    FormFreqString(PanelDigits, NavMode, COM_RightPanelStr);

    if (leftb)
    {
		NavMode = SwVHF1;
		SetParameters();
		PanelDigits = GetActiveFreq(NavMode);
		FormFreqString(PanelDigits, NavMode, COM_LeftPanelStr);
		PanelDigits = GetStbyFreq(NavMode);
		FormFreqString(PanelDigits, NavMode, COM_RightPanelStr);

        if (InsideCircle(x, y, COM_OffKnobX, COM_OffKnobY, 20))
        {
            if (x < COM_OffKnobX)
            {
			    if (COM_OffKnobAngle > 1)
				{
				    COM_OffKnobAngle -= 1;
				}
			}
			else if (COM_OffKnobAngle < 360)
			{
			    COM_OffKnobAngle += 1;
			}
			PowerSwitch[0] = (COM_OffKnobAngle > 30);
        }
         
        if (InsideCircle(x, y, COM_KnobX, COM_KnobY, 30))
        {
            PanelDigits = GetStbyFreq(SwVHF1);

            if (InsideCircle(x, y, COM_KnobX, COM_KnobY, 15))
            {
                if (x < COM_KnobX)
                {
				    COM_InnerKnobAngle -= 1;
                    if (ticks % ticklimit == 0)
					{
                        PanelDigits -= 5;
                    }
    			}
                else
                {
                    COM_InnerKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits += 5;
                    }
                }
			}
			else
            {
                if (x < COM_KnobX)
                {
				    COM_OuterKnobAngle -= 1;
					if (ticks % ticklimit == 0)
                    { 
                        PanelDigits -= 100;
                    }
    			}
                else
				{
                    COM_OuterKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits += 100;
                    }
                }
			}
			
			if (PanelDigits > DigitsMax)
			{
			    PanelDigits = DigitsMax;
			}
			else if (PanelDigits < DigitsMin)
			{
			    PanelDigits = DigitsMin;
			}

			FormFreqString(PanelDigits, SwVHF1, COM_RightPanelStr);
			SetFreq(PanelDigits, SwVHF1);
        }
    }

	if (leftb && !oldleftb)
	{
	    if (InsideBox(x, y, COM_ChangeoverX, COM_ChangeoverY, 24, 12))
		{
		    ChangeoverSwitch();
		}
	}
	
    if (leftb)
    {
		NavMode = SwVOR;
		SetParameters();
		PanelDigits = GetActiveFreq(NavMode);
		FormFreqString(PanelDigits, NavMode, NAV_LeftPanelStr);
		PanelDigits = GetStbyFreq(NavMode);
		FormFreqString(PanelDigits, NavMode, NAV_RightPanelStr);
		
        if (InsideCircle(x, y, NAV_OffKnobX, NAV_OffKnobY, 20))
        {
            if (x < NAV_OffKnobX)
            {
			    if (NAV_OffKnobAngle > 1)
				{
				    NAV_OffKnobAngle -= 1;
				}
			}
			else if (NAV_OffKnobAngle < 360)
			{
			    NAV_OffKnobAngle += 1;
			}
			PowerSwitch[1] = (NAV_OffKnobAngle > 30);
        }
         
        if (InsideCircle(x, y, NAV_KnobX, NAV_KnobY, 30))
        {
            PanelDigits = GetStbyFreq(SwVOR);

            if (InsideCircle(x, y, NAV_KnobX, NAV_KnobY, 15))
            {
                if (x < NAV_KnobX)
                {
				    NAV_InnerKnobAngle -= 1;
                    if (ticks % ticklimit == 0)
					{
                        PanelDigits -= 5;
                    }
    			}
                else
                {
                    NAV_InnerKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits += 5;
                    }
                }
			}
			else
            {
                if (x < NAV_KnobX)
                {
				    NAV_OuterKnobAngle -= 1;
					if (ticks % ticklimit == 0)
                    { 
                        PanelDigits -= 100;
                    }
    			}
                else
				{
                    NAV_OuterKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits += 100;
                    }
                }
			}
			
			if (PanelDigits > DigitsMax)
			{
			    PanelDigits = DigitsMax;
			}
			else if (PanelDigits < DigitsMin)
			{
			    PanelDigits = DigitsMin;
			}

			FormFreqString(PanelDigits, SwVOR, NAV_RightPanelStr);
			SetFreq(PanelDigits, SwVOR);
        }
	}
	
	if (leftb && !oldleftb)
	{
	    if (InsideBox(x, y, NAV_ChangeoverX, NAV_ChangeoverY, 24, 12))
		{
		    ChangeoverSwitch();
		}
	}
	
    if (leftb)
    {
		NavMode = SwADF;
		SetParameters();
		PanelDigits = GetActiveFreq(NavMode);
		FormFreqString(PanelDigits, NavMode, ADF_LeftPanelStr);
		PanelDigits = GetStbyFreq(NavMode);
		FormFreqString(PanelDigits, NavMode, ADF_RightPanelStr);
		
        if (InsideCircle(x, y, ADF_OffKnobX, ADF_OffKnobY, 20))
        {
            if (x < ADF_OffKnobX)
            {
			    if (ADF_OffKnobAngle > 1)
				{
				    ADF_OffKnobAngle -= 1;
				}
			}
			else if (ADF_OffKnobAngle < 360)
			{
			    ADF_OffKnobAngle += 1;
			}
			PowerSwitch[2] = (ADF_OffKnobAngle > 30);
        }
         
        if (InsideCircle(x, y, ADF_KnobX, ADF_KnobY, 30))
        {
            PanelDigits = GetStbyFreq(SwADF);

            if (InsideCircle(x, y, ADF_KnobX, ADF_KnobY, 15))
            {
                if (x < ADF_KnobX)
                {
				    ADF_InnerKnobAngle -= 1;
                    if (ticks % ticklimit == 0)
					{
                        PanelDigits -= 5;
                    }
    			}
                else
                {
                    ADF_InnerKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits += 5;
                    }
                }
			}
			else
            {
                if (x < ADF_KnobX)
                {
				    ADF_OuterKnobAngle -= 1;
					if (ticks % ticklimit == 0)
                    { 
                        PanelDigits -= 100;
                    }
    			}
                else
				{
                    ADF_OuterKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        PanelDigits += 100;
                    }
                }
			}
			
			if (PanelDigits > DigitsMax)
			{
			    PanelDigits = DigitsMax;
			}
			else if (PanelDigits < DigitsMin)
			{
			    PanelDigits = DigitsMin;
			}

			FormFreqString(PanelDigits, SwADF, ADF_RightPanelStr);
			SetFreq(PanelDigits, SwADF);
        }
	}
	
	if (leftb && !oldleftb)
	{
	    if (InsideBox(x, y, ADF_ChangeoverX, ADF_ChangeoverY, 24, 12))
		{
		    ChangeoverSwitch();
		}
	}
	
    if (leftb)
    {
        if (InsideCircle(x, y, XPDR_KnobX, XPDR_KnobY, 30))
        {
            if (InsideCircle(x, y, XPDR_KnobX, XPDR_KnobY, 15))
            {
                if (x < XPDR_KnobX)
                {
				    XPDR_InnerKnobAngle -= 1;
                    if (ticks % ticklimit == 0)
					{
					    if ((XPDR_Code & 077) > 0)
						{
                            XPDR_Code -= 1;
						}
                    }
    			}
                else
                {
                    XPDR_InnerKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        if ((XPDR_Code & 077) < 077)
						{
						    XPDR_Code += 1;
						}
                    }
                }
			}
			else
            {
                if (x < XPDR_KnobX)
                {
				    XPDR_OuterKnobAngle -= 1;
					if (ticks % ticklimit == 0)
                    {
                        if (XPDR_Code > 0100)
                        { 
                            XPDR_Code -= 0100;
						}
                    }
    			}
                else
				{
                    XPDR_OuterKnobAngle += 1;
                    if (ticks % ticklimit == 0)
                    {
                        if (XPDR_Code < 07700)
						{
						    XPDR_Code += 0100;
						}
                    }
                }
			}
			
            XPDR_RightPanelStr[0] = (char) ((XPDR_Code >> 9) & 07) + '0';
            XPDR_RightPanelStr[1] = (char) ((XPDR_Code >> 6) & 07) + '0';
            XPDR_RightPanelStr[2] = (char) ((XPDR_Code >> 3) & 07) + '0';
            XPDR_RightPanelStr[3] = (char) (XPDR_Code & 07) + '0';
            XPDR_RightPanelStr[4] = '\0';
        }
	}
	
	if (leftb && !oldleftb)
	{
		if (InsideCircle(x, y, XPDR_SelectorX, XPDR_SelectorY, 30))
		{
            if (x < XPDR_SelectorX)
            {
			    XPDR_SelectorAngle -= 45;
			}
			else
			{
			    XPDR_SelectorAngle += 45;
			}
			if (XPDR_SelectorAngle < 90)
			{
			    XPDR_SelectorAngle = 90;
			}
			else if (XPDR_SelectorAngle > 270)
			{
			    XPDR_SelectorAngle = 270;
			}
		}
		
		if (InsideBox(x, y, DME_OffSwitchX, DME_OffSwitchY, 30, 12))
		{
		    PowerSwitch[3] = x > (DME_OffSwitchX + 15);
		}
	}
	
	oldleftb = leftb;
	
    glDisable(GL_TEXTURE_2D);
	
    if (PowerSwitch[0])
    {
        DisplayPanel(COM_LeftPanelStr, COM_LeftPanelX + 30, COM_LeftPanelY + 9);
        DisplayPanel(COM_RightPanelStr, COM_RightPanelX + 30, COM_RightPanelY + 9);
	}
	
    if (PowerSwitch[1])
    {
        DisplayPanel(NAV_LeftPanelStr, NAV_LeftPanelX + 20, NAV_LeftPanelY + 9);
        DisplayPanel(NAV_RightPanelStr, NAV_RightPanelX + 20, NAV_RightPanelY + 9);
    }
	
    if (PowerSwitch[2])
    {
        DisplayPanel(ADF_LeftPanelStr, ADF_LeftPanelX + 10, ADF_LeftPanelY + 9);
        DisplayPanel(ADF_RightPanelStr, ADF_RightPanelX + 10, ADF_RightPanelY + 9);
    }
	
	if (XPDR_SelectorAngle > 90)
	{
	    int fl = (int) (-NavLink_AeroPkt.Pz * 3.280840) / 100;
		Display_XPDR(fl);
	}
	
	if (PowerSwitch[3])
	{
	    Display_DME();
	}
    DME_OffButton(PowerSwitch[3]);
}

/* ---------------------------------------------- */
static void DME_OffButton(bool onstate)
{
    int i;
	int b = (onstate) ? 15 : 0;
	
    Glib_Colour(Glib_GREY);
    for (i=0; i<=12; i+=3)
	{
	    Glib_Draw(DME_OffSwitchX + i + b, DME_OffSwitchY, DME_OffSwitchX + i + b, DME_OffSwitchY + 9);
	}
}

/* ---------------------------------------------- */
static void Display_DME()
{
    int d;
	int s;
	int t;
	char str[10];
    float gspeed = sqrt(NavLink_AeroPkt.Vn * NavLink_AeroPkt.Vn + NavLink_AeroPkt.Ve * NavLink_AeroPkt.Ve);
	
    Glib_SetFont(Glib_LFONT30, 0);
	
	if (Nav_VOR1.BeaconStatus)
	{
	    d = (int) (Nav_VOR1.SlantDistance * 0.00054 + 0.5);
	    s = (int) (gspeed * 1.943844 + 0.5);
		if (s > 0)
		{
    		t = (int) ((Nav_VOR1.SlantDistance / gspeed) / 60.0 + 0.5);
		}
		else
		{
		    t = 0;
		}
	}
	else
    {
        d = -1;
		s = -1;
		t = -1;
	}
	str[0] = (d % 1000) / 100 + '0';
	str[1] = (d % 100) / 10 + '0';
	str[2] = (d % 10) + '0';
	str[3] = '\0';
    if (d >= 0)
	{
        DisplayPanel(str, DME_PanelX + 10, DME_PanelY);
	}
	else
	{
        DisplayPanel("---", DME_PanelX + 10, DME_PanelY);
	}

	str[0] = (s % 1000) / 100 + '0';
	str[1] = (s % 100) / 10 + '0';
	str[2] = (s % 10) + '0';
	str[3] = '\0';
    if (s >= 0)
	{
        DisplayPanel(str, DME_PanelX + 70, DME_PanelY);
	}
	else
	{
        DisplayPanel("---", DME_PanelX + 70, DME_PanelY);
	}

    if (t > 99)
	{
	    t = 99;
	}
	str[0] = (t % 100) / 10 + '0';
	str[1] = (t % 10) + '0';
	str[2] = '\0';
    if (t >= 0)
	{
        DisplayPanel(str, DME_PanelX + 130, DME_PanelY);
	}
	else
	{
        DisplayPanel("--", DME_PanelX + 130, DME_PanelY);
	}

	Glib_Colour(Glib_RED);
	Glib_SetFont(Glib_GFONT10, 7);
	Glib_Chars("NM", DME_PanelX, DME_PanelY - 12);
	Glib_Chars("KT", DME_PanelX + 70, DME_PanelY - 12);
	Glib_Chars("MIN", DME_PanelX + 130, DME_PanelY - 12);
}

/* ---------------------------------------------- */
static void Display_XPDR(int fl)
{
	XPDR_LeftPanelStr[0] = (char) (fl / 100) + '0';
	XPDR_LeftPanelStr[1] = (char) ((fl % 100) / 10) + '0';
	XPDR_LeftPanelStr[2] = (char) (fl % 10) + '0';
	XPDR_LeftPanelStr[3] = '\0';
	
	Glib_Colour(Glib_RED);
	Glib_SetFont(Glib_LFONT20, 7);
	Glib_Chars("FL", XPDR_LeftPanelX - 5, XPDR_RightPanelY + 5);
	if ((XPDR_SelectorAngle == 270) || ((XPDR_SelectorAngle == 180) && (Clocks_ClockSecs % 6 == 0)))
	{
		int i;
		
		Glib_Colour(Glib_GREEN);
		for (i=0; i<=5; i+=1)
		{
			Glib_Draw(XPDR_RightPanelX + 70, XPDR_RightPanelY + 15 + i, XPDR_RightPanelX + 80, XPDR_RightPanelY + 15 + i);
		}
	}

    DisplayPanel(XPDR_LeftPanelStr, XPDR_LeftPanelX + 10, XPDR_LeftPanelY + 9);
    DisplayPanel(XPDR_RightPanelStr, XPDR_RightPanelX + 10, XPDR_RightPanelY + 9);
}

/* ---------------------------------------------- */
static void FormFreqString(unsigned int f, unsigned int mode, char v[])
{
    char d1, d2, d3, d4, d5;

    d1 = f / 10000 + '0';
    d2 = f % 10000 / 1000 + '0';
    d3 = f % 1000 / 100 + '0';
    d4 = f % 100 / 10 + '0';
    d5 = f % 10 + '0';
    if (mode == SwADF)
    {
        v[0] = ' ';
        v[1] = ' ';
        v[2] = d2;
        v[3] = d3;
        v[4] = d4;
        v[5] = '.';
        v[6] = d5;
        v[7] = '0';
        v[8] = 0;
    }
    else
    {
        v[0] = ' ';
        v[1] = ' ';
        v[2] = d1;
        v[3] = d2;
        v[4] = d3;
        v[5] = '.';
        v[6] = d4;
        v[7] = d5;
        v[8] = 0;
    }
}

/* ---------------------------------------------- */
static void SetParameters()
{
    switch (NavMode)
    {
    case SwVOR:
        DigitsMin   = 10800;
        DigitsMax   = 11795;
        DigitsInc   = 5;
        PanelDigits = Radio_Radios[0].NavVOR.Stby;
        break;
    case SwILS:
        DigitsMin   = 10800;
        DigitsMax   = 11195;
        DigitsInc   = 5;
        PanelDigits = Radio_Radios[0].NavILS.Stby;
        break;
    case SwADF:
        DigitsMin   = 1900;
        DigitsMax   = 5350;
        DigitsInc   = 5;
        PanelDigits = Radio_Radios[0].NavADF.Stby;
        break;
    case SwVHF1:
        DigitsMin   = 11800;
        DigitsMax   = 13595;
        DigitsInc   = 5;
        PanelDigits = Radio_Radios[0].ComVHF1.Stby;
        break;
    default:
        DigitsMin   = 0;
        DigitsMax   = 0;
        DigitsInc   = 0;
        PanelDigits = 0;
        break;
    }
}

/* ---------------------------------------------- */
static void SetFreq(unsigned int f, unsigned int mode)
{
    switch (mode)
    {
    case SwVOR:
        Radio_Radios[0].NavVOR.Stby = f;
        break;
    case SwILS:
        Radio_Radios[0].NavILS.Stby = f;
        break;
    case SwADF:
        Radio_Radios[0].NavADF.Stby = f;
        break;
    case SwVHF1:
        Radio_Radios[0].ComVHF1.Stby = f;
        break;
    default:
        return;
        break;
    }
}

/* ---------------------------------------------- */
static unsigned int GetStbyFreq(unsigned int mode)
{
    switch (mode)
    {
    case SwVOR:
        return Radio_Radios[0].NavVOR.Stby;
        break;
    case SwILS:
        return Radio_Radios[0].NavILS.Stby;
        break;
    case SwADF:
        return Radio_Radios[0].NavADF.Stby;
        break;
    case SwVHF1:
        return Radio_Radios[0].ComVHF1.Stby;
        break;
    default:
        return 0;
        break;
    }
}

/* ---------------------------------------------- */
static unsigned int GetActiveFreq(unsigned int mode)
{
    switch (mode)
    {
    case SwVOR:
        return Radio_Radios[0].NavVOR.Active;
        break;
    case SwILS:
        return Radio_Radios[0].NavILS.Active;
        break;
    case SwADF:
        return Radio_Radios[0].NavADF.Active;
        break;
    case SwVHF1:
        return Radio_Radios[0].ComVHF1.Active;
        break;
    default:
        return 0;
        break;
    }
}

/* ---------------------------------------------- */
static void ChangeoverSwitch()
{
    unsigned int       t;
    NavDefn_RadioPanel *w;

    {
        w = &Radio_Radios[0];

        switch (NavMode)
        {
        case SwVOR:
            t                = w->NavVOR.Active;
            w->NavVOR.Active = w->NavVOR.Stby;
            w->NavVOR.Stby   = t;
            FormFreqString(w->NavVOR.Active, NavMode, NAV_LeftPanelStr);
            FormFreqString(w->NavVOR.Stby, NavMode, NAV_RightPanelStr);
            break;
        case SwILS:
            t                = w->NavILS.Active;
            w->NavILS.Active = w->NavILS.Stby;
            w->NavILS.Stby   = t;
            FormFreqString(w->NavILS.Active, NavMode, NAV_LeftPanelStr);
            FormFreqString(w->NavILS.Stby, NavMode, NAV_RightPanelStr);
            break;
        case SwADF:
            t                = w->NavADF.Active;
            w->NavADF.Active = w->NavADF.Stby;
            w->NavADF.Stby   = t;
            FormFreqString(w->NavADF.Active, NavMode, ADF_LeftPanelStr);
            FormFreqString(w->NavADF.Stby, NavMode, ADF_RightPanelStr);
            break;
        case SwVHF1:
            t                 = w->ComVHF1.Active;
            w->ComVHF1.Active = w->ComVHF1.Stby;
            w->ComVHF1.Stby   = t;
            FormFreqString(w->ComVHF1.Active, NavMode, COM_LeftPanelStr);
            FormFreqString(w->ComVHF1.Stby, NavMode, COM_RightPanelStr);
            break;
        default:
            return;
            break;
        }
    }
}

/* ---------------------------------------------- */
static void DisplayPanel(char str[], unsigned int x, unsigned int y)
{
    Glib_Colour(Glib_RED);  // needs bright red
    Glib_SetFont(Glib_LFONT30, 10);
    Glib_Chars(str, x, y);	
}

/* ---------------------------------------------- */
void Radio_SaveRMP(NavDefn_NavDataPkt *pkt)
{
    memcpy(&pkt->SavedRadios[0], &Radio_Radios[0], sizeof(NavDefn_RadioPanel));
}

/* ---------------------------------------------- */
void Radio_RestoreRMP(IosDefn_RestoreVectorRecord v)
{
    memcpy(&Radio_Radios[0], &v.SavedRadios[0], sizeof(NavDefn_RadioPanel));

    SetParameters();
    PanelDigits = GetActiveFreq(SwVHF1);
    FormFreqString(PanelDigits, SwVHF1, COM_LeftPanelStr);
    PanelDigits = GetStbyFreq(SwVHF1);
    FormFreqString(PanelDigits, SwVHF1, COM_RightPanelStr);

    PanelDigits = GetActiveFreq(SwVOR);
    FormFreqString(PanelDigits, SwVOR, NAV_LeftPanelStr);
    PanelDigits = GetStbyFreq(SwVOR);
    FormFreqString(PanelDigits, SwVOR, NAV_RightPanelStr);

    PanelDigits = GetActiveFreq(SwADF);
    FormFreqString(PanelDigits, SwADF, ADF_LeftPanelStr);
    PanelDigits = GetStbyFreq(SwADF);
    FormFreqString(PanelDigits, SwADF, ADF_RightPanelStr);

    OldVor1 = 0;
    OldAdf1 = 0;
}

/* ---------------------------------------------- */
/* N.B. for some reason, (x0, y0) is the centre of the radio panel! */

void Radio_Radio(int x0, int y0, GLuint texobj_stack, GLuint texobj_simpleknob, GLuint texobj_innerknob, GLuint texobj_outerknob)
{
    Glib_SetTexture(texobj_stack);
    Glib_DrawTexture(x0, y0, 360, 360, 0.0, 0.0, 1.0, 1.0, 1.0);

    Glib_SetTexture(texobj_simpleknob);
    Glib_DrawTextureRotated(COM_OffKnobX, COM_OffKnobY, 24, 24, 0.0, 0.0, 1.0, 1.0, (float) (45-COM_OffKnobAngle), 1.0);
    Glib_SetTexture(texobj_outerknob);
    Glib_DrawTextureRotated(COM_KnobX, COM_KnobY, 48, 48, 0.0, 0.0, 1.0, 1.0, (float) -COM_OuterKnobAngle, 1.0);
    Glib_SetTexture(texobj_innerknob);
    Glib_DrawTextureRotated(COM_KnobX, COM_KnobY, 24, 24, 0.0, 0.0, 1.0, 1.0, (float) -COM_InnerKnobAngle, 1.0);
	
    Glib_SetTexture(texobj_simpleknob);
    Glib_DrawTextureRotated(NAV_OffKnobX, NAV_OffKnobY, 24, 24, 0.0, 0.0, 1.0, 1.0, (float) (45-NAV_OffKnobAngle), 1.0);
    Glib_SetTexture(texobj_outerknob);
    Glib_DrawTextureRotated(NAV_KnobX, NAV_KnobY, 48, 48, 0.0, 0.0, 1.0, 1.0, (float) -NAV_OuterKnobAngle, 1.0);
    Glib_SetTexture(texobj_innerknob);
    Glib_DrawTextureRotated(NAV_KnobX, NAV_KnobY, 24, 24, 0.0, 0.0, 1.0, 1.0, (float) -NAV_InnerKnobAngle, 1.0);
	
    Glib_SetTexture(texobj_simpleknob);
    Glib_DrawTextureRotated(ADF_OffKnobX, ADF_OffKnobY, 24, 24, 0.0, 0.0, 1.0, 1.0, (float) (45-ADF_OffKnobAngle), 1.0);
    Glib_SetTexture(texobj_outerknob);
    Glib_DrawTextureRotated(ADF_KnobX, ADF_KnobY, 48, 48, 0.0, 0.0, 1.0, 1.0, (float) -ADF_OuterKnobAngle, 1.0);
    Glib_SetTexture(texobj_innerknob);
    Glib_DrawTextureRotated(ADF_KnobX, ADF_KnobY, 24, 24, 0.0, 0.0, 1.0, 1.0, (float) -ADF_InnerKnobAngle, 1.0);
	
    Glib_SetTexture(texobj_simpleknob);
    Glib_DrawTextureRotated(XPDR_SelectorX, XPDR_SelectorY, 32, 32, 0.0, 0.0, 1.0, 1.0, (float) (90-XPDR_SelectorAngle), 1.0);
    Glib_SetTexture(texobj_outerknob);
    Glib_DrawTextureRotated(XPDR_KnobX, XPDR_KnobY, 48, 48, 0.0, 0.0, 1.0, 1.0, (float) -XPDR_OuterKnobAngle, 1.0);
    Glib_SetTexture(texobj_innerknob);
    Glib_DrawTextureRotated(XPDR_KnobX, XPDR_KnobY, 24, 24, 0.0, 0.0, 1.0, 1.0, (float) -XPDR_InnerKnobAngle, 1.0);
}

/* --------------------------------------------------- */
void Radio_SetRadio(unsigned int n, unsigned int chn, float f)
{
    int x = (int) (f * 100.0 +0.001);
	
	switch (chn)
	{
	    case 0:
		    Radio_Radios[0].NavILS.Active = x;
			NavMode = SwILS;
			break;
	    case 1:
		    Radio_Radios[1].NavILS.Active = x;
			NavMode = SwILS;
			break;
	    case 2:
		    Radio_Radios[0].NavVOR.Active = x;
			NavMode = SwVOR;
			break;
	    case 3:
		    Radio_Radios[1].NavVOR.Active = x;
			NavMode = SwVOR;
			break;
	    case 4:
		    Radio_Radios[0].NavADF.Active = x;
			NavMode = SwADF;
			break;
	    case 5:
		    Radio_Radios[1].NavADF.Active = x;
			NavMode = SwADF;
			break;
	}

    PanelDigits = GetActiveFreq(NavMode);
    FormFreqString(PanelDigits, NavMode, NAV_LeftPanelStr);
    DisplayPanel(NAV_LeftPanelStr, NAV_LeftPanelX + 8, NAV_LeftPanelY + 9);
    Radio_UpdateRadio();
}

/* ---------------------------------------------- */
void BEGIN_Radio()
{
    unsigned int i, j;
    NavDefn_RadioPanel *w;

    for (i=0; i<=1; i+=1)
	{
		w                 = &Radio_Radios[i];
		w->NavVOR.Active  = 10800;
		w->NavVOR.Stby    = 10800;
		w->NavILS.Active  = 10800;
		w->NavILS.Stby    = 10800;
		w->NavADF.Active  = 1900;
		w->NavADF.Stby    = 1900;
		w->ComVHF1.Active = 11800;
		w->ComVHF1.Stby   = 11800;
		w->ComVHF2.Active = 11800;
		w->ComVHF2.Stby   = 11800;
		w->ComVHF3.Active = 11800;
		w->ComVHF3.Stby   = 11800;
		w->ComHF1.Active  = 0;
		w->ComHF1.Stby    = 0;
		w->ComHF2.Active  = 0;
		w->ComHF2.Stby    = 0;
		w->ComAM.Active   = 0;
		w->ComAM.Stby     = 0;
		w->CrsKnob        = 360;
		w->NavGuard       = true;
		w->PowerSwitch     = false;
		for (j=1; j<=13; j+=1)
		{
		    w->PushSwitches[j] = false; 
		}
		w->Mode = 0;
	}

    for (i=0; i<NumberOfPowerSwitches; i+=1)
    {
        PowerSwitch[i] = false;
    }

    NavMode              = 0;
	COM_OffKnobAngle     = 1;
	COM_OffKnobAngle     = 1;
    NAV_InnerKnobAngle   = 1;
    COM_OuterKnobAngle   = 1;
    NAV_InnerKnobAngle   = 1;
    NAV_OuterKnobAngle   = 1;
	ADF_OffKnobAngle     = 1;
    ADF_InnerKnobAngle   = 1;
    ADF_OuterKnobAngle   = 1;
    XPDR_SelectorAngle   = 90;
    XPDR_InnerKnobAngle  = 1;
    XPDR_OuterKnobAngle  = 1;
	XPDR_Code            = 0;
    oldleftb             = false;
    COM_LeftPanelStr[0]  = '\0';
    COM_RightPanelStr[0] = '\0';
    NAV_LeftPanelStr[0]  = '\0';
    NAV_RightPanelStr[0] = '\0';
    ADF_LeftPanelStr[0]  = '\0';
    ADF_RightPanelStr[0] = '\0';
	XPDR_LeftPanelStr[0] = '\0';
	strcpy(XPDR_RightPanelStr, "0000");
    ticks                = 0;
    DigitsMin            = 11800;
    DigitsMax            = 13595;
    DigitsInc            = 5;
    OldVor1              = 0;
    OldAdf1              = 0;
}
