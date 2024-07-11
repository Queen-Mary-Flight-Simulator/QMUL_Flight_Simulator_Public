#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"
#include "XPLMCamera.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#if IBM
#include <windows.h>
#endif

#define MAX_ITEMS 11

static XPLMDataRef gPositionDataRef[MAX_ITEMS];

static char DataRefString[MAX_ITEMS][255] = {	"sim/flightmodel/position/local_x", "sim/flightmodel/position/local_y", "sim/flightmodel/position/local_z",
										"sim/flightmodel/position/lat_ref", "sim/flightmodel/position/lon_ref",	"sim/flightmodel/position/theta",
										"sim/flightmodel/position/phi", "sim/flightmodel/position/psi",
										"sim/flightmodel/position/latitude", "sim/flightmodel/position/longitude", "sim/flightmodel/position/elevation"};

static char DataRefDesc[MAX_ITEMS][255] = {"Local x", "Local y", "Local z", "Lat Ref", "Lon Ref", "Theta", "Phi", "Psi"};
static char Description[3][255] = {"Latitude", "Longitude", "Elevation"};

static int	Element = 0, IntVals[128];
static float FloatVals[128];
static int ByteVals[128];

static int MenuItem1;

static XPWidgetID			PositionWidget = NULL, PositionWindow = NULL;
static XPWidgetID			PositionApplyButton = NULL;
static XPWidgetID			PositionText[MAX_ITEMS] = {NULL};
static XPWidgetID			PositionEdit[MAX_ITEMS] = {NULL};
static XPWidgetID			UpArrow[MAX_ITEMS] = {NULL};
static XPWidgetID			DownArrow[MAX_ITEMS] = {NULL};
static XPWidgetID			LatLonAltApplyButton = NULL, LatLonRefApplyButton = NULL, ReloadSceneryButton = NULL;
static XPWidgetID			Position2Text[3] = {NULL};
static XPWidgetID			Position2Edit[3] = {NULL};

static void PositionMenuHandler(void *, void *);

static void CreatePosition(int x1, int y1, int w, int h);

static int PositionHandler(
						XPWidgetMessage			inMessage,
						XPWidgetID				inWidget,
						intptr_t				inParam1,
						intptr_t				inParam2);

static void DisplayFindDataRef(void);
static void ApplyValues(void);
static void ApplyLatLonRefValues(void);
static void ApplyLatLonAltValues(void);

inline	float	HACKFLOAT(float val)
{
	return val;
}
/*
#if IBM
inline	float	HACKFLOAT(float val)
{
	return val;
}
#else
inline long long HACKFLOAT(float val)
{
	double	d = val;
	long long temp;
	temp = *((long long *) &d);
	return temp;
}
#endif
*/

PLUGIN_API int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{
	XPLMMenuID	id;
	int			item;

	strcpy(outName, "Position");
	strcpy(outSig, "xpsdk.examples.position");
	strcpy(outDesc, "A plug-in that allows positioning of lat/lon etc.");

	item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "Position", NULL, 1);

	id = XPLMCreateMenu("Position", XPLMFindPluginsMenu(), item, PositionMenuHandler, NULL);
	XPLMAppendMenuItem(id, "Position", (void *)"Position", 1);

	MenuItem1 = 0;

	for (int Item=0; Item<MAX_ITEMS; Item++)
		gPositionDataRef[Item] = XPLMFindDataRef(DataRefString[Item]);

	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
	if (MenuItem1 == 1)
	{
		XPDestroyWidget(PositionWidget, 1);
		MenuItem1 = 0;
	}
}

PLUGIN_API void XPluginDisable(void)
{
}

PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam)
{
}

void PositionMenuHandler(void * mRef, void * iRef)
{
	if (!strcmp((char *) iRef, "Position"))
	{
		if (MenuItem1 == 0)
		{
			CreatePosition(300, 600, 300, 550);
			MenuItem1 = 1;
		}
		else
			if(!XPIsWidgetVisible(PositionWidget))
				XPShowWidget(PositionWidget);
	}
}


void CreatePosition(int x, int y, int w, int h)
{
	int x2 = x + w;
	int y2 = y - h;
	float FloatValue[MAX_ITEMS];
	double DoubleValue[3];

	char buffer[512];
	int Item;

	for (Item=0; Item<MAX_ITEMS; Item++)
		FloatValue[Item] = XPLMGetDataf(gPositionDataRef[Item]);

	/// X, Y, Z, Lat, Lon, Alt
	XPLMLocalToWorld(FloatValue[0], FloatValue[1], FloatValue[2], &DoubleValue[0], &DoubleValue[1], &DoubleValue[2]);
	DoubleValue[2] *= 3.28;

	PositionWidget = XPCreateWidget(x, y, x2, y2,
					1,	// Visible
					"Position",	// desc
					1,		// root
					NULL,	// no container
					xpWidgetClass_MainWindow);

	XPSetWidgetProperty(PositionWidget, xpProperty_MainWindowHasCloseBoxes, 1);

	PositionWindow = XPCreateWidget(x+50, y-50, x2-50, y2+50,
					1,	// Visible
					"",	// desc
					0,		// root
					PositionWidget,
					xpWidgetClass_SubWindow);

	XPSetWidgetProperty(PositionWindow, xpProperty_SubWindowType, xpSubWindowStyle_SubWindow);

	for (Item=0; Item<MAX_ITEMS-3; Item++)
	{
		PositionText[Item] = XPCreateWidget(x+60, y-(70 + (Item*30)), x+115, y-(92 + (Item*30)),
							1,	// Visible
							DataRefDesc[Item],// desc
							0,		// root
							PositionWidget,
							xpWidgetClass_Caption);

		sprintf(buffer, "%f", HACKFLOAT(FloatValue[Item]));
		PositionEdit[Item] = XPCreateWidget(x+120, y-(70 + (Item*30)), x+210, y-(92 + (Item*30)),
							1, buffer, 0, PositionWidget,
							xpWidgetClass_TextField);

		XPSetWidgetProperty(PositionEdit[Item], xpProperty_TextFieldType, xpTextEntryField);

		UpArrow[Item] = XPCreateWidget(x+212, y-(66 + (Item*30)), x+224, y-(81 + (Item*30)),
							1, "", 0, PositionWidget,
							xpWidgetClass_Button);

		XPSetWidgetProperty(UpArrow[Item], xpProperty_ButtonType, xpLittleUpArrow);

		DownArrow[Item] = XPCreateWidget(x+212, y-(81 + (Item*30)), x+224, y-(96 + (Item*30)),
							1, "", 0, PositionWidget,
							xpWidgetClass_Button);

		XPSetWidgetProperty(DownArrow[Item], xpProperty_ButtonType, xpLittleDownArrow);
	}

	PositionApplyButton = XPCreateWidget(x+50, y-310, x+140, y-332,
					1, "Apply Data", 0, PositionWidget,
					xpWidgetClass_Button);

	XPSetWidgetProperty(PositionApplyButton, xpProperty_ButtonType, xpPushButton);


	LatLonRefApplyButton = XPCreateWidget(x+145, y-310, x+240, y-332,
					1, "Apply LatLonRef", 0, PositionWidget, 
					xpWidgetClass_Button);

	XPSetWidgetProperty(LatLonRefApplyButton, xpProperty_ButtonType, xpPushButton);


	for (Item=0; Item<3; Item++)
	{
		Position2Text[Item] = XPCreateWidget(x+60, y-(350 + (Item*30)), x+115, y-(372 + (Item*30)),
							1,	// Visible
							Description[Item],// desc
							0,		// root
							PositionWidget,
							xpWidgetClass_Caption);
	
		sprintf(buffer, "%lf", HACKFLOAT(DoubleValue[Item]));
		Position2Edit[Item] = XPCreateWidget(x+120, y-(350 + (Item*30)), x+210, y-(372 + (Item*30)),
							1, buffer, 0, PositionWidget,
							xpWidgetClass_TextField);

		XPSetWidgetProperty(PositionEdit[Item], xpProperty_TextFieldType, xpTextEntryField);
	}
	
	LatLonAltApplyButton = XPCreateWidget(x+70, y-440, x+220, y-462,
						1, "Apply LatLonAlt", 0, PositionWidget, 
						xpWidgetClass_Button);

	XPSetWidgetProperty(LatLonAltApplyButton, xpProperty_ButtonType, xpPushButton);

	ReloadSceneryButton = XPCreateWidget(x+70, y-465, x+220, y-487,
						1, "Reload Scenery", 0, PositionWidget, 
						xpWidgetClass_Button);

	XPSetWidgetProperty(ReloadSceneryButton, xpProperty_ButtonType, xpPushButton);

	XPAddWidgetCallback(PositionWidget, PositionHandler);
}

int	PositionHandler(
						XPWidgetMessage			inMessage,
						XPWidgetID				inWidget,
						intptr_t				inParam1,
						intptr_t				inParam2)
{
	float FloatValue[MAX_ITEMS];
	char buffer[512];
	int Item;

	for (Item=0; Item<MAX_ITEMS; Item++)
		FloatValue[Item] = XPLMGetDataf(gPositionDataRef[Item]);

	if (inMessage == xpMessage_CloseButtonPushed)
		{
			if (MenuItem1 == 1)
			{
				XPHideWidget(PositionWidget);
			}
			return 1;
		}

	if (inMessage == xpMsg_PushButtonPressed)
	{
		if (inParam1 == (intptr_t)PositionApplyButton)
		{
			ApplyValues();
			return 1;
		}

		if (inParam1 == (intptr_t)LatLonRefApplyButton)
		{
			ApplyLatLonRefValues();
			return 1;
		}

		if (inParam1 == (intptr_t)LatLonAltApplyButton)
		{
			ApplyLatLonAltValues();
			return 1;
		}

		if (inParam1 == (intptr_t)ReloadSceneryButton)
		{
			XPLMReloadScenery();
			return 1;
		}

		for (Item=0; Item<MAX_ITEMS-3; Item++)
		{
			if (inParam1 == (intptr_t)UpArrow[Item])
			{
				FloatValue[Item] += 1.0;
				sprintf(buffer, "%f", HACKFLOAT(FloatValue[Item]));
				XPSetWidgetDescriptor(PositionEdit[Item], buffer);
				XPLMSetDataf(gPositionDataRef[Item], FloatValue[Item]);
				return 1;
			}
		}

		for (Item=0; Item<MAX_ITEMS-3; Item++)
		{
			if (inParam1 == (intptr_t)DownArrow[Item])
			{
				FloatValue[Item] -= 1.0;
				sprintf(buffer, "%f", HACKFLOAT(FloatValue[Item]));
				XPSetWidgetDescriptor(PositionEdit[Item], buffer);
				XPLMSetDataf(gPositionDataRef[Item], FloatValue[Item]);
				return 1;
			}
		}
	}
	return 0;
}						

void ApplyValues(void)
{
	char	buffer[512];

	for (int Item=0; Item<MAX_ITEMS-3; Item++)
	{
		XPGetWidgetDescriptor(PositionEdit[Item], buffer, 512);
		XPLMSetDataf(gPositionDataRef[Item], atof(buffer));
	}
}

void ApplyLatLonRefValues(void)
{
	float FloatValue;
	char	buffer[512];

	XPGetWidgetDescriptor(PositionEdit[3], buffer, 512);
	FloatValue = atof(buffer);
	XPLMSetDataf(gPositionDataRef[3], FloatValue);

	XPGetWidgetDescriptor(PositionEdit[4], buffer, 512);
	FloatValue = atof(buffer);
	XPLMSetDataf(gPositionDataRef[4], FloatValue);
}

void ApplyLatLonAltValues(void)
{
	float FloatValue[3];
	double DoubleValue[3];
	char	buffer[512];
	int Item;

	// This gets the lat/lon/alt from the widget text fields
	for (Item=0; Item<3; Item++)
	{
		XPGetWidgetDescriptor(Position2Edit[Item], buffer, 512);
		FloatValue[Item] = atof(buffer);
	}

	/// Lat, Lon, Alt, X, Y, Z
	XPLMWorldToLocal(FloatValue[0], FloatValue[1], FloatValue[2] / 3.28, &DoubleValue[0], &DoubleValue[1], &DoubleValue[2]);  

	for (Item=0; Item<3; Item++)
	{
		// This writes out the lat/lon/alt from the widget text fields back to the datarefs
		XPLMSetDataf(gPositionDataRef[Item+8], FloatValue[Item]);
		// This writes out the x,y,z datarefs after conversion from lat/lon/alt back to the datarefs
		XPLMSetDataf(gPositionDataRef[Item], DoubleValue[Item]);
	}

	ApplyLatLonRefValues();
}
