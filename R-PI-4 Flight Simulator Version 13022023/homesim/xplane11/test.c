#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "XPLMPlanes.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"

int AircraftLoaded = 0;
char AircraftPath[1024], AirportCode[256];

float DefaultAircraftLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon);

PLUGIN_API int XPluginStart(
char *        outName,
char *        outSig,
char *        outDesc)
{
    char PluginDataFile[1024];
    char TempAircraftPath[1024];
    FILE *InStream;

    strcpy(outName, "DefaultAircraft");
    strcpy(outSig, "xpsdk.sandybarbour.DefaultAircraft");
    strcpy(outDesc, "A plug-in for loading a default aircraft at startup.");

    #if IBM
    char *pFileName = "Resources\\Plugins\\DefaultAircraft.ini";
    #elif APL
    char *pFileName = "Resources:Plugins:DefaultAircraft.ini";
    #else
    char *pFileName = "Resources/plugins/DefaultAircraft.ini";
    #endif

    XPLMGetSystemPath(PluginDataFile);
    strcat(PluginDataFile, pFileName);
    InStream = fopen(PluginDataFile, "r");
    if (InStream != NULL)
    {
        fgets(TempAircraftPath, 255, InStream);
        if (AircraftPath[strlen(TempAircraftPath)-1] == '\r')
            TempAircraftPath[strlen(TempAircraftPath)-1] = 0;
        if (TempAircraftPath[strlen(TempAircraftPath)-1] == '\n')
            TempAircraftPath[strlen(TempAircraftPath)-1] = 0;

        fgets(AirportCode, 255, InStream);
        if (AirportCode[strlen(AirportCode)-1] == '\r')
            AirportCode[strlen(AirportCode)-1] = 0;
        if (AirportCode[strlen(AirportCode)-1] == '\n')
            AirportCode[strlen(AirportCode)-1] = 0;

        XPLMGetSystemPath(AircraftPath);
        strcat(AircraftPath, TempAircraftPath);

        fclose(InStream);
    }
    else
    {
        XPLMDebugString("DefaultAircaft.xpl, Error - DefaultAircaft.ini file not found\n");
        AircraftLoaded = 1;
    }

    XPLMRegisterFlightLoopCallback(DefaultAircraftLoopCB, 1.0, NULL);
    return 1;
}

PLUGIN_API void    XPluginStop(void)
{
}

PLUGIN_API int XPluginEnable(void)
{
return 1;
}

PLUGIN_API void XPluginDisable(void)
{
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, long inMsg, void * inParam)
{
}

float DefaultAircraftLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
    if (!AircraftLoaded)
    {
        XPLMSetUsersAircraft(AircraftPath);
        XPLMPlaceUserAtAirport(AirportCode);
        AircraftLoaded = 1;
    }
    return 1.0;
}
