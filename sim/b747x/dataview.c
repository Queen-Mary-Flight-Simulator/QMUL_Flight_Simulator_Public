#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SIM/maths.h>
#include <SIM/aerodefn.h>
#include <SIM/glibx.h>

#include "ioslink.h"
#include "iolib.h"
#include "map.h"

#define X0 (Glibx_SCREENWIDTH / 2)
#define FontWidth 9  /* font12.fnt */

void DisplayVariable(char str1[], char str2[], float variable, int x, int y);

void DisplayVariable(char str1[], char str2[], float variable, int x, int y)
{
    char v[100];

    Glibx_Colour(Glibx_BLUE);
    Map_Chars(str1, x, y);
    sprintf(v, str2, variable);
    Glibx_Colour(Glibx_BLACK);
    Map_Chars(v, x + 200 - FontWidth * strlen(str2), y);
}

void DataView_Display(void)
{
    float OAT = 15.0 + 0.0065 * IosLink_AeroPkt.Pz;

    //Glibx_SetFont(Glibx_BITMAP_HELVETICA_18, 10);

    DisplayVariable("Pitch (deg)", " %10.2f", IosLink_AeroPkt.Pitch * Maths_ONERAD, X0 + 10, 710);
    DisplayVariable("Roll  (deg)", " %10.2f", IosLink_AeroPkt.Roll * Maths_ONERAD, X0 + 320, 710);
    DisplayVariable("Yaw   (deg)", " %10.2f", IosLink_AeroPkt.Yaw * Maths_ONERAD, X0 + 630, 710);
    DisplayVariable("P   (deg/s)", " %10.2f", IosLink_AeroPkt.P * Maths_ONERAD, X0 + 10, 680);
    DisplayVariable("Q   (deg/s)", " %10.2f", IosLink_AeroPkt.Q * Maths_ONERAD, X0 + 320, 680);
    DisplayVariable("R   (deg/s)", " %10.2f", IosLink_AeroPkt.R * Maths_ONERAD, X0 + 630, 680);
    DisplayVariable("Pdot (deg/s/s)", " %10.2f", IosLink_AeroPkt.PDot * Maths_ONERAD, X0 + 10, 650);
    DisplayVariable("Qdot (deg/s/s)", " %10.2f", IosLink_AeroPkt.QDot * Maths_ONERAD, X0 + 320, 650);
    DisplayVariable("Rdot (deg/s/s)", " %10.2f", IosLink_AeroPkt.RDot * Maths_ONERAD, X0 + 630, 650);
    DisplayVariable("Vn (Kt)", " %10.2f", IosLink_AeroPkt.Vn * 1.943844, X0 + 10, 620);
    DisplayVariable("Ve  (Kt)", " %10.2f", IosLink_AeroPkt.Ve * 1.943844, X0 + 320, 620);
    DisplayVariable("Vd    (fpm)", " %10.2f", -IosLink_AeroPkt.Vd * 196.850394, X0 + 630, 620);
    DisplayVariable("Pz (ft)", " %10.2f", -IosLink_AeroPkt.Pz * 3.28084, X0 + 10, 590);
    DisplayVariable("Latitude (deg)", " %10.2f", IosLink_AeroPkt.Latitude * Maths_ONERAD, X0 + 320, 590);
    DisplayVariable("Longitude (deg)", " %10.2f", IosLink_AeroPkt.Longitude * Maths_ONERAD, X0 + 630, 590);
    DisplayVariable("U (m/s)", " %10.2f", IosLink_AeroPkt.U, X0 + 10, 560);
    DisplayVariable("V (m/s)", " %10.2f", IosLink_AeroPkt.V, X0 + 320, 560);
    DisplayVariable("W (m/s)", " %10.2f", IosLink_AeroPkt.W, X0 + 630, 560);
    DisplayVariable("Udot (m/s/s)", " %10.2f", IosLink_AeroPkt.UDot, X0 + 10, 530);
    DisplayVariable("Vdot (m/s/s)", " %10.2f", IosLink_AeroPkt.VDot, X0 + 320, 530);
    DisplayVariable("Wdot (m/s/s)", " %10.2f", IosLink_AeroPkt.WDot, X0 + 630, 530);
    DisplayVariable("Pmt (Nm)", " %10.1f", IosLink_AeroPkt.Pmt, X0 + 10, 500);
    DisplayVariable("Rmt (Nm)", " %10.1f", IosLink_AeroPkt.Rmt, X0 + 320, 500);
    DisplayVariable("Ymt (Nm)", " %10.1f", IosLink_AeroPkt.Ymt, X0 + 630, 500);
    DisplayVariable("Alpha (deg)", " %10.2f", IosLink_AeroPkt.Alpha * Maths_ONERAD, X0 + 10, 470);
    DisplayVariable("Alphadot (deg/s)", " %10.2f", IosLink_AeroPkt.AlphaDot * Maths_ONERAD, X0 + 320, 470);
    DisplayVariable("Beta (deg)", " %10.2f", IosLink_AeroPkt.Beta, X0 + 630, 470);
    DisplayVariable("Betadot (deg/s)", " %10.2f", IosLink_AeroPkt.BetaDot, X0 + 10, 440);
    DisplayVariable("Cl", " %10.2f", IosLink_AeroPkt.Cl, X0 + 320, 440);
    DisplayVariable("Cd", " %10.2f", IosLink_AeroPkt.Cd, X0 + 630, 440);
    DisplayVariable("Lift (N)", " %10.1f", IosLink_AeroPkt.Lift, X0 + 10, 410);
    DisplayVariable("Thrust (N)", " %10.1f", IosLink_AeroPkt.Thrust, X0 + 320, 410);
    DisplayVariable("Drag (N)", " %10.1f", IosLink_AeroPkt.Drag, X0 + 630, 410);
    DisplayVariable("Sideforce (N)", " %10.1f", IosLink_AeroPkt.SideForce, X0 + 10, 380);
    DisplayVariable("Xforce (N)", " %10.1f", IosLink_AeroPkt.XForce, X0 + 320, 380);
    DisplayVariable("Yforce (N)", " %10.1f", IosLink_AeroPkt.YForce, X0 + 630, 380);
    DisplayVariable("Zforce (N)", " %10.1f", IosLink_AeroPkt.ZForce, X0 + 10, 350);
    DisplayVariable("Vc (Kt)", " %10.2f", IosLink_AeroPkt.Vc * 1.943844, X0 + 320, 350);
    DisplayVariable("Mach", " %10.2f", IosLink_AeroPkt.MachNumber, X0 + 630, 350);
    DisplayVariable("Rho", " %10.2f", IosLink_AeroPkt.Rho, X0 + 10, 320);
    DisplayVariable("OAT", " %10.2f", OAT, X0 + 320, 320);
    DisplayVariable("EPR", " %10.2f", IosLink_EngPkt.Engines[0].Epr, X0 + 630, 320);
    DisplayVariable("Flap", " %10.2f", IosLink_AeroPkt.FlapPosition, X0 + 10, 290);
    DisplayVariable("Gear", " %10.2f", IosLink_AeroPkt.GearPosition, X0 + 320, 290);
    DisplayVariable("Throttle", " %10.2f", IosLink_EngPkt.EngineLevers[0], X0 + 630, 290);
    DisplayVariable("Elevator", " %10.2f", IosLink_AeroPkt.Elevator, X0 + 10, 260);
    DisplayVariable("Aileron", " %10.2f", IosLink_AeroPkt.Aileron, X0 + 320, 260);
    DisplayVariable("Rudder", " %10.2f", IosLink_AeroPkt.Rudder, X0 + 630, 260);
    DisplayVariable("Elevator trim", " %10.2f", IosLink_AeroPkt.ElevatorTrim, X0 + 10, 230);
    DisplayVariable("Aileron trim", " %10.2f", IosLink_AeroPkt.AileronTrim, X0 + 320, 230);
    DisplayVariable("Rudder trim", " %10.2f", IosLink_AeroPkt.RudderTrim, X0 + 630, 230);
    DisplayVariable("Flight data #1", " %10.2f", IosLink_AeroPkt.FlightData[0], X0 + 10, 200);
    DisplayVariable("Flight data #2", " %10.2f", IosLink_AeroPkt.FlightData[1], X0 + 320, 200);
    DisplayVariable("Flight data #3", " %10.2f", IosLink_AeroPkt.FlightData[2], X0 + 630, 200);
}

void BEGIN_Dataview()
{
}

