/* +------------------------------+---------------------------------+
   | Module      : nav.c          | Version         : 1.1           | 
   | Last Edit   : 30-03-2016     | Reference Number: 03-01-05      |
   +------------------------------+---------------------------------+
   | Computer    : DELL2                                            |
   | Directory   : /dja/aerosoft/cranfield/software/nfd/            |
   | Compiler    : gcc 4.8.1                                        |
   | OS          : Windows7                                         |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : Navigation library                               |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#include <SIM/maths.h>
#include <SIM/navlib.h>
#include <SIM/navdefn.h>
#include <SIM/iodefn.h>

#include "navlink.h"
#include "radio.h"
#include "fcu.h"
#include "nav.h"

#define ONERAD        (180.0L / M_PI)
#define TWOPI         (M_PI * 2.0L)
#define PIBY2         (M_PI / 2.0L)
#define DEG2          (2.0 / ONERAD)
#define DEG3          (3.0 / ONERAD)
#define DEG25         (25.0 / ONERAD)
#define DEG60         (60.0 / ONERAD)
#define DEG170        (170.0 / ONERAD)

#define PapiDistance  0.0

unsigned int       Nav_MorseChannel;
char               Nav_MorseIdent[4];

unsigned int       Nav_NumberOfWayPoints = 0;
Nav_WayPointRecord Nav_WayPoints[Nav_MaxWayPoints];

unsigned int       Nav_CurrentRunway;
Nav_NavRecord      Nav_ILS1;
Nav_NavRecord      Nav_VOR1;
Nav_NavRecord      Nav_VOR2;
Nav_NavRecord      Nav_ADF1;
Nav_NavRecord      Nav_ADF2;
Nav_NavRecord      Nav_WayPoint;

float              Nav_GroundLevel;
float              Nav_Rmi_Dir1, Nav_Rmi_Dir2;
int                Nav_HSI_Crs, Nav_HSI_Hdg, Nav_VOR_Obs;
float              Nav_HSI_Localiser, Nav_HSI_GlideSlope;
bool               Nav_HSI_ILSMode, Nav_HSI_Status;
float              Nav_VOR_Localiser, Nav_VOR_GlideSlope;
bool               Nav_VOR_ILSMode, Nav_VOR_Status;
float              Nav_DmeDistance;
bool               Nav_OuterMarker, Nav_MiddleMarker;
bool               Nav_InnerMarker, Nav_MarkerTest;
float              Nav_Track;
unsigned int       Nav_TrackRange;
float              Nav_MagneticVariation;
unsigned int       Nav_BaroPressure1;
unsigned int       Nav_BaroPressure2;
unsigned int       Nav_RegionalQNH;
unsigned int       Nav_FlightPlan_Segment = 0;

static unsigned int oldFlightPlan_Segment = 0;
static bool OldFDOn;

static void  NavCalculations(Nav_NavRecord *Nav, float Obs, bool Operational);
static void  UpdateMarkers();
static float GetTAS(float IAS, float Z);

/* ------------------------------------------------------ */
static float GetTAS(float IAS, float Z)   /* nb. IAS (m/s), Z +ve altitude (m) */
{
    const double Rs  =   8314.32;  /* Nm/(kmol K), gas constant */
    const double M0  =   28.9644;  /* kg/kmol, mean molecular weight of air */
    const double g0  =   9.80665;  /* m/s^2, acceleration of gravity at 45.5425 deg lat. */
    const double r0  = 6356766.0;  /* m, Earth radius at g0 */
    const double P0  =  101325.0;  /* Pa, air pressure at g0 */
    const double T0  =    288.15;  /* K, standard sea-level temperature */

    double As;   /* constant */
    double H;    /* geopotential height m */
    double P;    /* pressure Pa */
    double T;    /* temperature K */

    double Rho;
    double DensityRatio;
  
    As = g0 * M0 / Rs;
  
    H = r0 * (double) Z / (r0 + (double) Z);
    T = T0 - 0.0065 * H;
    P = P0 * exp(As * log(T0 / T) / (-0.0065));
  
    Rho = (P / T) * (M0 / Rs);
    DensityRatio = Rho / 1.225;
    return IAS / sqrt(DensityRatio);
}

/* ------------------------------------------------------ */
void Nav_CheckFlightPlan()
{
    static double dturn = 0.0;       /* distance to start turn to next wp */
    static double cur_trk = 0.0;     /* track to next way point */
    static double next_trk = 0.0;    /* track after next way point */
    static double vseg = 0.0;        /* segment speed */
    static double dtrk = 0.0;        /* angle between current segment and next segment */
    static bool newseg = false;      /* used to intiate speed and alt change on new segment */
    static float maxbankangle = 0.0; /* normally 25, but less for light aircraft */

    double bearing;                  /* bearing from a/c to next wp */
    double wpdist;                   /* distance to next way point */
    double trackerr;                 /* off track error */
    double beta;                     /* used in wpdist calc */
    double yrate;                    /* used to compute max bank angle */

    if ((Nav_NumberOfWayPoints < 2) || (Nav_FlightPlan_Segment < 1)) 
    {
        Nav_WayPoint.BeaconStatus = false;  /* deactivate flight plan */
        return;
    }

    if (Nav_FlightPlan_Segment <= (Nav_NumberOfWayPoints - 1)) 
    {
        if (oldFlightPlan_Segment != Nav_FlightPlan_Segment) 
        {
            Nav_WayPoint.BeaconStatus = true;  /* activate flight plan */
            if (Nav_FlightPlan_Segment == 1)
            {
                FCU_ALT = Nav_WayPoints[Nav_FlightPlan_Segment].WayPointAltitude;
                FCU_SPD = Nav_WayPoints[Nav_FlightPlan_Segment].WayPointSpeed;
            }
            FCU_ALTKnob = NavDefn_Middle;
            FCU_HDGKnob = NavDefn_Middle;
            FCU_SPDKnob = NavDefn_Middle;
            FCU_APPR = false;

            oldFlightPlan_Segment = Nav_FlightPlan_Segment;

            cur_trk = NavLib_Bearing((double) Nav_WayPoints[Nav_FlightPlan_Segment].WayPointLatitude,
                                     (double) Nav_WayPoints[Nav_FlightPlan_Segment].WayPointLongitude,
                                     (double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLatitude,
                                     (double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLongitude);
            Nav_WayPoint.RunwayQdm = (float) cur_trk - Nav_MagneticVariation;  /* heading hold must be magnetic */

            if (Nav_FlightPlan_Segment <= (Nav_NumberOfWayPoints - 2)) 
            {
                next_trk = NavLib_Bearing((double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLatitude,
                                          (double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLongitude,
                                          (double) Nav_WayPoints[Nav_FlightPlan_Segment+2].WayPointLatitude,
                                          (double) Nav_WayPoints[Nav_FlightPlan_Segment+2].WayPointLongitude);

                dtrk = next_trk - cur_trk;
                dtrk = Maths_Double_Normalise(dtrk);
                if (fabs(dtrk) < DEG2) 
                {
                    dturn = 1000.0;
                }
                else if (fabs(dtrk) > DEG170) 
                {
                    dturn = 10000.0;
                }
                else
                {
                    vseg = GetTAS(Nav_WayPoints[Nav_FlightPlan_Segment].WayPointSpeed / 1.944,
                                  Maths_Metres(Nav_WayPoints[Nav_FlightPlan_Segment].WayPointAltitude));
                    beta = M_PI - dtrk;
                    beta = Maths_Double_Normalise(beta);
                    beta = fabs(beta);
                    yrate = 9.81 * tan(DEG25) / vseg;
                    if (yrate > DEG3)
                    {
                        maxbankangle = atan(DEG3 * vseg / 9.81);
                    }
                    else
                    {
                        maxbankangle = DEG25;
                    }
                    dturn = (vseg * vseg) / (9.81 * tan(maxbankangle) * tan(beta / 2.0)) + vseg * 5.0;
                }
            }
        }

        bearing = NavLib_Bearing(NavLink_AeroPkt.Latitude,
                                 NavLink_AeroPkt.Longitude,
                                 (double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLatitude,
                                 (double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLongitude);
        wpdist = NavLib_Distance((double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLatitude,
                                 (double) Nav_WayPoints[Nav_FlightPlan_Segment+1].WayPointLongitude,
                                 NavLink_AeroPkt.Latitude,
                                 NavLink_AeroPkt.Longitude);

        trackerr = bearing - cur_trk;
        trackerr = Maths_Double_Normalise(trackerr);
        Nav_WayPoint.LocaliserError = (float) trackerr;

        if (Nav_FlightPlan_Segment >= (Nav_NumberOfWayPoints - 1)) 
        {
            if (wpdist < 1000.0) 
            {
                Nav_FlightPlan_Segment = 0;
                oldFlightPlan_Segment = 0;
                Nav_WayPoint.BeaconStatus = false;  /* deactivate flight plan */
                FCU_ALTKnob = NavDefn_Middle;
                FCU_HDGKnob = NavDefn_Middle;
                return;
            }
        }

        if ((fabs(trackerr) < 0.01) && (newseg))
        {
            FCU_ALT = Nav_WayPoints[Nav_FlightPlan_Segment].WayPointAltitude;
            FCU_SPD = Nav_WayPoints[Nav_FlightPlan_Segment].WayPointSpeed;
            newseg = false;
        }

        if (wpdist < dturn) 
        {
            Nav_FlightPlan_Segment = Nav_FlightPlan_Segment + 1;
            newseg = true;
        }
    }
}

/* ------------------------------------------------------ */
void Nav_UpdateNav()
{
    Nav_HSI_Crs = Radio_Radios[0].CrsKnob;
    NavCalculations(&Nav_ILS1, Maths_Rads((float) Nav_HSI_Crs), true);
    NavCalculations(&Nav_VOR1, Maths_Rads((float) Nav_HSI_Crs), true);
    NavCalculations(&Nav_VOR2, Maths_Rads((float) Nav_HSI_Crs), true);
    if (FCU_ModeSelector == NavDefn_ModeILS) 
    {
        Nav_HSI_Localiser = Nav_ILS1.LocaliserError;
        Nav_HSI_GlideSlope = Nav_ILS1.GlideSlopeError;
        Nav_HSI_ILSMode = Nav_ILS1.IlsFrequency;
        Nav_HSI_Status = Nav_ILS1.BeaconStatus;
        if (Nav_ILS1.BeaconStatus) 
        {
            Nav_DmeDistance = Nav_ILS1.SlantDistance;
        } else 
        {
            Nav_DmeDistance = -10000.0;
        }
    } 
    else 
    {
        Nav_HSI_Localiser = Nav_VOR1.LocaliserError;
        Nav_HSI_GlideSlope = Nav_VOR1.GlideSlopeError;
        Nav_HSI_ILSMode = Nav_VOR1.IlsFrequency;
        Nav_HSI_Status = Nav_VOR1.BeaconStatus;
        if (Nav_VOR1.BeaconStatus) 
        {
            Nav_DmeDistance = Nav_VOR1.SlantDistance;
        } 
        else 
        {
            Nav_DmeDistance = -10000.0;
        }
    }
    NavCalculations(&Nav_ADF1, 0.0, true);
    NavCalculations(&Nav_ADF2, 0.0, true);
    if (FCU_NavSwitch1 == NavDefn_NavADF) 
    {
        Nav_Rmi_Dir1 = Nav_ADF1.BearingToStation - NavLink_AeroPkt.Yaw;
    } 
    else if (FCU_NavSwitch1 == NavDefn_NavVOR) 
    {
        Nav_Rmi_Dir1 = Nav_VOR1.BearingToStation - NavLink_AeroPkt.Yaw;
    } 
    else 
    {
        Nav_Rmi_Dir1 = PIBY2;
    }
    if (FCU_NavSwitch2 == NavDefn_NavVOR) 
    {
        Nav_Rmi_Dir2 = Nav_VOR2.BearingToStation - NavLink_AeroPkt.Yaw;
    } 
    else if (FCU_NavSwitch2 == NavDefn_NavADF) 
    {
        Nav_Rmi_Dir2 = Nav_ADF2.BearingToStation - NavLink_AeroPkt.Yaw;
    } 
    else 
    {
        Nav_Rmi_Dir2 = PIBY2;
    }

    if (Nav_CurrentRunway == 0 || (!NavLink_AeroPkt.OnTheGround) || NavLink_Restored)
    {
        if (NavLink_Restored)
        {
            Nav_CurrentRunway = NavLink_IosPkt.RestoreVector.CurrentRunway;
            NavLink_Restored = false;
        }
        else if (!NavLink_AeroPkt.OnTheGround)
        {
            Nav_CurrentRunway = NavLib_NearestRunway(NavLink_AeroPkt.Latitude, NavLink_AeroPkt.Longitude);
        }
    }
    
    if (Nav_CurrentRunway > 0)
    {
        Nav_GroundLevel = -(float) (NavLib_Runways[Nav_CurrentRunway].Elevation);
    }
    else
    {
        Nav_GroundLevel = 0.0;
    }
    Nav_BaroPressure1 = 1013;
    Nav_BaroPressure2 = 1013;
    UpdateMarkers();
}

/* ------------------------------------------------------ */
static void NavCalculations(Nav_NavRecord *Nav, float Obs, bool Operational)
{
    bool OK;
    bool DmeOnly;
    unsigned int BeaconNumber;
    bool IlsBeacon;
    bool VorBeacon;
    bool AdfBeacon;
    float z;
    float dground;
    float dslant;
    float MaxRange;
    float qTrue;
    float t1, t2;
    NavLib_BeaconRecord *bcn;

    BeaconNumber = Nav->SelectedBeacon;
    OK = BeaconNumber > 0 && Operational;
    DmeOnly = false;
    IlsBeacon = false;
    MaxRange = 0.0;
    if (OK) 
    {
        bcn = &NavLib_Beacons[BeaconNumber];

        IlsBeacon = NavLib_ILS & bcn->Navaid;
        VorBeacon = NavLib_VOR & bcn->Navaid;
        AdfBeacon = NavLib_NDB & bcn->Navaid;
        if (IlsBeacon | VorBeacon)  /* VHF line-of-sight */ 
        {
            z = -(float) (NavLib_Beacons[BeaconNumber].Elevation) - (float) NavLink_AeroPkt.Pz;
        } 
        else 
        {
            z = Nav_GroundLevel - (float) NavLink_AeroPkt.Pz;
        }
        if (z < 0.0) 
        {
            z = 0.0;
        }
        MaxRange = 3568.0 * (sqrt(z) + sqrt(-NavLink_AeroPkt.Pz));
        if (MaxRange < 3000.0) 
        {
            MaxRange = 3000.0;
        } 
        else if (MaxRange > 160000.0) 
        {
            MaxRange = 160000.0;
        }
        if (AdfBeacon) 
        {
            MaxRange = (float) bcn->Range * 1852.0;
        }
        dground = (float) NavLib_Distance(NavLink_AeroPkt.Latitude, 
                                          NavLink_AeroPkt.Longitude, 
                                          (double) bcn->BeaconLatitude, 
                                          (double) bcn->BeaconLongitude);
        dslant = sqrt(dground * dground + z * z);
        if (dslant <= MaxRange) 
        {
            Nav->BearingToStation = (float) NavLib_Bearing(NavLink_AeroPkt.Latitude, 
                                                           NavLink_AeroPkt.Longitude, 
                                                           (double) bcn->BeaconLatitude, 
                                                           (double) bcn->BeaconLongitude);
            if (IlsBeacon) 
            {
                //Nav->RunwayQdm = NavLib_Beacons[BeaconNumber].Qdm - Nav_MagneticVariation;  /* magnetic */
                Nav->RunwayQdm = NavLib_Beacons[BeaconNumber].Qdm;  /* magnetic */
                Nav->RunwayQdm = Maths_Normalise(Nav->RunwayQdm);
                qTrue = Nav->RunwayQdm + Nav_MagneticVariation;
                qTrue = Maths_Normalise(qTrue);
                Nav->LocaliserError = Nav->BearingToStation - qTrue;
                t1 = dground * sin(Nav->LocaliserError);
                t2 = dground * cos(Nav->LocaliserError);
                t2 += 416.0;  /* G/S transmitter typically 400m past start of the runway */
                dground = sqrt(t1 * t1 + t2 * t2);
                dslant = sqrt(dground * dground + z * z);
                Nav->GlideSlopeError = 0.052359877 - atan2(z, dground);
                t2 -= 416.0;  /* reset to start of runway */
                if (Nav_CurrentRunway > 0)
                {
                    t2 = t2 + NavLib_Runways[Nav_CurrentRunway].Length;  /* localiser transmitter 300m past end of the runway */
                }
                else
                {
                    t2 = t2 + 3000.0;
                }
                Nav->LocaliserError  = atan2(t1, t2);
            } 
            else 
            {
                Nav->RunwayQdm = 0.0;
                Nav->LocaliserError = Nav->BearingToStation - Obs - Nav_MagneticVariation;
            }
            Nav->LocaliserError = Maths_Normalise(Nav->LocaliserError);
            Nav->SlantDistance = dslant;
            if (bcn->Navaid == NavLib_DME) 
            {
                DmeOnly = true;
            }
        } 
        else 
        {
            OK = false;
        }
    }

    Nav->BeaconStatus = OK && !DmeOnly;
    Nav->IlsFrequency = IlsBeacon;
}

/* ------------------------------------------------------ */
static void UpdateMarkers()
{
    float Dme1;
    float OM1Range;
    float MM1Range;

    Nav_OuterMarker = false;
    Nav_MiddleMarker = false;
    Nav_InnerMarker = true;
  
    Dme1 = sqrt(Nav_ILS1.SlantDistance * Nav_ILS1.SlantDistance - NavLink_AeroPkt.Pz * NavLink_AeroPkt.Pz);
    if (Nav_ILS1.BeaconStatus && 
        Nav_ILS1.IlsFrequency && 
        (Nav_ILS1.LocaliserError > -0.1) && 
        (Nav_ILS1.LocaliserError < 0.1)) 
    {
        OM1Range         = 9260.0;  /* OM always at 5NM */
        Nav_OuterMarker  = (Dme1 > (OM1Range - 200.0)) && (Dme1 < (OM1Range + 200.0));
        MM1Range         = 1852.0; /* MM always at 1NM */
        Nav_MiddleMarker = (Dme1 > (MM1Range - 200.0)) && (Dme1 < (MM1Range + 200.0));
        //printf("OM=%c MM=%c\n", Nav_OuterMarker ? 'T' : 'F', Nav_MiddleMarker ? 'T' : 'F');
    }

    //if (NavLink_IOPkt.LampsTestButton == IODefn_On) 
    //{
    //    Nav_OuterMarker = true;
    //    Nav_MiddleMarker = true;
    //    Nav_InnerMarker = true;
    //}
}

/* ------------------------------------------------------ */
void BEGIN_Nav()
{
  Nav_NavRecord *w;

    Nav_CurrentRunway = 0;
    Nav_GroundLevel = 0.0;
    Nav_Rmi_Dir1 = 0.0;
    Nav_Rmi_Dir2 = 0.0;
    Nav_HSI_Crs = 0;
    Nav_HSI_Hdg = 0;
    Nav_Track = 0.0;
    Nav_TrackRange = 0;
    Nav_HSI_Localiser = 0.0;
    Nav_HSI_GlideSlope = 0.0;
    Nav_DmeDistance = 0.0;
    Nav_OuterMarker = false;
    Nav_MiddleMarker = false;
    Nav_InnerMarker = false;
    Nav_MarkerTest = false;

    w = &Nav_ILS1;
    w->SelectedBeacon = 0;
    w->LocaliserError = 0.0;
    w->GlideSlopeError = 0.0;
    w->SlantDistance = 0.0;
    w->BearingToStation = 0.0;
    w->BeaconStatus = false;
    w->IlsFrequency = false;
    w->RunwayQdm = 0.0;

    w = &Nav_VOR1;
    w->SelectedBeacon = 0;
    w->LocaliserError = 0.0;
    w->GlideSlopeError = 0.0;
    w->SlantDistance = 0.0;
    w->BearingToStation = 0.0;
    w->BeaconStatus = false;
    w->IlsFrequency = false;
    w->RunwayQdm = 0.0;

    w = &Nav_VOR2;
    w->SelectedBeacon = 0;
    w->LocaliserError = 0.0;
    w->GlideSlopeError = 0.0;
    w->SlantDistance = 0.0;
    w->BearingToStation = 0.0;
    w->BeaconStatus = false;
    w->IlsFrequency = false;
    w->RunwayQdm = 0.0;

    w = &Nav_ADF1;
    w->SelectedBeacon = 0;
    w->LocaliserError = 0.0;
    w->GlideSlopeError = 0.0;
    w->SlantDistance = 0.0;
    w->BearingToStation = 0.0;
    w->BeaconStatus = false;
    w->IlsFrequency = false;
    w->RunwayQdm = 0.0;

    w = &Nav_ADF2;
    w->SelectedBeacon = 0;
    w->LocaliserError = 0.0;
    w->GlideSlopeError = 0.0;
    w->SlantDistance = 0.0;
    w->BearingToStation = 0.0;
    w->BeaconStatus = false;
    w->IlsFrequency = false;
    w->RunwayQdm = 0.0;

    w = &Nav_WayPoint;
    w->SelectedBeacon = 0;
    w->LocaliserError = 0.0;
    w->GlideSlopeError = 0.0;
    w->SlantDistance = 0.0;
    w->BearingToStation = 0.0;
    w->BeaconStatus = false;
    w->IlsFrequency = false;
    w->RunwayQdm = 0.0;

    Nav_MagneticVariation = 0.0;
    Nav_BaroPressure1 = 1013;
    Nav_BaroPressure2 = 1013;
    Nav_RegionalQNH = 1013;
    OldFDOn = false;

    Nav_NumberOfWayPoints = 0;
    Nav_FlightPlan_Segment = 0;
    oldFlightPlan_Segment = 0;

    Nav_MorseChannel = 0;
    Nav_MorseIdent[0] = '\0';
}
