/* +------------------------------+---------------------------------+
   | Module      : inslib.c       | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-05              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : INS library                                      |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/inslib.h>
#include <SIM/maths.h>

#define Re        6378137.0L     /* Earth radius equator WGS-84 (m) */
#define Rp        6356752.3142L  /* Earth radius polar WGS-84 (m) */
#define EarthRate 7.292115E-5L   /* 24 hour rotation rate */

bool InsLib_Active;

const double  StepLength = 0.02; /* 50 Hz update */

static double A11, A12, A13;
static double A21, A22, A23;
static double A31, A32, A33;
static double e0, e1, e2, e3;

static double Pitch, Roll, Yaw;
static double P, Q, R;
static double Ax, Ay, Az;
static double An, Ae, Ad;
static double Vn, Ve, Vd;
static double latitude, longitude, h;
static double g;

void InsLib_Quaternions(double P, double Q, double R);
void SetQuaternions(double Pitch, double Roll, double Yaw);
void SetDCM();
void body2nav(double *n, double *e, double *d, double u, double v, double w);
void nav2body(double *u, double *v, double *w, double n, double e, double d);
double InsLib_Gravity(double lambda, double h);
double Integrate(double P, double V);

/* ------------------------------------------------- */
double InsLib_Gravity(double lambda, double h)
/* WGS-84 values */
{
    double s, s2;
    double g0;
    
    s = sin(lambda);
    s2 = s * s; 
    g0 = 9.78032667714 * ((1.0 + 0.001931851138639 * s2) / sqrt(1.0 - 0.0066943799013 * s2));
    return g0 * (Re * Re) / ((Re + h) * (Re + h));
}
 
/* ------------------------------------------------- */
void nav2body(double *u, double *v, double *w, double n, double e, double d)
/* transform from nav frame to body frame */
{
    *u = n * A11 + e * A21 + d * A31;
    *v = n * A12 + e * A22 + d * A32;
    *w = n * A13 + e * A23 + d * A33;
}
                  
/* ------------------------------------------------- */
void body2nav(double *n, double *e, double *d, double u, double v, double w)
/* transform from body frame to nav frame */{
    *n = u * A11 + v * A12 + w * A13;
    *e = u * A21 + v * A22 + w * A23;
    *d = u * A31 + v * A32 + w * A33;
}

/* ------------------------------------------------- */
double Integrate(double P, double V)
/* 1st order forward Euler integration */
{
    return P + StepLength * V;
}

/* ------------------------------------------------- */
void SetQuaternions(double Pitch, double Roll, double Yaw)
/* initialise quaternions from Euler angles */
{
  double p, r, y;
  double sp, cp, sr, cr, sy, cy;

  p = Pitch * 0.5;
  r = Roll * 0.5;
  y = Yaw * 0.5;
  sp = sin(p);
  cp = cos(p);
  sr = sin(r);
  cr = cos(r);
  sy = sin(y);
  cy = cos(y);
  e0 = cr * cp * cy + sr * sp * sy;
  e1 = sr * cp * cy - cr * sp * sy;
  e2 = cr * sp * cy + sr * cp * sy;
  e3 = cr * cp * sy - sr * sp * cy;
}

/* ------------------------------------------------- */
void InsLib_Quaternions(double P, double Q, double R)
/* update quaternion rates and quaternions */
{
    double emag;
    double e0Dot, e1Dot, e2Dot, e3Dot;
    
    e0Dot = 0.5 * (-e1 * P - e2 * Q - e3 * R);
    e1Dot = 0.5 * ( e0 * P - e3 * Q + e2 * R);
    e2Dot = 0.5 * ( e3 * P + e0 * Q - e1 * R);
    e3Dot = 0.5 * (-e2 * P + e1 * Q + e0 * R );

    e0 = Integrate(e0, e0Dot);
    e1 = Integrate(e1, e1Dot);
    e2 = Integrate(e2, e2Dot);
    e3 = Integrate(e3, e3Dot);

    emag = sqrt(e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3);
    e0 = e0 / emag;
    e1 = e1 / emag;
    e2 = e2 / emag;
    e3 = e3 / emag;
}

/* ------------------------------------------------- */
void SetDCM()
/* set DCM from quaternions */
{
    double e00, e11, e22, e33;

    e00 = e0 * e0;
    e11 = e1 * e1;
    e22 = e2 * e2;
    e33 = e3 * e3;

    A11 = e00 + e11 - e22 - e33;
    A12 = 2.0 * (e1 * e2 - e0 * e3);
    A13 = 2.0 * (e0 * e2 + e1 * e3);
    A21 = 2.0 * (e1 * e2 + e0 * e3);
    A22 = e00 - e11 + e22 - e33;
    A23 = 2.0 * (e2 * e3 - e0 * e1);
    A31 = 2.0 * (e1 * e3 - e0 * e2); 
    A32 = 2.0 * (e2 * e3 + e0 * e1);
    A33 = e00 - e11 - e22 + e33;
}

/* ------------------------------------------------- */
void InsLib_Align(double a_latitude, double a_longitude, double a_altitude, double a_Vn, double a_Ve, double a_Vd, double a_pitch, double a_roll, double a_yaw)
{
    latitude = a_latitude;
	longitude = a_longitude;
	h = a_altitude;
	Vn = a_Vn;
	Ve = a_Ve;
	Vd = a_Vd;
	Pitch = a_pitch;
	Roll = a_roll;
	Yaw = a_yaw;
}

/* ------------------------------------------------- */
void InsLib_Update(double xb, double yb, double zb, double p, double q, double r)
{
    double Wn1, Wn2, Wn3;  /* nav rates */
    double Wb1, Wb2, Wb3;  /* body rates */
    double VnDot, VeDot, VdDot;
    double latitudeDot, longitudeDot, hDot;
    
    g = InsLib_Gravity(latitude, h);

    Pitch = asin(-A31);
    Roll = atan2(A32, A33);
    Yaw = atan2(A21, A11);

    Ax = xb;
    Ay = yb;
    Az = zb;
        
	Wn1 = EarthRate * cos(latitude) + Ve / (Re + h);
	Wn2 = -Vn / (Re + h);
	Wn3 = -EarthRate * sin(latitude) - (Ve * tan(latitude)) / (Re + h);
	nav2body(&Wb1, &Wb2, &Wb3, Wn1, Wn2, Wn3);
        
    P = p + EarthRate * cos(latitude) - Wb1; // + Radians((double) (0.015 / 60.0 / 60.0));
    Q = q - Wb2;
    R = r - EarthRate * sin(latitude) - Wb3;
         
	InsLib_Quaternions(P, Q, R);
	SetDCM();
	
	body2nav(&An, &Ae, &Ad, Ax, Ay, Az);
	
	VnDot = An - 
			2.0 * Ve * EarthRate * sin(latitude) + 
			(Vn * Vd - Ve * Ve * tan(latitude)) / (Re + h);
	VeDot = Ae + 
			2.0 * Vn * EarthRate * sin(latitude) + 
			2.0 * Vd * EarthRate * cos(latitude) +
			(Vn * Ve * tan(latitude) + Ve * Vd) / (Re + h);
	VdDot = Ad - 
			2.0 * Ve * EarthRate * cos(latitude) -
			(Ve * Ve + Vn * Vn) / (Re + h) + g;
	Vn = Integrate(Vn, VnDot);
	Ve = Integrate(Ve, VeDot);
	Vd = Integrate(Vd, VdDot);

	latitudeDot = Vn / (Re + h);
	longitudeDot = Ve / ((Re + h) * cos(latitude));
	hDot = -Vd;
	
	latitude = Integrate(latitude, latitudeDot);
	longitude = Integrate(longitude, longitudeDot);
	h = Integrate(h, hDot);

    printf("lat=%f long=%f h=%f pitch=%f roll=%f yaw=%f\n", Maths_Degrees(latitude), Maths_Degrees(longitude), Maths_Feet(h), Maths_Degrees(Pitch), Maths_Degrees(Roll), Maths_Degrees(Yaw)); // ***
}

/* ------------------------------------------------- */
void InsLib_GetData(double *a_latitude, double *a_longitude, double *a_altitude, double *a_pitch, double *a_roll, double *a_yaw)
{
    *a_latitude = latitude;
	*a_longitude = longitude;
	*a_altitude = h;
	*a_pitch = Pitch;
	*a_roll = Roll;
	*a_yaw = Yaw;
}

/* ------------------------------------------------- */
void BEGIN_InsLib()
{
    InsLib_Active = true;
}
