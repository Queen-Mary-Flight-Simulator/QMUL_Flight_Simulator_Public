#include <stdlib.h>
#include <math.h>

#include "model.h"
#include "aero.h"

float Aero_Ixx, Aero_Iyy, Aero_Izz;
float Aero_Ixz, Aero_Ixy, Aero_Iyz;
float Aero_Mass;
float Aero_CgPosition;

static float IxxMoment();
static float IyyMoment();
static float IzzMoment();
static float IxzMoment();


float Aero_AeroMaxAlpha()
{
  return 0.262;
}

float Aero_AeroCz0()
{
  return 0.1 + Model_Flaps * 0.42;
}

float Aero_AeroCz1()
{
  return 5.9;
}

float Aero_AeroCl()
{
  float MaxCl, DeltaAlpha, AbsAlpha, MaxAlpha;
  float Cz0, Cz1;

  AbsAlpha = fabs(Model_Alpha);
  MaxAlpha = Aero_AeroMaxAlpha();
  Cz0 = Aero_AeroCz0();
  Cz1 = Aero_AeroCz1();
  if (AbsAlpha > MaxAlpha) {
    MaxCl = Cz0 + Cz1 * MaxAlpha;
    DeltaAlpha = AbsAlpha - MaxAlpha;
    if (Model_Alpha > 0.0) {
      return MaxCl - DeltaAlpha * 0.917;
    } else {
      return -MaxCl + DeltaAlpha * 0.917;
    }
  } else {
    return Cz0 + Cz1 * Model_Alpha;
  }
}

float Aero_AeroClTail()
{
  return 0.427;
}

float Aero_AeroClfw()
{
  return 0.0;
}

float Aero_AeroCd()
{
  float AbsAlpha;
  float MaxAlpha;
  float MaxCl;
  float Cz0, Cz1;
  float Cl;

  AbsAlpha = fabs(Model_Alpha);
  MaxAlpha = Aero_AeroMaxAlpha();
  Cz0 = Aero_AeroCz0();
  Cz1 = Aero_AeroCz1();
  MaxCl = Cz0 + Cz1 * MaxAlpha;
  Cl = Aero_AeroCl();
  if (AbsAlpha > MaxAlpha) {
    return 0.036 + 0.038 * MaxCl * MaxCl + Model_Flaps * 0.049 + (AbsAlpha - MaxAlpha) * 2.0;
  } else {
    return 0.036 + Model_Flaps * 0.049 + 0.038 * Cl * Cl;
  }
}

float Aero_AeroCyBeta()
{
  return -0.308;
}

float Aero_AeroCyDr()
{
  return 0.187;
}

float Aero_AeroCm0()
{
  float Cm0, AbsAlpha, DeltaAlpha, MaxAlpha;

  MaxAlpha = Aero_AeroMaxAlpha();
  Cm0 = 0.05 - 0.075 * Model_Flaps;
  AbsAlpha = fabs(Model_Alpha);
  if (AbsAlpha > MaxAlpha) {
    DeltaAlpha = AbsAlpha - MaxAlpha;
    return Cm0 - DeltaAlpha * 1.0;
  } else {
    return Cm0;
  }
}

float Aero_AeroCmAlpha()
{
  return -0.885;
}

float Aero_AeroCmDe()
{
  return -1.283;
}

float Aero_AeroCmQ()
{
  return -12.434;
}

float Aero_AeroCmAlphaDot()
{
  return -5.24;
}

float Aero_AeroClBeta()
{
  return -0.089;
}

float Aero_AeroClDr()
{
  return 0.015;
}

float Aero_AeroClDa()
{
  return 0.177;
}

float Aero_AeroClP()
{
  return -0.471;
}

float Aero_AeroClR()
{
  return 0.096;
}

float Aero_AeroCnBeta()
{
  return 0.064;
}

float Aero_AeroCnBetaDot()
{
  return 0.0;
}

float Aero_AeroCnDr()
{
  return -0.066;
}

float Aero_AeroCnDa()
{
  return 0.053;
}

float Aero_AeroCnP()
{
  return -0.029;
}

float Aero_AeroCnR()
{
  return -0.096;
}

static float IxxMoment()
{
  return 1350.0;
}

static float IyyMoment()
{
  return 2700.0;
}

static float IzzMoment()
{
  return 3800.0;
}

static float IxzMoment()
{
  return 0.0;
}

void BEGIN_Aero()
{
    Aero_Mass = 1200.0;
    Aero_Ixx = IxxMoment();
    Aero_Iyy = IyyMoment();
    Aero_Izz = IzzMoment();
    Aero_Ixz = IxzMoment();
    Aero_CgPosition = 0.25;
}
