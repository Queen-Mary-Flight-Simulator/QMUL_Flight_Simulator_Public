#include <math.h>

#include "maths.h"

#define StepLength 0.02

float Maths_arctanXY(float x, float y)
{
  float a, xx, yy;

  xx = fabs(x);
  yy = fabs(y);
  if (xx < 0.001) {
    a = Maths_PIBY2;
  } else {
    a = atan(yy / xx);
  }
  if (x < 0.0) {
    a = Maths_PI - a;
  }
  if (y < 0.0) {
    return -a;
  } else {
    return a;
  }
}

void Maths_Limit(float *x, float LowerBound, float UpperBound)
{
  if (*x < LowerBound) {
    *x = LowerBound;
  } else {
    if (*x > UpperBound) {
      *x = UpperBound;
    }
  }
}

void Maths_Normalise(float *x)
{
  while (*x > Maths_PI) {
    *x = *x - Maths_TWOPI;
  }
  while (*x < -Maths_PI) {
    *x = *x + Maths_TWOPI;
  }
}

float Maths_Rads(float x)
{
  return x / 57.29577951;
}

float Maths_Degrees(float x)
{
  return x * 57.29577951;
}

float Maths_Metres(float x)
{
  return x * 0.30479999;
}

float Maths_Feet(float x)
{
  return x * 3.280840;
}

void Maths_Integrate(float *P, float V)
{
  *P = *P + StepLength * V;
}

void Maths_Double_Integrate(double *P, float V)
{
  *P = *P + (double) (StepLength * V);
}

void BEGIN_Maths()
{
}
