#ifndef Maths_H
#define Maths_H

#define intround(x) ((x) >= 0.0 ? (int)((x)+0.5) : (int)((x)-0.5))

extern float  Maths_Limit(float x, float LowerBound, float UpperBound);
extern float  Maths_Normalise(float x);
extern float  Maths_Rads(float x);
extern float  Maths_Degrees(float x);
extern float  Maths_Metres(float x);
extern float  Maths_Feet(float x);
extern float  Maths_Integrate(float P, float V);
extern double Maths_Double_Integrate(double P, float V);
extern double Maths_Double_Normalise(double a);
extern void BEGIN_Maths();

#endif
