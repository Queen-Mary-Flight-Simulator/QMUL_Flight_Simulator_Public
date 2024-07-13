#ifndef Maths_H
#define Maths_H

#ifdef __cplusplus
extern "C" {
#endif

#define Maths_PI  3.141592654
#define Maths_ONERAD (180.0 / Maths_PI)
#define Maths_TWOPI  (Maths_PI * 2.0)
#define Maths_PIBY2  (Maths_PI / 2.0)

#define intround(x) ((x) >= 0.0 ? (int)((x)+0.5) : (int)((x)-0.5))

extern float Maths_arctanXY(float x, float y);
extern void Maths_Limit(float *x, float LowerBound, float UpperBound);
extern void Maths_Normalise(float *x);
extern float Maths_Rads(float x);
extern float Maths_Degrees(float x);
extern float Maths_Metres(float x);
extern float Maths_Feet(float x);
extern void Maths_Integrate(float *P, float V);
extern void Maths_Double_Integrate(double *P, float V);
extern void BEGIN_Maths();

#ifdef __cplusplus
}
#endif

#endif
