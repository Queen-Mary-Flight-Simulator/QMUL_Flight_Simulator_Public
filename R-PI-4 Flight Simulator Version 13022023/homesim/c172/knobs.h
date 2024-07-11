#ifndef Knobs_H
#define Knobs_H

extern unsigned int Knobs_BaroPressure;
extern unsigned int Knobs_Obs;
extern unsigned int Knobs_HI;

extern void Knobs_Check(int x, int y, bool leftb, bool middleb, bool rightb);

extern void BEGIN_Knobs();

#endif
