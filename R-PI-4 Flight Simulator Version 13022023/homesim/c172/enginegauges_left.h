#ifndef EngineGauges_H
#define EngineGauges_H

extern void EngineGauges_Ammeter(int x0, int y0, float rpm, int gauge, int needle);
extern void EngineGauges_Suction(int x0, int y0, float rpm, int gauge, int needle);

extern void BEGIN_EngineGauges_left();

#endif
