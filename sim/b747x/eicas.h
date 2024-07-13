#ifndef EICAS_H
#define EICAS_H

#include <SIM/IODefn.h>

extern void EICAS_EprGauge(unsigned int EngineNumber);
extern void EICAS_RpmGauge(unsigned int EngineNumber);
extern void EICAS_EgtGauge(unsigned int EngineNumber);
void EICAS_DisplayGear(float GearPosition);
void EICAS_FlapsIndicator(float FlapPosition, unsigned int FlapSetting);
void EICAS_ParkBrake(IODefn_SwitchPosition brake);

extern void BEGIN_EICAS();

#endif
