#ifndef EICAS_H
#define EICAS_H

extern void EICAS_EprGauge(int EprX, int EprY, unsigned int EngineNumber);
extern void EICAS_RpmGauge(int RpmX, int RpmY, unsigned int EngineNumber);
extern void EICAS_EgtGauge(int EgtX, int EgtY, unsigned int EngineNumber);
extern void EICAS_DisplayGear(int GearX, int GearY, float GearPosition);
extern void EICAS_FlapsIndicator(int FlapsX, int FlapsY, float FlapPosition, unsigned int FlapSetting);
extern void EICAS_ParkBrake(int ParkBrakeX, int ParkBrakeY, IODefn_SwitchPosition brake);

extern void BEGIN_EICAS();

#endif
