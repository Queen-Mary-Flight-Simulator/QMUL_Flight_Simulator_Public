#include <stdbool.h>

void Sounds(bool Enabled, unsigned int EngineType);

void GearWarning(bool Enabled);

void OuterMarkerIdent(bool Enabled);

void MiddleMarkerIdent(bool Enabled);

bool OuterMarkerSound();

bool MiddleMarkerSound();

void Morse(char Str[], bool ILS);

void StallWarning(bool Enabled);

void FireWarning(bool Enabled);

void StickWarning(bool Enabled);

void ConfigurationWarning(bool Enabled);

void AirConditioning(bool Enabled);

void ElectricalNoise(bool Enabled);

void Slipstream(float Airspeed);

void JetEngine(unsigned int EngineNumber, float N1, bool ReverseThrust);

void PistonEngine(unsigned int EngineNumber, float RPM);

void TurboPropEngine(unsigned int EngineNumber, float RPM);

void GroundRumble(bool Enabled, float Airspeed);

void GearMotor(bool Enabled);

void GearBuffet(float GearPosition, float Airspeed);

void BEGIN_SoundLib(int *argc, char *argv[]);

void END_SoundLib();
