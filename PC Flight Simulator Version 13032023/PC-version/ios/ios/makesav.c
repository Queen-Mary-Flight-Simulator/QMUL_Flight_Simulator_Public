#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <SIM/iosdefn.h>

IosDefn_IosDataPkt IosPkt;

void LoadRestoreFile(char Filename[]);
void PrintRestoreFile(IosDefn_RestoreVectorRecord a);
void ModifyAndWriteRestoreFile(IosDefn_RestoreVectorRecord *a);
float Degrees(float x);
float Radians(float x);
float Feet(float x);
float Metres(float x);

/* ---------------------------------- */
void LoadRestoreFile(char Filename[])
{
    FILE *f;

    f = fopen(Filename, "rb"); 
    if (f == NULL) 
    {
        printf("Error opening file %s\n", Filename);
        exit(1);
    }
    fread(&IosPkt.RestoreVector, sizeof(IosDefn_RestoreVectorRecord), 1, f);
    fclose(f);
}

/* ---------------------------------- */
float Degrees(float x)
{
  return x * (180.0/M_PI);
}

/* ---------------------------------- */
float Radians(float x)
{
  return x * (M_PI/180.0);
}

/* ---------------------------------- */
float Feet(float x)
{
  return x * 3.280840;
}

/* ---------------------------------- */
float Metres(float x)
{
  return x / 3.280840;
}

/* ---------------------------------- */
int main(int argc, char *argv[])
{
  if (argc > 1) 
  {
    LoadRestoreFile(argv[1]);
    PrintRestoreFile(IosPkt.RestoreVector);
	//ModifyAndWriteRestoreFile(&IosPkt.RestoreVector);
  }
  return 0;
}

void ModifyAndWriteRestoreFile(IosDefn_RestoreVectorRecord *a)
{
	a->Latitude = Radians(51.1273);
	a->Longitude = Radians(-0.3356);
    a->Pz = metres(1808.0);
	a->Pitch = Radians(-1.2);
	a->Roll = 0.0;
    a->Yaw = Radians(77.2);
	a->U = 80.0;
	a->V = 0.0;
	a->W = 3.3;
	a->P = 0.0;
	a->Q = 0.0;
	a->R = 0.0;
	a->CurrentRunway = 83
	
	a->FCU_BaroPressure = 1013;
	
	// Write
	FILE *f;
    f = fopen("gat6m.sav", "wb"); 
    if (f == NULL) 
    {
        printf("Error opening file mod.sav\n");
        exit(1);
    }
    fwrite(&IosPkt.RestoreVector, sizeof(IosDefn_RestoreVectorRecord), 1, f);
    fclose(f);
}
