#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <SIM/iosdefn.h>

IosDefn_IosDataPkt IosPkt;

void LoadRestoreFile(char Filename[]);
void PrintRestoreFile(IosDefn_RestoreVectorRecord a);
void ModifyAndWriteRestoreFile(char Filename[], IosDefn_RestoreVectorRecord *a);
float Degrees(float x);
float Feet(float x);

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
int main(int argc, char *argv[])
{
  if (argc > 1) 
  {
    LoadRestoreFile(argv[1]);
	ModifyAndWriteRestoreFile(argv[1], &IosPkt.RestoreVector);
    PrintRestoreFile(IosPkt.RestoreVector);
  }
  return 0;
}

/* ---------------------------------- */
void PrintRestoreFile(IosDefn_RestoreVectorRecord a)
{
  unsigned int i;
  
  printf("Lat: %0.4f deg\n", Degrees(a.Latitude));
  printf("Long: %0.4f deg\n", Degrees(a.Longitude));
  printf("Pz: %d ft\n", (unsigned int) Feet(-a.Pz));

  printf("Pitch: %0.1f deg\n", Degrees(a.Pitch));
  printf("Roll: %0.1f deg\n", Degrees(a.Roll));
  printf("Yaw: %0.1f deg\n", Degrees(a.Yaw));

  printf("U: %0.1f Kt\n", a.U * 1.944);
  printf("V: %0.1f m/s\n", a.V);
  printf("W: %0.1f fpm\n", -a.W * 3.280840 * 60.0);

  printf("P: %0.4f deg/s\n", Degrees(a.P));
  printf("Q: %0.4f deg/s\n", Degrees(a.Q));
  printf("R: %0.4f deg/s\n", Degrees(a.R));

  printf("Runway: %d\n", a.CurrentRunway);

  printf("\nFCU_Baro: %d\n", (int) a.FCU_BaroPressure);
  printf("FCU_BaroMode: %s\n", (a.FCU_BaroHg) ? "Hg" : "HPa");
  printf("FCU_BaroSTD: %s\n", (a.FCU_BaroKnob) ? "STD" : "Baro");
  printf("FCU_Metric: %s\n", (a.FCU_Metric_Button) ? "YES" : "NO");
  printf("FCU_ALT: %d\n", (int) a.FCU_ALT);
  printf("FCU_VS: %d\n", (int) a.FCU_VS);
  printf("FCU_HDG: %d\n", (int) a.FCU_HDG);
  printf("FCU_SPD: %d\n", (int) a.FCU_SPD);
  printf("FCU_FD: %s\n", (a.FCU_FD) ? "YES" : "NO");
  printf("FCU_LS: %s\n", (a.FCU_LS) ? "YES" : "NO");
  printf("FCU_LOC: %s\n", (a.FCU_LOC) ? "YES" : "NO");
  printf("FCU_AP1: %s\n", (a.FCU_AP1) ? "YES" : "NO");
  printf("FCU_AP2: %s\n", (a.FCU_AP2) ? "YES" : "NO");
  printf("FCU_ATHR: %s\n", (a.FCU_ATHR) ? "YES" : "NO");
  printf("FCU_EXPED: %s\n", (a.FCU_EXPED) ? "YES" : "NO");
  printf("FCU_APPR: %s\n", (a.FCU_APPR) ? "YES" : "NO");
  printf("FCU_SPD_MACH: %s\n", (a.FCU_SPD_MACH) ? "SPD" : "MACH");
  printf("FCU_HDG_TRK: %s\n", (a.FCU_HDG_TRK) ? "HDG" : "TRK");
  printf("FCU_ALT_HOLD: %s\n", (a.FCU_ALTKnob) ? "YES" : "NO");
  printf("FCU_HDG_HOLD: %s\n", (a.FCU_HDGKnob) ? "YES" : "NO");
  printf("FCU_SPD_HOLD: %s\n", (a.FCU_SPDKnob) ? "YES" : "NO");
  printf("FCU_VS_HOLD: %s\n\n", (a.FCU_VSKnob) ? "YES" : "NO");
  
  for (i=0; i<= 1; i++)
  {
      printf("VOR %d: %d\n", i, a.SavedRadios[i].NavVOR.Active);
      printf("ILS %d: %d\n", i, a.SavedRadios[i].NavILS.Active);
      printf("ADF %d: %d\n", i, a.SavedRadios[i].NavADF.Active);
      printf("ComHF1 %d: %d\n", i, a.SavedRadios[i].ComHF1.Active);
      printf("ComHF2 %d: %d\n", i, a.SavedRadios[i].ComHF2.Active);
      printf("ComVHF1 %d: %d\n", i, a.SavedRadios[i].ComVHF1.Active);
      printf("ComVHF2 %d: %d\n", i, a.SavedRadios[i].ComVHF2.Active);
      printf("ComVHF3 %d: %d\n", i, a.SavedRadios[i].ComVHF3.Active);
      printf("ComAM %d: %d\n", i, a.SavedRadios[i].ComAM.Active);
      printf("CRS Knob %d: %d\n", i, a.SavedRadios[i].CrsKnob);
  }
}

void ModifyAndWriteRestoreFile(char Filename[], IosDefn_RestoreVectorRecord *a)
{
    int i, j;
	
//	for (i=0; i<=1; i+=1)
//	{
//        a->SavedRadios[i].NavVOR.Active  = 11400;
//		a->SavedRadios[i].NavVOR.Stby    = 11790;
//    	a->SavedRadios[i].NavILS.Active  = 11090;
//		a->SavedRadios[i].NavILS.Stby    = 10950;
//		a->SavedRadios[i].NavADF.Active  = 3160;
//		a->SavedRadios[i].NavADF.Stby    = 4255;
//        a->SavedRadios[i].ComVHF1.Active = 11800;
//        a->SavedRadios[i].ComVHF1.Stby   = 11800;
//        a->SavedRadios[i].ComVHF2.Active = 11800;
//        a->SavedRadios[i].ComVHF2.Stby   = 11800;
//        a->SavedRadios[i].ComVHF3.Active = 11800;
//        a->SavedRadios[i].ComVHF3.Stby   = 11800;
//        a->SavedRadios[i].ComHF1.Active  = 0;
//        a->SavedRadios[i].ComHF1.Stby    = 0;
//        a->SavedRadios[i].ComHF2.Active  = 0;
//        a->SavedRadios[i].ComHF2.Stby    = 0;
//        a->SavedRadios[i].ComAM.Active   = 0;
//        a->SavedRadios[i].ComAM.Stby     = 0;
//        a->SavedRadios[i].CrsKnob        = 360;
//        for (j=0; j<=13; j+=1)
//	    {
//	        a->SavedRadios[i].PushSwitches[j] = false;
//	    }
//    }

    a->FCU_HDG_TRK  = false;
	a->FCU_VS = 0.0;
	// Write
	FILE *f;
    f = fopen(Filename, "wb"); 
    if (f == NULL) 
    {
        printf("Error opening file mod.sav\n");
        exit(1);
    }
    fwrite(&IosPkt.RestoreVector, sizeof(IosDefn_RestoreVectorRecord), 1, f);
    fclose(f);
}
