/*
Sound test program
D J Allerton 8 December 2021
Emulates the simulator sounds at 50 FPS
Each sound is activated by a single chacacter command with arguments
use 'make clean' and 'make' to build the program, then
./soundtest following the guide shown
n.b. very little checking of user commands
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include <sys/time.h>
#include <pthread.h>

#include <SIM/engdefn.h>
#include <AL/alut.h>
#include <SIM/soundlib.h>

bool         MiddleMarkerState;
bool         OuterMarkerState;
bool         GearWarningState;
bool         StallWarningState;
bool         FireWarningState;
bool         ConfigWarningState;
bool         AirConditioningState;
bool         ElectricalNoiseState;
bool         GearMotorState;
bool         FreezeState;

float        Airspeed;
float        MinAirspeed;
float        MaxAirspeed;
float        Groundspeed;
unsigned int EngineNo;
float        EngineRpm;
unsigned int EngineType;
float        MinRpm;
float        MaxRpm;
float        dRpm;
unsigned int GearPosition;
bool         MorseMode;
char         MorseString[80];
bool         Stopping = false;

static pthread_t sound_thread;
static pthread_t cmd_thread;

/* prototypes */
void *ReadCommands();
void *UpdateSounds();
void HelpMessage();
char Cap(char Ch);

/* --------------------------------------------- */
int main(int argc, char *argv[])
{
    MiddleMarkerState = false;
    OuterMarkerState = false;
    GearWarningState = false;
    StallWarningState = false;
    FireWarningState = false;
    ConfigWarningState = false;
    AirConditioningState = false;
    ElectricalNoiseState = false;
    GearMotorState = false;
    FreezeState = false;
    EngineType = EngDefn_UnknownEngine;	
	Airspeed = 0.0;
	Groundspeed = 0.0;
	MorseMode = false;

    BEGIN_SoundLib(&argc, argv);
    
    printf("OpenAL Sound System Test Program\n");
    printf("D J Allerton (modified for OpenAL by G T Spence)\n");
    printf("OpenAL version %d.%d\n\n", alutGetMajorVersion(), alutGetMinorVersion());
    HelpMessage();

    if (pthread_create(&sound_thread, NULL, UpdateSounds, NULL))  /* Posix thread for the sound generation loop */
    {
        printf("Unable to create sound thread\n");
        exit(1);
    }

    if (pthread_create(&cmd_thread, NULL, ReadCommands, NULL))  /* Posix thread for user input */
    {
        printf("Unable to create cmd thread\n");
        exit(1);
    }

    pthread_join(sound_thread, NULL);  /* wait for both threads to terminate */
    pthread_join(cmd_thread, NULL);

    pthread_cancel(sound_thread);
    pthread_cancel(cmd_thread);
}

/* --------------------------------------------- */
void *ReadCommands()
{
    while(1)
    {
	    int          cmd;
        char         ch;
        char         str[80];
        unsigned int i;
    
		
        fgets(str, 1000, stdin);
        cmd = Cap(str[0]);
		
		for (i=0; i<=strlen(str); i+=1)
		{
		    str[i] = str[i+1];
		}
		
        switch (cmd)
		{
            case 'M':
                MiddleMarkerState = !MiddleMarkerState;
                break;
       
            case 'O':
                OuterMarkerState = !OuterMarkerState;
                break;
                 
            case 'G':
                GearWarningState = !GearWarningState;
                break;
                
            case 'H':
                FreezeState = !FreezeState;
                break;
                
            case 'S':
                StallWarningState = !StallWarningState;
                break;
                
            case 'F': 
                FireWarningState = !FireWarningState;
                break;
                
            case 'C': 
                ConfigWarningState = !ConfigWarningState;
                break;

            case 'A': 
                AirConditioningState = !AirConditioningState;
                break;
                
            case 'E':
                ElectricalNoiseState = !ElectricalNoiseState;
                break;

            case 'Z': 
			    GearMotorState = true;
                break;
                
            case 'R': 
                sscanf(str, "%c %f", &ch, &Groundspeed);
                break;
                
            case 'W': 
                sscanf(str, "%c %f", &ch, &Airspeed);
                break;
                
            case 'U': 
                sscanf(str, "%c %u %f", &ch, &GearPosition, &Airspeed);
                break;
                
            case 'P': 
                EngineType = EngDefn_Piston;
                sscanf(str, "%c %u %f", &ch, &EngineNo, &EngineRpm);
                MinRpm = 650.0;             
                MaxRpm = 2700.0;
                dRpm = 5.0;
                break;
                
            case 'T': 
                EngineType = EngDefn_Turboprop;
                sscanf(str, "%c %u %f", &ch, &EngineNo, &EngineRpm);
                MinRpm = 650.0;             
                MaxRpm = 2700.0;
                dRpm = 1000.0;
                break;
                
            case 'J': 
                EngineType = EngDefn_Turbofan;
                sscanf(str, "%c %u %f", &ch, &EngineNo, &EngineRpm);
                MinRpm = 10.0;
                MaxRpm = 100.0;
                dRpm = 2.0;
                break;

            case 'I':
                for (i=0; i<=2; i+=1)
                {
                    MorseString[i] = Cap(str[i]);
                }
				MorseString[3] = '\0';
				MorseMode = true;
                break;
 
            case '+':
			    EngineRpm += dRpm;
				if (EngineRpm > MaxRpm)
				{
				    EngineRpm = MaxRpm;
				}
				break;
				
            case '-':
			    EngineRpm -= dRpm;
				if (EngineRpm < MinRpm)
				{
				    EngineRpm = MinRpm;
				}
				break;
				
            case '>':
			    Airspeed += 10.0;
				if (Airspeed > 400.0)
				{
				    Airspeed = 400.0;
				}
				break;
				
            case '<':
			    Airspeed -= 10.0;
				if (Airspeed < 0.0)
				{
				    Airspeed = 0.0;
				}
				break;
				
            case 'Q':
				fflush(stdout);
				Stopping = true;
                break;
			
			case '?':
                HelpMessage();
                break;
            
            default:
                break;
        }
    }
	return NULL;
}

/* --------------------------------------------- */
void *UpdateSounds()
{
    unsigned int Timer1 = 0;
    unsigned int Timer2;
    struct timeval frametime;

	while (1)
	{
	    if (Stopping)
		{
            Sounds(false, EngDefn_UnknownEngine);
            Close_SoundLib();
			exit(0);
		    break;
		}

        Sounds(!FreezeState, EngineType);
        MiddleMarkerIdent(MiddleMarkerState);
        OuterMarkerIdent(OuterMarkerState);
        GearWarning(GearWarningState);
        StallWarning(StallWarningState);
        FireWarning(FireWarningState);
        ConfigurationWarning(ConfigWarningState);
        AirConditioning(AirConditioningState);
        ElectricalNoise(ElectricalNoiseState);
		if (GearMotorState)
		{
            GearMotor(true);
	    	GearMotorState = false;
		}
        GroundRumble(true, Groundspeed);
        Slipstream(Airspeed);
        GearBuffet((float) GearPosition / 100.0, (float) Airspeed);

        if (EngineType == EngDefn_Turbofan)
        {
            JetEngine(EngineNo, EngineRpm, false);
        }
        else if (EngineType == EngDefn_Piston)
        {
            PistonEngine(EngineNo, EngineRpm);
        }
        else if (EngineType == EngDefn_Turboprop)
        {
            TurboPropEngine(EngineNo, EngineRpm);
        }
        
        if (MorseMode)
        {
            Morse(MorseString, false);
			MorseMode = false;
		}
		
        while (1)  /* 50 Hz frame rate sync */
        {
            gettimeofday(&frametime, NULL);
            Timer2 = frametime.tv_usec / 20000L;  /* frame ticks */
         	if (Timer1 != Timer2)
            {
    			Timer1 = Timer2;
                break;
            }
        }

    }
	return NULL;
}

/* --------------------------------------------- */
void HelpMessage()
{
    printf("A  Airconditioning ON/OFF\n");
    printf("C  Configuration warning ON/OFF\n");
    printf("E  Electrical noise ON/OFF\n");
    printf("F  Fire warning ON/OFF\n");
    printf("G  Gear warning ON/OFF\n");
    printf("H  Toggle HOLD ON/OFF\n");
    printf("I  Ident <3-4 chars>\n");
    printf("J  Jet Engine <EngineNo> <Rpm%%>\n");
    printf("M  Middle marker ON/OFF\n");
    printf("O  Outer marker ON/OFF\n");
    printf("P  Piston Engine <EngineNo> <Rpm%%>\n");
    printf("Q  Quit\n");
    printf("R  Rumble <v> kts\n");
    printf("S  Stall warning ON/OFF\n");
    printf("T  Turboprop Engine <EngineNo> <Rpm%%>\n");
    printf("U  Undercarriage <Position%%> <Speed>\n");
    printf("W  Wind <n> kts\n");
    printf("Z  Gear Motor\n");
	printf("?  This guide\n");
    fflush(stdout);
}

/* --------------------------------------------- */
char Cap(char Ch)
{
    return ((Ch >= 'a') && (Ch <= 'z')) ? Ch - 'a' + 'A' : Ch;
}
