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

#include <SIM/engdefn.h>
#include <AL/alut.h>
#include <SIM/soundlib.h>

/* prototypes */
void UpdateSounds();

/* --------------------------------------------- */
int main(int argc, char *argv[])
{
    BEGIN_SoundLib(&argc, argv);
    
    printf("OpenAL Sound System RPM Test Program\n");
    printf("D J Allerton\n");
    printf("OpenAL version %d.%d\n\n", alutGetMajorVersion(), alutGetMinorVersion());
    UpdateSounds();
}

/* --------------------------------------------- */
void UpdateSounds()
{
    unsigned int Timer1 = 0;
    unsigned int Timer2;
    struct timeval frametime;
    float drpm = 0.05;
	float EngineRpm = 0.0;
    unsigned int EngineType = EngDefn_Turbofan;	
    bool FreezeState = false;
	
	while (1)
	{
        Sounds(!FreezeState, EngineType);

        EngineRpm += drpm;
		if (EngineRpm > 100.0)
		{
		    EngineRpm = 100.0;
			drpm = -drpm;
		}
		else if (EngineRpm < 0.0)
		{
		    EngineRpm = 0.0;
			drpm = -drpm;
		}
		
        JetEngine(0, EngineRpm, false);
        JetEngine(1, EngineRpm, false);
        JetEngine(2, EngineRpm, false);
        JetEngine(3, EngineRpm, false);

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
}
