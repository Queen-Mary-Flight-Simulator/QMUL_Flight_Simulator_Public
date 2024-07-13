#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>
#include <SIM/engdefn.h>
#include <SIM/soundlib.h>
#include <AL/alut.h>

/* Sound identifiers */
typedef enum
{
    SNDID_OUTER_MARKER = 0, // Outer marker (400 Hz) LOOPED SAMPLE INC PAUSES
    SNDID_MIDDLE_MARKER,    // Middle marker (1300 Hz) LOOPED SAMPLE INC PAUSES
    SNDID_GEAR_WARNING,     // 250 ms on, 250 ms off (365 Hz) LOOPED SAMPLE INC PAUSES
    SNDID_MORSE_TONE,       // Morse message (750 Hz)
    SNDID_CONFIG_WARNING,   // Combined 250, 500, 1000 Hz
    SNDID_STALL_WARNING,    // Continous tone (245 Hz) LOOPED GENERATED TONE
    SNDID_FIRE_WARNING,     // LOOPED SAMPLE INC PAUSES
    SNDID_STICK_WARNING,    // control stick warning
    SNDID_COCKPIT_AC,       // LOOPED SAMPLE (PREFILTERED)
    SNDID_COCKPIT_ELEC,     // Combined 1200,2000,2800 Hz elec PREMIXED LOOPED SAMPLE 
    SNDID_SLIPSTREAM,       // Aerodynamically generated noise. LOOPED SAMPLE
    SNDID_GEAR_MOTOR,       // Landing gear motors LOOPED SAMPLE
    SNDID_GROUND_RUMBLE,    // Ground rumble 
    SNDID_GEAR_BUFFET,      // Nose wheel buffet 
    SNDID_WHN_0_0,
    SNDID_WHN_0_1,
    SNDID_WHN_0_2,
    SNDID_WHN_0_3,
    SNDID_WHN_0_4,
    SNDID_WHN_0_5,
    SNDID_REVRS_0,
    SNDID_WHN_1_0,
    SNDID_WHN_1_1,
    SNDID_WHN_1_2,
    SNDID_WHN_1_3,
    SNDID_WHN_1_4,
    SNDID_WHN_1_5,
    SNDID_REVRS_1,
    SNDID_WHN_2_0,
    SNDID_WHN_2_1,
    SNDID_WHN_2_2,
    SNDID_WHN_2_3,
    SNDID_WHN_2_4,
    SNDID_WHN_2_5,
    SNDID_REVRS_2,
    SNDID_WHN_3_0,
    SNDID_WHN_3_1,
    SNDID_WHN_3_2,
    SNDID_WHN_3_3,
    SNDID_WHN_3_4,
    SNDID_WHN_3_5,
    SNDID_REVRS_3,
    SNDID_PISTON_ENGINE_0,     // Single propeller piston engine (max 2700 RPM)
    SNDID_PISTON_ENGINE_1,
    SNDID_TURBOPROP_ENGINE_0,  // Turboprop engine (max 2700 RPM)
    SNDID_TURBOPROP_ENGINE_1
} SoundIdentType;

/* Sound source identifiers */
typedef enum
{
    SNDSRC_OUTER_MARKER = 0,
    SNDSRC_MIDDLE_MARKER,
    SNDSRC_GEAR_WARNING,
    SNDSRC_MORSE_TONE,
    SNDSRC_CONFIG_WARNING,
    SNDSRC_STALL_WARNING,
    SNDSRC_FIRE_WARNING,
    SNDSRC_STICK_WARNING,
    SNDSRC_COCKPIT_AC,
    SNDSRC_COCKPIT_ELEC,
    SNDSRC_SLIPSTREAM,
    SNDSRC_GEAR_MOTOR,
    SNDSRC_GROUND_RUMBLE,
    SNDSRC_GEAR_BUFFET,
    SNDSRC_WHN_0_0,
    SNDSRC_WHN_0_1,
    SNDSRC_WHN_0_2,
    SNDSRC_WHN_0_3,
    SNDSRC_WHN_0_4,
    SNDSRC_WHN_0_5,
    SNDSRC_REVRS_0,
    SNDSRC_WHN_1_0,
    SNDSRC_WHN_1_1,
    SNDSRC_WHN_1_2,
    SNDSRC_WHN_1_3,
    SNDSRC_WHN_1_4,
    SNDSRC_WHN_1_5,
    SNDSRC_REVRS_1,
    SNDSRC_WHN_2_0,
    SNDSRC_WHN_2_1,
    SNDSRC_WHN_2_2,
    SNDSRC_WHN_2_3,
    SNDSRC_WHN_2_4,
    SNDSRC_WHN_2_5,
    SNDSRC_REVRS_2,
    SNDSRC_WHN_3_0,
    SNDSRC_WHN_3_1,
    SNDSRC_WHN_3_2,
    SNDSRC_WHN_3_3,
    SNDSRC_WHN_3_4,
    SNDSRC_WHN_3_5,
    SNDSRC_REVRS_3,
    SNDSRC_PISTON_ENGINE_0,
    SNDSRC_PISTON_ENGINE_1,
    SNDSRC_TURBOPROP_ENGINE_0,
    SNDSRC_TURBOPROP_ENGINE_1
} SoundSourceType;

#define MAX_SOUND_BUFFERS   50 /* ram dependent */
#define MAX_SOUND_SOURCES   50 /* sound hw dependent */

#define DOT                  6 /* 120 ms */
#define DASH                18 /* 360 ms */
#define GAP                  6 /* 120 ms */
#define EOM                 30 /* 600 ms */

typedef unsigned int WaveformType[5];

typedef void (*PulseProc)();
  
typedef struct
{
    bool         Enabled;
    bool         Sound;
    unsigned int Pulse;
    unsigned int Timeout;
    PulseProc    OnProc;
    PulseProc    OffProc;
    PulseProc    StopProc;
    WaveformType Waveform;
} PulseRecord;

const WaveformType GearWaveform         = {0, 10, 10, 10, 10};
const WaveformType ConfigWaveform       = {0, 10, 10, 10, 10};
const WaveformType OuterMarkerWaveform  = {0, 18,  6, 18,  6};
const WaveformType MiddleMarkerWaveform = {0, 6,  6, 18,  6};

/* Initial gains for each sound source */
ALuint soundBuffers[MAX_SOUND_BUFFERS];
ALuint soundSources[MAX_SOUND_SOURCES];
float  soundDefaultGains[MAX_SOUND_SOURCES];

bool         OldStallWarning;
bool         OldFireWarning;
bool         OldStickWarning;
bool         OldMiddleMarker;
bool         OldOuterMarker;
bool         OldGearWarning;
bool         OldConfigWarning;
bool         OldAirconditioning;
bool         OldElectricalNoise;
bool         OldGearMotor;
bool         OldSoundOn;
unsigned int OldEngineType;  

PulseRecord  GearWarningRecord;
PulseRecord  ConfigWarningRecord;
PulseRecord  OuterMarkerRecord;
PulseRecord  MiddleMarkerRecord;
    
const unsigned int MorseWaveform [26][9] = 
   {{DOT,  GAP,  DASH, GAP,  EOM,  EOM,  EOM,  EOM, EOM},  /* A */
    {DASH, GAP,  DOT,  GAP,  DOT,  GAP,  DOT,  GAP, EOM},  /* B */
    {DASH, GAP,  DOT,  GAP,  DASH, GAP,  DOT,  GAP, EOM},  /* C */
    {DASH, GAP,  DOT,  GAP,  DOT,  GAP,  EOM,  EOM, EOM},  /* D */
    {DOT,  GAP,  EOM,  EOM,  EOM,  EOM,  EOM,  EOM, EOM},  /* E */
    {DOT,  GAP,  DOT,  GAP,  DASH, GAP,  DOT,  GAP, EOM},  /* F */
    {DASH, GAP,  DASH, GAP,  DOT,  GAP,  EOM,  EOM, EOM},  /* G */
    {DOT,  GAP,  DOT,  GAP,  DOT,  GAP,  DOT,  GAP, EOM},  /* H */
    {DOT,  GAP,  DOT,  GAP,  EOM,  EOM,  EOM,  EOM, EOM},  /* I */
    {DOT,  GAP,  DASH, GAP,  DASH, GAP,  DASH, GAP, EOM},  /* J */
    {DASH, GAP,  DOT,  GAP,  DASH, GAP,  EOM,  EOM, EOM},  /* K */
    {DOT,  GAP,  DASH, GAP,  DOT,  GAP,  DOT,  GAP, EOM},  /* L */
    {DASH, GAP,  DASH, GAP,  EOM,  EOM,  EOM,  EOM, EOM},  /* M */
    {DASH, GAP,  DOT,  GAP,  EOM,  EOM,  EOM,  EOM, EOM},  /* N */
    {DASH, GAP,  DASH, GAP,  DASH, GAP,  EOM,  EOM, EOM},  /* O */
    {DOT,  GAP,  DASH, GAP,  DASH, GAP,  DOT,  GAP, EOM},  /* P */
    {DASH, GAP,  DASH, GAP,  DOT,  GAP,  DASH, GAP, EOM},  /* Q */
    {DOT,  GAP,  DASH, GAP,  DOT,  GAP,  EOM,  EOM, EOM},  /* R */
    {DOT,  GAP,  DOT,  GAP,  DOT,  GAP,  EOM,  EOM, EOM},  /* S */
    {DASH, GAP,  EOM,  EOM,  EOM,  EOM,  EOM,  EOM, EOM},  /* T */
    {DOT,  GAP,  DOT,  GAP,  DASH, GAP,  EOM,  EOM, EOM},  /* U */
    {DOT,  GAP,  DOT,  GAP,  DOT,  GAP,  DASH, GAP, EOM},  /* V */
    {DOT,  GAP,  DASH, GAP,  DASH, GAP,  EOM,  EOM, EOM},  /* W */
    {DASH, GAP,  DOT,  GAP,  DOT,  GAP,  DASH, GAP, EOM},  /* X */
    {DASH, GAP,  DOT,  GAP,  DASH, GAP,  DASH, GAP, EOM},  /* Y */
    {DASH, GAP,  DASH, GAP,  DOT,  GAP,  DOT,  GAP, EOM}}; /* Z */

bool         MorseEnabled;      /* Morse string enabled  TRUE */
char         MorseSymbol;       /* current morse charcater */
unsigned int MorseDigit;        /* current symbol in morse string */
char         MorseString[5];
bool         MorseSound;        /* sound on  TRUE */
unsigned int MorseTimeout;      /* time to go to end of pulse in 1/50 sec */
unsigned int MorsePulse;        /* pulse number of pulse train */

/* prototypes */
int isSourcePlayingAL(ALuint src);
int createSourceWithProperties(ALuint srcIdent,  bool loopSound);
short int* create16bitSineTone(float toneFrequency, float sampleFrequency, float peakAmplitude, unsigned int *pointsPerCycle);
short int* create16bitRectifiedSineTone(float toneFrequency, float sampleFrequency, float peakAmplitude, unsigned int *pointsPerCycle);
short int* create16bitWhiteNoise(float peakAmplitude, unsigned int numberOfSamplePoints);
ALuint loadBufferFromFile(char *filename);
void sourcePlay(ALuint srcIdent);
void sourceStop(ALuint srcIdent);
void sourcePause(ALuint srcIdent);
void sourceAttachSound(ALuint srcIdent, ALuint soundIdent);
void sourceAttachSoundAndPlay(ALuint srcIdent, ALuint soundIdent);
void sourceReplaceSound(ALuint srcIdent, ALuint soundIdent); /* detach old buffer before attaching new buffer */
void sourceReplaceSoundAndPlay(ALuint srcIdent, ALuint soundIdent);
void sourceDetach(ALuint srcIdent);
void sourceStopandDetach(ALuint srcIdent);
float interpolate(unsigned int n, float n1, float x[], float y[]);
void UpdateMorse();
void UpdatePulse(PulseRecord *p);
void SetPulseRecord(PulseRecord *p, PulseProc PrOn, PulseProc PrOff, PulseProc PrStop, const WaveformType w);
void StartPulseRecord(PulseRecord *p, bool On);
void fatal(char *fmt, ...);
int ChannelInitWav(char wavfilename[], SoundSourceType src, SoundIdentType ident, float defaultgain, bool looping);
int ChannelInitRectifiedTone(float frequency, unsigned int SampleRate, SoundSourceType src, SoundIdentType ident, float defaultgain, bool looping);
int SoundInit(int *argc, char *argv[]);

/* functions for sound system interface */
int  GearWarningInit();
void GearWarningOn();
void GearWarningOff();
int  ConfigInit();
void ConfigWarningOn();
void ConfigWarningOff();
int  OuterMarkerInit();
void OuterMarkerOn();
void OuterMarkerOff();
int  MiddleMarkerInit();
void MiddleMarkerOn();
void MiddleMarkerOff();
int  MorseInit();
void MorseOn();
void MorseOff();
int  SlipstreamInit();
int  BuffetInit();
int  AirConditioningInit();
int  ElectricalNoiseInit();
int  StallWarningInit();
int  GearMotorInit();  
int  FireWarningInit();  
int  StickWarningInit();  
int  JetEnginesInit();
int  PistonEnginesInit();
int  TurboPropEnginesInit();

/* --------------------------------------------- */
void fatal(char *fmt, ...)
{
    va_list list;
    char    *p, *r;
    int     e;
    float   f;
    
    va_start(list, fmt);

    for (p=fmt; *p; ++p)
    {
        if (*p != '%')
        {
            printf("%c", *p);
        } 
        else 
        {
            switch (*++p)
            {
                case 's':
                {
                    r = va_arg(list, char *);
                    printf("%s", r);
                    continue;
                }
 
                case 'i':
                {
                    e = va_arg(list, int);
                    printf("%i", e);
                    continue;
                }
 
                case 'd':
                {
                    e = va_arg(list, int);
                    printf("%d", e);
                    continue;
                }
 
                case 'x':
                {
                    e = va_arg(list, int);
                    printf("%x", e);
                    continue;
                }
 
                case 'f':
                {
                    f = va_arg(list, double);
                    printf("%f", f);
                    continue;
                }
 
                default: 
                     printf("%c", *p);
            }
        }
    }
    
    va_end(list);

    exit(1);
}

/* --------------------------------------------- */
int isSourcePlayingAL(ALuint src)
{
    ALenum state;
    
    alGetSourcei(src, AL_SOURCE_STATE, &state);
    return (state == AL_PLAYING);
}

/* --------------------------------------------- */
int createSourceWithProperties(ALuint srcIdent, bool loopSound)
{
    /* If this is already a valid source, return 1 */
    if (alIsSource(soundSources[srcIdent]) == AL_TRUE) 
    {
        printf("createSourceWithProperties() - source exists\n");
        return 1;
    }

    alGenSources(1, &soundSources[srcIdent]);
    if (alGetError() != AL_NO_ERROR) 
    {
        printf("createSourceWithProperties() alGenSources error.\n");
        return 0;
    }
    
    /* Properties - looping */
    alGetError();
    if (loopSound)
    {
        alSourcei(soundSources[srcIdent], AL_LOOPING, AL_TRUE);
    }
    else
    {
        alSourcei(soundSources[srcIdent], AL_LOOPING, AL_FALSE);
    }
    if (alGetError() != AL_NO_ERROR) 
    {
        printf("createSourceWithProperties() alGenSources error.\n");
        return 0;
    }
    
    return 1;
}

/* --------------------------------------------- 
    samples per wave = sample freq / tone freq
    toneFrequency : audible tone frequency
    sampleFrequency : eg 22050, 44100 are common sample frequencies
    peakAmplitude : range 0.0 - 1.0
    pointsPerCycle : number of points to construct a single wave cycle */
    
short int* create16bitSineTone(float toneFrequency, float sampleFrequency, float peakAmplitude, unsigned int *pointsPerCycle)
{
    unsigned int i;
    short int*   buffer;
    unsigned int numberOfSamplePoints = (unsigned int) (sampleFrequency / toneFrequency + 0.5);
    
    if (numberOfSamplePoints & 1)
    {
        numberOfSamplePoints -= 1;
    }

    buffer = malloc(sizeof(short int) * numberOfSamplePoints);
    if (buffer == NULL)
    {
        return NULL;
    }
    
    for (i=0; i<numberOfSamplePoints; i++) 
    {
        buffer[i] = (short int) (sin((2.0f * M_PI * toneFrequency) / sampleFrequency * (float) i) * 32767.0f * peakAmplitude);
    }

    *pointsPerCycle = numberOfSamplePoints;
    /* printf("sine tone : %f points : %d\n", toneFrequency, numberOfSamplePoints); */
    return buffer;
}

/* --------------------------------------------- */
short int* create16bitRectifiedSineTone(float toneFrequency, float sampleFrequency, float peakAmplitude, unsigned int *pointsPerCycle)
{
    unsigned int i;
    short int*   buffer;
    unsigned int numberOfSamplePoints = (unsigned int) (sampleFrequency / toneFrequency + 0.5);

    if (numberOfSamplePoints & 1)
    {
        numberOfSamplePoints -= 1;
    }

    buffer = malloc(sizeof(short int) * numberOfSamplePoints);
    if (buffer == NULL)
    {
        return NULL;
    }
    
    /* Create the sine tone */
    for (i=0; i<numberOfSamplePoints; i++) 
    {
        buffer[i] = (short int) fabs(sin((2.0f * M_PI * toneFrequency) / sampleFrequency * (float) i) * 32768.0f * peakAmplitude);
    }
    
    *pointsPerCycle = numberOfSamplePoints;
    /* printf("rectified sine tone : %f points : %d\n", toneFrequency, numberOfSamplePoints); */
    return buffer;
}

/* --------------------------------------------- */
short int* create16bitWhiteNoise(float peakAmplitude, unsigned int numberOfSamplePoints)
{
    unsigned int i;
    short int*   buffer;
    
    buffer = malloc(sizeof(short int) * numberOfSamplePoints);
    if (buffer == NULL)
    {
        return NULL;
    }
    
    for (i=0; i<numberOfSamplePoints; i++) 
    {
        buffer[i] = (short int) ((float) (rand() - 16384) * peakAmplitude);
    }
    return buffer;
}

/* --------------------------------------------- */
ALuint loadBufferFromFile(char *filename)
{
    ALuint buf = alutCreateBufferFromFile(filename);
    
    return buf; /* AL_NONE (0) on failure */
}

/*
    Convenience functions to wrap-up OpenAL
*/

/* --------------------------------------------- */
void sourcePlay(ALuint srcIdent)
{
    alSourcePlay(soundSources[srcIdent]);
}

/* --------------------------------------------- */
void sourceStop(ALuint srcIdent)
{
    alSourceStop(soundSources[srcIdent]);
}

/* --------------------------------------------- */
int sourceIsPlaying(ALuint srcIdent)
{
    ALint attr = 0;
    
    alGetSourcei(soundSources[srcIdent], AL_SOURCE_STATE, &attr);
    if (attr != AL_PLAYING)
    {
        return 0;
    }
    
    return 1;
}

/* --------------------------------------------- */
ALint sourceCurrentBufferId(ALuint srcIdent)
{
    ALint attr = 0;
    
    alGetSourcei(soundSources[srcIdent], AL_BUFFER, &attr);
    return attr;
}

/* --------------------------------------------- */
void sourceDetach(ALuint srcIdent)
{
    /* According to Creative, the standard way to detach buffer */
    alSourcei(soundSources[srcIdent], AL_BUFFER, 0);
}

/* --------------------------------------------- */
void sourceStopandDetach(ALuint srcIdent)
{
    alSourceStop(soundSources[srcIdent]);
    sourceDetach(srcIdent);
}

/* --------------------------------------------- */
void sourceAttachSound(ALuint srcIdent, ALuint soundIdent)
{
    alSourcei(soundSources[srcIdent], AL_BUFFER, soundBuffers[soundIdent]);
}

/* --------------------------------------------- */
void sourceReplaceSound(ALuint srcIdent, ALuint soundIdent)
{
    sourceStop(srcIdent);
    sourceDetach(srcIdent);
    alSourcei(soundSources[srcIdent], AL_BUFFER, soundBuffers[soundIdent]);
}

/* --------------------------------------------- */

void sourceAttachSoundAndPlay(ALuint srcIdent, ALuint soundIdent)
{
    alSourcei(soundSources[srcIdent], AL_BUFFER, soundBuffers[soundIdent]);
    alSourcePlay(soundSources[srcIdent]);
}

/* --------------------------------------------- */

void sourceReplaceSoundAndPlay(ALuint srcIdent, ALuint soundIdent)
{
    sourceStop(srcIdent);
    sourceDetach(srcIdent);
    alSourcei(soundSources[srcIdent], AL_BUFFER, soundBuffers[soundIdent]);
    alSourcePlay(soundSources[srcIdent]);
}

/* --------------------------------------------- */
int SoundInit(int *argc, char *argv[])
{
    alutInit(argc, argv);
    
    /* Init the sound buffer array */
    if (alGetError() != AL_NO_ERROR)
    {
        return 0;
    }
    
    alGenBuffers(MAX_SOUND_BUFFERS, soundBuffers);
    if (alGetError() != AL_NO_ERROR)
    {
        return 0;
    }
    
    OldEngineType = EngDefn_UnknownEngine;
    OldSoundOn = false;

    return 1;
}

/* ---------------------------------------- */
void sourcePause(ALuint srcIdent)
{
    alSourcePause(soundSources[srcIdent]);
}

/* ---------------------------------------- */    
void Sounds(bool SoundOn, unsigned int EngineType)
{
    if (SoundOn != OldSoundOn) 
    { 
        OldSoundOn = SoundOn;
        if (SoundOn)
        {
            if (EngineType == EngDefn_Turbofan)
            {
                sourcePlay(SNDSRC_WHN_0_0);
                sourcePlay(SNDSRC_WHN_0_1);
                sourcePlay(SNDSRC_WHN_0_2);
                sourcePlay(SNDSRC_WHN_0_3);
                sourcePlay(SNDSRC_WHN_0_4);
                sourcePlay(SNDSRC_WHN_0_5);
                sourcePlay(SNDSRC_REVRS_0);
                sourcePlay(SNDSRC_WHN_1_0);
                sourcePlay(SNDSRC_WHN_1_1);
                sourcePlay(SNDSRC_WHN_1_2);
                sourcePlay(SNDSRC_WHN_1_3);
                sourcePlay(SNDSRC_WHN_1_4);
                sourcePlay(SNDSRC_WHN_1_5);
                sourcePlay(SNDSRC_REVRS_1);
                sourcePlay(SNDSRC_WHN_2_0);
                sourcePlay(SNDSRC_WHN_2_1);
                sourcePlay(SNDSRC_WHN_2_2);
                sourcePlay(SNDSRC_WHN_2_3);
                sourcePlay(SNDSRC_WHN_2_4);
                sourcePlay(SNDSRC_WHN_2_5);
                sourcePlay(SNDSRC_REVRS_2);
                sourcePlay(SNDSRC_WHN_3_0);
                sourcePlay(SNDSRC_WHN_3_1);
                sourcePlay(SNDSRC_WHN_3_2);
                sourcePlay(SNDSRC_WHN_3_3);
                sourcePlay(SNDSRC_WHN_3_4);
                sourcePlay(SNDSRC_WHN_3_5);
                sourcePlay(SNDSRC_REVRS_3);
            }
            else if (EngineType == EngDefn_Piston)
            {
                sourcePlay(SNDSRC_PISTON_ENGINE_0);
                sourcePlay(SNDSRC_PISTON_ENGINE_1);
            }
            else if (EngineType == EngDefn_Turboprop)
            {
                sourcePlay(SNDSRC_TURBOPROP_ENGINE_0);
                sourcePlay(SNDSRC_TURBOPROP_ENGINE_1);
            }

            sourcePlay(SNDSRC_OUTER_MARKER);
            sourcePlay(SNDSRC_MIDDLE_MARKER);
            sourcePlay(SNDSRC_GEAR_WARNING);
            sourcePlay(SNDSRC_MORSE_TONE);
            sourcePlay(SNDSRC_STALL_WARNING);
            sourcePlay(SNDSRC_FIRE_WARNING);
            //sourcePlay(SNDSRC_STICK_WARNING);
            sourcePlay(SNDSRC_COCKPIT_AC);
            sourcePlay(SNDSRC_COCKPIT_ELEC);
            sourcePlay(SNDSRC_CONFIG_WARNING);
            sourcePlay(SNDSRC_SLIPSTREAM);
            //sourcePlay(SNDSRC_GEAR_MOTOR);
            sourcePlay(SNDSRC_GROUND_RUMBLE);
            sourcePlay(SNDSRC_GEAR_BUFFET);
        }
        else 
        {
            sourcePause(SNDSRC_OUTER_MARKER);
            sourcePause(SNDSRC_MIDDLE_MARKER);
            sourcePause(SNDSRC_GEAR_WARNING);
            sourcePause(SNDSRC_MORSE_TONE);
            sourcePause(SNDSRC_STALL_WARNING);
            sourcePause(SNDSRC_FIRE_WARNING);
            //sourcePause(SNDSRC_STICK_WARNING);
            sourcePause(SNDSRC_COCKPIT_AC);
            sourcePause(SNDSRC_COCKPIT_ELEC);
            sourcePause(SNDSRC_CONFIG_WARNING);
            sourcePause(SNDSRC_SLIPSTREAM);
            sourcePause(SNDSRC_GEAR_MOTOR);
            sourcePause(SNDSRC_GROUND_RUMBLE);
            sourcePause(SNDSRC_GEAR_BUFFET);
            sourcePause(SNDSRC_WHN_0_0);
            sourcePause(SNDSRC_WHN_0_1);
            sourcePause(SNDSRC_WHN_0_2);
            sourcePause(SNDSRC_WHN_0_3);
            sourcePause(SNDSRC_WHN_0_4);
            sourcePause(SNDSRC_WHN_0_5);
            sourcePause(SNDSRC_REVRS_0);
            sourcePause(SNDSRC_WHN_1_0);
            sourcePause(SNDSRC_WHN_1_1);
            sourcePause(SNDSRC_WHN_1_2);
            sourcePause(SNDSRC_WHN_1_3);
            sourcePause(SNDSRC_WHN_1_4);
            sourcePause(SNDSRC_WHN_1_5);
            sourcePause(SNDSRC_REVRS_1);
            sourcePause(SNDSRC_WHN_2_0);
            sourcePause(SNDSRC_WHN_2_1);
            sourcePause(SNDSRC_WHN_2_2);
            sourcePause(SNDSRC_WHN_2_3);
            sourcePause(SNDSRC_WHN_2_4);
            sourcePause(SNDSRC_WHN_2_5);
            sourcePause(SNDSRC_REVRS_2);
            sourcePause(SNDSRC_WHN_3_0);
            sourcePause(SNDSRC_WHN_3_1);
            sourcePause(SNDSRC_WHN_3_2);
            sourcePause(SNDSRC_WHN_3_3);
            sourcePause(SNDSRC_WHN_3_4);
            sourcePause(SNDSRC_WHN_3_5);
            sourcePause(SNDSRC_REVRS_3);
            sourcePause(SNDSRC_PISTON_ENGINE_0);
            sourcePause(SNDSRC_PISTON_ENGINE_1);
            sourcePause(SNDSRC_TURBOPROP_ENGINE_0);
            sourcePause(SNDSRC_TURBOPROP_ENGINE_1);
        }
    }
    
    if (SoundOn) 
    {
        UpdatePulse(&GearWarningRecord);
        UpdatePulse(&ConfigWarningRecord);
        UpdatePulse(&OuterMarkerRecord);
        UpdatePulse(&MiddleMarkerRecord);
        UpdateMorse();
        
        if (EngineType != OldEngineType)
        {
            if (EngineType == EngDefn_Turbofan)
            {
                sourcePause(SNDSRC_PISTON_ENGINE_0);
                sourcePause(SNDSRC_PISTON_ENGINE_1);
                sourcePause(SNDSRC_TURBOPROP_ENGINE_0);
                sourcePause(SNDSRC_TURBOPROP_ENGINE_1);
                sourcePlay(SNDSRC_WHN_0_0);
                sourcePlay(SNDSRC_WHN_0_1);
                sourcePlay(SNDSRC_WHN_0_2);
                sourcePlay(SNDSRC_WHN_0_3);
                sourcePlay(SNDSRC_WHN_0_4);
                sourcePlay(SNDSRC_WHN_0_5);
                sourcePlay(SNDSRC_REVRS_0);
                sourcePlay(SNDSRC_WHN_1_0);
                sourcePlay(SNDSRC_WHN_1_1);
                sourcePlay(SNDSRC_WHN_1_2);
                sourcePlay(SNDSRC_WHN_1_3);
                sourcePlay(SNDSRC_WHN_1_4);
                sourcePlay(SNDSRC_WHN_1_5);
                sourcePlay(SNDSRC_REVRS_1);
                sourcePlay(SNDSRC_WHN_2_0);
                sourcePlay(SNDSRC_WHN_2_1);
                sourcePlay(SNDSRC_WHN_2_2);
                sourcePlay(SNDSRC_WHN_2_3);
                sourcePlay(SNDSRC_WHN_2_4);
                sourcePlay(SNDSRC_WHN_2_5);
                sourcePlay(SNDSRC_REVRS_2);
                sourcePlay(SNDSRC_WHN_3_0);
                sourcePlay(SNDSRC_WHN_3_1);
                sourcePlay(SNDSRC_WHN_3_2);
                sourcePlay(SNDSRC_WHN_3_3);
                sourcePlay(SNDSRC_WHN_3_4);
                sourcePlay(SNDSRC_WHN_3_5);
                sourcePlay(SNDSRC_REVRS_3);
            }
            else if (EngineType == EngDefn_Piston)
            {
                sourcePause(SNDSRC_WHN_0_0);
                sourcePause(SNDSRC_WHN_0_1);
                sourcePause(SNDSRC_WHN_0_2);
                sourcePause(SNDSRC_WHN_0_3);
                sourcePause(SNDSRC_WHN_0_4);
                sourcePause(SNDSRC_WHN_0_5);
                sourcePause(SNDSRC_REVRS_0);
                sourcePause(SNDSRC_WHN_1_0);
                sourcePause(SNDSRC_WHN_1_1);
                sourcePause(SNDSRC_WHN_1_2);
                sourcePause(SNDSRC_WHN_1_3);
                sourcePause(SNDSRC_WHN_1_4);
                sourcePause(SNDSRC_WHN_1_5);
                sourcePause(SNDSRC_REVRS_1);
                sourcePause(SNDSRC_WHN_2_0);
                sourcePause(SNDSRC_WHN_2_1);
                sourcePause(SNDSRC_WHN_2_2);
                sourcePause(SNDSRC_WHN_2_3);
                sourcePause(SNDSRC_WHN_2_4);
                sourcePause(SNDSRC_WHN_2_5);
                sourcePause(SNDSRC_REVRS_2);
                sourcePause(SNDSRC_WHN_3_0);
                sourcePause(SNDSRC_WHN_3_1);
                sourcePause(SNDSRC_WHN_3_2);
                sourcePause(SNDSRC_WHN_3_3);
                sourcePause(SNDSRC_WHN_3_4);
                sourcePause(SNDSRC_WHN_3_5);
                sourcePause(SNDSRC_REVRS_3);
                sourcePause(SNDSRC_TURBOPROP_ENGINE_0);
                sourcePause(SNDSRC_TURBOPROP_ENGINE_1);
                sourcePlay(SNDSRC_PISTON_ENGINE_0);
                sourcePlay(SNDSRC_PISTON_ENGINE_1);
            }
            else if (EngineType == EngDefn_Turboprop)
            {
                sourcePause(SNDSRC_PISTON_ENGINE_0);
                sourcePause(SNDSRC_PISTON_ENGINE_1);
                sourcePause(SNDSRC_WHN_0_0);
                sourcePause(SNDSRC_WHN_0_1);
                sourcePause(SNDSRC_WHN_0_2);
                sourcePause(SNDSRC_WHN_0_3);
                sourcePause(SNDSRC_WHN_0_4);
                sourcePause(SNDSRC_WHN_0_5);
                sourcePause(SNDSRC_REVRS_0);
                sourcePause(SNDSRC_WHN_1_0);
                sourcePause(SNDSRC_WHN_1_1);
                sourcePause(SNDSRC_WHN_1_2);
                sourcePause(SNDSRC_WHN_1_3);
                sourcePause(SNDSRC_WHN_1_4);
                sourcePause(SNDSRC_WHN_1_5);
                sourcePause(SNDSRC_REVRS_1);
                sourcePause(SNDSRC_WHN_2_0);
                sourcePause(SNDSRC_WHN_2_1);
                sourcePause(SNDSRC_WHN_2_2);
                sourcePause(SNDSRC_WHN_2_3);
                sourcePause(SNDSRC_WHN_2_4);
                sourcePause(SNDSRC_WHN_2_5);
                sourcePause(SNDSRC_REVRS_2);
                sourcePause(SNDSRC_WHN_3_0);
                sourcePause(SNDSRC_WHN_3_1);
                sourcePause(SNDSRC_WHN_3_2);
                sourcePause(SNDSRC_WHN_3_3);
                sourcePause(SNDSRC_WHN_3_4);
                sourcePause(SNDSRC_WHN_3_5);
                sourcePause(SNDSRC_REVRS_3);
                sourcePlay(SNDSRC_TURBOPROP_ENGINE_0);
                sourcePlay(SNDSRC_TURBOPROP_ENGINE_1);
            }
   		    OldEngineType = EngineType;
        }
    }
}

/* ---------------------------------------- */    
void UpdateMorse()
{
    unsigned int cpos;

    if (MorseEnabled)
    {
        if (MorseTimeout > 0)
        {
            MorseTimeout -= 1;
        }
        else 
        {
            cpos = (unsigned int) (MorseSymbol - 'A');  /* A-Z -> 0-25 */
            if (MorseWaveform[cpos][MorsePulse] == EOM)
            {
                MorseDigit += 1;
                if ((MorseString[MorseDigit] == '\0') || (MorseDigit > 3))
                {
                    MorseOff();
                    MorseEnabled = false;
                }
                else
                {
                    MorseSymbol = MorseString[MorseDigit];
                    cpos = (unsigned int) (MorseSymbol - 'A');  /* A-Z -> 0-25 */
                    MorsePulse = 0;
                    MorseTimeout = MorseWaveform[cpos][0];
                    MorseSound = true;
                    MorseOn();
                }
            }
            else
            {
                MorsePulse += 1;
                MorseTimeout = MorseWaveform[cpos][MorsePulse];
                if (MorseTimeout == EOM)
                {
                    MorseSound = false;  /* inter gap off */
                }
                else
                {
                    MorseSound = !MorseSound;
                }
                if (MorseSound)
                {
                    MorseOn();
                }
                else
                {
                     MorseOff();
                }
            }
        }
    }
}

/* ---------------------------------------- */    
void UpdatePulse(PulseRecord *p)
{
    if (p->Enabled)
    {
        if (p->Timeout > 0)
        {
            p->Timeout -= 1;
        }
        else
        {
            p->Pulse += 1;
            if (p->Pulse > 4)
            {
                p->Pulse = 1;
            }
            p->Timeout = p->Waveform[p->Pulse];
            p->Sound = !p->Sound;
            if (p->Sound)
            {
                (p->OnProc)();
            }
            else
            {
                (p->OffProc)();
            }
        }    
    }
}

/* ---------------------------------------- */    
void SetPulseRecord(PulseRecord *p, PulseProc PrOn, PulseProc PrOff, PulseProc PrStop, const WaveformType w)
{
    unsigned int i;
  
    p->Enabled = false;
    p->OnProc = PrOn;
    p->OffProc = PrOff;
    p->StopProc = PrStop;
    for (i=1; i<=4; i+=1)
    {
        p->Waveform[i] = w[i];
    }
}
 
/* ---------------------------------------- */    
void StartPulseRecord(PulseRecord *p, bool On)
{
    if (On == p->Enabled)
    {
        return;
    }
    
    if (On)
    {
        p->Sound = false;
        p->Pulse = 4;
        p->Timeout = 0;
    }
    else
    {
        (p->StopProc)();
    }
    
    p->Enabled = On;
}

/* ---------------------------------------- */
int GearWarningInit()
{
    int r = ChannelInitWav("../wav/gear_warning.wav", SNDSRC_GEAR_WARNING, SNDID_GEAR_WARNING, 1.0, true);
    
    SetPulseRecord(&GearWarningRecord, GearWarningOn, GearWarningOff, GearWarningOff, GearWaveform);
    return r;
}

/* ---------------------------------------- */    
void GearWarning(bool Enabled)
{
    if (OldGearWarning != Enabled)
    {
        StartPulseRecord(&GearWarningRecord, Enabled);
        OldGearWarning = Enabled;
    }
}

/* ---------------------------------------- */    
void GearWarningOn()
{
    alSourcef(soundSources[SNDSRC_GEAR_WARNING], AL_GAIN, soundDefaultGains[SNDSRC_GEAR_WARNING]);
}

/* ---------------------------------------- */    
void GearWarningOff()
{
    alSourcef(soundSources[SNDSRC_GEAR_WARNING], AL_GAIN, 0.0);
}

/* ---------------------------------------- */
int ConfigInit()
{
    int r = ChannelInitWav("../wav/config_warning.wav", SNDSRC_CONFIG_WARNING, SNDID_CONFIG_WARNING, 0.5, true);
    
    SetPulseRecord(&ConfigWarningRecord, ConfigWarningOn, ConfigWarningOff, ConfigWarningOff, ConfigWaveform);
    return r;
}

/* ---------------------------------------- */    
void ConfigurationWarning(bool Enabled)
{
    if (OldConfigWarning != Enabled)
    {
        StartPulseRecord(&ConfigWarningRecord, Enabled);
        OldConfigWarning = Enabled;
    }
}

/* ---------------------------------------- */    
void ConfigWarningOn()
{
    alSourcef(soundSources[SNDSRC_CONFIG_WARNING], AL_GAIN, soundDefaultGains[SNDSRC_CONFIG_WARNING]);
}

/* ---------------------------------------- */    
void ConfigWarningOff()
{
    alSourcef(soundSources[SNDSRC_CONFIG_WARNING], AL_GAIN, 0.0);
}

/* ---------------------------------------- */
int OuterMarkerInit()
{
    int r = ChannelInitWav("../wav/w400.wav", SNDSRC_OUTER_MARKER, SNDID_OUTER_MARKER, 0.2, true);
    
    SetPulseRecord(&OuterMarkerRecord, OuterMarkerOn, OuterMarkerOff, OuterMarkerOff, OuterMarkerWaveform);
    return r;
}

/* ---------------------------------------- */    
void OuterMarkerIdent(bool Enabled)
{
    if (OldOuterMarker != Enabled)
    {
        StartPulseRecord(&OuterMarkerRecord, Enabled);
        OldOuterMarker = Enabled;
    }
}

/* ---------------------------------------- */    
void OuterMarkerOn()
{
    alSourcef(soundSources[SNDSRC_OUTER_MARKER], AL_GAIN, soundDefaultGains[SNDSRC_OUTER_MARKER]);
}

/* ---------------------------------------- */    
void OuterMarkerOff()
{
    alSourcef(soundSources[SNDSRC_OUTER_MARKER], AL_GAIN, 0.0);
}

/* ---------------------------------------- */    
int MiddleMarkerInit()
{
    int r = ChannelInitWav("../wav/w1300.wav", SNDSRC_MIDDLE_MARKER, SNDID_MIDDLE_MARKER, 0.2, true);
    
    SetPulseRecord(&MiddleMarkerRecord, MiddleMarkerOn, MiddleMarkerOff, MiddleMarkerOff, MiddleMarkerWaveform);
    return r;
}

/* ---------------------------------------- */
void MiddleMarkerIdent(bool Enabled)
{
    if (OldMiddleMarker != Enabled)
    {
        StartPulseRecord(&MiddleMarkerRecord, Enabled);
        OldMiddleMarker = Enabled;
    }
}

/* ---------------------------------------- */    
void MiddleMarkerOn()
{
    alSourcef(soundSources[SNDSRC_MIDDLE_MARKER], AL_GAIN, soundDefaultGains[SNDSRC_MIDDLE_MARKER]);
}

/* ---------------------------------------- */    
void MiddleMarkerOff()
{
    alSourcef(soundSources[SNDSRC_MIDDLE_MARKER], AL_GAIN, 0.0);
}

/* ---------------------------------------- */
int MorseInit()
{
    int r = ChannelInitWav("../wav/w750.wav", SNDSRC_MORSE_TONE, SNDID_MORSE_TONE, 0.1, true);
    
    MorseEnabled = false;
    return r;
}

/* ---------------------------------------- */    
void Morse(char Str[], bool ILS)
{
    unsigned int i;
    unsigned int p;
    
    if (ILS)
    {
        MorseString[0] = 'I';
        p = 1;
    }
    else
    {
        p = 0;
    }
    
    for (i=0; i<=3; i+=1)    /* max 4 digits, with trailing spaces */
    {
        char ch = Str[i];
        if (ch < 'A' || ch > 'Z')
        {
            MorseString[p] = '\0';
            break;
        }
        MorseString[p] = Str[i];
        p += 1;
    }  

    MorseSymbol = MorseString[0];
    MorseDigit = 0;
    MorsePulse = 0;
    MorseTimeout = MorseWaveform[(unsigned int) (MorseSymbol - 'A')][0];  /* A-Z -> 0-25 */
    MorseSound = true;
    MorseOn();
    MorseEnabled = true;
}

/* ---------------------------------------- */    
void MorseOn()
{
    alSourcef(soundSources[SNDSRC_MORSE_TONE], AL_GAIN, soundDefaultGains[SNDSRC_MORSE_TONE]);
}

/* ---------------------------------------- */    
void MorseOff()
{
    alSourcef(soundSources[SNDSRC_MORSE_TONE], AL_GAIN, 0.0);
}

/* ---------------------------------------- */
int ChannelInitWav(char wavfilename[], SoundSourceType src, SoundIdentType ident, float defaultgain, bool looping)
{
    ALuint buf;
    
    buf = loadBufferFromFile(wavfilename);
    if (buf != AL_NONE)
    {
        soundBuffers[ident] = buf;
    }
    else
    {
        printf("ChannelInitWav: Unable to load wav file %s\n", wavfilename);
        return 0;
    }
    
    /* create a source */
    if (!createSourceWithProperties(src, looping))
    {
        printf("ChannelInitWav: CreateSourceWithProperties failure %d\n", src);
        return 0;
    }
    
    /* Attach buffer to source */
    sourceAttachSound(src, ident);
    if (alGetError() != AL_NO_ERROR)
    {
        printf("ChannelInitWav: sourceAttachSound failure %d %d\n", src, ident);
        return 0;
    }
    
    /* Modify gain to init level */
    soundDefaultGains[src] = defaultgain;
    sourcePlay(src);
    if (alGetError() != AL_NO_ERROR)
    {
        printf("ChannelInitWav: sourcePlay failure %d\n", src);
        return 0;
    }
    alSourcef(soundSources[src], AL_GAIN, 0.0);  /* turn off for now */
    
    return 1;
}

/* ---------------------------------------- */
int StallWarningInit()
{
    return ChannelInitWav("../wav/stall_warning.wav", SNDSRC_STALL_WARNING, SNDID_STALL_WARNING, 0.8, true);
}

/* ---------------------------------------- */
void StallWarning(bool Enabled)
{
    if (Enabled != OldStallWarning)
    {
        if (Enabled) 
        {
            alSourcef(soundSources[SNDSRC_STALL_WARNING], AL_GAIN, soundDefaultGains[SNDSRC_STALL_WARNING]);
        }
        else 
        {
            alSourcef(soundSources[SNDSRC_STALL_WARNING], AL_GAIN, 0.0);
        }
        OldStallWarning = Enabled;
    }
}

/* ---------------------------------------- */
int StickWarningInit()
{
    return ChannelInitWav("../wav/stick_warning.wav", SNDSRC_STICK_WARNING, SNDID_STICK_WARNING, 0.1, true);
}

/* ---------------------------------------- */
void StickWarning(bool Enabled)
{
    if (OldStickWarning != Enabled)
    {
        if (Enabled) 
        {
            alSourcef(soundSources[SNDSRC_STICK_WARNING], AL_GAIN, soundDefaultGains[SNDSRC_STICK_WARNING]);
        }
        else 
        {
            alSourcef(soundSources[SNDSRC_STICK_WARNING], AL_GAIN, 0.0);
        }
        OldStickWarning = Enabled;
    }
}

/* ---------------------------------------- */
int SlipstreamInit()
{
    return ChannelInitWav("../wav/aerodynamic_noise.wav", SNDSRC_SLIPSTREAM, SNDID_SLIPSTREAM, 0.8, true);
}

/* ---------------------------------------- */
void Slipstream(float v)    /* v = airspeed Kts */  
{
    float aero_noise_gain_x[7] = { 0.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0 };
    float aero_noise_gain_y[7] = { 0.0,   0.1,   0.15,  0.2,   0.3,   0.5,   0.8 };

    float aero_noise_frequency_x[7] = {   0.0,  100.0,  150.0,  200.0,  250.0,  300.0,  350.0 };
    float aero_noise_frequency_y[7] = { 819.0, 1147.0, 1475.0, 1802.0, 2130.0, 2458.0, 2786.0 };

    /* Aerodynamic noise variation with V
       Base frequency is 1475 Hz @ 150 Kts */
    float gain = 0.1;
    float pitchFactor = 1.0;
    
    if (v > 350.0)
    {
        v = 350.0;
    }
    gain = interpolate(sizeof(aero_noise_gain_x), v, aero_noise_gain_x, aero_noise_gain_y);
    pitchFactor = interpolate(sizeof(aero_noise_frequency_x), v, aero_noise_frequency_x, aero_noise_frequency_y) / 1475.0;
    
    alSourcef(soundSources[SNDSRC_SLIPSTREAM], AL_GAIN, soundDefaultGains[SNDSRC_SLIPSTREAM] * gain);
    alSourcef(soundSources[SNDSRC_SLIPSTREAM], AL_PITCH, pitchFactor);
}

/* ---------------------------------------- */
int FireWarningInit()
{
    return ChannelInitWav("../wav/fire_warning.wav", SNDSRC_FIRE_WARNING, SNDID_FIRE_WARNING, 0.8, true);
}

/* ---------------------------------------- */
void FireWarning(bool Enabled)
{
    if (OldFireWarning != Enabled)
    {
        if (Enabled) 
        {
            alSourcef(soundSources[SNDSRC_FIRE_WARNING], AL_GAIN, soundDefaultGains[SNDSRC_FIRE_WARNING]);
        }
        else 
        {
            alSourcef(soundSources[SNDSRC_FIRE_WARNING], AL_GAIN, 0.0);
        }
        OldFireWarning = Enabled;
    }
}

/* ---------------------------------------- */
int AirConditioningInit()
{
    return ChannelInitWav("../wav/airconditioning.wav", SNDSRC_COCKPIT_AC, SNDID_COCKPIT_AC, 0.5, true);
}

/* ---------------------------------------- */
void AirConditioning(bool Enabled)
{
    if (OldAirconditioning != Enabled)
    {
        if (Enabled) 
        {
            alSourcef(soundSources[SNDSRC_COCKPIT_AC], AL_GAIN, soundDefaultGains[SNDSRC_COCKPIT_AC]);
        }
        else 
        {
            alSourcef(soundSources[SNDSRC_COCKPIT_AC], AL_GAIN, 0.0);
        }
        OldAirconditioning = Enabled;
    }
}

/* ---------------------------------------- */
int ElectricalNoiseInit()
{
    return ChannelInitWav("../wav/electrical_noise.wav", SNDSRC_COCKPIT_ELEC, SNDID_COCKPIT_ELEC, 0.02, true);
}

/* ---------------------------------------- */
void ElectricalNoise(bool Enabled)
{
    if (OldElectricalNoise != Enabled)
    {
        if (Enabled) 
        {
            alSourcef(soundSources[SNDSRC_COCKPIT_ELEC], AL_GAIN, soundDefaultGains[SNDSRC_COCKPIT_ELEC]);
        }
        else 
        {
            alSourcef(soundSources[SNDSRC_COCKPIT_ELEC], AL_GAIN, 0.0);
        }
        OldElectricalNoise = Enabled;
    }
}

/* ---------------------------------------- */
int GearMotorInit()
{
    return ChannelInitWav("../wav/gear_motor.wav", SNDSRC_GEAR_MOTOR, SNDID_GEAR_MOTOR, 1.0, false);
}

/* ---------------------------------------- */
void GearMotor(bool Enabled)
{
    if (Enabled) 
    {
        sourcePlay(SNDSRC_GEAR_MOTOR);
        alSourcef(soundSources[SNDSRC_GEAR_MOTOR], AL_GAIN, soundDefaultGains[SNDSRC_GEAR_MOTOR]);
    }
    else 
    {
        alSourcef(soundSources[SNDSRC_GEAR_MOTOR], AL_GAIN, 0.0);
    }
}

/* ---------------------------------------- */
int GroundRumbleInit()
{
    return ChannelInitWav("../wav/rumble.wav", SNDSRC_GROUND_RUMBLE, SNDID_GROUND_RUMBLE, 1.0, true);
}

/* ---------------------------------------- */
void GroundRumble(bool Enabled, float Airspeed)  /* airspeed in Kts */
{
    if (Airspeed > 100.0)
    {
        Airspeed = 100.0;
    }
    
    if (Enabled) 
    {
        alSourcef(soundSources[SNDSRC_GROUND_RUMBLE], AL_GAIN, soundDefaultGains[SNDSRC_GROUND_RUMBLE] * Airspeed / 100.0);
    }
    else 
    {
        alSourcef(soundSources[SNDSRC_GROUND_RUMBLE], AL_GAIN, 0.0);
    }
}

/* ---------------------------------------- */
int GearBuffetInit()
{
    return ChannelInitWav("../wav/gear_buffet.wav", SNDSRC_GEAR_BUFFET, SNDID_GEAR_BUFFET, 0.8, true);
}

/* ---------------------------------------- */
void GearBuffet(float GearPosition, float v)
{
    float buffet_gain_x[7] = { 0.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0 };
    float buffet_gain_y[7] = { 0.0,   0.07,  0.13,  0.25,  0.5,   1.0,   1.0 };

    float buffet_frequency_x[7] = {   0.0,  100.0,  150.0,  200.0,  250.0,  300.0,  350.0 };
    float buffet_frequency_y[7] = { 245.0,  262.0,  278.0,  295.0,  311.0,  328.0,  344.0 };

    /* Aerodynamic noise variation with V
       Base frequency is 1475 Hz @ 150 Kts */
    float gain = 0.1;
    float pitchFactor = 1.0;
    
    if (v > 350.0)
    {
        v = 350.0;
    }
    gain = interpolate(sizeof(buffet_gain_x), v, buffet_gain_x, buffet_gain_y);
    pitchFactor = interpolate(sizeof(buffet_frequency_x), v, buffet_frequency_x, buffet_frequency_y);
    
    alSourcef(soundSources[SNDSRC_GEAR_BUFFET], AL_GAIN, soundDefaultGains[SNDSRC_GEAR_BUFFET] * gain * GearPosition);
    alSourcef(soundSources[SNDSRC_GEAR_BUFFET], AL_PITCH, pitchFactor / 344.0);
}

/* ---------------------------------------- */    
int PistonEnginesInit()
{
    int c0 = ChannelInitWav("../wav/c172.wav", SNDSRC_PISTON_ENGINE_0, SNDID_PISTON_ENGINE_0, 0.0, true);
    int c1 = ChannelInitWav("../wav/c172.wav", SNDSRC_PISTON_ENGINE_1, SNDID_PISTON_ENGINE_1, 0.0, true);

	if (c0 == 0 || c1 == 0)
	{
	    return 0;
	}
	else
	{
	    return c1;
	}
}

/* ---------------------------------------- */    
void PistonEngine(unsigned int EngineNumber, float RPM)
/*                             1-2                 0-2700   */
{
    if (EngineNumber == 0)
	{
        alSourcef(soundSources[SNDSRC_PISTON_ENGINE_0], AL_GAIN, 1.0);
        alSourcef(soundSources[SNDSRC_PISTON_ENGINE_0], AL_PITCH, RPM / 2000.0);  /* wav file based on 2000 RPM (?)*/
	}
	else if (EngineNumber == 1)
	{
        alSourcef(soundSources[SNDSRC_PISTON_ENGINE_1], AL_GAIN, 1.0);
        alSourcef(soundSources[SNDSRC_PISTON_ENGINE_1], AL_PITCH, RPM / 2000.0);  /* wav file based on 2000 RPM (?)*/
	}
}	

/* ---------------------------------------- */    
int TurboPropEnginesInit()
{
    int c0 = ChannelInitWav("../wav/turboprop.wav", SNDSRC_TURBOPROP_ENGINE_0, SNDID_TURBOPROP_ENGINE_0, 0.0, true);
    int c1 = ChannelInitWav("../wav/turboprop.wav", SNDSRC_TURBOPROP_ENGINE_1, SNDID_TURBOPROP_ENGINE_1, 0.0, true);

	if (c0 == 0 || c1 == 0)
	{
	    return 0;
	}
	else
	{
	    return c1;
	}
}

/* ---------------------------------------- */    
void TurboPropEngine(unsigned int EngineNumber, float RPM)
/*                                1-2                 0-2700  */
{
    if (EngineNumber == 0)
	{
        alSourcef(soundSources[SNDSRC_TURBOPROP_ENGINE_0], AL_GAIN, 1.0);
        alSourcef(soundSources[SNDSRC_TURBOPROP_ENGINE_0], AL_PITCH, RPM / 2200.0);  /* wav file based on 2200 RPM */
	}
	else
	{
        alSourcef(soundSources[SNDSRC_TURBOPROP_ENGINE_1], AL_GAIN, 1.0);
        alSourcef(soundSources[SNDSRC_TURBOPROP_ENGINE_1], AL_PITCH, RPM / 2200.0);  /* wav file based on 2200 RPM */
	}
}

/* --------------------------------------------------------------- */
float interpolate(unsigned int n, float v, float x[], float y[])
{
    int   x1;
    int   x2;
    float r = 0.0;
    
    n = n / sizeof(float);  /* bytes -> floats */
    x1 = n - 2;
    x2 = n - 1;
    
    while (x1 >= 0)
    {
        if (v >= x[x1])
        {
            r = y[x1] + (y[x2] - y[x1]) * (v - x[x1]) / (x[x2] - x[x1]);
            break;
        }
        x1 -= 1;
        x2 -= 1;
    }
    if (r < 0.0)
    {
        return 0.0;
    }
    else
    {
        return r;
    }
}

/* --------------------------------------------- */
int ChannelInitRectifiedTone(float frequency, unsigned int SampleRate, SoundSourceType src, SoundIdentType ident, float defaultgain, bool looping)
{
    short int*   pData = NULL;
    unsigned int numberOfSamplePoints = 0;

    pData = create16bitRectifiedSineTone(frequency, (float) SampleRate, 1.0f, &numberOfSamplePoints);
    if (pData != NULL)
    {
        alBufferData(soundBuffers[ident], AL_FORMAT_MONO16, pData, numberOfSamplePoints, SampleRate); 
        if (pData != NULL)
        {
            free(pData);
        }
        if (alGetError() != AL_NO_ERROR)
        {
            return 0;
        }
    }
    else 
    {
        return 0;
    }
    
    /* create sources */
    if (!createSourceWithProperties(src, looping))
    {
        return 0;
    }
    
    /* Attach buffers to sources */
    sourceAttachSound(src, ident);
    if (alGetError() != AL_NO_ERROR)
    {
        return 0;
    }
    
    /* Modify gains to init level */
    soundDefaultGains[src] = defaultgain;

    alSourcef(soundSources[src], AL_GAIN, 0.0);
    if (alGetError() != AL_NO_ERROR)
    {
        return 0;
    }

    sourcePlay(src);
    
    return 1;
}

/* --------------------------------------------- */
int JetEnginesInit()
{
    if (ChannelInitWav("../wav/A_WHN_0.wav", SNDSRC_WHN_0_0, SNDID_WHN_0_0, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 1 whine 0\n");
    }
    if (ChannelInitWav("../wav/A_WHN_1.wav", SNDSRC_WHN_0_1, SNDID_WHN_0_1, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 1 whine 1\n");
    }
    if (ChannelInitWav("../wav/A_WHN_2.wav", SNDSRC_WHN_0_2, SNDID_WHN_0_2, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 1 whine 2\n");
    }
    if (ChannelInitWav("../wav/A_WHN_3.wav", SNDSRC_WHN_0_3, SNDID_WHN_0_3, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 1 whine 3\n");
    }
    if (ChannelInitWav("../wav/A_WHN_4.wav", SNDSRC_WHN_0_4, SNDID_WHN_0_4, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 1 whine 4\n");
    }
    if (ChannelInitWav("../wav/A_WHN_5.wav", SNDSRC_WHN_0_5, SNDID_WHN_0_5, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 1 whine 5\n");
    }
    if (ChannelInitWav("../wav/A_REVRS.wav", SNDSRC_REVRS_0, SNDID_REVRS_0, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 1 reverse thrust 5\n");
    }
	
    if (ChannelInitWav("../wav/A_WHN_0.wav", SNDSRC_WHN_1_0, SNDID_WHN_1_0, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 2 whine 0\n");
    }
    if (ChannelInitWav("../wav/A_WHN_1.wav", SNDSRC_WHN_1_1, SNDID_WHN_1_1, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 2 whine 1\n");
    }
    if (ChannelInitWav("../wav/A_WHN_2.wav", SNDSRC_WHN_1_2, SNDID_WHN_1_2, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 2 whine 2\n");
    }
    if (ChannelInitWav("../wav/A_WHN_3.wav", SNDSRC_WHN_1_3, SNDID_WHN_1_3, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 2 whine 3\n");
    }
    if (ChannelInitWav("../wav/A_WHN_4.wav", SNDSRC_WHN_1_4, SNDID_WHN_1_4, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 2 whine 4\n");
    }
    if (ChannelInitWav("../wav/A_WHN_5.wav", SNDSRC_WHN_1_5, SNDID_WHN_1_5, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 2 whine 5\n");
    }
    if (ChannelInitWav("../wav/A_REVRS.wav", SNDSRC_REVRS_1, SNDID_REVRS_1, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 2 reverse thrust 5\n");
    }
	
    if (ChannelInitWav("../wav/A_WHN_0.wav", SNDSRC_WHN_2_0, SNDID_WHN_2_0, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 3 whine 0\n");
    }
    if (ChannelInitWav("../wav/A_WHN_1.wav", SNDSRC_WHN_2_1, SNDID_WHN_2_1, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 3 whine 1\n");
    }
    if (ChannelInitWav("../wav/A_WHN_2.wav", SNDSRC_WHN_2_2, SNDID_WHN_2_2, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 3 whine 2\n");
    }
    if (ChannelInitWav("../wav/A_WHN_3.wav", SNDSRC_WHN_2_3, SNDID_WHN_2_3, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 3 whine 3\n");
    }
    if (ChannelInitWav("../wav/A_WHN_4.wav", SNDSRC_WHN_2_4, SNDID_WHN_2_4, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 3 whine 4\n");
    }
    if (ChannelInitWav("../wav/A_WHN_5.wav", SNDSRC_WHN_2_5, SNDID_WHN_2_5, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 3 whine 5\n");
    }
    if (ChannelInitWav("../wav/A_REVRS.wav", SNDSRC_REVRS_2, SNDID_REVRS_2, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 3 reverse thrust 5\n");
    }
	
    if (ChannelInitWav("../wav/A_WHN_0.wav", SNDSRC_WHN_3_0, SNDID_WHN_3_0, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 4 whine 0\n");
    }
    if (ChannelInitWav("../wav/A_WHN_1.wav", SNDSRC_WHN_3_1, SNDID_WHN_3_1, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 4 whine 1\n");
    }
    if (ChannelInitWav("../wav/A_WHN_2.wav", SNDSRC_WHN_3_2, SNDID_WHN_3_2, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 4 whine 2\n");
    }
    if (ChannelInitWav("../wav/A_WHN_3.wav", SNDSRC_WHN_3_3, SNDID_WHN_3_3, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 4 whine 3\n");
    }
    if (ChannelInitWav("../wav/A_WHN_4.wav", SNDSRC_WHN_3_4, SNDID_WHN_3_4, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 4 whine 4\n");
    }
    if (ChannelInitWav("../wav/A_WHN_5.wav", SNDSRC_WHN_3_5, SNDID_WHN_3_5, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 4 whine 5\n");
    }
    if (ChannelInitWav("../wav/A_REVRS.wav", SNDSRC_REVRS_3, SNDID_REVRS_3, 0.0, true) == 0)
    {
        fatal("Unable to initialise channel: engine 4 reverse thrust 5\n");
    }
	
    return 1;
}

/* ---------------------------------------- */
void JetEngine(unsigned int EngineNumber, float N1, bool ReverseThrust)
/*                          1-4                 0-100%                    */
{
    float A_REVRS_frequency_x[2] = { 0.000000,1.000000 };
    float A_REVRS_frequency_y[2] = { 1.000000,1.800000 };
    float A_REVRS_gain_x[3] = { 0.021000,0.70000,1.000000 };
    float A_REVRS_gain_y[3] = { 0.000000,0.30000,0.936000 };

    float whn_0_frequency_x [2] = { 0.200000, 0.400000 };
    float whn_0_frequency_y [2] = { 0.975000, 1.075000 };
    float whn_0_gain_x [8] = { 0.000000, 0.100000,  0.157000,  0.221000,  0.305000,  0.370000, 0.439000, 0.606000 };
    float whn_0_gain_y [8] = { 0.000000, 5.000000, 34.853001, 33.914001, 23.458000, 12.332000, 4.692000, 0.000000 };
    
    float whn_1_frequency_x [2] = { 0.310000, 0.660000 };
    float whn_1_frequency_y [2] = { 1.030000, 1.200000 };
    float whn_1_gain_x [8] = { 0.282000,  0.341000,  0.379000,  0.423000,  0.497000, 0.517000, 0.560000, 0.626000 };
    float whn_1_gain_y [8] = { 0.000000, 20.000000, 25.000000, 25.000000, 14.000000, 9.500000, 3.191000, 0.000000 };

    float whn_2_frequency_x [2] = { 0.400000, 0.600000 };
    float whn_2_frequency_y [2] = { 0.900000, 1.200000 };
    float whn_2_gain_x [8] = { 0.333000,  0.414000,  0.483226,  0.556129, 0.581935, 0.618065, 0.669677, 0.731000 };
    float whn_2_gain_y [8] = { 0.000000, 10.372000, 15.292553, 11.037234, 7.978724, 5.053192, 2.127660, 0.000000 };

    float whn_3_frequency_x [2] = { 0.560000, 0.910000 };
    float whn_3_frequency_y [2] = { 0.600000, 1.340000 };
    float whn_3_gain_x [8] = { 0.530000, 0.603000,  0.641000,  0.672000,  0.732000,  0.808000,  0.881000, 0.990000 };
    float whn_3_gain_y [8] = { 0.000000, 7.447000, 17.420000, 27.926001, 40.425999, 41.355999, 31.915001, 0.000000 };

    float whn_4_frequency_x [2] = { 0.322000, 1.000000 };
    float whn_4_frequency_y [2] = { 1.000000, 1.977000 };
    float whn_4_gain_x [8] = { 0.400000, 0.600000, 0.700000, 0.765806,  0.800000,  0.850000,   0.900000,   0.950000 };
    float whn_4_gain_y [8] = { 0.000000, 0.000000, 0.000000, 8.643617, 50.000000, 75.000000, 100.000000, 100.000000 };

    float whn_5_frequency_x [2] = { 0.880000, 0.970000 };
    float whn_5_frequency_y [2] = { 0.920000, 1.012000 };
    float whn_5_gain_x[6] = { 0.875000,  0.890000,  0.905000,  0.920000,  0.950000,  0.970000 };
    float whn_5_gain_y[6] = { 0.000000, 10.000000, 30.000000, 50.000000, 65.000000, 80.000000 };
    
    float Gain;
    float PitchFactor;

    if (ReverseThrust)
    {
        Gain = interpolate(sizeof(A_REVRS_gain_x), N1 / 100.0, A_REVRS_gain_x, A_REVRS_gain_y);
        PitchFactor = interpolate(sizeof(A_REVRS_frequency_x), N1 / 100.0, A_REVRS_frequency_x, A_REVRS_frequency_y);
		
		switch (EngineNumber)
		{
		    case 0:
                alSourcef(soundSources[SNDSRC_REVRS_0], AL_GAIN, Gain);
                alSourcef(soundSources[SNDSRC_REVRS_0], AL_PITCH, PitchFactor);

                alSourcef(soundSources[SNDSRC_WHN_0_0], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_0_1], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_0_2], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_0_3], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_0_4], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_0_5], AL_GAIN, 0.0);
				break;
		    case 1:
                alSourcef(soundSources[SNDSRC_REVRS_1], AL_GAIN, Gain);
                alSourcef(soundSources[SNDSRC_REVRS_1], AL_PITCH, PitchFactor);

                alSourcef(soundSources[SNDSRC_WHN_1_0], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_1_1], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_1_2], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_1_3], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_1_4], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_1_5], AL_GAIN, 0.0);
				break;
		    case 2:
                alSourcef(soundSources[SNDSRC_REVRS_2], AL_GAIN, Gain);
                alSourcef(soundSources[SNDSRC_REVRS_2], AL_PITCH, PitchFactor);

                alSourcef(soundSources[SNDSRC_WHN_2_0], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_2_1], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_2_2], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_2_3], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_2_4], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_2_5], AL_GAIN, 0.0);
				break;
		    case 3:
                alSourcef(soundSources[SNDSRC_REVRS_3], AL_GAIN, Gain);
                alSourcef(soundSources[SNDSRC_REVRS_3], AL_PITCH, PitchFactor);

                alSourcef(soundSources[SNDSRC_WHN_3_0], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_3_1], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_3_2], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_3_3], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_3_4], AL_GAIN, 0.0);
                alSourcef(soundSources[SNDSRC_WHN_3_5], AL_GAIN, 0.0);
				break;
		}
    }
    else
    {
	    switch (EngineNumber)
		{
		    case 0:
                Gain = interpolate(sizeof(whn_0_gain_x), N1 / 100.0, whn_0_gain_x, whn_0_gain_y);
                PitchFactor = interpolate(sizeof(whn_0_frequency_x), N1 / 100.0, whn_0_frequency_x, whn_0_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_0_0], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_0_0], AL_PITCH, PitchFactor);

                Gain = interpolate(sizeof(whn_1_gain_x), N1 / 100.0, whn_1_gain_x, whn_1_gain_y);
                PitchFactor = interpolate(sizeof(whn_1_frequency_x), N1 / 100.0, whn_1_frequency_x, whn_1_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_0_1], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_0_1], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_2_gain_x), N1 / 100.0, whn_2_gain_x, whn_2_gain_y);
                PitchFactor = interpolate(sizeof(whn_2_frequency_x), N1 / 100.0, whn_2_frequency_x, whn_2_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_0_2], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_0_2], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_3_gain_x), N1 / 100.0, whn_3_gain_x, whn_3_gain_y);
                PitchFactor = interpolate(sizeof(whn_3_frequency_x), N1 / 100.0, whn_3_frequency_x, whn_3_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_0_3], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_0_3], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_4_gain_x), N1 / 100.0, whn_4_gain_x, whn_4_gain_y);
                PitchFactor = interpolate(sizeof(whn_4_frequency_x), N1 / 100.0, whn_4_frequency_x, whn_4_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_0_4], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_0_4], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_5_gain_x), N1 / 100.0, whn_5_gain_x, whn_5_gain_y);
                PitchFactor = interpolate(sizeof(whn_5_frequency_x), N1 / 100.0, whn_5_frequency_x, whn_5_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_0_5], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_0_5], AL_PITCH, PitchFactor);
                
                alSourcef(soundSources[SNDSRC_REVRS_0], AL_GAIN, 0.0);
				break;
		    case 1:
                Gain = interpolate(sizeof(whn_0_gain_x), N1 / 100.0, whn_0_gain_x, whn_0_gain_y);
                PitchFactor = interpolate(sizeof(whn_0_frequency_x), N1 / 100.0, whn_0_frequency_x, whn_0_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_1_0], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_1_0], AL_PITCH, PitchFactor);

                Gain = interpolate(sizeof(whn_1_gain_x), N1 / 100.0, whn_1_gain_x, whn_1_gain_y);
                PitchFactor = interpolate(sizeof(whn_1_frequency_x), N1 / 100.0, whn_1_frequency_x, whn_1_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_1_1], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_1_1], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_2_gain_x), N1 / 100.0, whn_2_gain_x, whn_2_gain_y);
                PitchFactor = interpolate(sizeof(whn_2_frequency_x), N1 / 100.0, whn_2_frequency_x, whn_2_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_1_2], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_1_2], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_3_gain_x), N1 / 100.0, whn_3_gain_x, whn_3_gain_y);
                PitchFactor = interpolate(sizeof(whn_3_frequency_x), N1 / 100.0, whn_3_frequency_x, whn_3_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_1_3], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_1_3], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_4_gain_x), N1 / 100.0, whn_4_gain_x, whn_4_gain_y);
                PitchFactor = interpolate(sizeof(whn_4_frequency_x), N1 / 100.0, whn_4_frequency_x, whn_4_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_1_4], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_1_4], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_5_gain_x), N1 / 100.0, whn_5_gain_x, whn_5_gain_y);
                PitchFactor = interpolate(sizeof(whn_5_frequency_x), N1 / 100.0, whn_5_frequency_x, whn_5_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_1_5], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_1_5], AL_PITCH, PitchFactor);
                
                alSourcef(soundSources[SNDSRC_REVRS_1], AL_GAIN, 0.0);
				break;
		    case 2:
                Gain = interpolate(sizeof(whn_0_gain_x), N1 / 100.0, whn_0_gain_x, whn_0_gain_y);
                PitchFactor = interpolate(sizeof(whn_0_frequency_x), N1 / 100.0, whn_0_frequency_x, whn_0_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_2_0], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_2_0], AL_PITCH, PitchFactor);

                Gain = interpolate(sizeof(whn_1_gain_x), N1 / 100.0, whn_1_gain_x, whn_1_gain_y);
                PitchFactor = interpolate(sizeof(whn_1_frequency_x), N1 / 100.0, whn_1_frequency_x, whn_1_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_2_1], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_2_1], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_2_gain_x), N1 / 100.0, whn_2_gain_x, whn_2_gain_y);
                PitchFactor = interpolate(sizeof(whn_2_frequency_x), N1 / 100.0, whn_2_frequency_x, whn_2_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_2_2], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_2_2], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_3_gain_x), N1 / 100.0, whn_3_gain_x, whn_3_gain_y);
                PitchFactor = interpolate(sizeof(whn_3_frequency_x), N1 / 100.0, whn_3_frequency_x, whn_3_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_2_3], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_2_3], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_4_gain_x), N1 / 100.0, whn_4_gain_x, whn_4_gain_y);
                PitchFactor = interpolate(sizeof(whn_4_frequency_x), N1 / 100.0, whn_4_frequency_x, whn_4_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_2_4], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_2_4], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_5_gain_x), N1 / 100.0, whn_5_gain_x, whn_5_gain_y);
                PitchFactor = interpolate(sizeof(whn_5_frequency_x), N1 / 100.0, whn_5_frequency_x, whn_5_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_2_5], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_2_5], AL_PITCH, PitchFactor);
                
                alSourcef(soundSources[SNDSRC_REVRS_2], AL_GAIN, 0.0);
				break;
		    case 3:
                Gain = interpolate(sizeof(whn_0_gain_x), N1 / 100.0, whn_0_gain_x, whn_0_gain_y);
                PitchFactor = interpolate(sizeof(whn_0_frequency_x), N1 / 100.0, whn_0_frequency_x, whn_0_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_3_0], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_3_0], AL_PITCH, PitchFactor);

                Gain = interpolate(sizeof(whn_1_gain_x), N1 / 100.0, whn_1_gain_x, whn_1_gain_y);
                PitchFactor = interpolate(sizeof(whn_1_frequency_x), N1 / 100.0, whn_1_frequency_x, whn_1_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_3_1], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_3_1], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_2_gain_x), N1 / 100.0, whn_2_gain_x, whn_2_gain_y);
                PitchFactor = interpolate(sizeof(whn_2_frequency_x), N1 / 100.0, whn_2_frequency_x, whn_2_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_3_2], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_3_2], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_3_gain_x), N1 / 100.0, whn_3_gain_x, whn_3_gain_y);
                PitchFactor = interpolate(sizeof(whn_3_frequency_x), N1 / 100.0, whn_3_frequency_x, whn_3_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_3_3], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_3_3], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_4_gain_x), N1 / 100.0, whn_4_gain_x, whn_4_gain_y);
                PitchFactor = interpolate(sizeof(whn_4_frequency_x), N1 / 100.0, whn_4_frequency_x, whn_4_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_3_4], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_3_4], AL_PITCH, PitchFactor);
                
                Gain = interpolate(sizeof(whn_5_gain_x), N1 / 100.0, whn_5_gain_x, whn_5_gain_y);
                PitchFactor = interpolate(sizeof(whn_5_frequency_x), N1 / 100.0, whn_5_frequency_x, whn_5_frequency_y);
                alSourcef(soundSources[SNDSRC_WHN_3_5], AL_GAIN, Gain / 100.0);
                alSourcef(soundSources[SNDSRC_WHN_3_5], AL_PITCH, PitchFactor);
                
                alSourcef(soundSources[SNDSRC_REVRS_3], AL_GAIN, 0.0);
				break;
		}
    }
}

/* ---------------------------------------- */
void BEGIN_SoundLib(int *argc, char *argv[])
{
//return; /* *** waiting to fix bug with openal */ 
    OldStallWarning = false;
    OldFireWarning = false;
    OldStickWarning = false;
    OldMiddleMarker = false;
    OldOuterMarker = false;
    OldGearWarning = false;
    OldConfigWarning = false;
    OldAirconditioning = false;
    OldElectricalNoise = false;
    OldGearMotor = false;

    if (!SoundInit(argc, argv))
    {
        fatal("Unable to initialise sound system\n");
    }
    if (!OuterMarkerInit())
    {
        fatal("Unable to initialise outer marker\n");
    }
    if (!MiddleMarkerInit())
    {
        fatal("Unable to initialise middle marker\n");
    }
    if (!GearWarningInit())
    {
        fatal("Unable to initialise gear warning\n");
    }
    if (!MorseInit())
    {
        fatal("Unable to initialise morse\n");
    }
    if (!ConfigInit())
    {
        fatal("Unable to initialise configuration warning\n");
    }
    if (!StallWarningInit())
    {
        fatal("Unable to initialise stall warning\n");
    }
    if (!FireWarningInit())
    {
        fatal("Unable to initialise fire warning\n");
    }
    if (!StickWarningInit())
    {
        fatal("Unable to initialise stick warning\n");
    }
    if (!ElectricalNoiseInit())
    {
        fatal("Unable to initialise electrical noise\n");
    }
    if (!AirConditioningInit())
    {
        fatal("Unable to initialise air conditioning noise\n");
    }
    if (!SlipstreamInit())
    {
        fatal("Unable to initialise slipstream\n");
    }
    if (!GearMotorInit())
    {
        fatal("Unable to initialise gear motor\n");
    }
    if (!GroundRumbleInit())
    {
        fatal("Unable to initialise ground rumble\n");
    }
    if (!GearBuffetInit())
    {
        fatal("Unable to initialise gear buffet\n");
    }
    if (!JetEnginesInit())
    {
        fatal("Unable to initialise jet engine\n");
    }
    if (!PistonEnginesInit())
    {
        fatal("Unable to initialise piston engine\n");
    }
    if (!TurboPropEnginesInit())
    {
        fatal("Unable to initialise turboprop engine\n");
    }
}

/* --------------------------------------------- */

void END_SoundLib()
{
    sourceStopandDetach(SNDSRC_OUTER_MARKER);
    sourceStopandDetach(SNDSRC_MIDDLE_MARKER);
    sourceStopandDetach(SNDSRC_MORSE_TONE);
    sourceStopandDetach(SNDSRC_COCKPIT_AC);
    sourceStopandDetach(SNDSRC_COCKPIT_ELEC);
    sourceStopandDetach(SNDSRC_SLIPSTREAM);
    sourceStopandDetach(SNDSRC_GEAR_BUFFET);
    sourceStopandDetach(SNDSRC_GEAR_WARNING);
    sourceStopandDetach(SNDSRC_GEAR_MOTOR);
    sourceStopandDetach(SNDSRC_STICK_WARNING);
    sourceStopandDetach(SNDSRC_FIRE_WARNING);
    sourceStopandDetach(SNDSRC_STALL_WARNING);
    sourceStopandDetach(SNDSRC_CONFIG_WARNING);
	
    sourceStopandDetach(SNDSRC_WHN_0_0);
    sourceStopandDetach(SNDSRC_WHN_0_1);
    sourceStopandDetach(SNDSRC_WHN_0_2);
    sourceStopandDetach(SNDSRC_WHN_0_3);
    sourceStopandDetach(SNDSRC_WHN_0_4);
    sourceStopandDetach(SNDSRC_WHN_0_5);
    sourceStopandDetach(SNDSRC_REVRS_0);
    sourceStopandDetach(SNDSRC_WHN_1_0);
    sourceStopandDetach(SNDSRC_WHN_1_1);
    sourceStopandDetach(SNDSRC_WHN_1_2);
    sourceStopandDetach(SNDSRC_WHN_1_3);
    sourceStopandDetach(SNDSRC_WHN_1_4);
    sourceStopandDetach(SNDSRC_WHN_1_5);
    sourceStopandDetach(SNDSRC_REVRS_1);
    sourceStopandDetach(SNDSRC_WHN_2_0);
    sourceStopandDetach(SNDSRC_WHN_2_1);
    sourceStopandDetach(SNDSRC_WHN_2_2);
    sourceStopandDetach(SNDSRC_WHN_2_3);
    sourceStopandDetach(SNDSRC_WHN_2_4);
    sourceStopandDetach(SNDSRC_WHN_2_5);
    sourceStopandDetach(SNDSRC_REVRS_2);
    sourceStopandDetach(SNDSRC_WHN_3_0);
    sourceStopandDetach(SNDSRC_WHN_3_1);
    sourceStopandDetach(SNDSRC_WHN_3_2);
    sourceStopandDetach(SNDSRC_WHN_3_3);
    sourceStopandDetach(SNDSRC_WHN_3_4);
    sourceStopandDetach(SNDSRC_WHN_3_5);
    sourceStopandDetach(SNDSRC_REVRS_3);
	
	alutExit();
}
