/*
Linux version
DJA 14 Jan 2024
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <math.h>

#include <SIM/navdefn.h>
#include <SIM/iosdefn.h>
#include <SIM/maths.h>

#include "navlink.h"
#include "fcu.h"
#include "rs232lib.h"

const double Rs    = 8314.32;   /* Nm/(kmol K), gas constant */
const double M0    = 28.9644;   /* kg/kmol, mean molecular weight of air */
const double r0    = 6356766.0; /* m, Earth radius at g0 */
const double T0    = 288.15;    /* K, standard sea-level temperature */
const double Gamma = 1.40;      /* Ratio of Specific heats for ideal diatomic gas */

bool            FCU_FD;
bool            FCU_LS;
bool            FCU_LOC;
bool            FCU_AP1;
bool            FCU_AP2;
bool            FCU_ATHR;
bool            FCU_EXPED;
bool            FCU_APPR;
bool            FCU_SPD_MACH_Button;
bool            FCU_HDG_TRK_Button;
unsigned int    FCU_BaroPressure;
bool            FCU_BaroHg;
bool            FCU_BaroSTD;
unsigned int    FCU_HDG;
unsigned int    FCU_TRK;
unsigned int    FCU_ALT;
unsigned int    FCU_SPD;
int             FCU_VS;
int             FCU_FPA;
bool            FCU_Metric_Button;

NavDefn_FCUKnob FCU_BaroKnob;
NavDefn_FCUKnob FCU_HDGKnob;
NavDefn_FCUKnob FCU_ALTKnob;
NavDefn_FCUKnob FCU_SPDKnob;
NavDefn_FCUKnob FCU_VSKnob;
unsigned int    FCU_ALT_Range;

NavDefn_FCUNav  FCU_NavSwitch1;
NavDefn_FCUNav  FCU_NavSwitch2;
NavDefn_FCUMode FCU_ModeSelector;
NavDefn_FCUData FCU_DataMode;
unsigned int    FCU_RangeSelector;

bool            FCU_CSTR_Button;
bool            FCU_WPT_Button;
bool            FCU_VORD_Button;
bool            FCU_NDB_Button;
bool            FCU_ARPT_Button;

NavDefn_FCUNav  LHS_NavSwitch1;
NavDefn_FCUNav  LHS_NavSwitch2;
NavDefn_FCUMode LHS_ModeSelector;
NavDefn_FCUData LHS_DataMode;
unsigned int    LHS_RangeSelector;
NavDefn_FCUNav  RHS_NavSwitch1;
NavDefn_FCUNav  RHS_NavSwitch2;
unsigned int    RHS_RangeSelector;
NavDefn_FCUMode RHS_ModeSelector;
NavDefn_FCUData RHS_DataMode;

bool            LHS_FD;
bool            LHS_LS;
unsigned int    LHS_Baro;
bool            LHS_BaroHg;
bool            LHS_BaroSTD;
bool            LHS_CSTR;
bool            LHS_WPT;
bool            LHS_VORD;
bool            LHS_NDB;
bool            LHS_ARPT;
unsigned int    Old_LHS_Baro;

bool            RHS_FD;
bool            RHS_LS;
unsigned int    RHS_Baro;
bool            RHS_BaroHg;
bool            RHS_BaroSTD;
bool            RHS_CSTR;
bool            RHS_WPT;
bool            RHS_VORD;
bool            RHS_NDB;
bool            RHS_ARPT;
unsigned int    Old_RHS_Baro;

#define ACK     0x6
#define NACK    0x15
#define bufsize 5000   /* bytes */

static int             port_handle;
static pthread_t       port_rxthread;
static pthread_t       port_txthread;
static unsigned int    rxiptr;
static unsigned int    rxoptr;
static unsigned char   rxbuf[bufsize];
static unsigned int    txiptr;
static unsigned int    txoptr;
static unsigned char   txbuf[bufsize];
static pthread_mutex_t rxbuf_mutex;
static pthread_mutex_t txbuf_mutex;
static sem_t           rxsem;
static sem_t           txsem;

static void            SendCard(char Cmd[], unsigned int Arg, unsigned int Width);
static void            SendInt(char Cmd[], int Arg, unsigned int Width);
static void            SendDLX(char Str[]);
static void            SendQ(char Str[]);
static unsigned int    GetCard(char Str[], unsigned int Width);
static int             GetInt(char Str[], unsigned int Width);
static unsigned int    Bin(char Ch);
static void            DecodeVCode(char VString[]);
static void            DecodeKCode(char KString[]);
static void            DecodeRCode(char RString[]);
static void            SendChars(char str[]);

static void            *serial_input();
static void            *serial_output();
static void            Decode(char v[]);
void 	               Start_SerialPort();

/* ------------------------------------------------------ */
void FCU_StopFCU()
{
    pthread_cancel(port_rxthread);
    pthread_cancel(port_txthread);
    RS232_Close(port_handle);
}

/* ------------------------------------------------------ */
void Start_SerialPort() /* create rx and tx threads, initialise semaphores, initialise mutexes, reset buffers */
{
    rxiptr       = 0;
    rxoptr       = 0;
    txiptr       = 0;
    txoptr       = 0;
    pthread_mutex_init(&rxbuf_mutex, NULL);
    pthread_mutex_init(&txbuf_mutex, NULL);

    if (sem_init(&rxsem, 0, 0) < 0)
    {
        printf("sem_init failed rx thread\n");
        exit(1);
    }
    if (sem_init(&txsem, 0, 0) < 0)
    {
        printf("sem_init failed tx thread\n");
        exit(1);
    }

    if (pthread_create(&port_rxthread, NULL, serial_input, NULL))  /* start these threads *after* the serial port is initialised */
    {
        printf("Unable to create serial port rx thread\n");
        exit(1);
    }

    if (pthread_create(&port_txthread, NULL, serial_output, NULL))
    {
        printf("Unable to create serial port tx thread\n");
        exit(1);
    }
}

/* ------------------------------------------------------ */
void *serial_input()
{
    while (1)  /* threads reads one byte from serial input indefinitely */
    {
        sem_wait(&rxsem);

        while (1)
        {
            unsigned char ch;
            int           dwBytesRead = RS232_Gets(port_handle, (char *) &ch, 1);

            if (dwBytesRead < 0)
            {
                printf("serial port read error 1\n");
                exit(1);
            }

            if (dwBytesRead == 0)  /* timeout? */
            {
                pthread_mutex_lock(&txbuf_mutex);
                if (txiptr == txoptr)
                {
                    pthread_mutex_unlock(&txbuf_mutex);
					continue;
                }
				else
                {
                    pthread_mutex_unlock(&txbuf_mutex);
                    sem_post(&txsem);
                    break;
                }
            }

            if (dwBytesRead != 1)
            {
                printf("serial port read error 2\n");
                exit(1);
            }

            pthread_mutex_lock(&rxbuf_mutex);
            if (!(ch ==  ACK || ch == NACK))
            {
                rxbuf[rxiptr] = ch;
                rxiptr += 1;
                if (rxiptr >= bufsize)
                {
                    rxiptr = 0;
                }
            }
            pthread_mutex_unlock(&rxbuf_mutex);
			
            if (ch == '\0')
            {
                char v[100];
                unsigned int p = 0;

                pthread_mutex_lock(&rxbuf_mutex);
                while (1)
                {
                    char chx = rxbuf[rxoptr];

                    v[p] = chx;
                    p += 1;
                    rxoptr += 1;
                    if (rxoptr >= bufsize)
                    {
                        rxoptr = 0;
                    }
                    if (chx == '\0')
                    {
                        break;
                    }
                }
                pthread_mutex_unlock(&rxbuf_mutex);
                //printf("Recvd: %s\n", v); // ***
                Decode(v);
            }
            else if (!(ch == ACK || ch == NACK))
            {
                continue;
            }

            pthread_mutex_lock(&txbuf_mutex);
            if (txiptr == txoptr)
            {
                pthread_mutex_unlock(&txbuf_mutex);
				continue;
            }
			else
            {
                pthread_mutex_unlock(&txbuf_mutex);
                sem_post(&txsem);
                break;
            }
        }
    }
}

/* ------------------------------------------------------ */
void *serial_output()
{
    int dwBytesWritten;

    while (1)
    {
        sem_wait(&txsem);

        while (1)
        {
            unsigned char ch;

            pthread_mutex_lock(&txbuf_mutex);
            ch = txbuf[txoptr];
            txoptr += 1;
            if (txoptr >= bufsize)
            {
                txoptr = 0;
            }

            dwBytesWritten = RS232_Puts(port_handle, (char *) &ch, 1);
            if (dwBytesWritten < 0)
            {
                printf("Serial port write error 1\n");
                exit(1);
            }
			
            if (dwBytesWritten != 1)
            {
                printf("Serial port write error 2\n");
                exit(1);
            }
            pthread_mutex_unlock(&txbuf_mutex);
            //printf("%c", ch == '\0' ? '\n' : ch); // ***

            if (ch == '\0')
            {
                sem_post(&rxsem);
                break;
            }
        }
    }
}

/* ------------------------------------------------------ */
static void SendChars(char str[])
{
    unsigned int p = 0;

    pthread_mutex_lock(&txbuf_mutex);
    while (1)
    {
        char ch = str[p];

        p += 1;
        txbuf[txiptr] = ch;
        txiptr += 1;
        if (txiptr >= bufsize)
        {
            txiptr = 0;
        }
        if (txiptr == txoptr)
        {
            printf("Serial buffer overflow\n");
            exit(1);
        }
        if (ch == '\0')
        {
            break;
        }
    }
    pthread_mutex_unlock(&txbuf_mutex);
}

/* --------------------------------------------------------------------- */
static void Decode(char v[])
{
    switch (v[0])
    {
        case 'K':
            DecodeKCode(v);
            break;
        case 'R':
            DecodeRCode(v);
            break;
        case 'V':
            DecodeVCode(v);
            break;
        default:
            printf("unknown command %s\n", v);
            exit(1);
    }
    
    FCU_FD = LHS_FD;
    FCU_LS = LHS_LS;
    FCU_NavSwitch1 = LHS_NavSwitch1;
    FCU_NavSwitch2 = LHS_NavSwitch2;
    FCU_ModeSelector = LHS_ModeSelector;
    FCU_DataMode = LHS_DataMode;
    FCU_RangeSelector = LHS_RangeSelector;

    FCU_BaroPressure = LHS_Baro;
    FCU_BaroHg = LHS_BaroHg;
    FCU_BaroSTD = LHS_BaroSTD;

    FCU_CSTR_Button = LHS_CSTR;
    FCU_WPT_Button = LHS_WPT;
    FCU_VORD_Button = LHS_VORD;
    FCU_NDB_Button = LHS_NDB;
    FCU_ARPT_Button = LHS_ARPT;
}

/* --------------------------------------------------------------------- */
void FCU_InitialiseFCU()
{
    port_handle = RS232_Open("/dev/ttySC0", 19200);  /* channel 1 */
    if (port_handle < 0)
    {
        printf("can't open serial port\n");
        exit(1);
    }
    Start_SerialPort();

    SendDLX("L003"); /* LHS BARO legend Off */
    SendDLX("L102"); /* LHS STD legend On */
    SendDLX("L024"); /* LHS CSTR lamp Off */
    SendDLX("L025"); /* LHS WPT lamp Off */
    SendDLX("L026"); /* LHS FD lamp Off */
    SendDLX("L027"); /* LHS LS lamp Off */
    SendDLX("L028"); /* LHS VOR.D lamp Off */
    SendDLX("L029"); /* LHS NDB lamp Off */
    SendDLX("L030"); /* LHS ARPT lamp Off */

    SendDLX("L033"); /* RHS BARO legend Off */
    SendDLX("L132"); /* RHS STD legend On */
    SendDLX("L054"); /* RHS CSTR lamp Off */
    SendDLX("L055"); /* RHS WPT lamp Off */
    SendDLX("L056"); /* RHS FD lamp Off */
    SendDLX("L057"); /* RHS LS lamp Off */
    SendDLX("L058"); /* RHS VOR.D lamp Off */
    SendDLX("L059"); /* RHS NDB lamp Off */
    SendDLX("L071"); /* RHS ARPT lamp Off */

    SendDLX("L062"); /* MACH legend Off */
    SendDLX("L161"); /* SPD legend On */
    SendDLX("L066"); /* TRK/FPA legend Off */
    SendDLX("L165"); /* HDG/VS legend On */
    SendDLX("L069"); /* LVL CHG legend Off */

    SendDLX("L075"); /* LOC lamp Off */
    SendDLX("L076"); /* HDG lamp Off */
    SendDLX("L077"); /* AP1 lamp Off */
    SendDLX("L078"); /* AP2 lamp Off */
    SendDLX("L079"); /* ATHR lamp Off */
    SendDLX("L080"); /* EXPED lamp Off */
    SendDLX("L081"); /* LVL CHG lamp Off */
    SendDLX("L082"); /* APPR lamp Off */
    SendDLX("L083"); /* SPD lamp Off */
	
    SendDLX("D060000");   /* SPD counter */
    SendDLX("D064000");   /* HDG counter */
    SendDLX("D06700000"); /* ALT counter */
    SendDLX("D072+00");   /* VS counter */

    SendDLX("L002");   /* Initialise the Baro Counters */
    SendDLX("X0001");  /* normal mode */
    SendDLX("L032");
    SendDLX("X0031");  /* normal mode */

    /* query BARO switch positions */
    SendQ("Q004");   /* get LH Hg/HPa switch position */
    SendQ("Q034");   /* get RH Hg/HPa switch position */

    if (LHS_BaroHg)
    {
        SendDLX("D0012992");
        LHS_Baro = 2992;
    }
    else
    {
        SendDLX("D0011013");
        LHS_Baro = 1013;
    }

    if (RHS_BaroHg)
    {
        SendDLX("D0312992");
        RHS_Baro = 2992;
    }
    else
    {
        SendDLX("D0311013");
        RHS_Baro = 1013;
    }

    SendQ("Q060");  /* Request Speed/Mach */
    SendQ("Q064");
    SendQ("Q067");
    SendQ("Q072");

    SendQ("Q018");  /* Initialise the NavAids */
    SendQ("Q021");
    SendQ("Q048");
    SendQ("Q051");

    SendQ("Q036");
    SendQ("Q042");
    SendQ("Q054");
    SendQ("Q006");
    SendQ("Q012");
    SendQ("Q024");

    sem_post(&txsem);
}

/* --------------------------------------------------------------------- */
static void SendCard(char Cmd[], unsigned int Arg, unsigned int Width)
{
    unsigned int       i;
    unsigned int       s;
    char               Str[20];
    const unsigned int Divisor[6] = {0, 1, 10, 100, 1000, 10000};

    s = strlen(Cmd);
    for (i = 0; i <= s - 1; i += 1)
    {
        Str[i] = Cmd[i];
    }
    for (i = Width; i >= 1; i += -1)
    {
        Str[s] = (char) (Arg / Divisor[i] % 10 + '0');
        s += 1;
    }
    Str[s] = '\0';
    SendDLX(Str);
}

/* --------------------------------------------------------------------- */
static void SendInt(char Cmd[], int Arg, unsigned int Width)
{
    unsigned int       i;
    unsigned int       s;
    unsigned int       x;
    char               Str[20];
    const unsigned int Divisor[6] = {0, 1, 10, 100, 1000, 10000};

    s = strlen(Cmd);
    for (i = 0; i <= s - 1; i += 1)
    {
        Str[i] = Cmd[i];
    }
    if (Arg >= 0)
    {
        x = Arg;
        Str[s] = '+';
    }
    else
    {
        Str[s] = '-';
        x = -Arg;
    }
    s += 1;
    for (i = Width; i >= 1; i += -1)
    {
        Str[s] = (char) (x / Divisor[i] % 10 + '0');
        s += 1;
    }
    Str[s] = '\0';
    SendDLX(Str);
}

/* --------------------------------------------------------------------- */
static void SendDLX(char Str[])
{
    SendChars(Str);
}

/* --------------------------------------------------------------------- */
static void SendQ(char Str[])
{
    SendChars(Str);
}

/* --------------------------------------------------------------------- */
static unsigned int GetCard(char Str[], unsigned int Width)
{
    unsigned int i;
    unsigned int n;

    n = 0;
    for (i = 3; i <= 3 + Width - 1; i += 1)
    {
        n = n * 10 + Bin(Str[i]);
    }
    return n;
}

/* --------------------------------------------------------------------- */
static int GetInt(char Str[], unsigned int Width)
{
    unsigned int i;
    int          n;
    bool         neg;

    n = 0;
    neg = Str[3] == '-';
    for (i = 4; i <= 3 + Width - 1; i += 1)
    {
        n = n * 10 + (int) Bin(Str[i]);
    }
    if (neg)
    {
        return -n;
    }
    else
    {
        return n;
    }
}

/* --------------------------------------------------------------------- */
static unsigned int Bin(char Ch)
{
    if (Ch >= '0' && Ch <= '9')
    {
        return (unsigned int) (Ch) - '0';
    }
    else
    {
        return 0;
    }
}

/* --------------------------------------------------------------------- */
static void DecodeVCode(char VString[])
{
    unsigned int Code;
    int          x;
    
    Code = Bin(VString[1]) * 10 + Bin(VString[2]);
    
    switch (Code)
    {
        case 1:
            LHS_Baro = GetCard(VString, 4);
            break;
            
        case 31:
            RHS_Baro = GetCard(VString, 4);
            break;
            
        case 60:
            x = GetCard(VString, 3);
            FCU_SPD = x;
            break;
            
        case 64:
            x = GetCard(VString, 3);
            if (FCU_HDG_TRK_Button)
            {
                FCU_HDG = x;
            }
            else
            {
                FCU_TRK = x;
            }
            break;
            
        case 67:
            FCU_ALT = GetCard(VString, 5);
            break;
            
        case 72:
            x = GetInt(VString, 3);
            if (FCU_HDG_TRK_Button)
            {
                FCU_VS = x;
            }
            else
            {
                FCU_FPA = x;
            }
            break;
    }
}

/* --------------------------------------------------------------------- */
static void DecodeKCode(char KString[])
{
    unsigned int Code;

    Code = Bin(KString[1]) * 100 + Bin(KString[2]) * 10 + Bin(KString[3]);
    
    switch (Code)
    {
        case 1:  /* Baro Ref */
            if (LHS_BaroSTD)
            {
                LHS_BaroSTD = false;
                SendDLX("L002");  /* STD legend off */
                SendCard("D001", LHS_Baro, 4);  /* reset Baro counter */
                SendDLX("L103");  /* BARO legend on */
            }
            Old_LHS_Baro = LHS_Baro;
            SendQ("Q001");
            break;
            
        case 2:  /* Pull Baro = STD */
            LHS_Baro = (LHS_BaroHg) ? 2992 : 1013;
            LHS_BaroSTD = true;
            SendDLX("L003");  /* BARO legend off */
            SendDLX("L102");  /* STD legend on */
            break;

        case 3:  /* Push Baro */
            LHS_Baro = Old_LHS_Baro;
            if (LHS_BaroSTD)
            {
                SendDLX("L002");  /* STD legend off */
 				SendDLX("L103");  /* BARO legend on */
                LHS_BaroSTD = false;
            }
            SendCard("D001", LHS_Baro, 4);  /* reset Baro counter */
            break;
            
        case 4:  /* Hg/hPa  */
            SendQ("Q004");
            break;
            
        case 6:  /* Mode */
            SendQ("Q006");
            break;
            
        case 12:  /* Range */
            SendQ("Q012");
            break;
            
        case 18:  /* NavAid 1*/
            SendQ("Q018");
            break;
            
        case 21:  /* NavAid 2  */
            SendQ("Q021");
            break;
            
        case 24:  /* CSTR button */
            if (LHS_CSTR)
            {
                SendDLX("L024");
            }
            else
            {
                SendDLX("L124");
            }
            LHS_CSTR = !LHS_CSTR;
            break;
            
        case 25:  /* WPT button */
            if (LHS_WPT)
            {
                SendDLX("L025");
            }
            else
            {
                SendDLX("L125");
            }
            LHS_WPT = !LHS_WPT;
            break;
            
        case 26:  /* FD Button */
            if (LHS_FD)
            {
                SendDLX("L026");
            }
            else
            {
                SendDLX("L126");
            }
            LHS_FD = !LHS_FD;
            break;
            
        case 27:  /* LS Button */
            if (LHS_LS)
            {
                SendDLX("L027");
            }
            else
            {
                SendDLX("L127");
            }
            
            LHS_LS = !LHS_LS;
            break;
            
        case 28:  /* VOR.D Button */
            if (LHS_VORD)
            {
                SendDLX("L028");
            }
            else
            {
                SendDLX("L128");
            }
            
            LHS_VORD = !LHS_VORD;
            break;
            
        case 29:  /* NDB Button */
            if (LHS_NDB)
            {
                SendDLX("L029");
            }
            else
            {
                SendDLX("L129");
            }
            
            LHS_NDB = !LHS_NDB;
            break;
            
        case 30:  /* ARPT Button */
            if (LHS_ARPT)
            {
                SendDLX("L030");
            }
            else
            {
                SendDLX("L130");
            }
            
            LHS_ARPT = !LHS_ARPT;
            break;
            
        case 31:  /* Baro Ref */
            if (RHS_BaroSTD)
            {
                RHS_BaroSTD = false;
                SendDLX("L032");  /* STD legend off */
                SendCard("D031", RHS_Baro, 4);  /* reset Baro counter */
                SendDLX("L133");  /* BARO legend on */
            }
            Old_RHS_Baro = RHS_Baro;
            SendQ("Q031");
            break;
            
        case 32:  /* Pull Baro = STD */
            RHS_Baro = (RHS_BaroHg) ? 2992 : 1013;
            RHS_BaroSTD = true;
            SendDLX("L103");  /* BARO legend off */
            SendDLX("L132");  /* STD legend on */
            break;

        case 33:  /* Push Baro */
            RHS_Baro = Old_RHS_Baro;
            if (RHS_BaroSTD)
            {
                SendDLX("L032");  /* STD legend off */
 				SendDLX("L133");  /* BARO legend on */
                RHS_BaroSTD = false;
            }
            SendCard("D031", RHS_Baro, 4);  /* reset Baro counter */
            break;
            
        case 34:  /* Hg/hPa  */
            SendQ("Q034");
            break;
            
        case 36:  /*  Mode  */
            SendQ("Q036");
            break;
            
        case 42:  /* Range  */
            SendQ("Q042");
            break;
            
        case 48:  /* NavAid 1  */
            SendQ("Q048");
            break;
            
        case 51:  /* NavAid 2  */
            SendQ("Q051");
            break;
            
        case 54:  /* CSTR button */
            if (RHS_CSTR)
            {
                SendDLX("L054");
            }
            else
            {
                SendDLX("L154");
            }
            RHS_CSTR = !RHS_CSTR;
            break;
            
        case 55:  /* WPT button */
            if (RHS_WPT)
            {
                SendDLX("L055");
            }
            else
            {
                SendDLX("L155");
            }
            RHS_WPT = !RHS_WPT;
            break;
            
        case 56:  /* FD Button */
            if (RHS_FD)
            {
                SendDLX("L056");
            }
            else
            {
                SendDLX("L156");
            }
            RHS_FD = !RHS_FD;
            break;
            
        case 57:  /* LS Button  */
            if (RHS_LS)
            {
                SendDLX("L057");
            }
            else
            {
                SendDLX("L157");
            }
            RHS_LS = !RHS_LS;
            break;
            
        case 58:  /* VOR.D button */
            if (RHS_VORD)
            {
                SendDLX("L058");
            }
            else
            {
                SendDLX("L158");
            }
            RHS_VORD = !RHS_VORD;
            break;
            
        case 59:  /* NDB button */
            if (RHS_NDB)
            {
                SendDLX("L059");
            }
            else
            {
                SendDLX("L159");
            }
            RHS_NDB = !RHS_NDB;
            break;
            
        case 60:  /*  SPD/MACH Counter  */
            SendQ("Q060");
            break;
            
        case 61:  /* SPD/MACH Pull */
            FCU_SPDKnob = NavDefn_Pulled;
            SendDLX("L183"); /* SPD lamp on */
            break;
            
        case 62:  /* SPD/MACH Push */
            FCU_SPDKnob = NavDefn_Pushed;
            SendDLX("L083"); /* SPD lamp off */
            break;

        case 63:  /* SPD/MACH Button */
            if (FCU_SPD_MACH_Button)
            {
                SendDLX("L061");
                SendDLX("L162");
                SendCard("D060", FCU_SPD, 3);
            }
            else
            {
                SendDLX("L161");
                SendDLX("L062");
                SendCard("D060", FCU_SPD, 3);
            }
            FCU_SPD_MACH_Button = !FCU_SPD_MACH_Button;
            break;
            
        case 64:  /* HDG Counter */
            SendQ("Q064");
            break;
            
        case 65:  /* HDG Pull  */
            FCU_HDGKnob = NavDefn_Pulled;
            SendDLX("L176"); /* HDG lamp on */
            break;
            
        case 66:  /* HDG Push  */
            FCU_HDGKnob = NavDefn_Pushed;
            SendDLX("L076"); /* HDG lamp off */
            break;
            
        case 67:  /* ALT Counter  */
            SendQ("Q067");
            break;
            
        case 68:  /* ALT Pull  */
            FCU_ALTKnob = NavDefn_Pulled;
			FCU_VSKnob = NavDefn_Middle;
            SendDLX("L181"); /* ALT lamp on */
            break;
            
        case 69:  /* ALT Push  */
            FCU_ALTKnob = NavDefn_Pushed;
            SendDLX("L081"); /* ALT lamp off */
            break;
            
        case 70:  /* ALT Range  */
            SendQ("Q070");
            break;
            
        case 71:  /* ARPT Button */
            if (RHS_ARPT)
            {
                SendDLX("L071");
            }
            else
            {
                SendDLX("L171");
            }
            
            RHS_ARPT = !RHS_ARPT;
            break;
            
        case 72:  /* VS Counter  */
            SendQ("Q072");
            break;
            
        case 73:  /* VS Pull  */
            FCU_VSKnob = NavDefn_Pulled;
            FCU_ALTKnob = NavDefn_Middle;
            SendDLX("L181"); /* ALT lamp on */
            break;

        case 74:  /* VS Push  */
            FCU_VSKnob = NavDefn_Pushed;
			FCU_VS = 0;  /* Level off */
            SendDLX("D072+00");   /* zero VS counter */
            break;

        case 75:  /* LOC  */
            if (FCU_LOC)
            {
                SendDLX("L075");
            }
            else
            {
                SendDLX("L175");
            }
            FCU_LOC = !FCU_LOC;
            break;
            
        case 76:  /* HDG/TRK  */
            if (FCU_HDG_TRK_Button)
            {
                SendDLX("L065");
                SendDLX("L166");
                SendCard("D064", FCU_TRK, 3);
                SendInt("D072", FCU_FPA, 2);
            }
            else
            {
                SendDLX("L165");
                SendDLX("L066");
                SendCard("D064", FCU_HDG, 3);
                SendInt("D072", FCU_VS, 2);
            }
            FCU_HDG_TRK_Button = !FCU_HDG_TRK_Button;
            break;
            
        case 77:  /* AP1  */
            if (FCU_AP1)
            {
                SendDLX("L077");
            }
            else
            {
                SendDLX("L177");
            }
            FCU_AP1 = !FCU_AP1;
            break;
            
        case 78:  /* AP2  */
            if (FCU_AP2)
            {
                SendDLX("L078");
            }
            else
            {
                SendDLX("L178");
            }
            FCU_AP2 = !FCU_AP2;
            break;
            
        case 79:  /* ATHR  */
            if (FCU_ATHR)
            {
                SendDLX("L079");
            }
            else
            {
                SendDLX("L179");
            }
            FCU_ATHR = !FCU_ATHR;
            break;
            
        case 80:  /* EXPED  */
            if (FCU_EXPED)
            {
                SendDLX("L080");
            }
            else
            {
                SendDLX("L180");
            }
            FCU_EXPED = !FCU_EXPED;
            break;
            
        case 81:  /* Metric Alt  */
            FCU_Metric_Button = !FCU_Metric_Button;
            //if (FCU_Metric_Button)
            //{
            //    FCU_ALT = ((unsigned int) ((float) FCU_ALT * 0.3048 + 0.5) / 100) * 100;
            //}
            //else
            //{
            //    FCU_ALT = ((unsigned int) ((float) FCU_ALT * 3.28084 + 0.5) / 100) * 100;
            //}
            break;

        case 82:  /* APPR  */
            if (FCU_APPR)
            {
                SendDLX("L082");
            }
            else
            {
                SendDLX("L182");
            }
            FCU_APPR = !FCU_APPR;
            break;
    }
}

/* --------------------------------------------------------------------- */
static void DecodeRCode(char RString[])
{
    unsigned int Code;

    Code = Bin(RString[1]) * 100 + Bin(RString[2]) * 10 + Bin(RString[3]);
    
    switch (Code)
    {
        case 4:  /* LH Baro Hg  */
            if (!LHS_BaroHg)
            {
                LHS_Baro = (unsigned int) ((float) LHS_Baro * 2992.0 / 1013.0 + 0.5);
            }
            LHS_BaroHg = true;
            SendCard("D001", LHS_Baro, 4);
            break;
            
        case 5:  /* LH Baro Hpa  */
            if (LHS_BaroHg)
            {
                LHS_Baro = (unsigned int) ((float) LHS_Baro * 1013.0 / 2992.0 + 0.5);
            }
            LHS_BaroHg = false;
            SendCard("D001", LHS_Baro, 4);
            break;
            
        case 6:  /* Mode ILS  */
            LHS_ModeSelector = NavDefn_ModeILS;
            break;
            
        case 7:  /* Mode VOR  */
            LHS_ModeSelector = NavDefn_ModeVOR;
            break;
            
        case 8:  /* Mode NAV  */
            LHS_ModeSelector = NavDefn_ModeNAV;
            break;
            
        case 9:  /* Mode ARC  */
            LHS_ModeSelector = NavDefn_ModeARC;
            break;
            
        case 10:  /* Mode PLAN  */
            LHS_ModeSelector = NavDefn_ModePLAN;
            break;
            
        case 12:  /* Range 10  */
            LHS_RangeSelector = 10;
            break;
            
        case 13:  /* Range 20  */
            LHS_RangeSelector = 20;
            break;
            
        case 14:  /* Range 40  */
            LHS_RangeSelector = 40;
            break;
            
        case 15:  /* Range 80  */
            LHS_RangeSelector = 80;
            break;
            
        case 16:  /* Range 160  */
            LHS_RangeSelector = 160;
            break;
            
        case 17:  /* Range 320  */
            LHS_RangeSelector = 320;
            break;
            
        case 18:  /* ADF1  */
            LHS_NavSwitch1 = NavDefn_NavADF;
            break;
            
        case 19:  /* OFF1  */
            LHS_NavSwitch1 = NavDefn_NavOFF;
            break;
            
        case 20:  /* VOR1  */
            LHS_NavSwitch1 = NavDefn_NavVOR;
            break;
            
        case 21:  /* ADF2  */
            LHS_NavSwitch2 = NavDefn_NavADF;
            break;
            
        case 22:  /* OFF2  */
            LHS_NavSwitch2 = NavDefn_NavOFF;
            break;
            
        case 23:  /* VOR2  */
            LHS_NavSwitch2 = NavDefn_NavVOR;
            break;
            
        case 34:  /* RH Baro Hg  */
            if (!RHS_BaroHg)
            {
                RHS_Baro = (unsigned int) ((float) RHS_Baro * 2992.0 / 1013.0 + 0.5);
            }
            RHS_BaroHg = true;
            SendCard("D031", RHS_Baro, 4);
            break;
            
        case 35:  /* RH Baro Hpa  */
            if (RHS_BaroHg)
            {
                RHS_Baro = (unsigned int) ((float) RHS_Baro * 1013.0 / 2992.0);
            }
            RHS_BaroHg = false;
            SendCard("D031", RHS_Baro, 4);
            break;
            
        case 36:  /* NAV/ILS  */
            RHS_ModeSelector = NavDefn_ModeILS;
            break;
            
        case 37:  /* NAV/VOR  */
            RHS_ModeSelector = NavDefn_ModeVOR;
            break;
            
        case 38:  /* NAV/NAV  */
            RHS_ModeSelector = NavDefn_ModeNAV;
            break;
            
        case 39:  /* NAV/ARC  */
            RHS_ModeSelector = NavDefn_ModeARC;
            break;
            
        case 40:  /* NAV/PLAN  */
            RHS_ModeSelector = NavDefn_ModePLAN;
            break;
            
        case 42:  /* Range 10  */
            RHS_RangeSelector = 10;
            break;
            
        case 43:  /* Range 20  */
            RHS_RangeSelector = 20;
            break;
            
        case 44:  /* Range 40  */
            RHS_RangeSelector = 40;
            break;
            
        case 45:  /* Range 80  */
            RHS_RangeSelector = 80;
            break;
            
        case 46:  /* Range 160  */
            RHS_RangeSelector = 160;
            break;
            
        case 47:  /* Range 320  */
            RHS_RangeSelector = 320;
            break;
            
        case 48:  /* ADF1  */
            RHS_NavSwitch1 = NavDefn_NavADF;
            break;
            
        case 49:  /* OFF1  */
            RHS_NavSwitch1 = NavDefn_NavOFF;
            break;
            
        case 50:  /* VOR1  */
            RHS_NavSwitch1 = NavDefn_NavVOR;
            break;
            
        case 51:  /* ADF2  */
            RHS_NavSwitch2 = NavDefn_NavADF;
            break;
            
        case 52:  /* OFF2  */
            RHS_NavSwitch2 = NavDefn_NavOFF;
            break;
            
        case 53:  /* VOR2  */
            RHS_NavSwitch2 = NavDefn_NavVOR;
            break;
            
        case 70:  /* A/R 100 */
            FCU_ALT_Range = 100;
            break;
            
        case 71:  /* A/R 1000 */
            FCU_ALT_Range = 1000;
            break;
    }
}

/* --------------------------------------------------------------------- */
void FCU_UpdateFCU(int leftb, int middleb, int rightb, int x, int y)
{
   return; /* for compatability with EFS */
}

/* --------------------------------------------------------------------- */
void FCU_RestoreFCU(IosDefn_RestoreVectorRecord v)
{
    FCU_FD              = v.FCU_FD;
    FCU_LS              = v.FCU_LS;
    FCU_LOC             = v.FCU_LOC;
    FCU_AP1             = v.FCU_AP1;
    FCU_AP2             = v.FCU_AP2;
    FCU_ATHR            = v.FCU_ATHR;
    FCU_EXPED           = v.FCU_EXPED;
    FCU_APPR            = v.FCU_APPR;
    FCU_SPD_MACH_Button = v.FCU_SPD_MACH;
    FCU_HDG_TRK_Button  = v.FCU_HDG_TRK;
    FCU_BaroKnob        = v.FCU_BaroPressure;
    FCU_HDG             = (unsigned int) v.FCU_HDG;
    FCU_ALT             = (unsigned int) v.FCU_ALT;
    FCU_SPD             = (unsigned int) v.FCU_SPD;
    FCU_VS              = (int) v.FCU_VS;
    LHS_BaroHg          = v.FCU_BaroHg;
    LHS_BaroSTD         = v.FCU_BaroKnob;
	FCU_Metric_Button   = v.FCU_Metric_Button;
    FCU_HDGKnob         = v.FCU_HDGKnob;
    FCU_ALTKnob         = v.FCU_ALTKnob;
    FCU_SPDKnob         = v.FCU_SPDKnob;
    FCU_VSKnob          = v.FCU_VSKnob;
    
    if (LHS_BaroSTD)
	{
        SendDLX("L003");  /* BARO legend off */
        SendDLX("L102");  /* STD legend on */
    }
	
    if (FCU_FD)
    {
        SendDLX("L126");  /* LHS FD lamp ON */
    }
    else
    {
        SendDLX("L026");  /* LHS FD lamp OFF */
    }
    if (FCU_LS)
    {
        SendDLX("L127");  /* LHS LS lamp ON */
    }
    else
    {
        SendDLX("L027");  /* LHS LS lamp OFF */
    }
    if (FCU_LOC)
    {
        SendDLX("L175");  /* LOC lamp ON */
    }
    else
    {
        SendDLX("L075");  /* LOC lamp OFF */
    }
    if (FCU_AP1)
    {
        SendDLX("L177");  /* AP1 lamp ON */
    }
    else
    {
        SendDLX("L077");  /* AP1 lamp OFF */
    }
    if (FCU_AP2)
    {
        SendDLX("L178");  /* AP2 lamp ON */
    }
    else
    {
        SendDLX("L078");  /* AP2 lamp OFF */
    }
    if (FCU_LOC)
    {
        SendDLX("L175");  /* LOC lamp ON */
    }
    else
    {
        SendDLX("L075");  /* LOC lamp OFF */
    }
    if (FCU_ATHR)
    {
        SendDLX("L179");  /* ATHR lamp ON */
    }
    else
    {
        SendDLX("L079");  /* ATHR lamp OFF */
    }
    if (FCU_EXPED)
    {
        SendDLX("L180");  /* EXPED lamp ON */
    }
    else
    {
        SendDLX("L080");  /* EXPED lamp OFF */
    }
    if (FCU_APPR)
    {
        SendDLX("L182");  /* APPR lamp ON */
    }
    else
    {
        SendDLX("L082");  /* APPR lamp OFF */
    }
    if (FCU_HDGKnob == NavDefn_Pushed)
    {
        SendDLX("L176");  /* HDG lamp ON */
    }
    else
    {
        SendDLX("L076");  /* HDG lamp OFF */
    }
    if (FCU_ALTKnob == NavDefn_Pushed)
    {
        SendDLX("L181");  /* ALT lamp ON */
    }
    else
    {
        SendDLX("L081");  /* ALT lamp OFF */
    }
    if (FCU_SPDKnob == NavDefn_Pushed)
    {
        SendDLX("L183");  /* SPD lamp ON */
    }
    else
    {
        SendDLX("L081");  /* SPD lamp OFF */
    }
    if (FCU_SPD_MACH_Button)
    {
        SendDLX("L161");
        SendDLX("L062");
        SendCard("D060", FCU_SPD, 3);
    }
    else
    {
        SendDLX("L061");
        SendDLX("L162");
        SendCard("D060", FCU_SPD, 3);
    }
    if (FCU_HDG_TRK_Button)
    {
        SendDLX("L165");
        SendDLX("L066");
        SendCard("D064", FCU_HDG, 3);
        SendInt("D072", FCU_VS, 2);
    }
    else
    {
        SendDLX("L065");
        SendDLX("L166");
        SendCard("D064", FCU_TRK, 3);
        SendInt("D072", FCU_FPA, 2);
    }
    SendCard("D067", FCU_ALT, 5);
}

/* --------------------------------------------------------------------- */
void BEGIN_FCU()
{
    FCU_FD = false;
    FCU_LS = false;
    FCU_LOC = false;
    FCU_AP1 = false;
    FCU_AP2 = false;
    FCU_ATHR = false;
    FCU_EXPED = false;
    FCU_APPR = false;
    FCU_SPD_MACH_Button = true;  /* default to SPD for now */
    FCU_HDG_TRK_Button = true;   /* default to HDG for now */
    FCU_BaroPressure = 1013;     /* default to 1013 for now */
    FCU_BaroHg = false;
    FCU_BaroSTD = false;
    FCU_HDG = 0;
    FCU_TRK = 0;
    FCU_ALT = 0;
    FCU_SPD = 0;
    FCU_VS = 0;
    FCU_FPA = 0;
    FCU_Metric_Button = false;
    FCU_HDGKnob = NavDefn_Middle;
    FCU_ALTKnob = NavDefn_Middle;
    FCU_SPDKnob = NavDefn_Middle;
    FCU_VSKnob = NavDefn_Middle;
    FCU_ALT_Range = 1000;

    FCU_NavSwitch1 = NavDefn_NavOFF;
    FCU_NavSwitch2 = NavDefn_NavOFF;
    FCU_ModeSelector = NavDefn_ModeILS;
    FCU_DataMode = NavDefn_DataOFF;
    FCU_RangeSelector = 1000;
    
    LHS_FD = false;
    LHS_LS = false;
    LHS_Baro = 1013;
    LHS_BaroHg = false;
    LHS_BaroSTD = false;
    LHS_CSTR = false;
    LHS_WPT = false;
    LHS_VORD = false;
    LHS_NDB = false;
    LHS_ARPT = false;
    Old_LHS_Baro = 1013;

    RHS_FD = false;
    RHS_LS = false;
    RHS_Baro = 1013;
    RHS_BaroHg = false;
    RHS_BaroSTD = false;
    RHS_CSTR = false;
    RHS_WPT = false;
    RHS_VORD = false;
    RHS_NDB = false;
    RHS_ARPT = false;
    Old_RHS_Baro = 1013;
    
    FCU_InitialiseFCU();
}
