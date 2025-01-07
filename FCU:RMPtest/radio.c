/*
    Linux version
    DJA 16 Jan 2024
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>

#include <SIM/navdefn.h>

#include "radio.h"
#include "rs232lib.h"

NavDefn_RadioPanel Radio_Radios[2];

unsigned int  RmpStatus;
unsigned int  CRCErrors;
unsigned int  RmpErrors;
unsigned char ComButtons;
unsigned char NavButtons;
bool          PowerSwitch[2];

#define CH0	'0'

#define SwAM      0x01
#define SwHF2     0x02
#define SwSEL     0x04
#define SwHF1     0x08
#define SwVHF3    0x10
#define SwVHF2    0x20
#define SwVHF1    0x40

#define SwPWR     0x01
#define SwBFO     0x02
#define SwADF     0x04
#define SwMLS     0x08
#define SwILS     0x10
#define SwVOR     0x20
#define SwNAV     0x40

const uint16_t CrcTab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067, 
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0 };

typedef struct 
{
    unsigned char RmpData[17];
    unsigned char CRC16[4];
    unsigned char EndByte;
} TxPktType;

typedef struct 
{
    unsigned char RmpData[9];
    unsigned char CRC16[4];
    unsigned char EndByte;
} RxPktType;

static bool            CrsMode;
static unsigned int    RMP_ticks;

static int             InnerFreq;
static int             OuterFreq;
static int             InnerKnob;
static int             OuterKnob;
static int             OldInnerKnob;
static int             OldOuterKnob;
static unsigned int    NAVSwitch_ticks;
static unsigned int    BFOSwitch_ticks;
static unsigned int    XFRSwitch_ticks;

static unsigned int    SelfTestTimeout;
 
static TxPktType       TxPkt;
static RxPktType       RxPkt;

static int             port_handle;
static pthread_t       port_rxthread;
static pthread_t       port_txthread;
static pthread_mutex_t rxbuf_mutex;
static pthread_mutex_t txbuf_mutex;
static sem_t           rxsem;
static sem_t           txsem;
static bool            rxpktavailable;

static uint16_t        CRC(unsigned char buff[], unsigned int buffsize);
static void            RmpFormCommand();
static void            SetVHF(unsigned int ActiveFreq, unsigned int StandbyFreq);
static void            SetVORILS(unsigned int ActiveFreq, unsigned int StandbyFreq);
static void            SetCRS(unsigned int ActiveFreq, unsigned int Crs);
static void            SetADF(unsigned int ActiveFreq, unsigned int StandbyFreq);
static void            SetBlanks();
static void            SetSelfTest();
static void            SetHF(unsigned int ActiveFreq, unsigned int StandbyFreq);
static void            SetAM(unsigned int ActiveFreq, unsigned int StandbyFreq);
static void            RmpDecode();
static void            SwapFrequency(NavDefn_RadioKnob *r);
static unsigned int    SetNavFrequency(unsigned int Frequency, unsigned int Sw, unsigned int Outerknob, unsigned int InnerKnob);
static unsigned int    SetComFrequency(unsigned int Frequency, unsigned int Sw, unsigned int Outerknob, unsigned int InnerKnob);
static unsigned int    SetCrs(unsigned int f, unsigned int InnerKnob);

static void            *serial_input();
static void            *serial_output();
static void            Initialise_RMP();
static void            Start_SerialPort();
static unsigned int    SetKnob(unsigned int f, unsigned int Outerknob, unsigned int InnerKnob, unsigned int OldOuterknob, unsigned int OldInnerKnob,
                               unsigned int DigitsMin, unsigned int DigitsMax, unsigned int DigitsInc, unsigned int Decade);

/* --------------------------------------------------------------- */
void Radio_StopRMP()
{
return; // RMO disbled for now 
    pthread_cancel(port_rxthread);
    pthread_cancel(port_txthread);
    RS232_Close(port_handle);
}

/* --------------------------------------------------------------- */
static void Start_SerialPort() /* create rx and tx threads, initialise semaphores, initialise mutexes */
{
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

/* --------------------------------------------------------------- */
void *serial_input()
{
    RxPktType tPkt;
	
    while (1)  /* thread reads a block from the RMP indefinitely */
    {
        int dwBytesRead = 0;

        sem_wait(&rxsem);

        rxpktavailable = false;

        dwBytesRead = RS232_Gets(port_handle, (char *) &tPkt, 14);
        if (dwBytesRead < 0)
        {
            printf("serial port read error 1\n");
            exit(1);
        }

        if (dwBytesRead != 14)
        {
            RmpErrors += 1;
        }
        pthread_mutex_lock(&rxbuf_mutex);
		memcpy(&RxPkt, &tPkt, dwBytesRead);
        pthread_mutex_unlock(&rxbuf_mutex);
        rxpktavailable = true;
    }
	return NULL;  /* can't happen */
}

/* --------------------------------------------------------------- */
void *serial_output()
{
    TxPktType tPkt;

    while (1)  /* thread reads a block from the RMP indefinitely */
    {
        int dwBytesWritten = 0;

        sem_wait(&txsem);
        pthread_mutex_lock(&txbuf_mutex);
		memcpy(&tPkt, &TxPkt, sizeof(TxPkt));
        pthread_mutex_unlock(&txbuf_mutex);

        dwBytesWritten = RS232_Puts(port_handle, (char *) &tPkt, 22);
        if (dwBytesWritten < 0)
        {
            printf("Serial port write error 1\n");
            exit(1);
        }
        if (dwBytesWritten != 22)
        {
            printf("Serial port write error 2\n");
            exit(1);
        }
        
        sem_post(&rxsem);
    }
}

/* --------------------------------------------------------------- */
static uint16_t CRC(unsigned char buff[], unsigned int buffsize)
{
    uint16_t     x = 0;
    unsigned int i;
    
    for (i = 0; i < buffsize; i += 1) 
    {
        x = (uint16_t) ((x << 8) ^ CrcTab[(x >> 8) ^ buff[i]]);
    }
    return x;
}

/* --------------------------------------------------------------- */
static void RmpFormCommand()
{
    uint16_t CRCValue;

    TxPktType *w = &TxPkt;

    w->RmpData[0] = 0x80;

    if (ComButtons != 0)
	{
		switch (ComButtons) 
		{
			case SwVHF1:
				SetVHF(Radio_Radios[0].ComVHF1.Active, Radio_Radios[0].ComVHF1.Stby);
				break;
				
			case SwVHF2:
				SetVHF(Radio_Radios[0].ComVHF2.Active, Radio_Radios[0].ComVHF2.Stby);
				break;
				
			case SwVHF3:
				SetVHF(Radio_Radios[0].ComVHF3.Active, Radio_Radios[0].ComVHF3.Stby);
				break;
				
			case SwHF1:
				SetHF(Radio_Radios[0].ComHF1.Active, Radio_Radios[0].ComHF1.Stby);
				break;
				
			case SwHF2:
				SetHF(Radio_Radios[0].ComHF2.Active, Radio_Radios[0].ComHF2.Stby);
				break;
				
			case SwAM:
				SetAM(Radio_Radios[0].ComAM.Active, Radio_Radios[0].ComAM.Stby);
				break;
				
			default:
				SetBlanks();
				break;
		}
    }
    
	if (NavButtons != 0)
	{
		switch (NavButtons & (SwVOR | SwILS | SwMLS | SwADF)) 
		{
			case SwVOR:
				if (CrsMode) 
				{
					SetCRS(Radio_Radios[0].NavVOR.Active, Radio_Radios[0].CrsKnob);
				} 
				else 
				{
					SetVORILS(Radio_Radios[0].NavVOR.Active, Radio_Radios[0].NavVOR.Stby);
				}
				break;
				
			case SwILS:
				if (CrsMode) 
				{
					SetCRS(Radio_Radios[0].NavILS.Active, Radio_Radios[0].CrsKnob);
				} 
				else 
				{
					SetVORILS(Radio_Radios[0].NavILS.Active, Radio_Radios[0].NavILS.Stby);
				}
				break;
				
			case SwADF:
				SetADF(Radio_Radios[0].NavADF.Active, Radio_Radios[0].NavADF.Stby);
				break;
				
			default:
				SetBlanks();
				break;
		}
    }
    
    w->RmpData[15] = ComButtons;
    w->RmpData[16] = NavButtons;

    if (!PowerSwitch[0])
	{
	    SetBlanks();
		SelfTestTimeout = 0;
	}
	if (PowerSwitch[0] & (SelfTestTimeout < 50))
	{
	    SetSelfTest();
		SelfTestTimeout += 1;
	}
	
    CRCValue = CRC(w->RmpData, 17);
    w->CRC16[0] = (unsigned char) (CRCValue / 4096);
    w->CRC16[1] = (unsigned char) ((CRCValue / 256) % 16);
    w->CRC16[2] = (unsigned char) ((CRCValue / 16) % 16);
    w->CRC16[3] = (unsigned char) (CRCValue % 16);

    w->EndByte = 0xFF;
}

/* --------------------------------------------------------------- */
static void SetVHF(unsigned int ActiveFreq, unsigned int StandbyFreq)
{
    unsigned int f;
    TxPktType    *w = &TxPkt;

    f = ActiveFreq / 1000;
    w->RmpData[1] = (unsigned char) (CH0 + f / 100);
    w->RmpData[2] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[3] = (unsigned char) (CH0 + f % 10);
    f = ActiveFreq % 1000;
    w->RmpData[4] = (unsigned char) (CH0 + f / 100);
    w->RmpData[5] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[6] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq / 1000;
    w->RmpData[7] = (unsigned char) (CH0 + f / 100);
    w->RmpData[8] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[9] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq % 1000;
    w->RmpData[10] = (unsigned char) (CH0 + f / 100);
    w->RmpData[11] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[12] = (unsigned char) (CH0 + f % 10);
    w->RmpData[13] = 8;
    w->RmpData[14] = 8;
}

/* --------------------------------------------------------------- */
static void SetVORILS(unsigned int ActiveFreq, unsigned int StandbyFreq)
{
    unsigned int f;
    TxPktType *w = &TxPkt;

    f = ActiveFreq / 100;
    w->RmpData[1] = ' ';
    w->RmpData[2] = (unsigned char) (CH0 + f / 100);
    w->RmpData[3] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[4] = (unsigned char) (CH0 + f % 10);
    f = ActiveFreq % 100;
    w->RmpData[5] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[6] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq / 100;
    w->RmpData[7] = ' ';
    w->RmpData[8] = (unsigned char) (CH0 + f / 100);
    w->RmpData[9] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[10] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq % 100;
    w->RmpData[11] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[12] = (unsigned char) (CH0 + f % 10);
    w->RmpData[13] = 4;
    w->RmpData[14] = 4;
}

/* --------------------------------------------------------------- */
static void SetCRS(unsigned int ActiveFreq, unsigned int Crs)
{
    unsigned int f;
    TxPktType    *w = &TxPkt;

    f = ActiveFreq / 100;
    w->RmpData[1] = ' ';
    w->RmpData[2] = (unsigned char) (CH0 + f / 100);
    w->RmpData[3] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[4] = (unsigned char) (CH0 + f % 10);
    f = ActiveFreq % 100;
    w->RmpData[5] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[6] = (unsigned char) (CH0 + f % 10);
    w->RmpData[7] = ' ';
    w->RmpData[8] = ' ';
    w->RmpData[9] = ' ';
    f = Crs;
    w->RmpData[10] = (unsigned char) (CH0 + f / 100);
    w->RmpData[11] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[12] = (unsigned char) (CH0 + f % 10);
    w->RmpData[13] = 4;
    w->RmpData[14] = 0;
}

/* --------------------------------------------------------------- */
static void SetADF(unsigned int ActiveFreq, unsigned int StandbyFreq)
{
    unsigned int f;
    TxPktType    *w = &TxPkt;

    f = ActiveFreq / 1000;
    w->RmpData[1] = ' ';
    w->RmpData[2] = ' ';
    if (f % 10 == 0) 
    {
        w->RmpData[3] = ' ';
    } 
    else 
    {
        w->RmpData[3] = (unsigned char) (CH0 + f % 10);
    }
    f = ActiveFreq % 1000;
    w->RmpData[4] = (unsigned char) (CH0 + f / 100);
    w->RmpData[5] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[6] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq / 1000;
    w->RmpData[7] = ' ';
    w->RmpData[8] = ' ';
    if (f % 10 == 0) 
    {
        w->RmpData[9] = ' ';
    } 
    else 
    {
        w->RmpData[9] = (unsigned char) (CH0 + f % 10);
    }
    f = StandbyFreq % 1000;
    w->RmpData[10] = (unsigned char) (CH0 + f / 100);
    w->RmpData[11] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[12] = (unsigned char) (CH0 + f % 10);
    w->RmpData[13] = 2;
    w->RmpData[14] = 2;
}

/* --------------------------------------------------------------- */
static void SetBlanks()
{
    unsigned int i;
    TxPktType    *w = &TxPkt;

    for (i = 1; i <= 12; i += 1) 
    {
        w->RmpData[i] = ' ';
    }
    w->RmpData[13] = 0;
    w->RmpData[14] = 0;
    w->RmpData[15] = 0;
    w->RmpData[16] = 0;
}

/* --------------------------------------------------------------- */
static void SetSelfTest()
{
    unsigned int i;
    TxPktType    *w = &TxPkt;

    for (i = 1; i <= 12; i += 1) 
    {
        w->RmpData[i] = '8';
    }
    w->RmpData[13] = 0x7f;
    w->RmpData[14] = 0x7f;
    w->RmpData[15] = 0x7f;
    w->RmpData[16] = 0x7f;
}

/* --------------------------------------------------------------- */
static void SetHF(unsigned int ActiveFreq, unsigned int StandbyFreq)
{
    unsigned int f;
    TxPktType    *w = &TxPkt;

    f = ActiveFreq / 1000;
    w->RmpData[1] = ' ';
    if (f % 100 / 10 == 0) 
    {
        w->RmpData[2] = ' ';
    } 
    else 
    {
        w->RmpData[2] = (unsigned char) (CH0 + f % 100 / 10);
    }
    w->RmpData[3] = (unsigned char) (CH0 + f % 10);
    f = ActiveFreq % 1000;
    w->RmpData[4] = (unsigned char) (CH0 + f / 100);
    w->RmpData[5] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[6] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq / 1000;
    w->RmpData[7] = ' ';
    if (f % 100 / 10 == 0) 
    {
        w->RmpData[8] = ' ';
    } 
    else 
    {
        w->RmpData[8] = (unsigned char) (CH0 + f % 100 / 10);
    }
    w->RmpData[9] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq % 1000;
    w->RmpData[10] = (unsigned char) (CH0 + f / 100);
    w->RmpData[11] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[12] = (unsigned char) (CH0 + f % 10);
    w->RmpData[13] = 8;
    w->RmpData[14] = 8;
}

/* --------------------------------------------------------------- */
static void SetAM(unsigned int ActiveFreq, unsigned int StandbyFreq)
{
    unsigned int f;
    TxPktType    *w = &TxPkt;

    f = ActiveFreq / 1000;
    w->RmpData[1] = (unsigned char) (CH0 + f / 100);
    w->RmpData[2] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[3] = (unsigned char) (CH0 + f % 10);
    f = ActiveFreq % 1000;
    w->RmpData[4] = (unsigned char) (CH0 + f / 100);
    w->RmpData[5] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[6] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq / 1000;
    w->RmpData[7] = (unsigned char) (CH0 + f / 100);
    w->RmpData[8] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[9] = (unsigned char) (CH0 + f % 10);
    f = StandbyFreq % 1000;
    w->RmpData[10] = (unsigned char) (CH0 + f / 100);
    w->RmpData[11] = (unsigned char) (CH0 + f % 100 / 10);
    w->RmpData[12] = (unsigned char) (CH0 + f % 10);
    w->RmpData[13] = 8;
    w->RmpData[14] = 8;
}

/* --------------------------------------------------------------- */
static void RmpDecode()
{
    unsigned int  i;
    uint16_t      CRCValue;
    bool          CRCOK;
    unsigned char tbuff[11];
    RxPktType     *w1 = &RxPkt;
    unsigned int  t;
    unsigned int  NAVSwitch;
    unsigned int  BFOSwitch;
	unsigned int  XFRSwitch;
	unsigned int  Divisor;
	
    RMP_ticks += 1;
	
    if (w1->RmpData[0] != 0x80 || w1->EndByte != 0xFF || w1->RmpData[8] != 0) 
    {
        RmpErrors += 1;
    }
    
    RmpStatus = (unsigned int) w1->RmpData[8];
    
    CRCOK = true;
    for (i=0; i<=7; i+=1) 
    {
        tbuff[i] = w1->RmpData[i + 1];
    }
    CRCValue = CRC(tbuff, 8);
    
    Divisor = 4096;
    for (i=0; i<=3; i+=1) 
    {
        if (w1->CRC16[i] != (unsigned char) (CRCValue / Divisor % 16)) 
        {
            CRCOK = false;
        }
        Divisor = Divisor / 16;
    }
    
    if (!CRCOK) 
    {
        CRCErrors += 1;
    }
    
	Radio_Radios[0].Mode = ((w1->RmpData[5]) << 8) | (w1->RmpData[6]) | ((w1->RmpData[7] & 0x01) << 7);
	/* Note: The radio buttons are saved in .Mode for the LFS - this is only used for diagnostics (rmptest) */

    t = (unsigned int) (w1->RmpData[5]);
    
    if (t != 0)  /* COM button pressed */ 
    {
	    if (!(t & SwSEL))  /* ignore SEL */
		{
            ComButtons = t;
            NavButtons = 0;
        }
	}
    
    t = (unsigned int) (w1->RmpData[6]);
    
    PowerSwitch[0] = (t & 0x01);
    t &= ~SwPWR;  /* clear power switch bit */

    NAVSwitch = t & SwNAV;
    if ((NAVSwitch == SwNAV) && (RMP_ticks > (NAVSwitch_ticks + 5)))
	{
	    NavButtons ^= SwNAV;  /* toggle NAV switch */
    	NAVSwitch_ticks = RMP_ticks;
   	}
	t &= ~SwNAV;  /* clear NAV bit */
	
    BFOSwitch = t & SwBFO;
    if ((BFOSwitch == SwBFO) && (RMP_ticks > (BFOSwitch_ticks + 5)))
	{
	    NavButtons ^= SwBFO;  /* toggle BFO switch */
    	BFOSwitch_ticks = RMP_ticks;
   	}
	t &= ~SwBFO;  /* clear BFO bit */
	
	if (t != 0)
    {
		{
    	    NavButtons = (NavButtons & SwNAV) | t;
        }
		if (t & (SwVOR | SwILS | SwMLS))
		{
		    CrsMode = true;
		}
		ComButtons = 0;
    }
	
    t = (unsigned int) (w1->RmpData[7]);
	XFRSwitch = t & 0x01;
    if ((XFRSwitch == 0x01) && (RMP_ticks > (XFRSwitch_ticks + 5))) 
    {
	    XFRSwitch_ticks = RMP_ticks;
		
        switch (ComButtons) 
        {
            case SwVHF1:
                SwapFrequency(&Radio_Radios[0].ComVHF1);
                break;
				
            case SwVHF2:
                SwapFrequency(&Radio_Radios[0].ComVHF2);
                break;
				
			case SwVHF3:
				SwapFrequency(&Radio_Radios[0].ComVHF3);
				break;
				
			case SwHF1:
				SwapFrequency(&Radio_Radios[0].ComHF1);
				break;
				
			case SwHF2:
				SwapFrequency(&Radio_Radios[0].ComHF2);
				break;
				
			case SwAM:
				SwapFrequency(&Radio_Radios[0].ComAM);
				break;
				
			default:
				break;
		}
    
		switch (NavButtons & (SwVOR | SwILS | SwMLS | SwADF)) 
		{
			case SwVOR:
				SwapFrequency(&Radio_Radios[0].NavVOR);
				break;
				
			case SwILS:
				SwapFrequency(&Radio_Radios[0].NavILS);
				break;
				
			case SwMLS:
				SwapFrequency(&Radio_Radios[0].NavMLS);
				break;
				
			case SwADF:
				SwapFrequency(&Radio_Radios[0].NavADF);
				break;
				
			default:
			    break;
		}
		CrsMode = false;
    } 

    InnerKnob = (unsigned int) (w1->RmpData[1]) * 16 + (unsigned int) (w1->RmpData[2]);
    OuterKnob = (unsigned int) (w1->RmpData[3]) * 16 + (unsigned int) (w1->RmpData[4]);

    switch (ComButtons) 
    {
        case SwVHF1:
            Radio_Radios[0].ComVHF1.Stby = SetComFrequency(Radio_Radios[0].ComVHF1.Stby, SwVHF1, OuterKnob, InnerKnob);
            break;
			
        case SwVHF2:
            Radio_Radios[0].ComVHF2.Stby = SetComFrequency(Radio_Radios[0].ComVHF2.Stby, SwVHF2, OuterKnob, InnerKnob);
            break;
			
        case SwVHF3:
            Radio_Radios[0].ComVHF3.Stby = SetComFrequency(Radio_Radios[0].ComVHF3.Stby, SwVHF3, OuterKnob, InnerKnob);
            break;
			
        case SwHF1:
            Radio_Radios[0].ComHF1.Stby = SetComFrequency(Radio_Radios[0].ComHF1.Stby, SwHF1, OuterKnob, InnerKnob);
            break;
			
        case SwHF2:
            Radio_Radios[0].ComHF2.Stby = SetComFrequency(Radio_Radios[0].ComHF2.Stby, SwHF2, OuterKnob, InnerKnob);
            break;
			
        case SwAM:
            Radio_Radios[0].ComAM.Stby = SetComFrequency(Radio_Radios[0].ComAM.Stby, SwAM, OuterKnob, InnerKnob);
            break;
			
        default:
            break;
    }
    
    switch (NavButtons & (SwVOR | SwILS | SwMLS | SwADF)) 
    {
        case SwVOR:
            if (CrsMode) 
            {
                Radio_Radios[0].CrsKnob = SetCrs(Radio_Radios[0].CrsKnob, InnerKnob);
            }
			else
			{
                Radio_Radios[0].NavVOR.Stby = SetNavFrequency(Radio_Radios[0].NavVOR.Stby, SwVOR, OuterKnob, InnerKnob);
            } 
            break;
			
        case SwILS:
            if (CrsMode) 
            {
                Radio_Radios[0].CrsKnob = SetCrs(Radio_Radios[0].CrsKnob, InnerKnob);
            }
			else
			{
                Radio_Radios[0].NavILS.Stby = SetNavFrequency(Radio_Radios[0].NavILS.Stby, SwILS, OuterKnob, InnerKnob);
            }
			break;
			
        case SwMLS:
            if (CrsMode) 
            {
                Radio_Radios[0].CrsKnob = SetCrs(Radio_Radios[0].CrsKnob, InnerKnob);
            }
			else
			{
                Radio_Radios[0].NavMLS.Stby = SetNavFrequency(Radio_Radios[0].NavMLS.Stby, SwMLS, OuterKnob, InnerKnob);
            }
			break;
			
        case SwADF:
            Radio_Radios[0].NavADF.Stby = SetNavFrequency(Radio_Radios[0].NavADF.Stby, SwADF, OuterKnob, InnerKnob);
            break;
			
        default:
            break;
    }
	OldOuterKnob = OuterKnob;
	OldInnerKnob = InnerKnob;
}

/* --------------------------------------------------------------- */
static void SwapFrequency(NavDefn_RadioKnob *r)
{
    unsigned int t;

    t = r->Active;
    r->Active = r->Stby;
    r->Stby = t;
}

/* --------------------------------------------------------------- */
static void Initialise_RMP()
{
    unsigned int i;
    TxPktType *w = &TxPkt;
return; // RMP disabled for now

    RS232_Open("/dev/ttySC1", 115200); /* channel 2 */
    Start_SerialPort();
    
    Radio_Radios[0].ComVHF1.Active = 123455;  /* only one RMP Radios[0] */
    Radio_Radios[0].ComVHF1.Stby = 123125;
    Radio_Radios[0].ComVHF2.Active = 111225;
    Radio_Radios[0].ComVHF2.Stby = 136975;
    Radio_Radios[0].ComVHF3.Active = 120300;
    Radio_Radios[0].ComVHF3.Stby = 136975;
    Radio_Radios[0].ComHF1.Active = 31000;
    Radio_Radios[0].ComHF1.Stby = 30000;
    Radio_Radios[0].ComHF2.Active = 32000;
    Radio_Radios[0].ComHF2.Stby = 30000;
    Radio_Radios[0].ComAM.Active = 227000;
    Radio_Radios[0].ComAM.Stby = 333000;
    Radio_Radios[0].NavADF.Active = 2000;
    Radio_Radios[0].NavADF.Stby = 5000;
    Radio_Radios[0].NavVOR.Active = 11230;
    Radio_Radios[0].NavVOR.Stby = 11640;
    Radio_Radios[0].NavILS.Active = 10890;
    Radio_Radios[0].NavILS.Stby = 11170;
    Radio_Radios[0].CrsKnob = 123;
	
    ComButtons = 0;
	NavButtons = SwVOR;
	
    OuterFreq = 0;
    InnerFreq = 0;
    OldInnerKnob = 0;
	OldOuterKnob = 0;
	
    for (i = 1; i <= 16; i += 1) 
    {
        w->RmpData[i] = 0;
    }

    CrsMode = false;

    NAVSwitch_ticks = 0;
    BFOSwitch_ticks = 0;
	XFRSwitch_ticks = 0;
	SelfTestTimeout = 0;
	RMP_ticks = 0;
}

/* --------------------------------------------------- */
void Radio_SaveRMP(NavDefn_NavDataPkt *pkt)
{
    pkt->SavedRadios[0].NavVOR.Active  = Radio_Radios[0].NavVOR.Active;
    pkt->SavedRadios[0].NavVOR.Stby    = Radio_Radios[0].NavVOR.Stby;
    pkt->SavedRadios[0].NavILS.Active  = Radio_Radios[0].NavILS.Active;
    pkt->SavedRadios[0].NavILS.Stby    = Radio_Radios[0].NavILS.Stby;
    pkt->SavedRadios[0].NavADF.Active  = Radio_Radios[0].NavADF.Active;
    pkt->SavedRadios[0].NavADF.Stby    = Radio_Radios[0].NavADF.Stby;
    pkt->SavedRadios[0].ComHF1.Active  = Radio_Radios[0].ComHF1.Active;
    pkt->SavedRadios[0].ComHF1.Stby    = Radio_Radios[0].ComHF1.Stby;
    pkt->SavedRadios[0].ComHF2.Active  = Radio_Radios[0].ComHF2.Active;
    pkt->SavedRadios[0].ComHF2.Stby    = Radio_Radios[0].ComHF2.Stby;
    pkt->SavedRadios[0].ComVHF1.Active = Radio_Radios[0].ComVHF1.Active;
    pkt->SavedRadios[0].ComVHF1.Stby   = Radio_Radios[0].ComVHF1.Stby;
    pkt->SavedRadios[0].ComVHF2.Active = Radio_Radios[0].ComVHF2.Active;
    pkt->SavedRadios[0].ComVHF2.Stby   = Radio_Radios[0].ComVHF2.Stby;
    pkt->SavedRadios[0].ComVHF3.Active = Radio_Radios[0].ComVHF3.Active;
    pkt->SavedRadios[0].ComVHF3.Stby   = Radio_Radios[0].ComVHF3.Stby;
    pkt->SavedRadios[0].ComAM.Active   = Radio_Radios[0].ComAM.Active;
    pkt->SavedRadios[0].ComAM.Stby     = Radio_Radios[0].ComAM.Stby;
    pkt->SavedRadios[0].CrsKnob        = Radio_Radios[0].CrsKnob;
}

/* --------------------------------------------------------------- */
void Radio_RestoreRMP(IosDefn_RestoreVectorRecord v)
{
    Radio_Radios[0].NavVOR.Active  = v.SavedRadios[0].NavVOR.Active;
    Radio_Radios[0].NavVOR.Stby    = v.SavedRadios[0].NavVOR.Stby;
    Radio_Radios[0].NavILS.Active  = v.SavedRadios[0].NavILS.Active;
    Radio_Radios[0].NavILS.Stby    = v.SavedRadios[0].NavILS.Stby;
    Radio_Radios[0].NavADF.Active = v.SavedRadios[0].NavADF.Active;
    Radio_Radios[0].NavADF.Stby   = v.SavedRadios[0].NavADF.Stby;
    Radio_Radios[0].ComHF1.Active  = v.SavedRadios[0].ComHF1.Active;
    Radio_Radios[0].ComHF1.Stby    = v.SavedRadios[0].ComHF1.Stby;
    Radio_Radios[0].ComHF2.Active  = v.SavedRadios[0].ComHF2.Active;
    Radio_Radios[0].ComHF2.Stby    = v.SavedRadios[0].ComHF2.Stby;
    Radio_Radios[0].ComVHF1.Active = v.SavedRadios[0].ComVHF1.Active;
    Radio_Radios[0].ComVHF1.Stby   = v.SavedRadios[0].ComVHF1.Stby;
    Radio_Radios[0].ComVHF2.Active = v.SavedRadios[0].ComVHF2.Active;
    Radio_Radios[0].ComVHF2.Stby   = v.SavedRadios[0].ComVHF2.Stby;
    Radio_Radios[0].ComVHF3.Active = v.SavedRadios[0].ComVHF3.Active;
    Radio_Radios[0].ComVHF3.Stby   = v.SavedRadios[0].ComVHF3.Stby;
    Radio_Radios[0].CrsKnob        = v.SavedRadios[0].CrsKnob;

	SetVHF(Radio_Radios[0].ComVHF1.Active, Radio_Radios[0].ComVHF1.Stby);
	SetVHF(Radio_Radios[0].ComVHF2.Active, Radio_Radios[0].ComVHF2.Stby);
	SetVHF(Radio_Radios[0].ComVHF3.Active, Radio_Radios[0].ComVHF3.Stby);
	SetHF(Radio_Radios[0].ComHF1.Active, Radio_Radios[0].ComHF1.Stby);
	SetHF(Radio_Radios[0].ComHF2.Active, Radio_Radios[0].ComHF2.Stby);
	SetAM(Radio_Radios[0].ComAM.Active, Radio_Radios[0].ComAM.Stby);
	SetCRS(Radio_Radios[0].NavVOR.Active, Radio_Radios[0].CrsKnob);
	SetVORILS(Radio_Radios[0].NavVOR.Active, Radio_Radios[0].NavVOR.Stby);
	SetVORILS(Radio_Radios[0].NavILS.Active, Radio_Radios[0].NavILS.Stby);
    SetADF(Radio_Radios[0].NavADF.Active, Radio_Radios[0].NavADF.Stby);
}

/* --------------------------------------------------------------- */
void Radio_UpdateRMP(int n, int leftb, int middleb, int rightb, int x, int y)  /* mouse not used with RMP box */
{
return; // RMP disabled for now
    pthread_mutex_lock(&txbuf_mutex);
    RmpFormCommand();
    pthread_mutex_unlock(&txbuf_mutex);
    
    pthread_mutex_lock(&rxbuf_mutex);
    if (rxpktavailable)
    {
        RmpDecode();
    }
    rxpktavailable = false;
    pthread_mutex_unlock(&rxbuf_mutex);
    sem_post(&txsem);  /* activate write/read cycle */
}

/* --------------------------------------------------------------- */
static unsigned int SetKnob(unsigned int f, unsigned int Outerknob, unsigned int InnerKnob, unsigned int OldOuterknob, unsigned int OldInnerKnob,
                            unsigned int DigitsMin, unsigned int DigitsMax, unsigned int DigitsInc, unsigned int Decade)
{
	int d_inner;
	int d_outer;
	int msmin;
	int msmax;
	int lsmin;
	int lsmax;
	int msdig;
	int lsdig;
	
	msmin = DigitsMin / Decade;
	msmax = DigitsMax / Decade;
	lsmin = DigitsMin % Decade;
	lsmax = DigitsMax % Decade;
	msdig = f / Decade;
	lsdig = f % Decade;
	
    if (InnerKnob > 192 && OldInnerKnob < 64) 
    {
        d_inner = -1;
    } 
    else if (InnerKnob < 64 && OldInnerKnob > 192) 
    {
        d_inner = 1;
    }
	else if (InnerKnob > OldInnerKnob)
	{
	    d_inner = 1;
	}
	else if (InnerKnob < OldInnerKnob)
	{
	    d_inner = -1;
	}
	else
	{
	    d_inner = 0;
	}
	
    if (OuterKnob > 192 && OldOuterKnob < 64) 
    {
        d_outer = -1;
    } 
    else if (OuterKnob < 64 && OldOuterKnob > 192) 
    {
        d_outer = 1;
    }
	else if (OuterKnob > OldOuterKnob)
	{
	    d_outer = 1;
	}
	else if (OuterKnob < OldOuterKnob)
	{
	    d_outer = -1;
	}
	else
	{
	    d_outer = 0;
	}

    msdig += d_outer;
    if (msdig < msmin)
    {
	    msdig = msmin;
    }
	else if (msdig > msmax)
	{
	    msdig = msmax;
	}
    lsdig += d_inner * DigitsInc;
    if (lsdig < lsmin)
    {
	    lsdig = lsmin;
    }
	else if (lsdig > lsmax)
	{
	    lsdig = lsmax;
	}
	return msdig * Decade + lsdig; 
}

/* --------------------------------------------------------------- */
static unsigned int SetComFrequency(unsigned int f, unsigned int Sw, unsigned int Outerknob, unsigned int InnerKnob)
{
    unsigned int DigitsMin;
    unsigned int DigitsMax;
    unsigned int DigitsInc;
	unsigned int Decade;

	switch (Sw)
    {
        case SwVHF1:
        case SwVHF2:
        case SwVHF3:
            DigitsMin   = 118000;
            DigitsMax   = 136975;
            DigitsInc   = 25;
			Decade      = 1000;
            break;

        case SwHF1:
        case SwHF2:
            DigitsMin   = 2000;
            DigitsMax   = 29990;
            DigitsInc   = 10;
			Decade      = 1000;
            break;

        case SwAM:
            DigitsMin   = 225000;
            DigitsMax   = 399975;
            DigitsInc   = 25;
			Decade      = 1000;
            break;

        default:
            return 0;
            break;
    }

    return SetKnob(f, Outerknob, InnerKnob, OldOuterKnob, OldInnerKnob,
                            DigitsMin, DigitsMax, DigitsInc, Decade);
}

/* --------------------------------------------------------------- */
static unsigned int SetNavFrequency(unsigned int f, unsigned int Sw, unsigned int Outerknob, unsigned int InnerKnob)
{
    unsigned int DigitsMin;
    unsigned int DigitsMax;
    unsigned int DigitsInc;
	unsigned int Decade;
	
	switch (Sw)
    {
        case SwVOR:
            DigitsMin   = 10800;
            DigitsMax   = 11795;
            DigitsInc   = 5;
			Decade      = 100;
            break;

        case SwILS:
            DigitsMin   = 10800;
            DigitsMax   = 11195;
            DigitsInc   = 5;
			Decade      = 100;
            break;

        case SwADF:
            DigitsMin   = 1900;
            DigitsMax   = 5395;
            DigitsInc   = 5;
			Decade      = 100;
            break;

        default:
		    return 0;
            break;
    }

    return SetKnob(f, Outerknob, InnerKnob, OldOuterKnob, OldInnerKnob,
                            DigitsMin, DigitsMax, DigitsInc, Decade);
}

/* --------------------------------------------------------------- */
static unsigned int SetCrs(unsigned int f, unsigned int InnerKnob)
{
    return SetKnob(f, 0, InnerKnob, 0, OldInnerKnob, 1, 360, 1, 1000);
}

/* --------------------------------------------------------------- */
void BEGIN_Radio()
{
    RmpErrors = 0;
    CRCErrors = 0;
    RmpStatus = 0;

    Initialise_RMP();
}
