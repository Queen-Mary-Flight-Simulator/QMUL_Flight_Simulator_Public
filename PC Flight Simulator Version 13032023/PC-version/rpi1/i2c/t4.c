/* DAC simple tests */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

#define LED_ADR 0x24  /* A5001 LEDS 8-bit reg at 0x21 */
#define DAC_ADR 0x60

#define RUDDER   0
#define AIRSPEED 1
#define ELEVATOR 2
#define AILERON  3

struct timeval             t;
int                        t1sec;
int                        t1usec;
int                        t2sec;
int                        t2usec;
unsigned int               Timer1;
unsigned int               Timer2;
struct timeval             frametime;

int                        i2c;
unsigned char              buf[3];
struct i2c_rdwr_ioctl_data packets;
struct i2c_msg             messages[1];

void DAC(int r, int s, int e, int a);
void Delay(int n);
void LEDs(int x);

/* ------------------------------------------- */
int main()
{
    int de;
	int da;
	int dr;
	int ds;
	
    i2c = open("/dev/i2c-1", O_RDWR);
    if (i2c < 0)
    {
        printf("unable to open i2c as a file\n");
        exit(1);
    }

    buf[0] = 0;  /* write 0 to reg 0 (set dir to output) */
    buf[1] = 0;

    messages[0].addr = LED_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to write to dir register\n");
        exit(1);
    }
     
	gettimeofday(&t, NULL);
	t1sec = t.tv_sec;
	t1usec = t.tv_usec;
	
    gettimeofday(&frametime, NULL);
    Timer1 = frametime.tv_usec / 20000L;

	printf("Centre test starting\n");    /* check stick self centres correctly */
	LEDs(1);
	
	de = 2048;
	dr = 2048;
	da = 2048;
	for (ds=2048; ds<=3000; ds+=4)       /* 250 steps @ 20ms = 12.5s */
	{
	    DAC(dr, ds, de, da);
		Delay(20);
	}
	
	printf("Elevator test starting\n");  /* check elevator  */
	LEDs(2);
	
	de = 2048;
	dr = 2048;
	da = 2048;
	ds = 3000;
	for (de=2048; de<=4095; de+=5)  /* 400 steps @ 20ms = 8s */
	{
	    DAC(dr, ds, de, da);
		Delay(20);
	}
	for (de=4095; de>=0; de-=10)    /* 400 steps @ 20ms 8s */
	{
	    DAC(dr, ds, de, da);
		Delay(20);
	}
	for (de=0; de<=2048; de+=5)    /* 400 steps @ 20ms = 8s */
	{
	    DAC(dr, ds, de, da);
		Delay(20);
	}
	
	printf("Aileron test starting\n");  /* check aileron  */
	LEDs(3);
	
	de = 2048;
	dr = 2048;
	da = 2048;
	ds = 3000;
	for (da=2048; da<=4095; da+=5)      /* 400 steps @ 20ms = 8s */
	{
	    DAC(dr, ds, da, da);
		Delay(20);
	}
	for (da=4095; da>=0; da-=10)         /* 400 steps @ 0ms = 8s */
	{
	    DAC(dr, ds, da, da);
		Delay(20);
	}
	for (da=0; da<=2048; da+=5)         /* 400 steps @20ms = 8s */
	{
	    DAC(dr, ds, da, da);
		Delay(20);
	}
	
	printf("Rudder test starting\n");  /* check rudder  */
	LEDs(4);
	
	de = 2048;
	dr = 2048;
	da = 2048;
	ds = 3000;
	for (dr=2048; dr<=4095; dr+=5)  /* 400 steps @ 20ms = 8s */
	{
	    DAC(dr, ds, dr, dr);
		Delay(20);
	}
	for (dr=4095; dr>=0; dr-=10)     /* 400 steps @ 20ms = 8s */
	{
	    DAC(dr, ds, dr, dr);
		Delay(20);
	}
	for (dr=0; dr<=2048; dr+=5)     /* 400 steps @ 20ms = 8s */
	{
	    DAC(dr, ds, dr, dr);
		Delay(20);
	}
	
	printf("Shutdown\n");  /* remove power from stick over 5s */
	LEDs(0x55);
	
	de = 2048;
	dr = 2048;
	da = 2048;
	for (ds=3000; ds>=2048; ds-=5)  /* 200 steps @ 5ms = 1s */
	{
	    DAC(dr, ds, de, da);
		Delay(20);
	}
	
	DAC(dr, 2048, de, da);
	
	printf("Test complete\n");
	LEDs(0xAA);
	
    return 0;
}

void DAC(int r, int s, int e, int a)
{
    unsigned char buf[9];
    int           val;

    val = 4095 - r;
    buf[0] = (unsigned char) (0x50);  /* select DAC channel */
    buf[1] = (unsigned char) ((val >> 8) | 0x90);
    buf[2] = (unsigned char) val & 0xff;

    val = 4095 - s;
    buf[3] = (unsigned char) ((val >> 8) | 0x90);
    buf[4] = (unsigned char) val & 0xff;

    val = 4095 - e;
    buf[5] = (unsigned char) ((val >> 8) | 0x90);
    buf[6] = (unsigned char) val & 0xff;

    val = 4095 - a;
    buf[7] = (unsigned char) ((val >> 8) | 0x90);
    buf[8] = (unsigned char) val & 0xff;

    messages[0].addr = DAC_ADR;
    messages[0].flags = 0;
    messages[0].len = 9;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to write to MCP4728 data registers\n");
        exit(1);
    }
}
/* ------------------------------------------- */
void Delay(int n)  // n ms delay
{
	int i;

    for (i=1; i<=n; i+=1)
	{
        while (1)
        {
            gettimeofday(&frametime, NULL);
            Timer2 = frametime.tv_usec / 1000L;  /* 1 ms ticks */
            if (Timer1 != Timer2)
            {
                Timer1 = Timer2;
                break;
            }
        }
    }
}

/* ------------------------------------------- */
void LEDs(int x)
{
	buf[0] = 9;  /* write pattern to reg 9 */
	buf[1] = ~x;

	messages[0].addr = LED_ADR;
	messages[0].flags = 0;
	messages[0].len = 2;
	messages[0].buf = buf;

	packets.msgs = messages;
	packets.nmsgs = 1;

	if (ioctl(i2c, I2C_RDWR, &packets) < 0)
	{
		printf("unable to write to MCP23008 data register\n");
		exit(1);
	}
}

