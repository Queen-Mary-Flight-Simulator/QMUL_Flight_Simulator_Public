/* CLS simple tests
   DJA 6 December 2018 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

#define intround(x) ((x) >= 0.0 ? (int)((x)+0.5) : (int)((x)-0.5))

#define DAC_ADR 0x60

#define RUDDER   0
#define AIRSPEED 1
#define ELEVATOR 2
#define AILERON  3

struct timeval             frametime;
unsigned int               Timer1;
unsigned int               Timer2;

int                        i2c;
unsigned char              buf[3];
struct i2c_rdwr_ioctl_data packets;
struct i2c_msg             messages[1];

int convert(float x);
void DAC(float r, float s, float e, float a);
void Delay(int n);
void LEDs(int x);

/* ------------------------------------------- */
int main()
{
    float x;
	
    i2c = open("/dev/i2c-1", O_RDWR);
    if (i2c < 0)
    {
        printf("unable to open i2c as a file\n");
        exit(1);
    }

    gettimeofday(&frametime, NULL);
    Timer1 = frametime.tv_usec / 20000L;

	printf("Centre test starting\n");    /* stick self centre test */
	
	for (x=0.0; x<=1.0; x+=0.004)        /* 250 steps @ 20ms = 5s */
	{
	    DAC(0.0, x, 0.0, 0.0);
	}
	
	printf("Elevator test starting\n");  /* elevator test  */
	
	for (x=0.0; x<=1.0; x+=0.004)        /* 250 steps @ 20ms = 5s, stick fully forward */
	{
	    DAC(0.0, 1.0, x, 0.0);
	}
	for (x=1.0; x>=-1.0; x-=0.004)      /* 500 steps @ 20ms 10s, stick fully back */
	{
	    DAC(0.0, 1.0, x, 0.0);
	}
	for (x=-1.0; x<=0.0; x+=0.004)       /* 250 steps @ 20ms = 5s, stick back to centre */
	{
	    DAC(0.0, 1.0, x, 0.0);
	}
	
	printf("Aileron test starting\n");   /* aileron test  */
	
	for (x=0.0; x<=1.0; x+=0.004)        /* 250 steps @ 20ms = 5s, stick fully left */
	{
	    DAC(0.0, 1.0, 0.0, x);
	}
	for (x=1.0; x>=-1.0; x-=0.004)      /* 500 steps @ 20ms 10s, stick fully right */
	{
	    DAC(0.0, 1.0, 0.0, x);
	}
	for (x=-1.0; x<=0.0; x+=0.004)       /* 250 steps @ 20ms = 5s, stick back to centre */
	{
	    DAC(0.0, 1.0, 0.0, x);
	}
	
	printf("Rudder test starting\n");    /* rudder test */
	for (x=0.0; x<=1.0; x+=0.004)        /* 250 steps @ 20ms = 5s, rudder fully left */
	{
	    DAC(x, 1.0, x, 0.0);
	}
	for (x=1.0; x>=-1.0; x-=0.004)      /* 500 steps @ 20ms 10s, rudder fully right */
	{
	    DAC(x, 1.0, x, 0.0);
	}
	for (x=-1.0; x<=0.0; x+=0.004)       /* 250 steps @ 20ms = 5s, rudder back to centre */
	{
	    DAC(x, 1.0, 0.0, 0.0);
	}
	
	printf("Shutdown\n");               /* remove power from stick */
	for (x=1.0; x>=0.0; x-=0.004)       /* 250 steps @ 20ms = 5s */
	{
	    DAC(0.0, x, 0.0, 0.0);
	}
	
	printf("Test complete\n");
	
    return 0;
}

/* ------------------------------------------- */
void eof()  /* 50 Hz frame sync */
{
    while (1)
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

/* ------------------------------------------- */
int convert(float x)  /* DAC outputs: 0=+10V, 2048=0V, 4095=-10V  */
{
    int r = 2048 - intround(x * 2048.0);
	
	if (r < 0)
	{
	    return 0;
	}
	else if (r > 4095)
	{
	    return 4095;
	}
	else
	{
	    return r;
	}
}

/* ------------------------------------------- */
void DAC(float r, float s, float e, float a)
{
    unsigned char buf[9];
    int           val;

    val = convert(r);
    buf[0] = (unsigned char) (0x50);  /* select DAC channel */
    buf[1] = (unsigned char) ((val >> 8) | 0x90);
    buf[2] = (unsigned char) val & 0xff;

    val = convert(s); /* speed value 0-10V */
	if (val > 2048)
	{
	    val = 2048;
	}
    buf[3] = (unsigned char) ((val >> 8) | 0x90);
    buf[4] = (unsigned char) val & 0xff;

    val = convert(e);
    buf[5] = (unsigned char) ((val >> 8) | 0x90);
    buf[6] = (unsigned char) val & 0xff;

    val = convert(a);
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
	
	eof();
}
