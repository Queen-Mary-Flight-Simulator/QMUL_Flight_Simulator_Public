/* DAC dynamic tests */
/* DAC output at 50 Hz to ouput triangluar waveform to rudder trim, airspeed, elevator trim and aileron trim */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>
#include <time.h>

#define LED_ADR 0x24  /* A5001 LEDS 8-bit reg */
#define DAC_ADR 0x60

void DAC(int r, int s, int e, int a);

int                        i2c;
unsigned char              buf[3];
struct i2c_rdwr_ioctl_data packets;
struct i2c_msg             messages[1];

unsigned int               Timer1;
unsigned int               Timer2;
struct timeval             frametime;
unsigned int               count = 0;

int main()
{
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
     
	buf[0] = 9;  /* write pattern to reg 9 */
	buf[1] = (unsigned char) ~(count >> 4);

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
	
    gettimeofday(&frametime, NULL);
    Timer1 = frametime.tv_usec / 20000L;

	while (1)
	{
    	DAC(count, count, count, count);

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
	
    return 0;
}

/* ---------------------------------------------------- */    

/*
DAC output 0-5V amplified to -10V to 10V 
DAC range -10V = 0, 0v = 2048, +10V = 4095
*/

void DAC(int r, int s, int e, int a)
{
    unsigned char              buf[9];
    int                        val;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[1];

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
	count += 1;
	if (count > 4095)
	{
	    count = 0;
	}
}