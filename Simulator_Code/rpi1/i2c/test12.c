/* DAC simple tests */
/* set speed and elevator position */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

#define LED_ADR 0x24  /* A5001 LEDS 8-bit reg */
#define DAC_ADR 0x60

void DAC(unsigned int chn, int val);

int                        i2c;
unsigned char              buf[3];
struct i2c_rdwr_ioctl_data packets;
struct i2c_msg             messages[1];

int main()
{
	unsigned int               chn = 0;        /* 0=rudder 1=airspeed 2=elevator 3=aileron */
	int                        pattern = 4095; /* 0=-10V, 2048=0V, 4095=+10V  */
	
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
	buf[1] = (unsigned char) ~(pattern >> 4);

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

	DAC(0, 4095);
	DAC(1, 4095);
	DAC(2, 4095);
	DAC(3, 4095);

    return 0;
}

void DAC(unsigned int chn, int val)
{
    unsigned char buf[3];

    if (chn > 3)
    {
        return;
    }    
    val = 4095 - val;
    buf[0] = (unsigned char) (0x58 | (chn << 1));           /* select DAC channel */
    buf[1] = (unsigned char) (((val >> 8) & 0x0f) | 0x90);  /* 0x90 = select internal voltage ref, *2 buffer amp */
    buf[2] = (unsigned char) val;

    messages[0].addr = DAC_ADR;
    messages[0].flags = 0;
    messages[0].len = 3;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to write to MCP4728 data registers\n");
        exit(1);
    }
}