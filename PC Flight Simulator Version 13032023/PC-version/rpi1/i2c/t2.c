/* DAC simple tests */
/* 0=-10V, 2048=0V, 4095=+10V  */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

#define LED_ADR 0x21  /* A5001 LEDS 8-bit reg at 0x21 */
#define DAC_ADR 0x60

void DAC(int a, int b, int c, int d);

int                        i2c;
unsigned char              buf[3];
struct i2c_rdwr_ioctl_data packets;
struct i2c_msg             messages[1];

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
    buf[1] = (unsigned char) (0xAA);

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

    while (1)
	{
	    int x;
		char str[50];
		printf("DAC value: ");
		fscanf(stdin, "%s", str);
		printf("str=%s\n", str);
		x = atoi(str);
		if (x < 2048)
		{
		    break;
		}
		if (x > 4095)
		{
		    x = 4095;
		}
		printf("x=%d\n", x);
        DAC(2048, x, 2048, 2048);  // rudder, airspeed, elevator, aileron
    }
	
    return 0;
}

void DAC(int a, int b, int c, int d)
{
    unsigned char buf[9];
    int           val;

    val = 4095 - a;
    buf[0] = (unsigned char) (0x50);  /* select DAC channel */
    buf[1] = (unsigned char) ((val >> 8) | 0x90);
    buf[2] = (unsigned char) val & 0xff;

    val = 4095 - b;
    buf[3] = (unsigned char) ((val >> 8) | 0x90);
    buf[4] = (unsigned char) val & 0xff;

    val = 4095 - c;
    buf[5] = (unsigned char) ((val >> 8) | 0x90);
    buf[6] = (unsigned char) val & 0xff;

    val = 4095 - d;
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
