/* digital output test
   set individual lamps
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

/* 0xFE nose
   0xFD right
   0xFB left
   0xF7 transit
*/

int main()
{
    int                        i2c;
    unsigned char              pattern;
    unsigned char              buf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[1];

    i2c = open("/dev/i2c-1", O_RDWR);
    if (i2c < 0)
    {
        printf("unable to open i2c as a file\n");
        exit(1);
    }

    buf[0] = 0;  /* write 0 to reg 0 (set dir to output) */
    buf[1] = 0;

    messages[0].addr = 0x20;
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

	pattern = 0xF7;
	buf[0] = 9;  /* write pattern to reg 9 */
	buf[1] = pattern;

	messages[0].addr = 0x20;
	messages[0].flags = 0;
	messages[0].len = 2;
	messages[0].buf = buf;

	packets.msgs = messages;
	packets.nmsgs = 1;

	if (ioctl(i2c, I2C_RDWR, &packets) < 0)
	{
		printf("unable to write to data register\n");
		exit(1);
	}

    return 0;
}
