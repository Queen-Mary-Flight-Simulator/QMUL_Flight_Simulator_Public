/* rolling bit pattern */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

/* LEDS 8-bit reg at 0x24 */

int main()
{
    int                        i2c;
    long unsigned int          Timer1;
    long unsigned int          Timer2;
    unsigned int               pattern = 1; 
    bool                       right = false;
    struct timeval             tv;
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

    messages[0].addr = 0x24;
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

    gettimeofday(&tv, NULL);
    Timer1 = tv.tv_usec;

    while (1)
    {
        do
        {
            gettimeofday(&tv, NULL);
            Timer2 = tv.tv_usec;
            if (Timer2 < Timer1)
            {
                Timer2 += 1000000L;
            }
        } while (Timer2 <= (Timer1 + 20000L));  /* 50 places repeating every second */
        Timer1 = (Timer1 + 20000L) % 1000000L;

        buf[0] = 9;  /* write pattern to reg 9 */
        buf[1] = (unsigned char) ~pattern;

        messages[0].addr = 0x24;
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

        if (!right) 
        {
            pattern = pattern << 1;
            if (pattern == 0x80)
            {
                right = true;
            }
        }
        else
        {
            pattern = pattern >> 1;
            if (pattern == 0x01)
            {
                right = false;
            }
        }
    }
    return 0;
}
