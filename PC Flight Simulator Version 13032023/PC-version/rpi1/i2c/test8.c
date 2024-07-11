/* digital output test
   set lamps in the following order at 1 second interval:
   1. gear transit lamp
   2. left gear lamp
   3. right gear lamp
   4. nose gear lamp
   5. all lamps
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

int main()
{
    int                        i2c;
    long unsigned int          Timer1;
    long unsigned int          Timer2;
    unsigned char              pattern;
    unsigned int               ticks = 0; 
    struct timeval             tv;
    unsigned char              buf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[1];
    unsigned char              patterns[5] = {0xF7, 0xFB, 0xFD, 0xFE, 0xF0 }; /* transit, left, right, nose, all */
    unsigned int               p = 0;

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

        ticks = (ticks + 1) % 50;
        if (ticks != 0)
        {
            continue;
        }

        pattern = patterns[p];
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

        p += 1;
        if (p > 4)
        {
            p = 0;
        }
    }
    return 0;
}
