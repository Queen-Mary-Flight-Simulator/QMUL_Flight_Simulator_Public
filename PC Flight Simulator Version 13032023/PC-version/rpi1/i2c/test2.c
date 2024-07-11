#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

/*
     repeatedly write to the LEDs
     8 bit cycle
*/
int main()
{
    int               file;
    unsigned char     buf[10];
    long unsigned int Timer1;
    long unsigned int Timer2;
    unsigned int      count = 0; 
    struct timeval    tv;
    unsigned int      t;

    file = open("/dev/i2c-1", O_RDWR);
    if (file < 0)
    {
        printf("unable to open i2c as a file\n");
        exit(1);
    }
    if (ioctl(file, I2C_SLAVE, 0x24) < 0)
    {
        printf("unable to access LEDS\n");
        exit(1);
    }
    buf[0] = 0;
    buf[1] = 0;
    if (write(file, buf, 2) != 2)
    {
        printf("unable to write to dir register\n");
        exit(1);
    }


    gettimeofday(&tv, NULL);
    Timer1 = tv.tv_usec;
    t = 0;

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
        } while (Timer2 <= (Timer1 + 20000L));
        Timer1 = (Timer1 + 20000L) % 1000000L;

        t += 1;
        if (t >= 50)
        {
            buf[0] = 9;
            buf[1] = (unsigned char) ~count;  /* remember LEDs are inverted */
            count = (count + 1) % 256;
            if (write(file, buf, 2) != 2)
            {
                printf("unable to write to data register\n");
                exit(1);
            }
            t = 0;
        }
    }
    return 0;
}

