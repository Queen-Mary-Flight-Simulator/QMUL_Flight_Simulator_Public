#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

/*
     digital input test
*/

void printbinary(int n);

void printbinary(int n)
{
    int m = 0x80;
    int i;

    for (i=1; i<=8; i+=1)
    {
        printf("%c", ((m & n) == 0) ? '0': '1');
        m = m >> 1;
    }
    printf(" ");
}

int main()
{
    int            i2c;
    unsigned int   chn = 0;
    struct timeval tv;
    unsigned char  buf[10];
    unsigned int   samples[4];
    unsigned int      oldsamples[4];

    i2c = open("/dev/i2c-1", O_RDWR);
    if (i2c < 0)
    {
        printf("unable to access i2c\n");
        exit(1);
    }
    if (ioctl(i2c, I2C_SLAVE, 0x20) < 0)
    {
        printf("unable to access digital input reg\n");
        exit(1);
    }
    buf[0] = 0;
    buf[1] = 0xff;  /* set for 8 inputs */
    if (write(i2c, buf, 2) < 0)
    {
        printf("unable to access dir register\n");
        exit(1);
    }

    if (ioctl(i2c, I2C_SLAVE, 0x21) < 0)
    {
        printf("unable to access mux chip\n");
        exit(1);
    }
    buf[0] = 0;
    buf[1] = 0;
    if (write(i2c, buf, 2) < 0)
    {
        printf("unable to access dir register\n");
        exit(1);
    }

    gettimeofday(&tv, NULL);
    //printf("timer1=%u %u\n", tv.tv_sec, tv.tv_usec);


    for (chn=0; chn<=3; chn+=1)
    {
        samples[chn] = 1;
        oldsamples[chn] = 0;
    }

    while (1)
    {
        for (chn=0; chn<=3; chn+=1)
        {
            //printf("%c%c%c", 27, '[', 'H');
            ioctl(i2c, I2C_SLAVE, 0x21);
            buf[0] = 9;
            buf[1] = (unsigned char) (chn << 5);
            write(i2c, buf, 2);

            ioctl(i2c, I2C_SLAVE, 0x20);
            buf[0] = 9;
            if (write(i2c, buf, 1) != 1)
            {
                printf("unable to set register\n");
                exit(1);
            }

            if (read(i2c, buf, 1) != 1)
            {
                printf("unable to read digital inputs\n");
                exit(1);
            }
            samples[chn] = (unsigned int) buf[0];
            if (samples[chn] != oldsamples[chn])
            {
                printbinary(samples[0]);
                printbinary(samples[1]);
                printbinary(samples[2]);
                printbinary(samples[3]);
                printf("\n");
                oldsamples[chn] = samples[chn];
            }
        //chn = (chn + 1) % 32;
       }
    }
    close(i2c);

    return 0;
}
