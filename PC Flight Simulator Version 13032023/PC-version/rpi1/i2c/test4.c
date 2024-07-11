#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

/*
     time 640 ADC samples
     0x23 analogue MUX
     0x4d ADC
*/
int main()
{
    int                        i2c;
    unsigned int               chn;
    struct timeval             tv;
    unsigned char              inbuf[3];
    unsigned char              outbuf[3];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[2];
    int                        elapsedtime;

    i2c = open("/dev/i2c-1", O_RDWR);
    if (i2c < 0)
    {
        printf("unable to access i2c\n");
        exit(1);
    }
	
    outbuf[0] = 0;  /* set reg 0 = 0 (output) */
    outbuf[1] = 0;

    messages[0].addr = 0x23;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = outbuf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to set the MUX dir reg\n");
        exit(1);
    }

    gettimeofday(&tv, NULL);
    elapsedtime = (int) tv.tv_usec;

    for (chn=0; chn<=31; chn+=1)  // *** was 639
    {
        outbuf[0] = 9;  /* set reg 9 = channel number for analogue mux */
        outbuf[1] = (unsigned char) (chn % 32);

        messages[0].addr = 0x23;
        messages[0].flags = 0;
        messages[0].len = 2;
        messages[0].buf = outbuf;

        messages[1].addr = 0x4d;
        messages[1].flags = I2C_M_RD;
        messages[1].len = 2;
        messages[1].buf = inbuf;

        packets.msgs = messages;
        packets.nmsgs = 2;

        if (ioctl(i2c, I2C_RDWR, &packets) < 0)
        {
            printf("unable to read ADC\n");
            exit(1);
        }
    }

    gettimeofday(&tv, NULL);
    elapsedtime = (int) tv.tv_usec - elapsedtime;
    if (elapsedtime <= 0)
    {
        elapsedtime += 1000000;
    }
    printf("elaspedtime: %d usec\n", elapsedtime);
    close(i2c);

    return 0;
}

