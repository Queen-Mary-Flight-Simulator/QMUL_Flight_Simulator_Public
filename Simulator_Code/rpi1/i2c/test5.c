#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

/*
     ADC test
*/
int main()
{
    int                        i2c;
    unsigned int               chn = 0;
    struct timeval             tv;
    unsigned char              inbuf[2];
    unsigned char              outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[2];

    i2c = open("/dev/i2c-1", O_RDWR);
    if (i2c < 0)
    {
        printf("unable to access i2c\n");
        exit(1);
    }
    if (ioctl(i2c, I2C_SLAVE, 0x4d) < 0)
    {
        printf("unable to access LEDS\n");
        exit(1);
    }
    if (ioctl(i2c, I2C_SLAVE, 0x21) < 0)
    {
        printf("unable to access mux chip\n");
        exit(1);
    }
		
    //gettimeofday(&tv, NULL);
    //printf("timer1=%u %u\n", tv.tv_sec, tv.tv_usec);

    chn = 5;
    while (1)
    {
        int val;
		
        outbuf[0] = 9;
        outbuf[1] = 0;
	    
        messages[0].addr = 0x21;
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
            printf("ADC read error\n");
            exit(1);
        }
		
        val = (((int) inbuf[0] & 0xf) << 8) | (int) inbuf[1];
        //printf("chn %d %5d%c", chn, val, 13);
        //chn = (chn + 1) % 32;
    }

    close(i2c);

    return 0;
}
