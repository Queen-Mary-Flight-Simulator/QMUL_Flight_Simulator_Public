#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

/*
    sample the NXP SE95 temperature sensor
*/

int main()
{
    int                        i2c;
    unsigned int               s;
    struct timeval             tv;
    unsigned char              buf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[1];
    unsigned int               i;
	float                      temperatures[60*60+1];
	
    i2c = open("/dev/i2c-1", O_RDWR);
    if (i2c < 0)
    {
        printf("unable to access i2c\n");
        exit(1);
    }

    messages[0].addr = 0x48;
    messages[0].flags = I2C_M_RD;
    messages[0].len = 2;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    gettimeofday(&tv, NULL);
	s = tv.tv_sec;
    //printf("timer1=%u %u\n", tv.tv_sec, tv.tv_usec);

    for (i=1; i<=10; i+=1)
    //for (i=1; i<=60*60; i+=1)
    {
        do
		{
            gettimeofday(&tv, NULL);
        } while (s == tv.tv_sec);  /* wait for next second tick */
        s = tv.tv_sec;

        buf[0] = 0;
        buf[1] = 0;
        ioctl(i2c, I2C_SLAVE, 0x48);
        if (read(i2c, buf, 2) != 2)
        {
            printf("unable to set register\n");
            exit(1);
        }
		temperatures[i] = (float) (((buf[0] << 8) + buf[1]) / 8) * 0.03125;
    }
    
	for (i=1; i<=10; i+=1)
	//for (i=1; i<=60*60; i+=1)
    {
		printf("%d %f\n", i, temperatures[i]);
    }
    close(i2c);

    return 0;
}
