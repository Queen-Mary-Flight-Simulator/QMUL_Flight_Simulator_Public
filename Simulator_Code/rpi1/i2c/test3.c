#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/*
     read the ADC
*/
int main()
{
    int           file;
    unsigned char buf[10];
 
    file = open("/dev/i2c-1", O_RDWR);
    if (file < 0)
    {
        printf("unable to open i2c as a file\n");
        exit(1);
    }
    if (ioctl(file, I2C_SLAVE, 0x4d) < 0)
    {
        printf("unable to access ADC\n");
        exit(1);
    }
    buf[0] = 0;
    buf[1] = 0;
    if (read(file, buf, 2) != 2)
    {
        printf("unable to read ADC\n");
        exit(1);
    }
    printf("ADC val = %x\n", ((int) (buf[0] & 0xf) << 8) | (int) buf[1]);

    close(file);
    return 0;
}
