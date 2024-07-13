#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/*
     write to the LEDs
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
    buf[0] = 9;
    buf[1] = ~0x55; /* remember LEDs are inverted */
    if (write(file, buf, 2) != 2)
    {
        printf("unable to write to data register\n");
        exit(1);
    }
    close(file);
    return 0;
}
