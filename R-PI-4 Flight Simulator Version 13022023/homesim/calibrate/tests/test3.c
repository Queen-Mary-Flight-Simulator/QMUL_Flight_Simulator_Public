/*
Thrustmaster rudder test (non-block version)
DJA 19 May 2018

Based on joystick.c by Jason White
See also: https://www.kernel.org/doc/Documentation/input/joystick-api.txt
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

/*
Read a joystick event from the joystick device.
Returns 0 on success. Otherwise -1 is returned.
*/

/* ------------------------------------------------ */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes > 0)
	{
        return 0;
    }
	else
	{
        return -1;  /* probably no event */
	}
}

/*
Returns the number of axes on the controller or 0 if an error occurs.
*/

/* ------------------------------------------------ */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/*
Returns the number of buttons on the controller or 0 if an error occurs.
*/

/* ------------------------------------------------ */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/* ------------------------------------------------ */
int main(int argc, char *argv[])
{
    int             js;
    struct js_event event;
	
    char            naxes;
	char            nbuttons;
	int             version;
	char            name[128];
	
    js = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);

    if (js == -1)
	{
	    printf("Unable to open joystick\n");
		exit(-1);
    }
	
    ioctl(js, JSIOCGAXES, &naxes);
	printf("Number of axes: %d\n", naxes);
    ioctl(js, JSIOCGBUTTONS, &nbuttons);
	printf("Number of buttons: %d\n", nbuttons);
    ioctl(js, JSIOCGVERSION, &version);
	printf("Version: %d\n", version);
    ioctl(js, JSIOCGNAME(sizeof(name)), name);
	printf("Name: %s\n", name);

    /* This loop will exit if the controller is unplugged. */
	while (1)
    {
        if (read_event(js, &event) == 0)
        {
			switch (event.type)
			{
				case JS_EVENT_BUTTON:
					printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
					break;
				case JS_EVENT_AXIS:
					printf("event.number=%d event.value=%d\n", event.number, event.value);
					break;
				default:
					/* Ignore init events. */
					break;
			}
		}
    }

    close(js);
    return 0;
}
