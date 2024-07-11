/*
Thrustmaster throttle test
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

struct axis_state
{
    short x, y;
};

/*
Read a joystick event from the joystick device.
Returns 0 on success. Otherwise -1 is returned.
*/

/* ------------------------------------------------ */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
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

/*
Keeps track of the current axis state.

NOTE: This function assumes that axes are numbered starting from 0, and that
the X axis is an even number, and the Y axis is an odd number. However, this
is usually a safe assumption.

 Returns the axis that the event indicated.
*/

/* ------------------------------------------------ */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

/* ------------------------------------------------ */
int main(int argc, char *argv[])
{
    const char        *device;
    int               js;
    struct js_event   event;
    struct axis_state axes[3] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };
    size_t            axis;

    char naxes;
	char nbuttons;
	int  version;
	char name[128];
	
    if (argc > 1)
        device = argv[1];
    else
        device = "/dev/input/js1";

    js = open(device, O_RDONLY);

    if (js == -1)
        perror("Could not open joystick");

    ioctl(js, JSIOCGAXES, &naxes);
	printf("Number of axes: %d\n", naxes);
    ioctl(js, JSIOCGBUTTONS, &nbuttons);
	printf("Number of buttons: %d\n", nbuttons);
    ioctl(js, JSIOCGVERSION, &version);
	printf("Version: %d\n", version);
    ioctl(js, JSIOCGNAME(sizeof(name)), name);
	printf("Name: %s\n", name);

    /* This loop will exit if the controller is unplugged. */
    while (read_event(js, &event) == 0)
    {
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
                printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
                break;
            case JS_EVENT_AXIS:
                axis = get_axis_state(&event, axes);
                //if (axis < 3)
                    printf("Axis %u at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
                break;
            default:
                /* Ignore init events. */
                break;
        }
    }

    close(js);
    return 0;
}
