/* I/O library for the ThrustMaster joystick, throttle and rudders
   reads 32 analogue inputs and 32 digital inputs from USB ports in non-blocking mode
   maintains 50 Hz clock

   DJA: modified 1/4/20 for dynamic allocation of USB ports
   
   Based on joystick.c by Jason White
   See also: https://www.kernel.org/doc/Documentation/input/joystick-api.txt
*/
 
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/joystick.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <SIM/iodefn.h>
#include <SIM/jslib.h>

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

unsigned char cmds[] =
{ 0x21, 0x81, 0xe7 };  /* start oscillator, display ON, blinking OFF, full brightness */
  
unsigned char digits[] =
{ 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,  /* chars 0-9, A-F */
  0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71 };
  
unsigned int               jsLib_AnalogueData[32];
unsigned char              jsLib_DigitalDataA;
unsigned char              jsLib_DigitalDataB;
unsigned char              jsLib_DigitalDataC;
unsigned char              jsLib_DigitalDataD;

int                        js0;  /* stick */
int                        js1;  /* throttle */
int                        js2;  /* rudder */
struct js_event            event;

/* prototypes */
int read_event(int fd, struct js_event *event);
size_t get_axis_count(int fd);
size_t get_button_count(int fd);
short unsigned int word16(int x);
int findport(int port, char str[]);

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

/* ---------------------------------------------------- */    
short unsigned int word16(int x)  /* Thrustmaster values are 16 bits in the range +/-32767 */
{
	return (short unsigned int) (x & 0xffff);
}

/* ---------------------------------------------------- */    
int findport(int port, char str[])
{
    if (strcmp(str, "Thustmaster Joystick - HOTAS Warthog") == 0)
	{
	    return 0;
	}
	if (strcmp(str, "Thrustmaster Throttle - HOTAS Warthog") == 0)
	{
	    return 1;
	}
	if (strcmp(str, "Thrustmaster T-Rudder") == 0)
	{
	    return 2;
	}
    return -1;
}

/* USB port are re-mapped so that js0=stick, js1=throttle, js2=rudder
   independent of the physical ordering of the USB connections */
/* ---------------------------------------------------- */    
void Start_USB()
{
    char naxes;
    char nbuttons;
    int  version;
    char name[128];
    int  port;
	int  p;
	int  n;
	int  portsfound = 0;
    char str[] = "/dev/input/jsX";
	
	for (p=0; p<=2; p+=1)  /* try first three USB ports */
	{
        str[13] = (char) p + '0';
		port = open(str, O_RDONLY | O_NONBLOCK);

        if (port < 0)  /* skip if nothing connected */
        {
            continue;
        }

        portsfound += 1;
        ioctl(port, JSIOCGNAME(sizeof(name)), name);
		
		n = findport(p, name);
		
		switch (n)
		{
		    case 0:
			    js0 = port;
				break;
			case 1:
			    js1 = port;
				break;
			case 2:
			    js2 = port;
				break;
			default:
			    printf("Unknown port (%d)\n", n);
				exit(-1);
		}
    }
	
	if (portsfound != 3)
	{
	    printf("Cannot find 3 USB ports\n");  /* s/w assumes 3 ThrustMaster devices */
		exit(-1);
	}
	
    ioctl(js0, JSIOCGNAME(sizeof(name)), name);
    ioctl(js0, JSIOCGVERSION, &version);
    printf("js0: %s Version %d ", name, version);
    ioctl(js0, JSIOCGAXES, &naxes);
    printf("%d axes ", naxes);
    ioctl(js0, JSIOCGBUTTONS, &nbuttons);
    printf("%d buttons\n", nbuttons);
    
    ioctl(js1, JSIOCGNAME(sizeof(name)), name);
    ioctl(js1, JSIOCGVERSION, &version);
    printf("js1: %s Version %d ", name, version);
    ioctl(js1, JSIOCGAXES, &naxes);
    printf("%d axes ", naxes);
    ioctl(js1, JSIOCGBUTTONS, &nbuttons);
    printf("%d buttons\n", nbuttons);
	
    ioctl(js2, JSIOCGNAME(sizeof(name)), name);
    ioctl(js2, JSIOCGVERSION, &version);
    printf("js2: %s Version %d ", name, version);
    ioctl(js2, JSIOCGAXES, &naxes);
    printf("%d axes ", naxes);
    ioctl(js2, JSIOCGBUTTONS, &nbuttons);
    printf("%d buttons\n", nbuttons);
}

/* ---------------------------------------------------- */    
void jsLib_UpdateIO(unsigned char DigitalOutputA, unsigned char DigitalOutputB)
{
    int e0, e1, e2;
    
    do
    {
        e0 = read_event(js0, &event);
        if (e0 == 0)
        {
            switch (event.type)
            {
                case JS_EVENT_BUTTON | JS_EVENT_INIT:
                case JS_EVENT_BUTTON:
				    switch (event.number)  /* joystick buttons/switches */
					{
					    case 0:
						    /* rear trigger (not used) */
							break;
					    case 1:
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT7 : jsLib_DigitalDataA & ~BIT7;  /* A7 stick red button (PTT and sim start) */
							break;
					    case 2:
						    /* stick rear grey button (IOS restore) */
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT6 : jsLib_DigitalDataA & ~BIT6; 
							break;
					    case 3:
						    /* stick rear lever (IOS HOLD) */
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT0 : jsLib_DigitalDataB & ~BIT0; 
							break;
					    case 4:
						    /* stick right grey button (A/P disconnect) */
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT5 : jsLib_DigitalDataA & ~BIT5; 
							break;
					    case 5:
						    /* stick rear trigger (not used) */
							break;
					    case 6:
						    /* stick black knurled button up (not used) */
							break;
					    case 7:
						    /* stick black knurled button right (not used) */
							break;
					    case 8:
						    /* stick black knurled button down (not used) */
							break;
					    case 9:
						    /* stick black knurled button left (not used) */
							break;
					    case 10:
						    /* stick black round button up (not used) */
							break;
					    case 11:
						    /* stick black round button right (not used) */
							break;
					    case 12:
						    /* stick black round button down (not used) */
							break;
					    case 13:
						    /* stick black round button left (not used) */
							break;
					    case 14:
						    /* stick left grey button forwards (not used) */
							break;
					    case 15:
						    /* stick left grey button right (not used) */
							break;
					    case 16:
						    /* stick left grey button back (not used) */
							break;
					    case 17:
						    /* stick left grey button left (not used) */
							break;
						default:
						    break;
					}
                    break;
					
                case JS_EVENT_AXIS | JS_EVENT_INIT:
				case JS_EVENT_AXIS:
				    switch (event.number)  /* joystick analogue inputs */
					{
					    case 0:
						    jsLib_AnalogueData[0] = word16(event.value);  /* P01 aileron */
							break;
						case 1:
						    jsLib_AnalogueData[1] = word16(event.value);  /* P02 elevator */
							break;
						case 2:
						    jsLib_AnalogueData[9] = word16(event.value);  /* P10 grey pad (rudder electric trim left/right) */
							break;
						case 3:
						    jsLib_AnalogueData[8] = word16(event.value);  /* P09 grey pad (elevator electric trim up/down) */
							break;
						default:
						    break;
					}
                    break;
					
                default:
                    /* ignore unspecified events. */
                    break;
            }
        }

        e1 = read_event(js1, &event);
        if (e1 == 0)
        {
            switch (event.type)
            {
                case JS_EVENT_BUTTON | JS_EVENT_INIT:
				case JS_EVENT_BUTTON:
				    switch (event.number)  /* throttle switches */
					{
					    case 0:
						    /* right black mini hat (not used) */
							break;
					    case 1:
						    /* right black mini hat (not used) */
							break;
					    case 2:
						    /* right black mini hat (not used) */
							break;
					    case 3:
						    /* right black mini hat (not used) */
							break;
					    case 4:
						    /* right black hat (not used) */
							break;
						case 5:
						    /* right black hat (not used) */
							break;
					    case 6:
						    /* upper grey button (not used) */
							break;
					    case 7:
						    /* upper grey button (not used) */
							break;
					    case 8:
						    /* lower grey button (not used) */
							break;
					    case 9:
						    /* lower grey button (not used) */
							break;
					    case 10:
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT4 : jsLib_DigitalDataA & ~BIT4;  /* A4 reverse thrust disengage */
							break;
					    case 11:
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT3 : jsLib_DigitalDataA & ~BIT3;  /* A3 reverse thrust engage */
							break;
					    case 12:
						    /* left throttle switch (not used) */
							break;
					    case 13:
						    /* left throttle switch (not used) */
							break;
					    case 14:
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT2 : jsLib_DigitalDataA & ~BIT2;  /* A2 TOGA */
							break;
					    case 15:
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT1 : jsLib_DigitalDataA & ~BIT1;  /* A1 left fuel switch */
							break;
					    case 16:
						    jsLib_DigitalDataA = (event.value) ? jsLib_DigitalDataA | BIT0 : jsLib_DigitalDataA & ~BIT0;  /* A0 right fuel switch */
							break;
					    case 17:
						    /* left ignition switch (not used) */
							break;
					    case 18:
						    /* right ignition switch (not used) */
							break;
					    case 19:
						    /* APU start (not used) */
							break;
					    case 20:
						    /* L/G silence (not used) */
							break;
					    case 21:
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT5 : jsLib_DigitalDataB & ~BIT5;  /* B5 flaps up */
							break;
					    case 22:
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT4 : jsLib_DigitalDataB & ~BIT4;  /* B4 flaps down */
							break;
					    case 23:
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT3 : jsLib_DigitalDataB & ~BIT3;  /* B3 park brake */
							break;
					    case 24:
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT2 : jsLib_DigitalDataB & ~BIT2;  /* B2 undercarriage */
							break;
					    case 25:
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT1 : jsLib_DigitalDataB & ~BIT1;  /* B1 A/T disengage */
							break;
					    case 26:
						    /* A/P PATH (not used) */
							break;
					    case 27:
						    /* A/P ALT (not used) */
							break;
					    case 28:
						    /* (not used) */
							break;
					    case 29:
						    /* (not used) */
							break;
					    case 30:
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT7 : jsLib_DigitalDataB & ~BIT7;  /* B7 left engine start */
							break;
					    case 31:
						    jsLib_DigitalDataB = (event.value) ? jsLib_DigitalDataB | BIT6 : jsLib_DigitalDataB & ~BIT6;  /* B6 right engine start */
							break;
						default:
						    break;
					}
                    break;
					
                case JS_EVENT_AXIS | JS_EVENT_INIT:
				case JS_EVENT_AXIS:
				    switch (event.number)  /* throttle analogue inputs */
					{
					    case 2:
						    jsLib_AnalogueData[4] = word16(event.value);  /* P05 right throttle */
							break;
						case 3:
						    jsLib_AnalogueData[5] = word16(event.value);  /* P06 left throttle */
							break;
						case 4:
						    jsLib_AnalogueData[3] = word16(event.value);  /* P04 elevator trim wheel */
							break;
						default:
						    break;
					}
                    break;
					
                default:
                    /* ignore unspecified events. */
                    break;
            }
        }

        e2 = read_event(js2, &event);
        if (e2 == 0)
        {
            switch (event.type)  /* rudder analogiue inputs */
            {
                case JS_EVENT_AXIS | JS_EVENT_INIT:
				case JS_EVENT_AXIS:
				    switch (event.number)
					{
					    case 0:
						    jsLib_AnalogueData[6] = word16(event.value);  /* P07 right toe brake */
							break;
						case 1:
						    jsLib_AnalogueData[7] = word16(event.value);  /* P08 left toe brake */
							break;
						case 2:
						    jsLib_AnalogueData[2] = word16(event.value);  /* P03 rudder */
							break;
						default:
						    break;
					}
                    break;
					
                default:
                    /* ignore unspecified events. */
                    break;
            }
        }
		
    } while (e0 >= 0 || e1 >= 0 || e2 >= 0);  /* check for pending events */
}
    
/* ---------------------------------------------------- */    
void jsLib_Wait()  /* wait for RED button to be pressed */
{
    while (1)
    {
        jsLib_UpdateIO(0, 0);
        if ((jsLib_DigitalDataA & 0x040))
        {
            break;  /* RPi waits for RED button to be pressed */
        }
    }
    while (1)
    {
        jsLib_UpdateIO(0, 0);
        if (!(jsLib_DigitalDataA & 0x040))
        {
            break;  /* RPi waits for RED button to be released */
        }
    }
}

/* ---------------------------------------------------- */    
void jsLib_Shutdown()
{
    close(js0);
    close(js1);
}

/* ---------------------------------------------------- */    
void BEGIN_jsLib()
{
	jsLib_AnalogueData[0] = 0;      /* aileron mid-position */
	jsLib_AnalogueData[1] = 0;      /* elevator mid-position */
	jsLib_AnalogueData[2] = 0;      /* rudder mid-position */
	jsLib_AnalogueData[3] = 0;      /* trim wheel mid-position */
	jsLib_AnalogueData[4] = 32767;  /* right throttle fully back */
	jsLib_AnalogueData[5] = 32767;  /* left throttle fully back */
	jsLib_AnalogueData[6] = 32767;  /* right toe brake off */
	jsLib_AnalogueData[7] = 32767;  /* left toe brake off */
	jsLib_AnalogueData[8] = 0;      /* elevator electric mid-position */
	jsLib_AnalogueData[9] = 0;      /* rudder electric mid-position */
	
    jsLib_DigitalDataA = 0;
    jsLib_DigitalDataB = 0;
    jsLib_DigitalDataC = 0;
    jsLib_DigitalDataD = 0;
	
	Start_USB();
}
