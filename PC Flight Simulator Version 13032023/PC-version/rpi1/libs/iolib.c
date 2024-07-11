/* RPi I2C version
   DJA 26 June 2018 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <SIM/iodefn.h>
#include <SIM/iolib.h>

#define DIGOUTA_ADR 0x20  /* MCP23008 */
#define DIGOUTB_ADR 0x21  /* MCP23008 */
#define DIGIN_ADR   0x22  /* MCP23008 */
#define MUX_ADR     0x23  /* MCP23008 */
#define LEDS_ADR    0x24  /* MCP23008 */
#define ADC_ADR     0x4d  /* MCP3221  */
#define DAC_ADR     0x60  /* MCP4728  */
#define TMP_ADR     0x48  /* SE95DP   */

#define MidPoint 1962  /* engine lever mid position */
#define threshold 20   /* max change in stick position */

static int    vtrimpos = 2048;
static int    etrimpos = 2048;
static int    atrimpos = 2048;
static int    rtrimpos = 2048;

unsigned int  IOLib_AnalogueData[32];
unsigned char IOLib_DigitalDataA;
unsigned char IOLib_DigitalDataB;
unsigned char IOLib_DigitalDataC;
unsigned char IOLib_DigitalDataD;

float         IOLib_Temperature;
bool          IOLib_Sidestick;

int           i2c;
int           steps = 0;
unsigned int  pattern = 1; 
bool          goingright = false;
bool          LEDs_free;

void DAC(int r, int s, int e, int a);
void Delay(int n);
       
/* ---------------------------------------------------- */    
void IOLib_UpdateCLS(float v, float e, float a, float r)
{
    int vtrim;
    int etrim;
    int atrim;
    int rtrim;
  
    if (IOLib_Sidestick)
    {
        v = 0.0;
        e = 0.0;
        a = 0.0;
        r = 0.0;
    }

    vtrim = 2048 + (int) (2048.0 * (v / 125.0)); /* full triming at 250 Kt */
    if (vtrim > (vtrimpos + threshold))
    {
        vtrim = vtrimpos + threshold;
    }
    else if (vtrim < (vtrimpos - threshold))
    {
        vtrim = vtrimpos - threshold;
    }
    if (vtrim < 2048)
    {
        vtrim = 2048;
    }
    else if (vtrim > 4095)
    {
        vtrim = 4095;
    }
    vtrimpos = vtrim;
  
    etrim = 2048 + (int) (e * 2048.0);
    if (etrim > (etrimpos + threshold))
    {
        etrim = etrimpos + threshold;
    }
    else if (etrim < (etrimpos - threshold))
    {
        etrim = etrimpos - threshold;
    }
    if (etrim < 0)
    {
        etrim = 0;
    }
    else if (etrim > 4095)
    {
        etrim = 4095;
    }
    etrimpos = etrim;
    
    atrim = 2048 + (int) (a * 2048.0);
    if (atrim > (atrimpos + threshold))
    {
        atrim = atrimpos + threshold;
    }
    else if (atrim < (atrimpos - threshold))
    {
        atrim = atrimpos - threshold;
    }
    if (atrim < 0)
    {
        atrim = 0;
    }
    else if (atrim > 4095)
    {
        atrim = 4095;
    }
    atrimpos = atrim;
    
    rtrim = 2048 + (int) (r * 2048.0);
    if (rtrim > (rtrimpos + threshold))
    {
        rtrim = rtrimpos + threshold;
    }
    else if (rtrim < (rtrimpos - threshold))
    {
        rtrim = rtrimpos - threshold;
    }
    if (rtrim < 0)
    {
        rtrim = 0;
    }
    else if (rtrim > 4095)
    {
        rtrim = 4095;
    }
    rtrimpos = rtrim;

    DAC(rtrim, vtrim, etrim, atrim);
}

/* ---------------------------------------------------- */    
void IOLib_RepositionCLS(float v, float e, float a, float r)
{
    IOLib_UpdateCLS(v, e, a, r);
}

/* ---------------------------------------------------- */    
void IOLib_ResetCLS()  /* remove power to stick */
{
	Delay(1000);  /* allow 1s before stick resets */
    DAC(2048, 2048, 2048, 2048);  /* airspeed = 0 */
}

/* ---------------------------------------------------- */    
void IOLib_StartIO()
{
    unsigned char              buf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[1];
    
    i2c = open("/dev/i2c-1", O_RDWR);  /* check I2C device is available */
    if (i2c < 0)
    {
        printf("unable to access I2C bus\n");
        exit(1);
    }
    if (ioctl(i2c, I2C_SLAVE, ADC_ADR) < 0)  /* check ADC can be accessed */
    {
        printf("unable to access ADC (%2x)\n", ADC_ADR);
        exit(1);
    }

    if (ioctl(i2c, I2C_SLAVE, DIGIN_ADR) < 0)  /* check digital inputs can be accessed */
    {
        printf("unable to access digital inputs (%2x)\n", DIGIN_ADR);
        exit(1);
    }
    
    if (ioctl(i2c, I2C_SLAVE, MUX_ADR) < 0)  /* check analogue/digital MUX can be accessed */
    {
        printf("unable to access MUX (%2x)\n", MUX_ADR);
        exit(1);
    }
    
    if (ioctl(i2c, I2C_SLAVE, DIGOUTA_ADR) < 0)  /* check digital outputs can be accessed */
    {
        printf("unable to access digital outputs A (%2x)\n", DIGOUTA_ADR);
        exit(1);
    }
    
    if (ioctl(i2c, I2C_SLAVE, DIGOUTB_ADR) < 0)  /* check digital outputs can be accessed */
    {
        printf("unable to access digital outputs B (%2x)\n", DIGOUTB_ADR);
        exit(1);
    }
    
    if (ioctl(i2c, I2C_SLAVE, TMP_ADR) < 0)  /* check temperature sensor can be accessed */
    {
        printf("unable to access temperature sensor (%x2)\n", TMP_ADR);
        exit(1);
    }

    if (ioctl(i2c, I2C_SLAVE, LEDS_ADR) < 0)  /* check LEDs can be accessed */
    {
        printf("unable to access LEDs (%x2)\n", LEDS_ADR);
        exit(1);
    }

    /* set direction reg for digital inputs */
    buf[0] = 0;
    buf[1] = 0xff;  /* set for 8 inputs */

    messages[0].addr = DIGIN_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to set digital inputs dir reg\n");
        exit(1);
    }
    
    /* set MUX direction reg for outputs*/
    buf[0] = 0;
    buf[1] = 0;  /* set for 8 outputs */

    messages[0].addr = MUX_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to set the MUX dir reg\n");
        exit(1);
    }

    /* set digital outputs direction reg A */
    buf[0] = 0;
    buf[1] = 0;  /* set for 8 outputs */

    messages[0].addr = DIGOUTA_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to set digital outputs dir reg\n");
        exit(1);
    }

    /* set digital outputs direction reg B */
    buf[0] = 0;
    buf[1] = 0;  /* set for 8 outputs */

    messages[0].addr = DIGOUTB_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to set digital outputs dir reg\n");
        exit(1);
    }

    /* set LEDs direction reg */
    buf[0] = 0;
    buf[1] = 0;  /* set for 8 outputs */

    messages[0].addr = LEDS_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to set LEDs dir reg\n");
        exit(1);
    }
}

/* ---------------------------------------------------- */    
void IOLib_StopIO()
{
    close(i2c);
}

/* ---------------------------------------------------- */    
void IOLib_UpdateIO(unsigned char DigitalOutputA, unsigned char DigitalOutputB)
{
    unsigned int               chn;
    unsigned char              inbuf[2];
    unsigned char              outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[2];

    steps += 1;
    for (chn=0; chn<=31; chn+=1)
    {
        outbuf[0] = 9;  /* set reg 9 = channel number for analogue mux */
        outbuf[1] = (unsigned char) chn;

        messages[0].addr = MUX_ADR;
        messages[0].flags = 0;
        messages[0].len = 2;
        messages[0].buf = outbuf;

        packets.msgs = messages;
        packets.nmsgs = 1;

        if (ioctl(i2c, I2C_RDWR, &packets) < 0)
        {
            printf("unable to set the MUX dir reg");
            exit(1);
        }
        messages[0].addr = ADC_ADR;
        messages[0].flags = I2C_M_RD;
        messages[0].len = 2;
        messages[0].buf = inbuf;

        packets.msgs = messages;
        packets.nmsgs = 1;

        if (ioctl(i2c, I2C_RDWR, &packets) < 0)
        {
            printf("unable to read ADC ch=%d steps=%d\n", chn, steps);
            exit(1);
        }
        IOLib_AnalogueData[chn] = (((unsigned int) inbuf[0] & 0xf) << 8) + (unsigned int) inbuf[1];
    }

    for (chn=0; chn<=3; chn+=1)
    {
        outbuf[0] = 9;   /* set mux register */
        outbuf[1] = (unsigned char) (chn << 5);

        messages[0].addr = MUX_ADR;
        messages[0].flags = 0;
        messages[0].len = 2;
        messages[0].buf = outbuf;

        packets.msgs = messages;
        packets.nmsgs = 1;

        if (ioctl(i2c, I2C_RDWR, &packets) < 0)
        {
            printf("unable to set digital MUX\n");
            exit(1);
        }

        outbuf[0] = 9;
        messages[0].addr = DIGIN_ADR;
        messages[0].flags = 0;
        messages[0].len = 1;
        messages[0].buf = outbuf;

        packets.msgs = messages;
        packets.nmsgs = 1;

        if (ioctl(i2c, I2C_RDWR, &packets) < 0)
        {
            printf("unable to set digital inputs dir reg\n");
            exit(1);
        }

        messages[0].addr = DIGIN_ADR;
        messages[0].flags = I2C_M_RD;
        messages[0].len = 1;
        messages[0].buf = inbuf;

        packets.msgs = messages;
        packets.nmsgs = 1;

        if (ioctl(i2c, I2C_RDWR, &packets) < 0)
        {
            printf("unable to read digital inputs\n");
            exit(1);
        }

        switch (chn)
        {
            case 0:  
                IOLib_DigitalDataA = inbuf[0];
                break;
            case 1:  
                IOLib_DigitalDataB = inbuf[0];
                break;
            case 2:  
                IOLib_DigitalDataC = inbuf[0];
                break;
            case 3:  
                IOLib_DigitalDataD = inbuf[0];
                break;
        }
    }
    
    /* Digital output Reg A */
    outbuf[0] = 9;  /* write pattern to reg 9 */
    outbuf[1] = DigitalOutputA;

    messages[0].addr = DIGOUTA_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = outbuf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to write to Digital output Reg A\n");
        exit(1);
    }

    /* Digital output Reg B */
    outbuf[0] = 9;  /* write pattern to reg 9 */
    outbuf[1] = DigitalOutputB;

    messages[0].addr = DIGOUTB_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = outbuf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to write to Digital output Reg A\n");
        exit(1);
    }

    /* Temperature sensor */
    messages[0].addr = TMP_ADR;
    messages[0].flags = I2C_M_RD;
    messages[0].len = 2;
    messages[0].buf = inbuf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to read temperature sensor\n");
        exit(1);
    }
    IOLib_Temperature = (float) ((inbuf[0] << 8) + inbuf[1]) / 256.0;

    /* bump on the LEDs (unless LEDs are in use for diagnostics */
    if (LEDs_free)
    {
        IOLib_LEDS(pattern);
        LEDs_free = true;  /* reset to free */
    
        if (goingright) 
        {
            pattern = pattern >> 1;
            if (pattern == 0x01)
            {
                goingright = false;
            }
        }
        else
        {
            pattern = pattern << 1;
            if (pattern == 0x80)
            {
                goingright = true;
            }
        }
    }
}

/* ---------------------------------------------------- */    
void IOLib_LEDS(unsigned char pattern)
{
    unsigned char              outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[2];

    outbuf[0] = 9;  /* write pattern to reg 9 */
    outbuf[1] = (unsigned char) ~pattern;  /* remember LEDs are inverted */
    
    messages[0].addr = LEDS_ADR;
    messages[0].flags = 0;
    messages[0].len = 2;
    messages[0].buf = outbuf;

    packets.msgs = messages;
    packets.nmsgs = 1;
    
    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to write to LEDs\n");
        exit(1);
    }

	LEDs_free = false;  /* inhibit moving pattern */
}

/* ---------------------------------------------------- */    

/*
DAC output 0-5V amplified to -10V to 10V 
DAC range -10V = 0, 0v = 2048, +10V = 4095
*/

void DAC(int r, int s, int e, int a)
{
    unsigned char              buf[9];
    int                        val;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg             messages[1];

    val = 4095 - r;
    buf[0] = (unsigned char) (0x50);  /* select DAC channel */
    buf[1] = (unsigned char) ((val >> 8) | 0x90);
    buf[2] = (unsigned char) val & 0xff;

    val = 4095 - s;
    buf[3] = (unsigned char) ((val >> 8) | 0x90);
    buf[4] = (unsigned char) val & 0xff;

    val = 4095 - e;
    buf[5] = (unsigned char) ((val >> 8) | 0x90);
    buf[6] = (unsigned char) val & 0xff;

    val = 4095 - a;
    buf[7] = (unsigned char) ((val >> 8) | 0x90);
    buf[8] = (unsigned char) val & 0xff;

    messages[0].addr = DAC_ADR;
    messages[0].flags = 0;
    messages[0].len = 9;
    messages[0].buf = buf;

    packets.msgs = messages;
    packets.nmsgs = 1;

    if (ioctl(i2c, I2C_RDWR, &packets) < 0)
    {
        printf("unable to write to MCP4728 data registers\n");
        exit(1);
    }
}
    
/* ---------------------------------------------------- */    
void IOLib_Start()  /* wait for AUTOM RESET to be pressed */
{
    unsigned int   p = 0x55;
    struct timeval t;
    int            t1;
    int            t2;
    
    gettimeofday(&t, NULL);
    t1 = t.tv_usec / 500000L;  /* 0.5s units */

    while (1)
    {
        IOLib_UpdateIO(0xff, 0xff);
        if ((IOLib_DigitalDataC & 0x40) == 0)
        {
            break;  /* RPi waits for reset switch to be pressed */
        }
        
        IOLib_LEDS(p);
        
        gettimeofday(&t, NULL);
        t2 = t.tv_usec / 500000L;
        if (t1 != t2)
        {
            t1 = t2;
            p = ~p;
        }
    }
 	LEDs_free = true;  /* inhibit moving pattern */
}

/* ---------------------------------------------------- */    
void Delay(int n)  // n ms delay
{
	int            i;
    int            t1;
    int            t2;
    struct timeval t;

    gettimeofday(&t, NULL);
    t1 = t.tv_usec / 1000L;  /* 1 ms ticks */

    for (i=1; i<=n; i+=1)
	{
        while (1)
        {
            gettimeofday(&t, NULL);
            t2 = t.tv_usec / 1000L;  /* 1 ms ticks */
            if (t1 != t2)
            {
                t1 = t2;
                break;
            }
        }
    }
}

/* ---------------------------------------------------- */    
void BEGIN_IOLib()
{
    unsigned int i;
    
    for (i=0; i<=31; i+=1)
    {
        IOLib_AnalogueData[i] = 0;
    }

    IOLib_DigitalDataA = 0xff;
    IOLib_DigitalDataB = 0xff;
    IOLib_DigitalDataC = 0xff;
    IOLib_DigitalDataD = 0xff;

    IOLib_Sidestick = false;

    LEDs_free = true;
    IOLib_StartIO();
}

/* ---------------------------------------------------- */    
void END_IOLib()
{
    IOLib_StopIO();
}
