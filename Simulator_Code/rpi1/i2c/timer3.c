#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

long unsigned int Timer1;
long unsigned int Timer2;

struct timeval tv;
unsigned int t;

int main()
{
    gettimeofday(&tv, NULL);
    Timer1 = tv.tv_usec;
    t = 0;

    while (1)
    {
        do
        {
            gettimeofday(&tv, NULL);
            Timer2 = tv.tv_usec;
            if (Timer2 < Timer1)
            {
                Timer2 += 1000000L;
            }
        } while (Timer2 <= (Timer1 + 20000L));
        Timer1 = (Timer1 + 20000L) % 1000000L;

        t += 1;
        if (t >= 50)
        {
            printf("%u\n", tv.tv_usec);
            t = 0;
        }
    }
}
