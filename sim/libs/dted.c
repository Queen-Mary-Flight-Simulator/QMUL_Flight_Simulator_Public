/*
    DTED library
    DJA 08 November 2019
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <SIM/dted.h>

/*
    Prototypes
*/
double       DTED_Bearing (double, double, double, double);
double       DTED_Distance (double, double, double, double);
void         DTED_GlobeToGrid(double Latitude, double Longitude, double *x, double *y);
void         DTED_Normalise(double *a);
double       PostHeight(double x, double y, bool *found);
double       Degrees (double r);
double       Rads(double a);

/*
    Global data
*/

const double PI          =  M_PI;
const double TWOPI       =  M_PI * 2.0;
const double ONERAD      =  180.0 / M_PI;
const double EarthRadius =  6378137.0;

double       OldPostHeight = 0.0;
double       OldLatitude = 0.0;
double       OldLongitude = 0.0;
double       incline = 0.0;
double       *Posts;

DTED_Record  DTED;

/* ------------------------------------------------------------------ */
void DTED_LoadDTED(char Filename[])
{
    char fname[50];
    FILE *f;
    int dsize;
    
    strcpy(fname, Filename);
    strcat(fname, ".dtd");
    
    if ((f = fopen(fname, "rb")) == NULL)
    {
        printf("Error opening DTED file %s\n", fname);
        return;
    }

    fread(&DTED, sizeof(DTED), 1, f);
    
    dsize = DTED.xPosts * DTED.yPosts * sizeof(double);
    
    if (Posts != NULL)  /* check and remove any loaded DTED */
    {
        free((double *) Posts);
    }

    Posts = (double *) malloc(dsize);
    if (!Posts)
    {
        printf("Error: Insufficient memory for DTED (%d K bytes)\n", dsize / 1000);
        exit(1);
    }
    //printf("DTED_LoadDTED: Malloc: Posts=%p\n", Posts); // ***
    fread(Posts, dsize, 1, f);

    fclose(f);
    //printf("DTED_LoadDTED: bx1=%d by1=%d bx2=%d by2=%d xPosts=%d yPosts=%d\n",   // ***
    //       DTED.bx1, DTED.by1, DTED.bx2, DTED.by2, DTED.xPosts, DTED.yPosts);
}

/* ------------------------------------------------------------------ */
bool DTED_Loaded()
{
    return (Posts != NULL);
}

/* ------------------------------------------------------------------ */
void DTED_Exit()
{
    if (Posts != NULL)
    {
        free((double *) Posts);
    }
}
        
/* ------------------------------------------------------------------ */
double DTED_Incline()
{
    return incline;
}

/* ------------------------------------------------------------------ */
double DTED_PostHeight(double latitude, double longitude, double AirfieldElevation)
{
    double h;

    if (Posts == NULL)
    {
        incline = 0.0;
        return AirfieldElevation;
    }
    else
    {
        double x;
        double y;
        bool   found;
    
        DTED_GlobeToGrid(latitude, longitude, &x, &y);
        h = PostHeight(x, y, &found);
        //printf("DTED_PostHeight: lat=%f long=%f x=%f y=%f h=%f\n", 
        //  Degrees(latitude), Degrees(longitude), x, y, h); // ***
        if (!found)
        {
            incline = 0.0;
            return AirfieldElevation;
        }
        else
        {
            double dx = DTED_Distance(latitude, longitude, OldLatitude, OldLongitude);
            
            if (dx > 1.0)  /* only update incline after 1m of movement */
            {
                double dy = h - OldPostHeight;

                incline = atan2(dy, dx);
                OldLatitude = latitude;
                OldLongitude = longitude;
                OldPostHeight = h;
                //printf("DTED_PostHeight: x=%f y=%f bx1=%d by1=%d bx2=%d by2=%d xPosts=%d yPosts=%d px=%d py=%d p=%d\n",
                //       x, y, DTED.bx1, DTED.by1, DTED.bx2, DTED.by2, DTED.xPosts, DTED.yPosts, px, py, p);
                //printf("dx=%f dy=%f h=%f slope=%f\n", dx, dy, h, Degrees(incline)); // ***
            }
        }
    }
    
    return -h;  /* negative up */
}

/* ------------------------------------------------------------------ */
            /* (x2,y2)    (x3,y3)   */
            /*    +----------+      */
            /*    |          |      */
            /*    |          |      */ 
            /*    |      +   |      */
            /*    |    (x,y) |      */
            /*    |          |      */
            /*    +----------+      */
            /* (x1,y1)     (x4,y4)  */

double PostHeight(double x, double y, bool *found)  /* x, y in model coords */
{
    double res = 0.0;

    if (x < (double) DTED.bx1 || y < (double) DTED.by1 || x > (double) DTED.bx2 || y > DTED.by2)
    {
        *found = false;
    }
    else
    {
        int px = (int) (x - (double) DTED.bx1) / 10;
        int py = (int) (y - (double) DTED.by1) / 10;
        int p = px * DTED.yPosts + py;
        
        if (p < 0 || p >= (DTED.xPosts * DTED.yPosts))
        {
            *found = false;
        }
        else
        {
            double x1, y1, z1;
            double x2, z2;
            double z3;
            double z4;
            double z14;
            double z23;

            *found = true;

            x1   = (double) (DTED.bx1 + px * 10);
            y1   = (double) (DTED.by1 + py * 10);
            z1   = Posts[p];
            x2   = x1;
            z2   = (double) Posts[p+1];
            z3   = (double) Posts[p+1+DTED.yPosts];
            z4   = (double) Posts[p+DTED.yPosts];
            z14  = z1 + (x - x1) * (z4 - z1) / 10.0;
            z23  = z2 + (x - x2) * (z3 - z2) / 10.0;

            res = z14 +  (y - y1) * (z23 - z14) / 10.0;
            //printf("GetPostHeight: x=%f y=%f bx1=%d by1=%d bx2=%d by2=%d xPosts=%d yPosts=%d px=%d py=%d p=%d\n",
            //       x, y, DTED.bx1, DTED.by1, DTED.bx2, DTED.by2, DTED.xPosts, DTED.yPosts, px, py, p);
            //printf("x1=%f y1=%f z1=%f x2=%f z2=%f z3=%f z4=%f z14=%f z23=%f HAT=%f\n", 
            //        x1,   y1,   z1,   x2,   z2,   z3,   z4,   z14,   z23,   res);
        }
    }
    
    return res;
}
        
/* ---------------------------------------------------- */
double Degrees(double r)     /* radians -> degrees */
{
    return r * (double) ONERAD;
}

/* ---------------------------------------------------- */
double Rads(double d)     /* degrees -> radians */
{
    return d / (double) ONERAD;
}

/* ---------------------------------------------------- */
double DTED_Distance(double Lat1, double Long1, double Lat2, double Long2)
{  
    double dLat;
    double dLong;
    double d;

    dLat = Lat2 - Lat1;
    dLong = Long2 - Long1;
    d = EarthRadius * sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
    return d;
}

/* ---------------------------------------------------- */
double DTED_Bearing(double Lat1, double Long1, double Lat2, double Long2)
{
    double x, y;
    double dLat;
    double dLong;
    double d;

    dLat = Lat2 - Lat1;
    dLong = Long2 - Long1;
    d = sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
    x = sin(Lat2) - sin(Lat1) * cos(d);
    y = cos(Lat2) * sin(dLong) * cos(Lat1);
    return atan2(y, x);
}

/* ---------------------------------------------------- */
void DTED_GlobeToGrid(double lat1, double long1, double *x, double *y)
{
    double lat0 = Rads(DTED.RLAT); // (lat0, long0) is the runway datum
    double long0 = Rads(DTED.RLONG); 
    double d = DTED_Distance(lat1, long1, lat0, long0);
    double b = DTED_Bearing(lat1, long1, lat0, long0);
    double a = TWOPI - Rads(DTED.RQDM) - DTED.RunwayRotation;
    double sina;
    double cosa;
    double x1;
    double y1;
    
    DTED_Normalise(&a);
    sina = sin(a);
    cosa = cos(a);
    
    x1 = -d * sin(b);
    y1 = -d * cos(b);
    
    *x =  x1 * cosa + y1 * sina + DTED.RunwayX;
    *y = -x1 * sina + y1 * cosa + DTED.RunwayY;
}

/* ------------------------------------------------------ */
void DTED_Normalise(double *a)
{
    double x = fmod(*a + M_PI, TWOPI);
    
    if (x < 0.0)
    {
        x += TWOPI;
    }
    *a = x - M_PI;
}

/* ------------------------------------------------------ */
void BEGIN_DTED()
{
    Posts = NULL;
}
