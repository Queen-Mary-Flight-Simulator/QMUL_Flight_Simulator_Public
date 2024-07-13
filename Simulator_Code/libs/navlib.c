/* +------------------------------+---------------------------------+
   | Module      : navlib.c       | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-07              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : navigation database library                      |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include <SIM/fileio.h>
#include <SIM/maths.h>
#include <SIM/navdefn.h>
#include <SIM/navlib.h>

#define UK_MinLatitude            49.0
#define UK_MaxLatitude            61.0
#define UK_MinLongitude           -8.0
#define UK_MaxLongitude           2.0

#define Australia_MinLatitude     -39.0
#define Australia_MaxLatitude     -20.0
#define Australia_MinLongitude    142.0
#define Australia_MaxLongitude    153.0

#define HashTableSize             500

#define                           ONERAD (180.0L / M_PI)
#define DEG80                     (80.0 / ONERAD)

typedef struct 
{
    unsigned int         Length;          /* m */
    unsigned int         Width;           /* not used */
    unsigned int         ILSmode;         /* 0=no ILS, 1=ILS */
    unsigned int         Frequency;       /* iff ILS */
    float                Latitude;        /* rads */
    float                Longitude;       /* rads */
    unsigned int         Elevation;       /* m */
    float                Rqdm;            /* QDM true */
    unsigned int         Runway;          /* 23R etc */
} NavLib_RunwayInfo;

unsigned int             NavLib_NumberOfBeacons;
unsigned int             NavLib_NumberOfRunways;
unsigned int             NavLib_NumberOfWayPoints;
unsigned int             NavLib_NextWayPoint;

NavLib_BeaconRecord      NavLib_Beacons[NavLib_MaxBeacons];
NavLib_RunwayRecord      NavLib_Runways[NavLib_MaxRunways];
NavLib_WayPointRecord    NavLib_WayPoints[NavLib_MaxWayPoints];

static NavLib_RunwayInfo LocalRunways[20];
static unsigned int      nLocalRunways;
static unsigned int      LineNumber;
static FILE              *FileStream;
static unsigned int      HashTable[HashTableSize];

static bool         Digit(int Ch);
static bool         Alphabetic(int Ch);
unsigned int        ReadWayPointIdent();
static float        ReadLatitude();
static float        ReadLongitude();
static void         Error(char *fmt, ...);
char*               BeaconType(unsigned int t);
static unsigned int Hash(unsigned int f);
static void         InitialiseHashTable();
static bool         Runways_Read;

/* --------------------------------------------------- */ 
unsigned int NavLib_NearestRunway(float latitude, float longitude)  /* latitude rads, longitude rads */
{
    unsigned int i;
    double       MinDistance = 1000000.0;
    unsigned int Nearest     = 0;

    for (i=1; i<=NavLib_NumberOfRunways; i+=1)
    {
        double d = NavLib_Distance((double) latitude, (double) longitude, (double) NavLib_Runways[i].RunwayLatitude, (double) NavLib_Runways[i].RunwayLongitude);
        
        if (d < MinDistance && (int) NavLib_Runways[i].Frequency > 0)
        {
            MinDistance = d;
            Nearest = i;
        }
    }
    if (Nearest > 0)
	{
        return Nearest;
	}

    MinDistance = 1000000.0;
    Nearest     = 0;
    for (i=1; i<=NavLib_NumberOfRunways; i+=1)
    {
        double d = NavLib_Distance((double) latitude, (double) longitude, (double) NavLib_Runways[i].RunwayLatitude, (double) NavLib_Runways[i].RunwayLongitude);
        
        if (d < MinDistance)
        {
            MinDistance = d;
            Nearest = i;
        }
    }
    return Nearest;
}

/* --------------------------------------------------------- */
static void Error(char *fmt, ...)
{
    va_list list;
    char    *p, *r;
    int     e;
    float   f;

    va_start(list, fmt);

    printf("Error at line %d\n", LineNumber);

    for (p = fmt; *p; ++p)
    {
        if (*p != '%')
        {
            printf("%c", *p);
        }
        else
        {
            switch (*++p)
            {
            case 's':
            {
                r = va_arg(list, char *);
                printf("%s", r);
                continue;
            }

            case 'i':
            {
                e = va_arg(list, int);
                printf("%i", e);
                continue;
            }

            case 'd':
            {
                e = va_arg(list, int);
                printf("%d", e);
                continue;
            }

            case 'f':
            {
                f = va_arg(list, double);
                printf("%f", f);
                continue;
            }

            default:
                printf("%c", *p);
            }
        }
    }

    va_end(list);

    exit(EXIT_FAILURE);
}

/* --------------------------------------------------------------------- */
double NavLib_Distance(double Lat1, double Long1, double Lat2, double Long2)
{
    double dLat;
    double dLong;

    dLat  = Lat2 - Lat1;
    dLong = Long2 - Long1;
    return EarthRadius * sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
}

/* --------------------------------------------------------------------- */
double NavLib_Bearing(double Lat1, double Long1, double Lat2, double Long2)
{
    double x, y;
    double dLat;
    double dLong;
    double d;

    dLat  = Lat2 - Lat1;
    dLong = Long2 - Long1;
    d     = sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
    x     = sin(Lat2) - sin(Lat1) * cos(d);
    y     = cos(Lat2) * sin(dLong) * cos(Lat1);
    return atan2(y, x);
}

/* --------------------------------------------------------------------- */
unsigned int ReadWayPointIdent(char a[])
{
    int      ch;
    unsigned int i;

    for (i = 0; i <= 3; i += 1)
    {
        do
        {
            ch = FileIO_Rdch();
        } while (ch == ' ' || ch == EOL);
        if (ch == EOF)
        {
            return 0;
        }
        if (!(Alphabetic(ch) || Digit(ch)))
        {
            Error("Bad WayPoint ID (%c)\n", ch);
        }
        a[i] = (char) ch;
    }
    ch = FileIO_Rdch();
    return 4;
}

/* --------------------------------------------------------------------- */
static bool Alphabetic(int Ch)
{
    return Ch >= 'A' && Ch <= 'Z';
}

/* --------------------------------------------------------------------- */
static bool Digit(int Ch)
{
    return Ch >= '0' && Ch <= '9';
}

/* --------------------------------------------------------------------- */
static float ReadLatitude()
{
    float f = FileIO_ReadFloat();

    if (f < -90.0 || f > 90.0)
    {
        Error("Bad latitude (%f)\n", f);
    }
    return f;
}

/* --------------------------------------------------------------------- */
static float ReadLongitude()
{
    float f = FileIO_ReadFloat();

    if (f < -180.0 || f > 180.0)
    {
        Error("Bad longitude (%f)\n", f);
    }
    return f;
}

/* --------------------------------------------------------------------- */
void NavLib_InitNavLib()
{
//    unsigned int i;
    
    printf("Nav Init\n");
    NavLib_ReadRunways(UK_MinLatitude, UK_MaxLatitude, UK_MinLongitude, UK_MaxLongitude); /* must go before ReadBeacons */
    NavLib_ReadBeacons(UK_MinLatitude, UK_MaxLatitude, UK_MinLongitude, UK_MaxLongitude);
    InitialiseHashTable();

    printf("%5d beacons\n%5d runways\n", NavLib_NumberOfBeacons, NavLib_NumberOfRunways);

    //NavLib_ReadRunways(Australia_MinLatitude, Australia_MaxLatitude, Australia_MinLongitude, Australia_MaxLongitude);
    //NavLib_ReadBeacons(Australia_MinLatitude, Australia_MaxLatitude, Australia_MinLongitude, Australia_MaxLongitude);

/*
    for (i=1; i<=NavLib_NumberOfRunways; i+=1)
    {
        printf("%d %s: lat=%f long=%f elevation=%d len=%d width=%d qdm=%f freq=%d\n", i, 
          NavLib_Runways[i].Name,
          Maths_Degrees(NavLib_Runways[i].RunwayLatitude), 
          Maths_Degrees(NavLib_Runways[i].RunwayLongitude), 
          NavLib_Runways[i].Elevation,
          NavLib_Runways[i].Length, 
          NavLib_Runways[i].Width,
          Maths_Degrees(NavLib_Runways[i].Qdm),
          NavLib_Runways[i].Frequency);
    }
//
    for (i=1; i<=NavLib_NumberOfBeacons; i+=1)
    {
        printf("%d: id=%s lat=%f long=%f freq=%d range=%d alt=%d qdm=%f %s\n", i,
          NavLib_Beacons[i].Ident,
          Maths_Degrees(NavLib_Beacons[i].BeaconLatitude),
          Maths_Degrees(NavLib_Beacons[i].BeaconLongitude),
          NavLib_Beacons[i].Frequency,
          NavLib_Beacons[i].Range,
          NavLib_Beacons[i].Elevation,
          Maths_Degrees(NavLib_Beacons[i].Qdm),
          BeaconType(NavLib_Beacons[i].Navaid));
    }
*/
}

/* --------------------------------------------------------------------- */
char* BeaconType(unsigned int t)
{
    switch (t)
    {
    case NavLib_DME:
        return "DME";
        break;
    case NavLib_VOR:
        return "VOR";
        break;
    case NavLib_ILS:
        return "ILS";
        break;
    case NavLib_VOR | NavLib_DME:
        return "VOR/DME";
        break;
    case NavLib_ILS | NavLib_DME:
        return "ILS/DME";
        break;
    case NavLib_NDB:
        return "NDB";
        break;
    default:
        printf("Unrecognised bcn %x\n", t);
        return "???";
        break;
    }
}

/* --------------------------------------------------------------------- */
void NavLib_ReadBeacons(float minlat, float maxlat, float minlong, float maxlong)  /* args in degrees */
{
    char         ident[50];
    char         name[200];
    unsigned int runway;
    float        freq;
    unsigned int range;
    unsigned int elevation;
    float        latitude;
    float        longitude;
    int          vor_flag;
    int          dme_flag;
    char         d1[100];
    
    if (!Runways_Read)
    {
        Error("Runways database must be read before navigation database\n");
        exit(1);
    }
    
    NavLib_NumberOfBeacons = 0;
    LineNumber             = 1;

    FileStream = FileIO_Open("../files/Navaids.txt");
    if (FileStream == NULL)
    {
        Error("Can't open navaids.txt");
        exit(1);
    }

    FileIO_Select(FileStream);
    
    while (1)
    {
        if (FileIO_ReadString(ident, ',') == EOF)
        {
            break; /* EOF encountered */
        }
        
        FileIO_ReadString(name, ',');
        if (ident[0] == 'I') /* ILS */
        {
            unsigned int p = 0;

            while (1)
            {
                if (name[p] == ' ' && name[p+1] == 'R' && name[p+2] == 'W')
                {
                    runway = (name[p+3] - '0') * 10 + name[p+4] - '0';
                    if (name[p+5] == 'L')
                    {
                        runway += 100;
                    }
                    else if (name[p+5] == 'R')
                    {
                        runway += 200;
                    }
                    break; 
                }
                else
                {
                    p += 1;
                    if (p > 5)
                    {
                        runway = 0;
                        break;
                    }
                }
            }
        }
        else
        {
            runway = 0;
        }
        
        freq      = FileIO_ReadFloat();
        vor_flag  = FileIO_ReadInt();
        dme_flag  = FileIO_ReadInt();
        range     = FileIO_ReadInt();
        range     = 25;                  /* range value unuseable - set to 25 nm instead */
        latitude  = FileIO_ReadFloat();  /* degrees - DME location mid-runway */
        longitude = FileIO_ReadFloat();  /* degrees */
        elevation =  Maths_Metres(FileIO_ReadInt());
        FileIO_ReadString(d1, ',');      /* country code */
        FileIO_ReadInt();                /* skip last number (mag/true?) */

        if (latitude > minlat && latitude < maxlat && longitude > minlong && longitude < maxlong)
        {
            unsigned int t = 0;

            NavLib_NumberOfBeacons += 1;
            if (NavLib_NumberOfBeacons > NavLib_MaxBeacons)
            {
                Error("Too many beacons (%d)\n", NavLib_MaxBeacons);
            }

            if (freq >= 190.0 && freq <= 1750.0)
            {
                if (dme_flag == 0 && vor_flag == 0)
                {
                    t = NavLib_NDB;
                }
                else
                {
                    printf("Navlib: NDB %s with invalid VOR or DME flag\n", ident); 
                }
            }
            if (dme_flag == 1)
            {
                t |= NavLib_DME;
            }
            if (vor_flag == 1)
            {
                t |= NavLib_VOR;
            }
            if (ident[0] == 'I')
            {
                t |= NavLib_ILS;
            }

            strcpy(NavLib_Beacons[NavLib_NumberOfBeacons].Ident, ident);
            
            NavLib_Beacons[NavLib_NumberOfBeacons].Runway          = runway;
            NavLib_Beacons[NavLib_NumberOfBeacons].BeaconLatitude  = Maths_Rads(latitude);
            NavLib_Beacons[NavLib_NumberOfBeacons].BeaconLongitude = Maths_Rads(longitude);
            if ((t & NavLib_NDB) != 0)
            {
                NavLib_Beacons[NavLib_NumberOfBeacons].Frequency = (unsigned int) (freq * 10.0 + 0.1);
            }
            else
            {
                NavLib_Beacons[NavLib_NumberOfBeacons].Frequency = (unsigned int) (freq * 100.0 + 0.1);
            }
            NavLib_Beacons[NavLib_NumberOfBeacons].Range     = (t & NavLib_NDB) ? range : 0;
            NavLib_Beacons[NavLib_NumberOfBeacons].Qdm       = 0;
            NavLib_Beacons[NavLib_NumberOfBeacons].Elevation = (int) Maths_Metres(elevation);
            NavLib_Beacons[NavLib_NumberOfBeacons].Navaid    = t;

            if (t & NavLib_ILS)
            {
                unsigned int ils = 0;
                unsigned int r;
                double       dmin = 1000000.0;
                
                for (r=1; r<=NavLib_NumberOfRunways; r+=1)
                {
                     if (NavLib_Beacons[NavLib_NumberOfBeacons].Frequency == NavLib_Runways[r].Frequency)
                     {
                         double d = NavLib_Distance(NavLib_Beacons[NavLib_NumberOfBeacons].BeaconLatitude,
                                                    NavLib_Beacons[NavLib_NumberOfBeacons].BeaconLongitude,
                             	 	 			    NavLib_Runways[r].RunwayLatitude,
                                                    NavLib_Runways[r].RunwayLongitude);
                         if (d < dmin)
                         {
                             ils = r;
                             dmin = d;
                         }
                     }
                }
                
                if (ils > 0)
                {
                    NavLib_Beacons[NavLib_NumberOfBeacons].BeaconLatitude  = NavLib_Runways[ils].RunwayLatitude;
                    NavLib_Beacons[NavLib_NumberOfBeacons].BeaconLongitude = NavLib_Runways[ils].RunwayLongitude;
                    NavLib_Beacons[NavLib_NumberOfBeacons].Qdm = NavLib_Runways[ils].Qdm;
                    NavLib_Beacons[NavLib_NumberOfBeacons].Elevation = NavLib_Runways[ils].Elevation;
                    //printf("ILS modified: %s lat=%f long=%f qdm=%f\n", NavLib_Beacons[NavLib_NumberOfBeacons].Ident,
                    //    Maths_Degrees(NavLib_Runways[ils].RunwayLatitude), Maths_Degrees(NavLib_Runways[ils].RunwayLongitude), Maths_Degrees(NavLib_Runways[ils].Qdm));
                }
            }
        }
    }
    
    FileIO_Close(FileStream);
}

/* --------------------------------------------------------------------- */
void NavLib_ReadRunways(float minlat, float maxlat, float minlong, float maxlong)  /* args in degrees */
{
    char         rec_id[10];
    char         ICAO_code[6];
    char         name[50];
    float        latitude;
    float        longitude;
    
    NavLib_NumberOfRunways = 0;
    LineNumber             = 1;
    Runways_Read           = true;
    
    FileStream = FileIO_Open("../files/Airports.txt");
    if (FileStream == NULL)
    {
        Error("Can't open Airports.txt\n");
        exit(1);
    }

    FileIO_Select(FileStream);
    
    FileIO_SkipLine();
    FileIO_SkipLine();

    while (1)
    {
        unsigned int i;

        if (FileIO_ReadString(rec_id, ',') == EOF)
        {
            break;  /* EOF encountered */
        }
        
        if (rec_id[0] != 'A')
        {
            if (rec_id[0] != 0)
            {
                FileIO_SkipLine();
            }
            continue;
        }

        FileIO_ReadString(ICAO_code, ',');
        FileIO_ReadString(name, ',');
        
        if (strlen(name) >= NavLib_NameSize)
        {
            name[NavLib_NameSize - 1] = '\0'; /* allow room for e.g. " 27L" */
        }
        
        latitude  = FileIO_ReadFloat();
        longitude = FileIO_ReadFloat();
        FileIO_ReadInt();  /* skip airfield elevation */
        FileIO_ReadInt();  /* skip transition altitude */
        FileIO_ReadInt();  /* skip unknown */
        FileIO_ReadInt();  /* skip longest runway */
        FileIO_ReadInt();  /* skip mag var M or T */

        if (latitude > minlat && latitude < maxlat && longitude > minlong && longitude < maxlong)
        {
            int runway = 0;
			
			nLocalRunways = 0;
            
            while (1)
            {
                char str[20];
                
                if (FileIO_ReadString(str, ',') == EOF)
                {
                    break; /* EOF encountered */
                }
                
                if (str[0] == 'R')
                {
                    nLocalRunways += 1;
                    
                    FileIO_ReadString(str, ',');
                    LocalRunways[nLocalRunways].Runway = (str[0] - '0') * 10 + str[1] - '0';
                    if (str[2] == 'L')
                    {
                        LocalRunways[nLocalRunways].Runway += 100;
                    } 
                    else if (str[2] == 'R')
                    {
                        LocalRunways[nLocalRunways].Runway += 200;
                    } 

                    LocalRunways[nLocalRunways].Rqdm       = Maths_Rads((float) FileIO_ReadInt());
                    LocalRunways[nLocalRunways].Length     = (int) Maths_Metres(FileIO_ReadInt());
                    LocalRunways[nLocalRunways].Width      = (int) Maths_Metres(FileIO_ReadInt());
                    LocalRunways[nLocalRunways].ILSmode    = FileIO_ReadInt();
                    LocalRunways[nLocalRunways].Frequency  = (unsigned int) (FileIO_ReadFloat() * 100.0 + 0.1);
                    FileIO_ReadInt();  /* skip ILS QDM */
                    LocalRunways[nLocalRunways].Latitude   = Maths_Rads(FileIO_ReadFloat());
                    LocalRunways[nLocalRunways].Longitude  = Maths_Rads(FileIO_ReadFloat());
                    LocalRunways[nLocalRunways].Elevation = (int) Maths_Metres(FileIO_ReadInt());
                    FileIO_ReadFloat(); /* skip g/s angle */
                    FileIO_ReadInt();   /* skip threshold overfly ht */
                    FileIO_ReadInt();   /* skip surface type */
                    FileIO_ReadInt();   /* skip runway status */
                }
                else
                {
                    break;
                }
            }
            
            for (i=1; i<=nLocalRunways; i+=1)
            {
			    if ((int) LocalRunways[i].Frequency > 0)
				{
				    runway = i;  /* first ILS */
					break;
				}
			}
			
            NavLib_NumberOfRunways += 1;
            if (NavLib_NumberOfRunways > NavLib_MaxRunways)
            {
                Error("Too many runways (%d)\n", NavLib_NumberOfRunways);
            }

            if (runway == 0)
			{
			    runway = 1;  /* pick first runway (non ILS) */
			}
            NavLib_Runways[NavLib_NumberOfRunways].RunwayLatitude  = LocalRunways[runway].Latitude;
            NavLib_Runways[NavLib_NumberOfRunways].RunwayLongitude = LocalRunways[runway].Longitude;
            NavLib_Runways[NavLib_NumberOfRunways].Frequency       = LocalRunways[runway].Frequency;
            NavLib_Runways[NavLib_NumberOfRunways].Qdm             = LocalRunways[runway].Rqdm;
            NavLib_Runways[NavLib_NumberOfRunways].Length          = LocalRunways[runway].Length;
            NavLib_Runways[NavLib_NumberOfRunways].Width           = LocalRunways[runway].Width;
            NavLib_Runways[NavLib_NumberOfRunways].Elevation       = LocalRunways[runway].Elevation;
            NavLib_Runways[NavLib_NumberOfRunways].Runway          = LocalRunways[runway].Runway;
            strcpy(NavLib_Runways[NavLib_NumberOfRunways].Name, name);
/*				
                printf("%s %s: Lat=%f Long=%f Freq=%d QDM=%f Length=%d Width=%d Elevation=%d\n", 
                    NavLib_Runways[NavLib_NumberOfRunways].Name, 
                    LocalRunways[i].Rwy,
                    Maths_Degrees(NavLib_Runways[NavLib_NumberOfRunways].RunwayLatitude),
                    Maths_Degrees(NavLib_Runways[NavLib_NumberOfRunways].RunwayLongitude),
                    NavLib_Runways[NavLib_NumberOfRunways].Frequency,
                    NavLib_Runways[NavLib_NumberOfRunways].Qdm,
                    NavLib_Runways[NavLib_NumberOfRunways].Length,
                    NavLib_Runways[NavLib_NumberOfRunways].Width,
                    NavLib_Runways[NavLib_NumberOfRunways].Elevation);
*/
        }
    }
    
    FileIO_Close(FileStream);
}

/* --------------------------------------------------------------------- */
void NavLib_ReadFlightPlan(char Filename[])
{
    unsigned int i;
    char         str[100];

    //printf("NavLib_ReadFlightPlan %s\n", Filename); // ***
    
    NavLib_NumberOfWayPoints = 0;
    NavLib_NextWayPoint      = 0;
    LineNumber               = 0;

    FileStream = FileIO_Open(Filename);
    if (FileStream == NULL)
    {
        Error("Can't open flight plan file: %s", Filename);
        return;
    }

    FileIO_Select(FileStream);
    
    while (1)
    {
        FileIO_ReadString(str, ' ');
        if (FileIO_EOF())
        {
            break;
        }

        LineNumber += 1;
        NavLib_NumberOfWayPoints += 1;
        if (NavLib_NumberOfWayPoints > NavLib_MaxWayPoints)
        {
            Error("Too many waypoints (%d)\n", NavLib_MaxWayPoints);
        }

        for (i = 0; i <= 3; i += 1)
        {
            NavLib_WayPoints[NavLib_NumberOfWayPoints].WayPointID[i] = str[i];
        }
        NavLib_WayPoints[NavLib_NumberOfWayPoints].WayPointLatitude  = ReadLatitude();
        NavLib_WayPoints[NavLib_NumberOfWayPoints].WayPointLongitude = ReadLongitude();
        NavLib_WayPoints[NavLib_NumberOfWayPoints].WayPointAltitude  = FileIO_ReadInt();
        NavLib_WayPoints[NavLib_NumberOfWayPoints].WayPointSpeed     = FileIO_ReadInt();
    }
    FileIO_Close(FileStream);
}

/* ------------------------------------------------------ */
static unsigned int Hash(unsigned int f)
{
    unsigned int p;

    p = (f * 137 + 92731) % HashTableSize;
    return p;
}

/* ------------------------------------------------------ */
static void InitialiseHashTable()
{
    unsigned int i, f, p;

    for (i = 0; i <= HashTableSize - 1; i += 1) 
    {
        HashTable[i] = 0;
    }
 
    for (i = 1; i <= NavLib_NumberOfBeacons; i += 1) 
    {
        f = NavLib_Beacons[i].Frequency;
        p = Hash(f);
        while (HashTable[p] != 0) 
        {
            p = p + 1;
            if (p >= HashTableSize) 
            {
                p = 0;
            }
        }
        HashTable[p] = i;
    }
}

/* ------------------------------------------------------ */
unsigned int NavLib_LookupChannel(float Latitude, float Longitude, unsigned int f,
                                  NavLib_BeaconType BType)
{
    unsigned int p;
    float d;
    float Maxd;
    unsigned int Result;
    unsigned int b;
    unsigned int i;

    p = Hash(f);
    Maxd = 160000.0;
    Result = 0;
    
    for (i = 1; i <= HashTableSize; i += 1) 
    {
        b = HashTable[p];
        if (b == 0) 
        {
            return Result;
        }

        if ((NavLib_Beacons[b].Frequency == f) && (BType & NavLib_Beacons[b].Navaid)) 
        {
            d = NavLib_Distance(Latitude, Longitude,
                                NavLib_Beacons[b].BeaconLatitude, 
                                NavLib_Beacons[b].BeaconLongitude);
            if (d < Maxd) 
            {
                Maxd = d;
                Result = b;
            }
        }

        p = p + 1;
        if (p >= HashTableSize) 
        {
            p = 0;
        }
    }
    return Result;
}

/* --------------------------------------------------------------------- */
void BEGIN_NavLib(void)
{
    Runways_Read = false;
    NavLib_InitNavLib();
}
