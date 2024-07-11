#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <GL/gl.h>

#include <SIM/glib.h>

#include "nfd.h"
#include "magcompass.h"

#define ONERAD (180.0f / M_PI)
#define TWOPI  (M_PI * 2.0f)
#define PIBY2  (M_PI / 2.0f)

typedef struct
{
    float u;
    float v;
} UV;

typedef struct
{
    double x;
    double y;
    double z;
} XYZ;

static XYZ *sphere_pos_coords = NULL;
static XYZ *sphere_nor_coords = NULL;
static UV  *sphere_tex_coords = NULL;
static int nvertices = 0;

void Compass_GenerateSphere(int n, float r);
void Compass_DrawSphere(int n);

/*---------------------------------------------------------------------------*/
void MagneticCompass_Compass(int CompassX, int CompassY, float Hdg, GLuint texobj1, GLuint texobj2)
{
    float HdgDeg;

    HdgDeg = Hdg * ONERAD;
	
    Glib_PushMatrix();
    Glib_LoadIdentity();
    Glib_Translate((float) NFD_MagCompassX, (float) NFD_MagCompassY);
    Glib_RotateYaw3D(-90.0f);
    Glib_RotateYaw3D(-HdgDeg);
	Glib_SetNormal();
	Glib_SetTexture(texobj1);
    glBindTexture(GL_TEXTURE_2D, texobj1);
    Compass_DrawSphere(36);
	Glib_PopMatrix();

    Glib_SetTexture(texobj2);
    Glib_DrawTexture(CompassX - 128, CompassY - 128, 256, 256, 0.0, 0.0, 1.0, 1.0, 1.0);
}

/*---------------------------------------------------------------------------*/
void Compass_DrawSphere(int n)
{
    int i, j;
    int k = 0;

    Glib_Flush();
	
    for (j=0; j<n/2; j+=1)
    {
        for (i=0; i<=n; i+=1)
        {
            Glib_DrawSegment(sphere_pos_coords[k].x, sphere_pos_coords[k].y, sphere_pos_coords[k].z,
                             sphere_nor_coords[k].x, sphere_nor_coords[k].y, sphere_nor_coords[k].z,
                             sphere_tex_coords[k].u, sphere_tex_coords[k].v);
		    k += 1;
            Glib_DrawSegment(sphere_pos_coords[k].x, sphere_pos_coords[k].y, sphere_pos_coords[k].z,
                             sphere_nor_coords[k].x, sphere_nor_coords[k].y, sphere_nor_coords[k].z,
                             sphere_tex_coords[k].u, sphere_tex_coords[k].v);
			k += 1;
		}
        Glib_Flush(); /* flush triangle strip */
    }
}

/*---------------------------------------------------------------------------*/
/* Generate and store local coords, texture uv and normals for a sphere */
/*
   n = precision
   r = sphere radius
 */
void Compass_GenerateSphere(int n, float r)
{
    int i, j;
    int k = 0;
    
    nvertices = n * (n + 1);
    sphere_pos_coords = malloc( sizeof(XYZ) * nvertices);
    sphere_nor_coords = malloc( sizeof(XYZ) * nvertices);
    sphere_tex_coords = malloc( sizeof(UV) * nvertices);
    
    if (sphere_pos_coords == NULL || sphere_nor_coords == NULL || sphere_tex_coords == NULL) 
    {
        printf("GenerateSphere : memory allocation error\n");
        return;
    }

    for (j=0; j<n/2; j+=1)  /* -90 -> +90 */
    {
        float lat1 = (float) j * 2.0 * M_PI / (float) n - M_PI / 2.0;
        float lat2 = (float) (j + 1) * 2.0 * M_PI / (float) n - M_PI / 2.0;
        
        for (i=0; i<=n; i+=1)   /* 0 -> 360 */
        {
            float long1 = (float) i * 2.0 * M_PI / (float) n;
            GLfloat x, y, z;
            
            x = r * cos(lat2) * cos(long1);
            y = r * sin(lat2);
            z = r * cos(lat2) * sin(long1);
            sphere_pos_coords[k].x = x;
            sphere_pos_coords[k].y = y;
            sphere_pos_coords[k].z = z;
            sphere_nor_coords[k].x = x / r;
            sphere_nor_coords[k].y = y / r;
            sphere_nor_coords[k].z = z / r;
            sphere_tex_coords[k].u = (float) i / (float) n;
            sphere_tex_coords[k].v = 2.0 * (float) (j+1) / (float) n;
            k += 1;

            x = r * cos(lat1) * cos(long1);
            y = r * sin(lat1);
            z = r * cos(lat1) * sin(long1);
            sphere_pos_coords[k].x = x;
            sphere_pos_coords[k].y = y;
            sphere_pos_coords[k].z = z;
            sphere_nor_coords[k].x = x / r;
            sphere_nor_coords[k].y = y / r;
            sphere_nor_coords[k].z = z / r;
            sphere_tex_coords[k].u = (float) i / (float) n;
            sphere_tex_coords[k].v = 2.0 * (float) j / (float) n;
            k += 1;
        }
    }
    printf("k=%d nvertices=%d\n", k, nvertices); // ***
}

/*---------------------------------------------------------------------------*/
void BEGIN_MagneticCompass()
{
    //MagCompass_GenerateSphere(32, 140.0f); // for 320x320
    Compass_GenerateSphere(36, 100.0f);
}
