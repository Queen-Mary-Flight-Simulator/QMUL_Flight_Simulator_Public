#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <SIM/glib.h>

#include "ai.h"
#include "systems.h"

#define AiHeight           250
#define AiWidth            250

#define ONERAD             (180.0 / M_PI)
#define TWOPI              (2.0 * M_PI)
#define PIBY2              (0.5 * M_PI)

#define BankAngleRadius    114

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

void GenerateSphere(int n, float r);
void DrawSphere(int n);

/* ------------------------------------------------- */
void Ai_AttitudeIndicator(int AiX, int AiY, float Pitch, float Roll, int texobj1, int texobj2)
{
    float Pitch_Deg;
    float Roll_Deg;

    Pitch_Deg = -Pitch * ONERAD;
    Roll_Deg  = Roll * ONERAD;

    Glib_PushMatrix();
    Glib_LoadIdentity();
    Glib_Translate((float) AiX, (float) AiY);
    Glib_RotateYaw3D(-90.0f);
    Glib_RotateRoll3D(Roll_Deg);
    Glib_RotatePitch3D(-Pitch_Deg);
    Glib_SetNormal();
    Glib_SetTexture(texobj1);
    glBindTexture(GL_TEXTURE_2D, texobj1);
    DrawSphere(36);
    Glib_PopMatrix();
    
    Glib_SetTexture(texobj2);
    Glib_DrawTexture(AiX - 160, AiY - 160, 320, 320, 0.0, 0.0, 1.0, 1.0, 1.0);

    Glib_PushMatrix();
	Glib_Colour(Glib_WHITE);
    Glib_Translate((float) AiX, (float) AiY);
    Glib_Rotate(Roll_Deg);
    Glib_Triangle(0, BankAngleRadius, -10, BankAngleRadius - 20, 10, BankAngleRadius - 20);
    Glib_PopMatrix();
}

/* ------------------------------------------------- */
void DrawSphere(int n)
{
    int i, j;
    int k = 0;
    
    Glib_Flush(); /* flush existing graphics */
    
    for (j=0; j<n/2; j+=1)  /* -90 -> +90 */
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
/* Generate and store local coords, normals and texture uv for a sphere
   n = precision
   r = sphere radius
*/
void GenerateSphere(int n, float r)
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

/* ------------------------------------------------- */
void BEGIN_Ai()
{
    GenerateSphere(36, 140.0f);
}
