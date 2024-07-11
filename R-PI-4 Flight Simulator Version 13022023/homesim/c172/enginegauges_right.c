#include <math.h>
#include <stdio.h>

#include <SIM/glib.h>
#include <SIM/maths.h>

#include "nfd.h"
#include "enginegauges_right.h"

/* ---------------------------------- */
void EngineGauges_RFuelQty(int x0, int y0, float fuelq, GLuint texobj_gauge, GLuint texobj_needle)
{
    float fq           = 0.0;
    float needle_start = -40.0f;
    float needle_rot   = 0.0f;

    fq         = fuelq * 0.32;
    needle_rot = needle_start + fq;

    Glib_SetTexture(texobj_gauge);
    Glib_DrawTexture(x0-64, y0-64, 128, 128, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_ClipWindow(NFD_FuelQRX - 64, NFD_FuelQRY + 24, 128, 32);
	Glib_SetTexture(texobj_needle);
    Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, needle_rot, 0.0);
    Glib_RemoveClipWindow();
}

/* ---------------------------------- */
void EngineGauges_LFuelQty(int x0, int y0, float fuelq, GLuint texobj_gauge, GLuint texobj_needle)
{
    float fq           = 0.0;
    float needle_start = -40.0f;
    float needle_rot   = 0.0f;

    fq         = fuelq * 0.32;
    needle_rot = needle_start + fq;

    Glib_SetTexture(texobj_gauge);
    Glib_DrawTexture(x0-64, y0-64, 128, 128, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_ClipWindow(NFD_FuelQLX - 64, NFD_FuelQLY + 24, 128, 32);
	Glib_SetTexture(texobj_needle);
    Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, needle_rot, 0.0);
    Glib_RemoveClipWindow();
}

/* ---------------------------------- */
void EngineGauges_FuelPrs(int x0, int y0, float rpm, GLuint texobj_gauge, GLuint texobj_needle)
{
    float p            = 0.0;
    float needle_start = 40.0f; // needle image prerotated to point in flt sim upwards
    float needle_rot   = 0.0f;

    if (rpm < 500.0)
    {
        p = 0.0;
    }
    else if (rpm > 1000.0)
    {
        p = 60.0;
    }
    else
    {
        p = (rpm - 500.0) / 8.0;
    }

    needle_rot = needle_start - p;

    Glib_SetTexture(texobj_gauge);
    Glib_DrawTexture(x0-64, y0-64, 128, 128, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_ClipWindow(NFD_FuelPrsX - 64, NFD_FuelPrsY + 24, 128, 32);
	Glib_SetTexture(texobj_needle);
    Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, needle_rot, 0.0);
    Glib_RemoveClipWindow();
}

void EngineGauges_OilPrs(int x0, int y0, float rpm, GLuint texobj_gauge, GLuint texobj_needle)
{
    float        p            = 0.0;
    static float oilprs       = 30.0;
    float        needle_start = 40.0f; // needle image prerotated to point in flt sim upwards
    float        needle_rot   = 0.0f;

    if (rpm > 1200.0)
    {
        p = 60.0;
    }
    else
    {
        p = rpm * 0.05;
    }
    oilprs = Maths_Integrate(oilprs, 0.4 * (p - oilprs));
    needle_rot = needle_start - oilprs;

    Glib_SetTexture(texobj_gauge);
    Glib_DrawTexture(x0-64, y0-64, 128, 128, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_ClipWindow(NFD_OilPrsX - 64, NFD_OilPrsY + 24, 128, 32);
	Glib_SetTexture(texobj_needle);
    Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, needle_rot, 0.0);
    Glib_RemoveClipWindow();
}

/* ---------------------------------- */
void EngineGauges_OilTemp(int x0, int y0, float rpm, float Vc, GLuint texobj_gauge, GLuint texobj_needle)
{
    float        t            = 0.0;
    static float oiltemp      = 0.0;
    float        needle_start = 40.0f; // needle image prerotated to point in flt sim upwards
    float        needle_rot   = 0.0f;

    t = rpm * 0.023 - Vc * 0.4;
    if (t < 0.0)
    {
        t = 0.0;
    }
    oiltemp = Maths_Integrate(oiltemp, 0.007 * (t - oiltemp));
    needle_rot = needle_start - oiltemp;

    Glib_SetTexture(texobj_gauge);
    Glib_DrawTexture(x0-64, y0-64, 128, 128, 0.0, 0.0, 1.0, 1.0, 1.0);
    Glib_ClipWindow(NFD_OilTempX - 64, NFD_OilTempY + 24, 128, 32);
	Glib_SetTexture(texobj_needle);
    Glib_DrawTextureRotated(x0, y0, 128, 128, 0.0, 0.0, 1.0, 1.0, needle_rot, 0.0);
    Glib_RemoveClipWindow();
}

/* ---------------------------------- */
void BEGIN_EngineGauges_right()
{
}
