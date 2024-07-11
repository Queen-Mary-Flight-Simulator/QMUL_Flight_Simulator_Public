/* +------------------------------+---------------------------------+
   | Module      : pnglib.c       | Version : 3.1                   | 
   | Last Edit   : 27-11-2021     | Ref     : 03-01-08              |
   +------------------------------+---------------------------------+
   | Computer    : PFD                                              |
   | Directory   : /c/dja/sim/pfd/libs/                             |
   | Compiler    : gcc 10.2.0                                       |
   | OS          : Windows10, msys2 (64-bit)                        |
   +----------------------------------------------------------------+
   | Authors     : D J Allerton                                     |
   |             :                                                  |
   +----------------------------------------------------------------+
   | Description : png file generation library                      |
   |                                                                |
   +----------------------------------------------------------------+
   | Revisions   : none                                             |
   |                                                                |
   +----------------------------------------------------------------+ */


/* modified for desktop simulator
   screen 1920x1080
   DJA 31 March 2020 */

#include <stdio.h>
#include <stdlib.h>

#include <GLFW/glfw3.h>

#include <SIM/glib.h>
#include <png.h>

int PngLib_SavePngFile(char FileName[], int x, int y, int width, int height)
{
    FILE        *fp;
    GLubyte     *pixels;
    png_structp png;
    png_infop   info;
    png_bytepp  rows;
    int         i;
    
    pixels = malloc(width * 3 * height);
    if (pixels == NULL) 
    {
        printf("unable to allocate pixel buffer\n");
        return 0;
    }

    glFinish();
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    glReadPixels(x, y, width, height, 0x80E0, GL_UNSIGNED_BYTE, pixels);

    fp = fopen(FileName, "wb");
    if (fp == NULL) 
    {
        printf("unable to open %s\n", FileName);
        free(pixels);
        return 0;
    }

    png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png == NULL)
    {
        printf("unable to create png write structure\n");
        free(pixels);
        return 0;
    }
    
    info = png_create_info_struct(png);
    if (info == NULL) 
    {
        printf("unable to create png info structure\n");
        png_destroy_write_struct(&png, &info);
        free(pixels);
        return 0;
    }

    png_init_io(png, fp);
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
    
    png_write_info(png, info);
    png_set_packing(png);

    rows = (png_bytepp) malloc(height * sizeof(png_bytep));
    
    for (i=0; i<height; i++)
    {
        rows[i] = (png_bytep) (pixels + (height - i - 1) * width * 3);
    }
    
    png_set_bgr(png);  /* must change RGB->BGR */
    png_write_image(png, rows);
    png_write_end(png, info);
    png_destroy_write_struct(&png, &info);

    fclose(fp);
    free(pixels);
    free(rows);

    return 1;
}

void BEGIN_PngLib(void)
{
}
