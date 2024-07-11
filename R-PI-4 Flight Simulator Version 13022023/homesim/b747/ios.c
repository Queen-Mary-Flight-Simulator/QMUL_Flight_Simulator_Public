#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <sys/time.h>
#include <time.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <SIM/maths.h>
#include <SIM/pnglib.h>
#include <SIM/udplib.h>
#include <SIM/glib.h>
#include <SIM/navlib.h>
#include <SIM/soundlib.h>
#include <SIM/dted.h>

#include "ios.h"
#include "map.h"
#include "aerolink.h"
#include "englink.h"
#include "navlink.h"
#include "ioslink.h"
#include "approach.h"
#include "plot.h"
#include "script.h"
#include "gui.h"
#include "dataview.h"
#include "scan.h"
#include "iolib.h"
#include "diagnostics.h"
#include "pfd.h"
#include "nfd.h"
#include "engines.h"
#include "panellib.h"

#define MP4         0

static bool DIAGNOSTICS = false;

bool                DEMO_MODE;
bool                IOS_Mode;
static unsigned int frames;
static GLFWwindow*  window;
static char         snapfilename[] = "snap00.png";

void appSetupRenderingContext(void);
void error_callback(int error, const char* description);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void DrawGreyBox();

/* ------------------------------------------------------------- */
void appSetupRenderingContext(void)
{
    if (!IOS_Mode)
    {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClearDepth(1.0);
        glDepthFunc(GL_LEQUAL);
    }
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1.0);
}

/* ------------------------------------------------------------- */
extern void IOS_Update(void)
{
    Plot_CheckPlayback();

    memcpy(&AeroLink_IosPkt, &IosLink_IosPkt, sizeof(IosDefn_IosDataPkt));
    memcpy(&EngLink_IosPkt,  &IosLink_IosPkt, sizeof(IosDefn_IosDataPkt));
    memcpy(&NavLink_IosPkt,  &IosLink_IosPkt, sizeof(IosDefn_IosDataPkt));

    /* Data recording. Copy io,aero and nav to data pkt */
    memcpy(&IosLink_IosPlotDataPkt.IOPkt1,  &IosLink_IOPkt1,  sizeof(IosLink_IOPkt1));
    memcpy(&IosLink_IosPlotDataPkt.IOPkt2,  &IosLink_IOPkt2,  sizeof(IosLink_IOPkt2));
    memcpy(&IosLink_IosPlotDataPkt.AeroPkt, &IosLink_AeroPkt, sizeof(IosLink_AeroPkt));
    memcpy(&IosLink_IosPlotDataPkt.EngPkt,  &IosLink_EngPkt,  sizeof(IosLink_EngPkt));
    memcpy(&IosLink_IosPlotDataPkt.NavPkt,  &IosLink_NavPkt,  sizeof(IosLink_NavPkt));

    Plot_SaveData();

    Map_UpdateTrackList();

    IosLink_CheckWayPoints(); /* any FP waypoints to add? */

    IosLink_CheckSystemChange();

    if (Script_Error == true)
    {
        // Show error message dialog
        Script_Error = false;
    }

    if (Script_Enabled)
    {
        Script_ExecuteScript(IosLink_ScriptFilename);
    }
}

/* ------------------------------------------------------------- */
void IOS_Display(void)
{
    Glib_LoadIdentity();
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glClear(GL_COLOR_BUFFER_BIT);
    Glib_AntiAliasing(true);

    Glib_ClipWindow(0, 0, Glib_SCREENWIDTH - 1, Glib_SCREENHEIGHT - 1);
    Glib_Colour(Glib_LIGHTGREY);
    Glib_Rectangle(Glib_SCREENWIDTH / 2, 0, Glib_SCREENWIDTH, Glib_SCREENHEIGHT - 1);
    
    if (IosLink_IOSMode == MapDisplay)
    {
        Map_DrawMap();
    }
    else if (IosLink_IOSMode == ApproachDisplay)
    {
        Approach_ShowApproach();
    }
    else if (IosLink_IOSMode == FlightDataDisplay)
    {
        Plot_ShowPlot();
    }
    else if (IosLink_IOSMode == RawDataDisplay)
    {
        DataView_Display();
    }

    Gui_Menu();
}

/* ------------------------------------------------------------- */
void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

/* ---------------------------------- */
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
	{
        if (key == GLFW_KEY_P)
        {
            snapfilename[5] = snapfilename[5] + 1;
            if (snapfilename[5] > '9')
            {
                snapfilename[5] = '0';
                snapfilename[4] = snapfilename[4] + 1;
            }
            PngLib_SavePngFile(snapfilename, 0, 0, 1920, 1080);
        }
		else if (key == GLFW_KEY_D)
		{
		    DIAGNOSTICS = !DIAGNOSTICS;
		}
	}
}

/* ---------------------------------- */
void IOS_Init()
{
    GLenum glew_status;
    struct timeval tv1;
    struct timeval tv2;
    struct timeval frametime;
    float          etime;
    time_t         simtime;
    unsigned int   h, m, s;
    unsigned int   Timer1;
    unsigned int   Timer2;

 #if MP4
    const char* cmd = "ffmpeg -r 50 -f rawvideo -pix_fmt rgba -s 1920x1080 -i - "
                      "-threads 0 -preset fast -y -pix_fmt yuv420p -crf 21 -vf vflip output.mp4";
    //const char* cmd = "ffmpeg -r 50 -f rawvideo -pix_fmt rgba -s 1920x1080 -i - "
    //                  "-threads 0 -preset fast -y -pix_fmt yuv420p -crf 21 -vf vflip output.mp4";
    FILE* ffmpeg;
    int* buffer;
    
    ffmpeg = popen(cmd, "wb");  /* N.B. use "w" for Linux */
    if (ffmpeg == NULL)
    {
        printf("unable to open pipe\n");
        exit(-1);
    }

    buffer = malloc(1920 * 1080 * sizeof(int));
    if (buffer == NULL)
    {
        printf("Unable to open mp4 buffer\n");
        exit(-1);
    }
#endif

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
    
    window = glfwCreateWindow(Glib_SCREENWIDTH, Glib_SCREENHEIGHT, "IOS", glfwGetPrimaryMonitor(), NULL);

    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwRestoreWindow(window);
    glfwSwapInterval(0);  // was 1

    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, Gui_Mouse_Button_Callback);
    glfwSetCursorPosCallback(window, Gui_Mouse_Cursor_Callback);
    glfwSetScrollCallback(window, Gui_Mouse_Scroll_Callback);

    glew_status = glewInit();
    if (GLEW_OK != glew_status) 
    {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
        exit(-1);
    }

    if (!GLEW_VERSION_2_0) 
    {
        fprintf(stderr, "No support for OpenGL 2.0 found\n");
        exit(-1);
    }

    appSetupRenderingContext();

    Glib_Info();
    Glib_Init();
    Glib_Errors();
    
    #ifdef _WIN32   /* no standard fonts for PFD */
      Glib_LoadFont("c:/windows/fonts/arial.ttf", 1, 12);  /* font no. 1 */
      Glib_LoadFont("c:/windows/fonts/arial.ttf", 2, 24);  /* font no. 2 */
    #else
      Glib_LoadFont("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf", 1, 12);
      Glib_LoadFont("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf", 2, 24);
    #endif

    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT8,  11);
    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT12, 16);
    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT16, 20);
    Glib_LoadFont("../fonts/B612-Regular.ttf",  Glib_EFONT24, 32);
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT20, 18);  /* N.B. smaller font */
    Glib_LoadFont("../fonts/DSEG7Classic-Bold.ttf", Glib_LFONT30, 32);
    
    Glib_LoadTexture("Textures/mosaicv6.png", 1);
    Glib_LoadTexture("Textures/MapSymbols32px.png", 2);
    Glib_LoadTexture("Textures/PFD.png", 3);

    gettimeofday(&tv1, NULL);
    Timer1 = 0;
    
    while (!glfwWindowShouldClose(window))
    {
        frames += 1;
        
        IosLink_SendCmd(IosDefn_EndOfPkt);
        IosLink_CmdPtr = 0;

    Diagnostics_SetTimer1(); /* ***************************** */
        PFD_Update();
        if (AeroLink_Stopping)
        {
            glfwSetWindowShouldClose(window, GL_TRUE);
        }
        Engines_Update();
        NFD_Update();
        IOS_Update();
        
        glViewport(0, 0, Glib_SCREENWIDTH, Glib_SCREENHEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);

        Glib_LoadIdentity();

    Diagnostics_SetTimer2(); /* ***************************** */
        if (IOS_Mode)
        {
            Glib_SetTexture(3);
            PFD_Display();
            Glib_SetTexture(2);
            IOS_Display();
        }
        else
        {
            Glib_SetTexture(3);
            PFD_Display();
            Glib_SetTexture(1);
            NFD_Display();
        }

        DrawGreyBox();
	    Gui_CheckMouse();
	
    Diagnostics_SetTimer3(); /* ***************************** */
        if (DIAGNOSTICS)
        {
             Diagnostics_Diagnostics(Glib_SCREENWIDTH/2, 10, 750);
        }
        
        Glib_Flush();  /* just in case there is any unrendered graphics */

        glfwSwapBuffers(window);

#if MP4
        glReadPixels(0, 0, 1920, 1080, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
        fwrite(buffer, 1920 * 1080 * sizeof(int), 1, ffmpeg);
#endif
        glfwPollEvents();

        while (1)  /* 50 Hz frame sync */
        {
            gettimeofday(&frametime, NULL);
            Timer2 = frametime.tv_usec / 20000L;  /* frame ticks */
         	if (Timer1 != Timer2)
            {
                Timer1 = Timer2;
                break;
            }
        }
    }

#if MP4
    pclose(ffmpeg);
#endif
    
    gettimeofday(&tv2, NULL);
    simtime = time(NULL);
    printf("%s", asctime(localtime(&simtime)));
    printf("frames: %u\n", frames);
    etime = (float) (tv2.tv_sec - tv1.tv_sec) + (float) ((int) (tv2.tv_usec) - (int) (tv1.tv_usec)) / 1000000.0;
    h = (unsigned int) etime / 3600;
    m = (((unsigned int) (etime)) % 3600) / 60;
    s = (unsigned int) etime % 60;
    printf("elapsed time: %u:%u:%u\n", h, m, s);
    printf("frame rate: %f fps\n", (float) frames / etime);
    
    DTED_Exit();
    Close_SoundLib();
    IOS_CloseWindow();
}

/* --------------------------------------------------- */
void DrawGreyBox()
{	
    int    x1 = Glib_SCREENWIDTH - 40;
    int    y1 = Glib_SCREENHEIGHT - 40;
    int    x2 = x1 + 20;
    int    y2 = y1 + 20;

    Glib_LoadIdentity();

    glLineWidth(2.0);
    Glib_Colour(Glib_GREY);
    Glib_Draw(x1, y1, x1, y2);
    Glib_Draw(x1, y2, x2, y2);
    Glib_Draw(x2, y2, x2, y1);
    Glib_Draw(x2, y1, x1, y1);
    Glib_Draw(x1, y1, x2, y2);
    Glib_Draw(x1, y2, x2, y1);

    Glib_LoadIdentity();
}

/* ---------------------------------- */
void IOS_CloseWindow()
{
    Glib_Close();
    glfwDestroyWindow(window);
    glfwTerminate();
}

/* ---------------------------------- */
void BEGIN_IOS()
{
    printf("IOS starting\n");
    
    IOS_Mode = true;
    frames   = 0;
}
