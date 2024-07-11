/*
    CGI Compatible with OpenSceneGraph Version 3.4
    Adapted by G T Spence from cgi.c for OpenGL Performer (originally by D J Allerton)
    
    Modified for UDPs, DJA September 2007
    Modified for OpenGL HUD, DJA January 2009
    Modified for OSG 2+, GTS May 2011
	Modified for LFS, GTS Feb 2017

    Things for the future :
    
    * osgSim/OpenFlightOptimizer and osgUtil/Optimizer
    * osgSim/DOFTransform (OSG version of Performer DOF node)
    * Better sky model. Try osgEphemeris - includes a skydome
    * Investigate better camera factilies in OSG.
    * Lightpoints -> Too big?
    * Billboard clouds
    * Shader sea model
    * Shader volumetric fog model
*/

#include "cgi.h"


/*
    Global data
*/

// Winsock data struct
WSADATA wsadata;

const double PI     =  3.141592654;
const double TWOPI  =  6.283185307;
const double ONERAD =  57.2957795;
const double PIBY2  =  PI / 2.0;
const double PIBY4  =  PI / 4.0;
const double DEG270 = 270.0 / ONERAD;

// Database coordinates
static double       RunwayX;
static double       RunwayY;
static double       RunwayZ;
static double       RunwayRotation;
static double       RunwayQDM;

// UDP packet descriptors
static int          sock = 0;
static struct sockaddr_in addr_info;
static unsigned int group_ip;

static char         buf[1500];
static IODefn_IODataPkt     IOPkt;
static AeroDefn_AeroDataPkt AeroPkt;
static NavDefn_NavDataPkt   NavPkt;
static IosDefn_IosDataPkt   IosPkt;

static unsigned int CmdPtr         = 0;
static float        cloudbase      = 15000.0;
static float        visibility     = 35000.0f;
static float        visrate        = 0.0f;
static float        minvis         = 10.0f;
static float        maxvis         = 80000.0f;
static unsigned int Oldimc         = 0;
static int          TargetLoaded   = 0;
static unsigned int CameraPosition = 0;
static Target_Info  Targets[2];
static int HUDMode = 0;  /* off by default */
TrafficFileReader * trafficfile;

osgViewer::Viewer viewer;
// Model positions and orientations. To get comps. vec.x(), vec.y() etc. To set - vec.set(x,y,z)
osg::Vec3f vecPosAircraft;
osg::Vec3f vecAttAircraft;
osg::Vec3f vecPosTarget;
osg::Vec3f vecAttTarget;

// Global scene graph objects/nodes
osg::ref_ptr<osg::Fog> fog = new osg::Fog();
osg::ref_ptr<osg::LightSource> sunLight;
osg::ref_ptr<osg::Node> TargetNode;
osg::ref_ptr<osg::PositionAttitudeTransform> TargetXForm = new osg::PositionAttitudeTransform();
osg::ref_ptr<osg::Group> SceneRoot = new osg::Group();

// File paths
osgDB::FilePathList pathList = osgDB::getDataFilePathList();
static char         filename[64];
static char         filepath[256];

static float Kp = 0.0;

char IosDefn_FileName[IosDefn_MaxFileNameSize + 1];

void printNavPkt();
/* ---------------------------------------------------- */

void drawHUD ( void )
{
  // draw the HUD - call DJA's HUD in hud.c
  if (HUDMode && CentreChannel) {
    HUD_DrawHUD(AeroPkt.Pitch, 
                AeroPkt.Roll, 
                AeroPkt.Yaw - (float) (NavPkt.MagneticVariation), 
                AeroPkt.U * sqrt(AeroPkt.Rho / 1.225), 
                -AeroPkt.Pz,
                -AeroPkt.Alpha, /* gamma is actually displayed as -alpha  not pitch-alpha*/
                AeroPkt.Beta, 
                AeroPkt.Q - AeroPkt.AlphaDot, 
                AeroPkt.BetaDot, 
                AeroPkt.UDot, 
                NavPkt.FCU_BaroPressure);
  }
};

/* ---------------------------------------------------- */

// Now the OSG wrapper for the above OpenGL code, the most complicated bit is computing
// the bounding box for the above example, normally you'll find this the easy bit.
class HUD : public osg::Drawable
{
    public:
		// Make sure we turn off display lists or HUD will not update
		// Display list is created on intialisation and all geometry is fixed
        HUD() {setUseDisplayList(false);}

        /** Copy constructor using CopyOp to manage deep vs shallow copy.*/
        HUD(const HUD& hud,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
            osg::Drawable(hud,copyop) {}

        META_Object(myHUDApp,HUD)
        
        // the draw immediate mode method is where OSG wraps up the drawing of
        // of OpenGL primitives.
        virtual void drawImplementation(osg::RenderInfo&) const
        {
	        // Call the OpenGL code.
            drawHUD();
        }
        
    protected:

        virtual ~HUD() {}

};

/* ---------------------------------------------------- */

osg::Node* createHUD()
{
    osg::Geode* geode = new osg::Geode();

    // add the teapot to the geode.
    geode->addDrawable( new HUD );

    //std::string timesFont("fonts/arial.ttf");

    // turn lighting off for the text and disable depth test to ensure its always ontop.
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    {
        osg::BoundingBox bb;
        for(unsigned int i=0;i<geode->getNumDrawables();++i)
        {
            bb.expandBy(geode->getDrawable(i)->getBound());
        }
    }

    osg::CameraNode* camera = new osg::CameraNode;

    // set the projection matrix
	camera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1280, 0, 1024));

    // set the view matrix    
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::CameraNode::POST_RENDER);

    camera->addChild(geode);

    return camera;
}

/* ---------------------------------------------------- */

/*
    Temporary function to set up a light source
*/
osg::ref_ptr<osg::LightSource> createSunLight(void)
{
    /*
    will have to create a function to dynamically alter the light levels
    dependent on the TimeOfDay. Could just do something similar to this func.
    
    Or use sunLight as declared in main and adjust its properites - better.
    eg. osg::Light* dynamicSunLight = sunLight->getLight()
    dynamicSunLight->setPosition
    dynamicSunLight->setAmbient
    dynamicSunLight->setDiffuse
    sunLight->setLight(dynamicSunLight);
    
    Same sort of thing for overall ambient light intensity in LightModel
    */
    
    osg::ref_ptr<osg::LightSource> sunLightSource = new osg::LightSource;
    osg::ref_ptr<osg::Light> sunLight = sunLightSource->getLight();
    sunLight->setPosition( osg::Vec4( 0.0f, 0.0f, 10000.0f, 1.0f ) );
    sunLight->setAmbient( osg::Vec4( 0.2f, 0.2f, 0.2f, 1.0f ) );
    sunLight->setDiffuse( osg::Vec4( 0.8f, 0.8f, 0.8f, 1.0f ) );
    sunLightSource->setLight( sunLight.get() );
    sunLightSource->setLocalStateSetModes( osg::StateAttribute::ON );
    
    // Forcing OVERRIDE for lighting - otherwise trees in terrain remain unlit!
    // The trees are actually "extern" refs from main terrain db, so it is
    // likely that this is the source of unlit trees
    sunLightSource->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    osg::ref_ptr<osg::LightModel> lightModel = new osg::LightModel;
    lightModel->setAmbientIntensity(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    sunLightSource->getOrCreateStateSet()->setAttribute( lightModel.get() );

    return sunLightSource;    
}

/* ---------------------------------------------------- */

int main (int argc, char* argv[])
{
    int s; // socket fd
	int retval; // return value from socket_get_data in main while loop
	
	/* TRANSIT */
	int     frameNum = 0;
    int     i;
    double  x, y, z, h, p, r;

    Waypoints * waypoints3d = new Waypoints();
    Planes * planes3d = new Planes();

    trafficfile = new TrafficFileReader();
    //trafficfile->loadFile("transit_traffic_constant.csv");
	trafficfile->loadFile("transit_traffic_ar.csv");
	//trafficfile->loadFile("man_simdata_armodel.txt");
	/*
	*
	*/
    if (argc <= 1)
    {
       printf("Error: No file name\n");
       exit(1);
    }

    if ( SelectDatabase(argv) )
    {
        printf("Error: No visual database\n");
        exit(1);
    }

    // Winsock related
    if (WSAStartup(MAKEWORD(2, 0), &wsadata) != 0)
        exit(1);

    if(CentreChannel)
        BEGIN_HUD();
      
    // Setting up a file paths for databases/models - and appending to the file path list
    osgDB::setDataFilePathList(pathList);
    
    // Load the 3d models. Uses "smart" pointers that manage memory better.
    osg::ref_ptr<osg::Node> TerrainNode = osgDB::readNodeFile(filename);
    if (!TerrainNode)
    {
        std::cerr << "Failed to load terrain database!\n";
        exit(1);
    }
    
    // Load the sky model - the thin cloud layer
    osg::ref_ptr<osg::Node> SkyNode = osgDB::readNodeFile("sky.flt");
    if (!SkyNode)
    {
        std::cerr << "Failed to load sky database!\n";
        exit(1);
    }
    
    // Using a sky box until I get fog working on clear buffer properly
    osg::ref_ptr<osg::Node> SkyBoxNode = osgDB::readNodeFile("models/skyb.ac");
    if (!SkyBoxNode)
    {
        std::cerr << "Failed to load skybox database!\n";
        exit(1);
    }
    
    // Set up a transform node to move the surrounding skybox a little lower
    // This stops the band of different colour (clear colour) showing through.
    osg::ref_ptr<osg::PositionAttitudeTransform> SkyBoxXForm = new osg::PositionAttitudeTransform();
    SkyBoxXForm->addChild(SkyBoxNode.get());
    osg::Vec3 SkyBoxPosit(0,0,-10000);
    SkyBoxXForm->setPosition( SkyBoxPosit );
    
    // Create blue sky - sets the clear buffer bits
    osg::ref_ptr<osg::ClearNode> backdrop = new osg::ClearNode;
    backdrop->setClearColor(osg::Vec4(0.8f,0.8f,1.0f,1.0f));
      
    // Enable fogging     
    fog->setMode(osg::Fog::EXP);
    fog->setDensity(0.00005f);
    fog->setColor(osg::Vec4d(0.8,0.8,0.8,1.0)); 
    fog->setStart(0.0f);     // only for LINEAR fog
    fog->setEnd(visibility); // only for LINEAR fog
    
    // Fog node state
    osg::ref_ptr<osg::StateSet> fogStateSet = new osg::StateSet();
    fogStateSet->setAttribute(fog.get(), osg::StateAttribute::ON); 
    fogStateSet->setMode(GL_FOG, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE); 
    
    //
    // Populate the scene graph. Could layout the graph a little more thoughfully.
    //
    SceneRoot->setStateSet(fogStateSet.get());
   
    sunLight = createSunLight();
    osg::ref_ptr<osg::Group> LitObjects = new osg::Group();
    LitObjects->addChild(sunLight.get());
    sunLight->addChild(TerrainNode.get());
    
    SceneRoot->addChild(SkyBoxXForm.get());
    SceneRoot->addChild(backdrop.get());
    SceneRoot->addChild(SkyNode.get());
    SceneRoot->addChild(LitObjects.get());

    // add the HUD to the scene
    if(CentreChannel)
        SceneRoot->addChild(createHUD());
   


    // Initialise the camera with some default settings
    osg::Matrixd myCameraMatrix;
    osg::Matrixd cameraRotation;
    osg::Matrixd cameraOffsetRotation;
    osg::Matrixd cameraTrans;
    
    // Camera offset attitude
    cameraOffsetRotation.makeRotate(
        osg::DegreesToRadians(0.0), osg::Vec3(0,1,0), // roll
        osg::DegreesToRadians(0.0), osg::Vec3(1,0,0) , // pitch
        osg::DegreesToRadians(ChannelOffset), osg::Vec3(0,0,1) ); // heading 
   
    /* TRANSIT */
    sunLight->addChild(waypoints3d->get());
    sunLight->addChild(planes3d->get());
	/***/
    // Configure UDP comms
    s = socket_init();
    if (s == EXIT_FAILURE)
    {
      printf("Unable to initialise socket\n");
      exit(1);
    }
    
    // Point the scene graph root to the viewer
    viewer.setSceneData( SceneRoot.get() );
    
	// Many viewer settings only work once viewer.realize() has been called
    viewer.realize();
    
    // Disable mouse cursor for all windows (we only use one)
    osgViewer::Viewer::Windows windows; // vector of GraphicsWindow
    viewer.getWindows(windows); // in ViewerBase (which Viewer inherits), get the list of GWs
    windows[0]->useCursor(false); 
    //for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end(); ++itr) {
    //  (*itr)->useCursor(false);
    //}
    
    // Field Of View settings
    
    // In Performer the near and far clipping planes = 5.0 and 50000.0 resp.
    // Performer Hfov and Vfov = 63.1 and 43.0 resp.
    // For OSG far set to be 200000 so sky box is drawn. This area needs improvement.
    // OSG 2.0 + doesnt provide a way to set fovx and fovy, so we use standard OpenGL interface.
    // See gluPerspective. fovy, aspect ratio, near, far
    // So OSG aspect ratio of wants tan(fovx/2)/tan(fovy/2)
    
    viewer.getCamera()->setProjectionMatrixAsPerspective( 40.0, 1.716814, 1.0, 200000.0 );

    // Stop OSG optimising near and far clipping planes

    viewer.getCamera()->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    
    // Disable small feature culling so we can see airfield lights from a distance

    osg::CullSettings::CullingMode cm = viewer.getCamera()->getCullingMode();
    cm &= ~(osg::CullSettings::SMALL_FEATURE_CULLING);
    viewer.getCamera()->setCullingMode(cm);
    
	bool TRANSIT_drawWaypoints = true;
    //
    // Enter rendering loop
    //
    while (!viewer.done())
    {
        // read pkt from flight model
		
		/* TRANSIT */
		if (TRANSIT_drawWaypoints)
		{
			waypoints3d->updateSingleWaypoint(AeroPkt.waypoint_info, AeroPkt.waypoint_count);
			//Todo no need to do this every single time.
			waypoints3d->redrawWaypoints();
		}
		//printf("AC position: lat=%f, long=%f\n", AeroPkt.Latitude, AeroPkt.Longitude); // testing
        planes3d->updatePlanes(trafficfile->getData(AeroPkt.frameSinceRestore));
		/*
		*
		*/
		
        retval = socket_get_data();
        
        // Code to manually position and orient the camera. Move to function for neatness
        cameraTrans.makeTranslate( vecPosAircraft.x(), vecPosAircraft.y(), vecPosAircraft.z() ); // x = +ve RIGHT : y = +ve FORWARDS : Z = +ve UP
        cameraRotation.makeRotate(
            osg::DegreesToRadians(vecAttAircraft.z()), osg::Vec3(0,1,0),   // roll
            osg::DegreesToRadians(vecAttAircraft.y()), osg::Vec3(1,0,0) ,  // pitch
            osg::DegreesToRadians(vecAttAircraft.x()), osg::Vec3(0,0,1) ); // heading
        myCameraMatrix = (cameraOffsetRotation * cameraRotation) * cameraTrans;
        osg::Matrixd i = myCameraMatrix.inverse(myCameraMatrix);
        osg::Matrixd xxx = osg::Matrixd::rotate( -M_PI*0.5, osg::Vec3(1,0,0) );
        viewer.getCamera()->setViewMatrix(i * xxx);
        
        // Fire off the cull and draw traversals of the scene
        viewer.frame();
    }

    // Winsock related
    closesocket(sock);
    WSACleanup();

    return 0;
}

/* ---------------------------------------------------- */

void setname(char *str)
{
    strcpy(filename, str);
}

/* ---------------------------------------------------- */

void setpath(char *str)
{
    strcpy(filepath, str);
}

/* ---------------------------------------------------- */

int SelectDatabase ( char *argv[] )
{
    if ( !strcmp ( argv[1], "bristol" ) )
    {
        RunwayX  =        0.0;
        RunwayY  =        0.0;
        RunwayZ  =        0.0;
        RunwayRotation =  0.0;			/* 360 degrees */
		RunwayQDM =      88.0;
        
	    setname("long_bristol.flt");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_bristol\\flt_files");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_bristol\\models");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_bristol\\texture");
    }
	else if ( !strcmp ( argv[1], "sheffield" ) )
    {
        RunwayX  =      7980.0;
        RunwayY  =      1530.0;
        RunwayZ  =        70.0;
        RunwayRotation = 85.0 * (M_PI/180.0);		/* 89.095 degrees */
        RunwayQDM =      0.0;

	    //setname("sheffield_terrain.flt");
        //setname("sheffield_terrain20.ive");
        setname("sheffield_terrain5.ive");
        pathList.push_back("c:\\SIM-DATA\\databases\\sheffield_output\\standard_openflight\\geometry");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_bristol\\models");
        pathList.push_back("c:\\SIM-DATA\\databases\\sheffield_output\\standard_openflight\\texture\\used_textures");
	}
    else if ( !strcmp ( argv[1], "bristol2" ) )
    {
        RunwayX  =   -11999.0;
        RunwayY  =   -18437.0;
        RunwayZ  =        0.0;
        RunwayRotation =  3.141592654;		/* 180 degrees */
    }
    else if ( !strcmp ( argv[1], "bristolsea" ) )
    {
        RunwayX  =        0.0;
        RunwayY  =     2554.0;
        RunwayZ  =        0.0;
        RunwayRotation =  3.141592654;		/* 180 degrees */
    }
    else if ( !strcmp ( argv[1], "bristol_carrier" ) )
    {
        RunwayX  =        0.0;
        RunwayY  =        0.0;
        RunwayZ  =        0.0;
        RunwayRotation =  0.0;			/* 360 degrees */
        
	    setname("long_bristol.flt");
	    pathList.push_back("c:\\SIM-DATA\\databases\\Jake\\Long_Bristol\\FLT_files");
    }
    else if ( !strcmp ( argv[1], "hong_kong" ) )
    {
        //RunwayX  =   -18552.0;
        //RunwayY  =    -3003.0;
        //RunwayZ  =        0.0;
        //RunwayRotation =  1.815142422;		/* 104 degrees */
        RunwayQDM =      250.5;             /* 25L QDM True */
		
		// data from EFS:
		RunwayX  =   -21745.0;
        RunwayY  =    -2205.0;
        RunwayZ  =        0.0;
        RunwayRotation = -1.3289;
		
		// setname("hk_sgi_demo.flt"); // old HK - up to 2018-10-12
		setname("hk_sgi_demo.osg");
        pathList.push_back("c:\\SIM-DATA\\databases\\hong_kong");
    }
    else if ( !strcmp ( argv[1], "monterey" ) )
    {
        RunwayX  =   142283.0;
        RunwayY  =    51346.5;
        RunwayZ  =       43.0;
        RunwayRotation =  0.924279685;		/* 52 degrees */
        setname("long_monterey.flt");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_monterey\\flt_files");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_monterey\\models");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_monterey\\textures");
    }
    else if ( !strcmp ( argv[1], "monterey2" ) )
    {
        RunwayX  =   141661.0;
        RunwayY  =   100366.0;
        RunwayZ  =       63.0;
        RunwayRotation =  3.8397;		/* 220 degrees */
        setname("long_monterey.flt");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_monterey\\flt_files");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_monterey\\models");
        pathList.push_back("c:\\SIM-DATA\\databases\\long_monterey\\textures");
    }
    else if ( !strcmp ( argv[1], "lebourget" ) )
    {
        RunwayX  =      137.0;
        RunwayY  =      270.0;
        RunwayZ  =        0.0;
        RunwayRotation =  2.6579;		/* 152 degrees */
    }
    else if ( !strcmp ( argv[1], "cranfly" ) )
    {
        RunwayX  =    44201.0;
        RunwayY  =    53897.0;
        RunwayZ  =      105.0;
        RunwayRotation =  2.80998;		/* 161 degrees */
	setname("cranfly2002.flt");
	//pathList.push_back("c:\\SIM-DATA\\databases\\Performer\\data");
	pathList.push_back("c:\\SIM-DATA\\databases\\cranfly2002\\flt_files");
    }
    else if ( !strcmp ( argv[1], "cranfly2002" ) )
    {
        RunwayX  =    44201.0;
        RunwayY  =    53897.0;
        RunwayZ  =      105.0;
        RunwayRotation =  2.80998;		/* 161 degrees */
    }
    else if ( !strcmp ( argv[1], "bayport" ) )
    {
        RunwayX  =    101511.0;
        RunwayY  =    99107.0;
        RunwayZ  =      3.0;
        RunwayRotation =  5.7767;		/* 331 degrees */
        setname("bayport.flt");
        pathList.push_back("c:\\SIM-DATA\\databases\\bayport\\flt_files");
        pathList.push_back("c:\\SIM-DATA\\databases\\bayport\\models");
        pathList.push_back("c:\\SIM-DATA\\databases\\bayport\\textures");
    }
    else if ( !strcmp ( argv[1], "fresno" ) )
    {
        RunwayX  =    142920.0;
        RunwayY  =    51735.0;
        RunwayZ  =      101.0;
        RunwayRotation =  0.9423;		/* 54 degrees */
        setname("fresno.flt");
        pathList.push_back("c:\\SIM-DATA\\databases\\fresno\\flt_files");
        pathList.push_back("c:\\SIM-DATA\\databases\\fresno\\models");
        pathList.push_back("c:\\SIM-DATA\\databases\\fresno\\textures");
    }
    else if ( !strcmp ( argv[1], "heathrow" ) )
    {
        RunwayX  =   -2043.617;
        RunwayY  =       7.750;
        RunwayZ  =      22.689;
        RunwayRotation =  4.70332;		/* 270.48 degrees */
        setname("master_sub.flt");
	pathList.push_back("c:\\SIM-DATA\\databases\\heathrow\\flt_files");
    }
    else if ( !strcmp ( argv[1], "london" ) )
    {
        RunwayX  =        0.0;
        RunwayY  =        0.0;
        RunwayZ  =        0.0;
        RunwayRotation =  0.0;			/* 360 degrees */
        
	    setname("london_tile.flt");
	    pathList.push_back("c:\\SIM-DATA\\databases\\london");
        pathList.push_back("c:\\SIM-DATA\\databases\\london\\textures");
    }
	else if ( !strcmp ( argv[1], "manchester" ) )
    {
        RunwayX  =         0.0;
        RunwayY  =         -1091.0;
        RunwayZ  =         -7.26;
        RunwayRotation =   0.0;
		RunwayQDM =        51.06;     /* 05L QDM true */

        setname("aerodrome-terrain.osgt");
        pathList.push_back("c:\\SIM-DATA\\databases\\manchester");
    }
    else
    {
        return -1;
    }
    
    pathList.push_back("c:\\SIM-DATA\\multigen");
    pathList.push_back("c:\\SIM-DATA\\multigen\\models");
    pathList.push_back("c:\\SIM-DATA\\multigen\\texture");
    
    return 0;

}


/* ---------------------------------------------------- */
/*                                                      */
/*  SetTime( Hour, Angle )                              */
/*  calculate the elevation and direction of the sun    */
/*                                                      */
/* ---------------------------------------------------- */

void SetTime( float hour, float angle )
{
    float d;

    if (hour < 6.0)
      d = 0.0;
    else
      if (hour > 18.0)
        d = 0.0;
      else 
        d = 1.0 - fabs(12.0 - hour) / 6.0;
    
    
    osg::ref_ptr<osg::Light> UpdatedLight = sunLight->getLight();
    
    // Alter position of sun?
    //UpdatedLight->setPosition( osg::Vec4( 0.0f, 0.0f, 10000.0f, 1.0f ) );
    
    UpdatedLight->setAmbient( osg::Vec4( d, d, 0.8f*d, 1.0f ) );
    UpdatedLight->setDiffuse( osg::Vec4( d, d, 0.8f*d, 1.0f ) );
    
    sunLight->setLight( UpdatedLight.get() );
    sunLight->setLocalStateSetModes( osg::StateAttribute::ON );
    sunLight->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    fog->setColor(osg::Vec4d(d,d,d,1.0f));
}

/* ---------------------------------------------------- */

float degrees( float r )     /* radians -> degrees */
{
    return r * ONERAD;
}

/* ---------------------------------------------------- */

float rads( float d )     /* degrees -> radians */
{
    return d / ONERAD;
}

/* ---------------------------------------------------- */

float normalise( float a )     /* in radians +- 180 degrees */
{
    float x = fmod(a + Maths_PI, Maths_TWOPI);

    if (x < 0.0)
    {
        x += Maths_TWOPI;
    }
    return (x - Maths_PI);
}

/* ---------------------------------------------------- */

unsigned char GetByte( void )

{   unsigned char x;

    x = IosPkt.CmdBuff[CmdPtr];
    CmdPtr = CmdPtr + 1;
    return x;
}

/* ---------------------------------------------------- */

int GetInt( void )

{   union cmdpktarg x;

    x.chars[0] = GetByte();
    x.chars[1] = GetByte();
    return x.int16;
}

/* ---------------------------------------------------- */

unsigned short int GetCard( void )

{   union cmdpktarg x;

    x.chars[0] = GetByte();
    x.chars[1] = GetByte();
    return x.card16;
}

/* ---------------------------------------------------- */

int GetBoolean( void )

{  
    return (GetByte() != 0) ? 1 : 0;
}

/* ---------------------------------------------------- */

float GetReal( void )

{   union cmdpktarg t;
  
    t.chars[0] = GetByte();
    t.chars[1] = GetByte();
    t.chars[2] = GetByte();
    t.chars[3] = GetByte();
    return t.real32;
}

/* ---------------------------------------------------- */

void GetFilename( char *Str )

{   unsigned int i = 0;

    do
    {   Str[i] = GetByte();
        i = i + 1;
    } while (Str[i - 1] != 0);
}

/* ---------------------------------------------------- */

void set_vis( float vis )
{
    float    density;

/*  For linear fog.....
    fog->setEnd(vis);    */

/*  For exp fog....    */
    density = 3.2f/vis;
    fog->setDensity(density);

/*   For exp2 fog....    
    density = 2.6f/vis;
    fog->setDensity(density);    */

}

/* ---------------------------------------------------- */

void CheckVisibility( float h )
    /* 0 VMC
       1 partial IMC
       2 full IMC */
{
    unsigned int imc = 1;
    float oldvis = visibility;
    
    visibility = visibility + visrate / 50.0;
    if (visibility < minvis)
    {
        visibility = minvis;
    }
    if (visibility > maxvis)
    {
        visibility = maxvis;
    }
    
    if (abs(visibility - oldvis) > 1.0)
    {
        set_vis(visibility);
    }
        
    if ( h < cloudbase )
        imc = 0;
    if ( h > (cloudbase + 50.0) )
        imc = 2;
    if (imc == 1)
        set_vis( (cloudbase + 50.0 - h) * 0.02 * visibility );
    else
    {   
        if (imc != Oldimc)
        {
            if ( imc == 2 )
                set_vis( 0.0 );
            else
                set_vis( visibility );
        }
    }   
    Oldimc = imc;
}

/* ---------------------------------------------------- */

unsigned int GetGroup( unsigned int addr )

{   union IPaddress x;

    x.int32 = htonl(addr);
    x.bytes[0] = x.bytes[1];
    x.bytes[1] = x.bytes[2];
    x.bytes[2] = x.bytes[3];
    x.bytes[3] = 0;
    return x.int32;
}

/* ---------------------------------------------------- */

unsigned int GetNode( unsigned int addr )

{   
  return htonl(addr) % 256;
}

/* ---------------------------------------------------- */

void DecodeIosPkt( void )

{   char               Filename[20];
    unsigned short int IosCmd;
    //pfNode             *TargetModel;
    
    CmdPtr = 0;
    
    for (;;)
    {
        IosCmd = GetCard();
        if (IosCmd != 0)
		
        switch (IosCmd)
        {
            case IosDefn_EndOfPkt: 
                return;

            case IosDefn_Exit:
                /* ignore */
                break;

            case IosDefn_SetKp:
                Kp = GetReal();
                break;

            case IosDefn_Visual:
                closesocket(sock);
                WSACleanup();
                GetFilename( Filename );
                execl("c:/IG/visual/cgi.exe", "cgi.exe", Filename, NULL);
                break;
      
            case IosDefn_SetCloudbase:
                cloudbase = GetReal();
                break;
    
            case IosDefn_SetVisibility:
                visibility = GetReal();
                set_vis( visibility );
                break;
      
            case IosDefn_SetVisRate:
                visrate = GetReal();
                break;
      
            case IosDefn_LoadTargetFile:
                GetFilename( Filename );
                
                if (TargetLoaded)
                {
                    // sunLight!!! Messy. Sort this scene graph out so it is slighty more logical!
                    sunLight->removeChild(TargetXForm.get());
                    TargetXForm->removeChild(TargetNode.get());
                }
		
                
                TargetNode = osgDB::readNodeFile(Filename);
                if (!TargetNode)
                {
                    std::cerr << "Failed to load target geometry!\n";
                    //viewer.setDone(true); // this way threads/etc get cleaned up correctly. Viewer needs to be global!
                    exit (1);
                }

                TargetXForm->addChild(TargetNode.get());
                sunLight->addChild(TargetXForm.get());   // already attached to root
		
                TargetLoaded = 1;
                break;
      
            case IosDefn_SwitchTargetOff:
                if (TargetLoaded)
                {
                    sunLight->removeChild(TargetXForm.get());
                    TargetXForm->removeChild(TargetNode.get());
                    TargetLoaded = 0;
                }
	        
                break;

            case IosDefn_SwitchHUDOff:
                HUDMode = GetBoolean();
                break;
            

            case IosDefn_PlaybackCamera:
                CameraPosition = GetInt();
                break;
			case 401:
                if(GetBoolean() == 0)
                {
                	trafficfile->loadFile("transit_traffic_constant.csv");
                }
                else
                {
                	trafficfile->loadFile("transit_traffic_ar.csv");
					//trafficfile->loadFile("man_simdata_armodel.txt");
                }
            	break;

            default: 
                return;
        }
    }
}

/* ---------------------------------------------------- */
double Distance(double Lat1, double Long1, double Lat2, double Long2)
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
double Bearing(double Lat1, double Long1, double Lat2, double Long2)
{
  double x, y;
  double ax, ay;
  double dLat;
  double dLong;
  double d;
  double psi;

  dLat = Lat2 - Lat1;
  dLong = Long2 - Long1;
  d = sqrt(dLat * dLat + cos(Lat1) * cos(Lat2) * dLong * dLong);
  x = sin(Lat2) - sin(Lat1) * cos(d);
  ax = fabs(x);
  y = cos(Lat2) * sin(dLong) * cos(Lat1);
  ay = fabs(y);
  psi = atan(ay / ax);
  if (x < 0.0) 
  {
    psi = PI - psi;
  }
  if (y < 0.0) 
  {
    return -psi;
  } 
  else 
  {
    return psi;
  }
}

/* ---------------------------------------------------- */
/*                                                      */
/*  socket_init                                         */
/*  initialise the UDP interface                        */
/*                                                      */
/* ---------------------------------------------------- */


/* ----------------------------------------- */
int socket_init(void)
{   
    addr_info.sin_family = AF_INET;
    addr_info.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_info.sin_port = htons(DEFAULT_PORT);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {   // perror("socket");
        return EXIT_FAILURE;
    }

    if (bind(sock, (struct sockaddr *) &addr_info, sizeof(addr_info)) < 0)
    {   //perror("bind");
        return EXIT_FAILURE;
    }

    group_ip = GetGroup(inet_addr(BROADCAST_IP));
    return EXIT_SUCCESS;
}


/* ----------------------------------------------------------- */
/*                                                             */
/*  socket_get_data                                            */
/*                                                             */
/*    called once/frame to acquire pkt from the host           */
/*    updates visual system parameters and responds to the IOS */
/*                                                             */
/* ----------------------------------------------------------- */

/* ---------------------------------------------------- */
int socket_get_data(void)
{
    static unsigned int OldTimeOfDay = 0;
    int retval;
 
    float q;
  
    double x;
    double y;
    double z;
    double Pitch;
    double Roll;
    double Yaw;
  
    double Tx;
    double Ty;
    double Tz;
    double TPitch;
    double TRoll;
    double TYaw;
  
    double a;
    double d;
    double b;
    double dx;
    double dy;

    double sina;
    double cosa;

    double px, py, pz;
    double h, p, r;
    
    int addr_len = sizeof(addr_info);
    int pkt2found = 0;

    do
    {
        retval = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *) &addr_info, (int*) &addr_len);
        if (retval < 0)
        {   perror("recvfrom:");
            exit(3);
        }

        if (GetGroup(addr_info.sin_addr.s_addr) == group_ip)
        { 
            switch (GetNode(addr_info.sin_addr.s_addr))
            {
            case 1:
                if (retval != sizeof(IOPkt))
                {   perror("IOPkt");
                    exit(4);
                }
                else
                {
                    //memcpy(&IOPkt, buf, retval);
                    //if (IOPkt.KeySwitch == IODefn_Off) { // *** LFS
                      //close(sock);
                      //closesocket(sock);
                      //exit(0);
                    //}
                }
                break;
			
/*	
			case 2: // Second IOPkt 
*/
            case 3:
                pkt2found = 1;
                if (retval != sizeof(AeroPkt))
                {   perror("AeroPkt");
                    exit(4);
                }
                else
                {
                    memcpy(&AeroPkt, buf, retval);
					
					/*
                    if (AeroPkt.Shutdown) // *** LFS
                    {
                        //close(sock);
                        closesocket(sock);
                        exit(0);
                    }
					*/

                    Pitch = AeroPkt.Pitch;
                    Roll = AeroPkt.Roll;
                    Yaw = AeroPkt.Yaw;
  
                    TPitch = AeroPkt.TPitch;
                    TRoll = AeroPkt.TRoll;
                    TYaw = AeroPkt.TYaw;
  
                    if (NavPkt.CurrentRunway > 0) 
                    {
                        q = normalise(rads((double) (NavPkt.RunwayQDM)) + (double) (NavPkt.MagneticVariation) );
                        d = Distance(AeroPkt.Latitude, AeroPkt.Longitude, 
                                     (double) NavPkt.RunwayLatitude, (double) NavPkt.RunwayLongitude);
                        b = Bearing(AeroPkt.Latitude, AeroPkt.Longitude, 
                                    (double) NavPkt.RunwayLatitude, (double) NavPkt.RunwayLongitude);
                        x = -d * sin(b) + (double) AeroPkt.Ey;
                        y = -d * cos(b) + (double) AeroPkt.Ex;
                        z = -(AeroPkt.Pz + AeroPkt.Ez) + NavPkt.GroundLevel;
                        //printf("d=%f x=%f y=%f lat=%f long=%f rlat=%f rlong=%f cr=%d\n", d, x, y, AeroPkt.Latitude*(180.0/M_PI), AeroPkt.Longitude*(180.0/M_PI), NavPkt.RunwayLatitude*(180.0/M_PI), NavPkt.RunwayLongitude*(180.0/M_PI), NavPkt.CurrentRunway);
                        d = Distance(AeroPkt.TLatitude, AeroPkt.TLongitude, 
                                     (double) NavPkt.RunwayLatitude, (double) NavPkt.RunwayLongitude);
                        b = Bearing(AeroPkt.TLatitude, AeroPkt.TLongitude, 
                                    (double) NavPkt.RunwayLatitude, (double) NavPkt.RunwayLongitude);
                        Tx = -d * sin(b);
                        Ty = -d * cos(b);
                        Tz = -AeroPkt.TPz + NavPkt.GroundLevel;
                        
                    } 
                    else 
                    {
                        x = 0.0;
                        y = 0.0;
                        z = -(AeroPkt.Pz + AeroPkt.Ez);
                        Tx = 0.0;
                        Ty = 0.0;
                        Tz = 0.0;
                        q = 0.0;
                    }
                    
                    Targets[0].x = x;
                    Targets[0].y = y;
                    Targets[0].z = z;
                    Targets[0].pitch = Pitch;
                    Targets[0].roll = Roll;
                    Targets[0].yaw = Yaw;
                    Targets[1].x = Tx;
                    Targets[1].y = Ty;
                    Targets[1].z = Tz;
                    Targets[1].pitch = TPitch;
                    Targets[1].roll = TRoll;
                    Targets[1].yaw = TYaw;

                    if (CameraPosition > 0) 
                    {
                        Targets[1].x = Targets[0].x; // target pos = aircraft pos
                        Targets[1].y = Targets[0].y;
                        Targets[1].z = Targets[0].z;
                        Targets[1].pitch = Targets[0].pitch;
                        Targets[1].roll = Targets[0].roll;
                        Targets[1].yaw = Targets[0].yaw;
                        
                        Targets[0].pitch = 0.0;
                        Targets[0].roll = 0.0;
                        
                        if (CameraPosition == 1) // rear view
                        {
                            Targets[0].x = x - 100.0 * sin(Yaw);
                            Targets[0].y = y - 100.0 * cos(Yaw);
                        } 
                        else if (CameraPosition == 2) // side view
                        { 
                            Targets[0].yaw = Targets[0].yaw - PIBY2;
                            Targets[0].yaw = normalise(Targets[0].yaw);
                            Targets[0].x = x + 100.0 * cos(Yaw);
                            Targets[0].y = y - 100.0 * sin(Yaw);

                        } 
                        else if (CameraPosition == 3) // wing man
                        {
                            // Down looking view - menus buttons max of 5
                            Targets[0].pitch = -1.5707963267;
                            Targets[0].roll = 0.0;
                            //Targets[0].yaw = Targets[0].yaw - PIBY4;
                            //Targets[0].yaw = normalise(Targets[0].yaw);
                            //Targets[0].x = x + 50.0 * cos(Yaw + PIBY4);
                            //Targets[0].y = y - 50.0 * sin(Yaw + PIBY4);
                        } 
                        else if (CameraPosition == 4) // tower
                        {
                            Targets[0].x = TowerX;
                            Targets[0].y = TowerY;
                            Targets[0].z = TowerZ;
                            dx = TowerX - x;
                            dy = TowerY - y;
							Targets[0].pitch = atan2(Targets[1].z, sqrt(dx * dx + dy * dy));
							Targets[0].yaw = DEG270 - atan2(TowerY - y, TowerX - x);
                            Targets[0].yaw = normalise(Targets[0].yaw);
                        }
                    }
                }

                a = normalise(TWOPI - q - RunwayRotation);
                sina = sin(a);
                cosa = cos(a);
  
                /* position the aircraft */

                px = Targets[0].x * cosa + Targets[0].y * sina + RunwayX;
                py = Targets[0].y * cosa - Targets[0].x * sina + RunwayY;
                pz = Targets[0].z + RunwayZ;
                p = degrees(Targets[0].pitch);
                h = degrees(RunwayRotation - Targets[0].yaw + q);
                r = degrees(Targets[0].roll);

                vecPosAircraft.set(px, py, pz);
                vecAttAircraft.set(h, p, r);

                /* position the target */

                px = Targets[1].x * cosa + Targets[1].y * sina + RunwayX;
                py = Targets[1].y * cosa - Targets[1].x * sina + RunwayY;
                pz = Targets[1].z + RunwayZ;
                p = degrees(Targets[1].pitch);
                h = degrees(RunwayRotation - Targets[1].yaw + q);
                r = degrees(Targets[1].roll);
  
                vecPosTarget.set(px, py, pz);
                vecAttTarget.set(h, p, r);

    
                /* target model */
      
                if (TargetLoaded)
                { 
                    TargetXForm->setAttitude( osg::Quat(osg::DegreesToRadians(vecAttTarget.z()), osg::Vec3(0,1,0),  // roll
                                              osg::DegreesToRadians(vecAttTarget.y()), osg::Vec3(1,0,0),  // pitch
                                              osg::DegreesToRadians(vecAttTarget.x()), osg::Vec3(0,0,1))); // heading
        
                    TargetXForm->setPosition( vecPosTarget );
                }

                if ( AeroPkt.TimeOfDay != OldTimeOfDay )
                {
                    SetTime( (float) (AeroPkt.TimeOfDay) / 60.0, float(a) );
                    OldTimeOfDay = AeroPkt.TimeOfDay;
                }

                CheckVisibility( z );

                break;

/*	
			case 4: // Engines Pkt
*/

            case 5:
                if (retval != sizeof(NavPkt))
                {   perror("NavPkt");
                    exit(4);
                }
                else
                {
                    memcpy(&NavPkt, buf, retval);
                }
                break;

            case 6:
                if (retval != sizeof(IosPkt))
                {   perror("IosPkt");
                    exit(4);
                }
                else
                {
                    memcpy(&IosPkt, buf, retval);
                    DecodeIosPkt();
                }
                break;
            }
        }
    } while (pkt2found == 0);

    return EXIT_SUCCESS;
}

MapPos Coord_GlobalToMap(GlobalPos const glob)
{
	MapPos map;
	  float q;

	  double x;
	  double y;
	  double z;

	  double a;
	  double d;
	  double b;

	q = normalise(rads((double) (NavPkt.RunwayQDM)) + (double) (NavPkt.MagneticVariation) );
	d = Distance(glob.lat, glob.lon,
			(double) NavPkt.RunwayLatitude, (double) NavPkt.RunwayLongitude);
	b = Bearing(glob.lat, glob.lon,
			(double) NavPkt.RunwayLatitude, (double) NavPkt.RunwayLongitude);
	x = -d * sin(b);
	y = -d * cos(b);
	z = -(glob.alt) + NavPkt.GroundLevel;

	a = normalise(TWOPI - q - RunwayRotation);
	float sina = sin(a);
	float cosa = cos(a);

	/* position the aircraft */

	//FIXME
	//IMPORTANT
	//Magic numbers are corrective factors for the hong_kong map + transit project
	map.x = 1.01*(x * cosa + y * sina) + RunwayX;
	map.y = 1.008*(y * cosa - x * sina) + RunwayY;
	map.z = z + RunwayZ;

    map.pitch = degrees(glob.pitch);
    map.yaw = degrees(RunwayRotation - glob.yaw + q);
    map.roll = degrees(glob.roll);

    return map;


	// double centre_lat = 22.345622 *M_PI/180;
	// double centre_lon = 114.107517 *M_PI/180;
	// double hdg_offset = -4.3*M_PI/180;

	// double dLat = glob.lat - centre_lat;
	// double dLon = glob.lon - centre_lon;

	// double dm_north = dLat*111100*180/M_PI;
	// double dm_east = dLon*111100*180/M_PI*cos(centre_lat);

	// MapPos map;
	// map.x = dm_east*cos(hdg_offset) - dm_north*sin(hdg_offset);
	// map.y = dm_east*sin(hdg_offset) + dm_north*cos(hdg_offset);;
	// map.z = glob.alt + 15;

	// map.pitch = degrees(glob.pitch);
	// map.yaw = degrees(glob.yaw - hdg_offset);
	// map.roll = degrees(glob.roll);

	// return map;




}