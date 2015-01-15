#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif

#include <memory>
#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "CinderOpenCV.h"
#include "DSAPI.h"
#include "DSAPIUtil.h"

using namespace ci;
using namespace ci::app;
using namespace std;

typedef std::shared_ptr<DSAPI> DSAPIRef;

class DepthDelApp : public AppNative
{
public:
	void prepareSettings(Settings *pSettings);
	void setup();
	void mouseDown( MouseEvent event );	
	void update();
	void draw();

private:
	DSAPIRef mDS;

};

void DepthDelApp::prepareSettings(Settings *pSettings)
{

}

void DepthDelApp::setup()
{
}

void DepthDelApp::mouseDown( MouseEvent event )
{
}

void DepthDelApp::update()
{
}

void DepthDelApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP_NATIVE( DepthDelApp, RendererGl )
