#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif
#include <memory>
#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "DSAPI.h"
#include "DSAPIUtil.h"
//#include "CinderOpenCV.h

using namespace ci;
using namespace ci::app;
using namespace std;

typedef std::shared_ptr<DSAPI> DSAPIRef;

class HP_WaitingRTApp : public AppNative
{
public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;

private:
	//DSAPI
	DSAPIRef mDSAPI;
	DSThird *mDSRGB;
	
	//point cloud
	vector<vec3> mCloudPoints;
	DSCalibIntrinsicsRectified mZIntrinsics;
	uint8_t *mRgbBuffer;
	uint16_t *mDepthBuffer;

};

void HP_WaitingRTApp::setup()
{
}

void HP_WaitingRTApp::mouseDown( MouseEvent event )
{
}

void HP_WaitingRTApp::update()
{
}

void HP_WaitingRTApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP_NATIVE( HP_WaitingRTApp, RendererGl )
