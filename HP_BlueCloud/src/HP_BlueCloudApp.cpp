#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/Channel.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "CiDSAPI.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

typedef pair<geom::BufferLayout, gl::VboRef> vboMapType;

class HP_BlueCloudApp : public AppNative
{
public:
	void prepareSettings(Settings *pSettings) override;
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
	void shutdown() override;

private:
	void setupGLSL();
	void setupVBO();

	CameraPersp mCamera;
	CinderDSRef mDSAPI;

	gl::GlslProgRef mGlslProg;
	gl::VboMeshRef mVboMesh;

	Channel16u mDepthChan;
	Surface8u mRgbSurf;
	Surface32f mUVSurf;

	gl::Texture2dRef mDepthTex;
	gl::Texture2dRef mRgbTex;
	gl::Texture2dRef mDepthToColorTex;
	gl::Texture2dRef mDepthToCamTex;
};

void HP_BlueCloudApp::prepareSettings(Settings *pSettings)
{
	pSettings->prepareWindow(Window::Format().size(1280, 960));
	pSettings->setFrameRate(60);
}

void HP_BlueCloudApp::setup()
{
	gl::enable(GL_TEXTURE_2D);
	setupGLSL();

	mDSAPI = CinderDSAPI::create();
	mDSAPI->init();
	mDSAPI->initDepth(FrameSize::DEPTHSD, 60);
	mDSAPI->initRgb(FrameSize::RGBVGA, 60);
	mDSAPI->start();

	mUVSurf = Surface32f(mDSAPI->getDepthWidth(), mDSAPI->getDepthHeight(), false);
}

void HP_BlueCloudApp::mouseDown( MouseEvent event )
{
}

void HP_BlueCloudApp::update()
{
	mDSAPI->update();
	mRgbSurf = mDSAPI->getRgbFrame();
	mDepthChan = mDSAPI->getDepthFrame();
	mUVSurf = Surface32f(mDSAPI->getDepthWidth(), mDSAPI->getDepthHeight(), false, SurfaceChannelOrder::RGB);

	vector<ivec2> cPositions = mDSAPI->mapDepthToColorFrame();
	ivec2 cRgbSize = mDSAPI->getRgbSize();

	Surface32f::Iter cSurfIter = mUVSurf.getIter();
	vector<ivec2>::iterator cPosIter = cPositions.begin();

	// create UVs
	while (cSurfIter.line())
	{
		while (cSurfIter.pixel())
		{
			cSurfIter.r() = (float)cPosIter->x / cRgbSize.x;
			cSurfIter.g() = (float)cPosIter->y / cRgbSize.y;
			cSurfIter.b() = 0.0f;
			++cPosIter;
		}
	}

	//create depthtocolor texture
	//create depthtocam texture


	
}

void HP_BlueCloudApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 

}

void HP_BlueCloudApp::setupGLSL()
{
	try
	{
		mGlslProg = gl::GlslProg::create(gl::GlslProg::Format()
			.vertex(loadAsset("cloud.vert"))
			.fragment(loadAsset("cloud.frag")));
	}
	catch (gl::GlslProgCompileExc ex)
	{
		console() << "GLSL Error: " << ex.what() << endl;
		quit();
	}
	catch (gl::GlslNullProgramExc ex)
	{
		console() << "GLSL Error: " << ex.what() << endl;
		quit();
	}
	catch (...)
	{
		console() << "Unknown GLSL Error" << endl;
		quit();
	}
}

void HP_BlueCloudApp::setupVBO()
{
	ivec2 cSize = mDSAPI->getDepthSize();
	vector<vec2> cVerts;
	for (int32_t dx = 0; dx < cSize.x; ++dx)
	{
		for (int32_t dy = 0; dy < cSize.y; ++dy)
		{
			cVerts.push_back(vec2(dx, dy) / vec2(cSize));
		}
	}

	gl::VboRef cVBO = gl::Vbo::create(GL_ARRAY_BUFFER, cVerts.size()*sizeof(vec2), cVerts.data(), GL_STATIC_DRAW);

	geom::BufferLayout cLayout;
	cLayout.append(geom::Attrib::POSITION, 2, cVerts.size()*sizeof(vec2), 0);
	
	vector<vboMapType> cVAOs = {make_pair(cLayout, cVBO)};
	mVboMesh = gl::VboMesh::create(cVerts.size(), GL_POINTS, cVAOs);
}

void HP_BlueCloudApp::shutdown()
{
	mDSAPI->stop();
}

CINDER_APP_NATIVE( HP_BlueCloudApp, RendererGl )
