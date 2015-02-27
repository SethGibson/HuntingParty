#ifdef _DEBUG
#pragma comment(lib, "DSAPI.dbg.lib")
#else
#pragma comment(lib, "DSAPI.lib")
#endif
#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Fbo.h"
#include "cinder/Camera.h"
#include "cinder/MayaCamUI.h"
#include "CiDSAPI.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

static ivec2 S_DIMS(480, 360);

class WFTE_v2App : public AppNative 
{
public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void mouseDrag(MouseEvent event) override;
	void update() override;
	void draw() override;

	void exit();

private:
	void setupDS();
	void setupMesh();

	void updateTextures();
	void updatePointCloud();
	void renderScene();

	struct CloudPoint
	{
		vec3 PPosition;
		vec4 PColor;
		vec2 PTexCoord;
		CloudPoint(vec3 pPos, vec4 pCol, vec2 pUv) :PPosition(pPos), PColor(pCol), PTexCoord(pUv){}
	};

	gl::BatchRef mDrawObj;
	gl::VboMeshRef mPointCloud;
	gl::VboRef mVertexData;
	geom::BufferLayout mVertexAttribs;
	vector<CloudPoint> mPoints;

	gl::GlslProgRef mBlurShader;
	gl::GlslProgRef mColorShader;
	gl::GlslProgRef mExposureShader;
	gl::GlslProgRef mTexReplaceShader;
	gl::TextureRef mTexRgb;

	gl::FboRef mBlurTarget;
	CinderDSRef mCinderDS;

	CameraPersp mCamera;
	MayaCamUI mMayaCam;
};

void WFTE_v2App::setup()
{
	setupDS();
	setupMesh();

	getWindow()->setSize(1280, 720);
	setFrameRate(60);

	mCamera.setPerspective(45.0f, getWindowAspectRatio(), 0.1f, 100.0f);
	mCamera.lookAt(vec3(0), vec3(0,0,-1), vec3(0, -1, 0));
	mCamera.setCenterOfInterestPoint(vec3(0,0,-3));
	mMayaCam.setCurrentCam(mCamera);

	mTexRgb = gl::Texture::create(640, 480);
	mBlurTarget = gl::Fbo::create(1280, 720);

	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableAdditiveBlending();
}

void WFTE_v2App::setupDS()
{
	mCinderDS = CinderDSAPI::create();
	mCinderDS->init();
	mCinderDS->initDepth(FrameSize::DEPTHSD, 60);
	mCinderDS->initRgb(FrameSize::RGBVGA, 60);
	mCinderDS->start();

	getSignalShutdown().connect(std::bind(&WFTE_v2App::exit, this));
}

void WFTE_v2App::setupMesh()
{
	try
	{
		mBlurShader = gl::GlslProg::create(loadAsset("WFTE_bloom_vert.glsl"), loadAsset("WFTE_bloom_frag.glsl"));
		mColorShader = gl::GlslProg::create(loadAsset("WFTE_vert.glsl"), loadAsset("WFTE_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error Loading Shaders: " << endl;
		console() << e.what() << endl;
	}

	mPoints.clear();
	for (int dy = 0; dy < S_DIMS.y; ++dy)
	{
		for (int dx = 0; dx < S_DIMS.x; ++dx)
		{
			float cx = lmap<float>(dx, 0, S_DIMS.x, -1.3333f, 1.3333f);
			float cy = lmap<float>(dy, 0, S_DIMS.y, -1.0f, 1.0f);
			float cz = 0.0f;
			float cr = lmap<float>(dx, 0, S_DIMS.x, 0.0f, 1.0f);
			float cg = lmap<float>(dy, 0, S_DIMS.y, 1.0f, 0.0f);
			float cb = 1.0f;
			float cu = dx / (float)S_DIMS.x;
			float cv = dy / (float)S_DIMS.y;
			mPoints.push_back(CloudPoint(vec3(cx, cy, cz), vec4(cr, cg, cb, 1.0), vec2(cu, cv)));
		}
	}

	mVertexData = gl::Vbo::create(GL_ARRAY_BUFFER, mPoints, GL_DYNAMIC_DRAW);
	mVertexAttribs.append(geom::POSITION, 3, sizeof(CloudPoint), offsetof(CloudPoint, PPosition));
	mVertexAttribs.append(geom::COLOR, 4, sizeof(CloudPoint), offsetof(CloudPoint, PColor));
	mVertexAttribs.append(geom::TEX_COORD_0, 2, sizeof(CloudPoint), offsetof(CloudPoint, PTexCoord));

	mPointCloud = gl::VboMesh::create(mPoints.size(), GL_POINTS, { { mVertexAttribs, mVertexData } });
	mDrawObj = gl::Batch::create(mPointCloud, mBlurShader);
}

void WFTE_v2App::mouseDown( MouseEvent event )
{
	mMayaCam.mouseDown(event.getPos());
}

void WFTE_v2App::mouseDrag(MouseEvent event)
{
	mMayaCam.mouseDrag(event.getPos(), event.isLeftDown(), false, event.isRightDown());
}

void WFTE_v2App::update()
{
	mCinderDS->update();
	updateTextures();
	updatePointCloud();
	renderScene();
}

void WFTE_v2App::updateTextures()
{
	mTexRgb->update(mCinderDS->getRgbFrame());
}

void WFTE_v2App::updatePointCloud()
{
	const uint16_t* cDepth = mCinderDS->getDepthFrame().getData();
	mPoints.clear();
	int id = 0;
	for (int dy = 0; dy < S_DIMS.y; ++dy)
	{
		for (int dx = 0; dx < S_DIMS.x; ++dx)
		{
			float cVal = (float)cDepth[id];
			if (cVal>100 && cVal < 1000 && dx%2==0&&dy%2==0)
			{
				float cx = lmap<float>(dx, 0, S_DIMS.x, -1.3333f, 1.3333f);
				float cy = lmap<float>(dy, 0, S_DIMS.y, -1.0f, 1.0f);
				float cz = lmap<float>(cVal, 100, 1000, -1, -5);

				vec2 cUV = mCinderDS->mapColorToDepth((float)dx, (float)dy, cVal);
				cUV.y = 1.0 - cUV.y;
				mPoints.push_back(CloudPoint(vec3(cx, cy, cz), vec4(0.25,0.75,1,1), cUV));
			}
			id++;
		}
	}

	mVertexData->bufferData(mPoints.size()*sizeof(CloudPoint), mPoints.data(), GL_DYNAMIC_DRAW);
	mPointCloud = gl::VboMesh::create(mPoints.size(), GL_POINTS, { { mVertexAttribs, mVertexData } });
	mDrawObj->replaceVboMesh(mPointCloud);
	mDrawObj->replaceGlslProg(mBlurShader);
}

void WFTE_v2App::renderScene()
{
	gl::ScopedFramebuffer cFBO(mBlurTarget);
	gl::clear(Color::black());
	gl::ScopedViewport cVP(ivec2(0), mBlurTarget->getSize());

	gl::setMatrices(mMayaCam.getCamera());
	gl::ScopedTextureBind cTex(mTexRgb);
	gl::pointSize(4.0f);
	mDrawObj->draw();

	gl::color(Color::white());
}

void WFTE_v2App::draw()
{
	gl::clear( Color( 0, 0, 0 ) );

	gl::color(Color::white());
	gl::setMatrices(mMayaCam.getCamera());
	gl::pointSize(1.0f);
	mTexRgb->bind();
	mDrawObj->replaceGlslProg(mColorShader);
	mDrawObj->draw();
	mTexRgb->unbind();
	
	gl::enableAdditiveBlending();
	gl::color(ColorA(1, 1, 1, 1));
	gl::setMatricesWindow(getWindowSize());
	gl::draw(mBlurTarget->getColorTexture(), vec2(0));
}

void WFTE_v2App::exit()
{
	mCinderDS->stop();
}

CINDER_APP_NATIVE( WFTE_v2App, RendererGl )
