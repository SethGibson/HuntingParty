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
#include "cinder/params/Params.h"
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
	void setupGUI();
	void setupDS();
	void setupMesh();
	void setupShaders();
	void setupFBOs();

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

	gl::BatchRef		mDrawObj;
	gl::VboMeshRef		mPointCloud;
	gl::VboRef			mVertexData;
	geom::BufferLayout	mVertexAttribs;
	vector<CloudPoint>	mPoints;

	gl::GlslProgRef		mShaderColor;
	gl::GlslProgRef		mShaderRender;
	gl::GlslProgRef		mShaderFilter;
	gl::GlslProgRef		mShaderResolve;
	
	gl::TextureRef		mTexRgb;

	gl::FboRef			mColorTarget;	//step 1: render scene and get brightness
	gl::FboRef			mLumTarget;		//step 2: blur brightness
	gl::FboRef			mFilterTarget;
	gl::FboRef			mFinalTarget;

	CinderDSRef			mCinderDS;

	CameraPersp			mCamera;
	MayaCamUI			mMayaCam;

	// UI Data
	params::InterfaceGlRef mGUI;
	float				mBloomMin,
						mBloomMax,
						mExposure,
						mBloomFactor,
						mSceneFactor,
						mDepthMin,
						mDepthMax,
						mDrawSize,
						mBloomSize;

	Color				mMaskColor;
};

void WFTE_v2App::setup()
{
	setupGUI();
	setupDS();
	setupShaders();
	setupMesh();
	setupFBOs();

	getWindow()->setSize(1280, 720);
	setFrameRate(60);

	mCamera.setPerspective(45.0f, getWindowAspectRatio(), 0.1f, 100.0f);
	mCamera.lookAt(vec3(0), vec3(0,0,-1), vec3(0, -1, 0));
	mCamera.setCenterOfInterestPoint(vec3(0,0,-3));
	mMayaCam.setCurrentCam(mCamera);

	mTexRgb = gl::Texture::create(640, 480);

	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableAdditiveBlending();
}

void WFTE_v2App::setupGUI()
{
	mBloomMin = 0.4f;
	mBloomMax = 1.2f;
	mExposure = 0.9f;
	mBloomFactor = 1.0f;
	mSceneFactor = 1.0f;
	mDepthMin = 100.0f;
	mDepthMax = 1000.0f;
	mDrawSize = 1.0f;
	mBloomSize = 2.0f;
	mMaskColor = Color(0.25f, 0.75f, 1.0f);

	mGUI = params::InterfaceGl::create("Settings", vec2(250, 300));
	mGUI->setPosition(vec2(20));
	mGUI->addText("Cloud Parameters");
	mGUI->addParam("Min Depth", &mDepthMin);
	mGUI->addParam("Max Depth", &mDepthMax);
	mGUI->addParam("Cloud Point Size", &mDrawSize);
	mGUI->addParam("Blur Point Size", &mBloomSize);
	mGUI->addParam("Mask Color", &mMaskColor);
	mGUI->addSeparator();
	mGUI->addText("Bloom Parameters");
	mGUI->addParam("Min Bloom", &mBloomMin);
	mGUI->addParam("Max Bloom", &mBloomMax);
	mGUI->addParam("Exposure", &mExposure);
	mGUI->addParam("Bloom Factor", &mBloomFactor);
	mGUI->addParam("Scene Factor", &mSceneFactor);
	

}

void WFTE_v2App::setupShaders()
{
	try
	{
		mShaderColor = gl::GlslProg::create(loadAsset("WFTE_vert.glsl"), loadAsset("WFTE_00_color_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading color shader: " << endl;
		console() << e.what() << endl;
	}
	try
	{
		mShaderRender = gl::GlslProg::create(loadAsset("WFTE_vert.glsl"), loadAsset("WFTE_01_render_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading render shader: " << endl;
		console() << e.what() << endl;
	}
	try
	{
		mShaderFilter = gl::GlslProg::create(loadAsset("WFTE_vert_fbo.glsl"), loadAsset("WFTE_02_filter_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading filter shader: " << endl;
		console() << e.what() << endl;
	}
	try
	{
		mShaderResolve = gl::GlslProg::create(loadAsset("WFTE_vert_fbo.glsl"), loadAsset("WFTE_03_resolve_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading tex resolve shader: " << endl;
		console() << e.what() << endl;
	}
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
	mDrawObj = gl::Batch::create(mPointCloud, mShaderColor);
}

void WFTE_v2App::setupFBOs()
{
	gl::Fbo::Format cColorFormat;
	cColorFormat.colorTexture(gl::Texture2d::Format().internalFormat(GL_RGBA16F));
	mColorTarget = gl::Fbo::create(1280, 720, cColorFormat);

	gl::Fbo::Format cLumFormat;
	cLumFormat.colorTexture(gl::Texture2d::Format().internalFormat(GL_RGBA32F).dataType(GL_FLOAT));
	mLumTarget = gl::Fbo::create(1280, 720, cLumFormat);

	gl::Fbo::Format cFilterFormat;
	cFilterFormat.colorTexture(gl::Texture2d::Format().internalFormat(GL_RGBA32F).dataType(GL_FLOAT));
	mFilterTarget = gl::Fbo::create(1280, 720, cFilterFormat);

	gl::Fbo::Format cFinalFormat;
	cFinalFormat.colorTexture(gl::Texture2d::Format().internalFormat(GL_RGBA16F));
	mFinalTarget = gl::Fbo::create(1280, 720, cFinalFormat);
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
			if (cVal>mDepthMin && cVal < mDepthMax && dy%2==0)
			{
				float cx = lmap<float>(dx, 0, S_DIMS.x, -1.3333f, 1.3333f);
				float cy = lmap<float>(dy, 0, S_DIMS.y, -1.0f, 1.0f);
				float cz = lmap<float>(cVal, mDepthMin, mDepthMax, -1, -5);

				vec2 cUV = mCinderDS->mapColorToDepth((float)dx, (float)dy, cVal);
				cUV.y = 1.0 - cUV.y;
				mPoints.push_back(CloudPoint(vec3(cx, cy, cz), vec4(mMaskColor.r, mMaskColor.g, mMaskColor.b,1), cUV));
			}
			id++;
		}
	}

	mVertexData->bufferData(mPoints.size()*sizeof(CloudPoint), mPoints.data(), GL_DYNAMIC_DRAW);
	mPointCloud = gl::VboMesh::create(mPoints.size(), GL_POINTS, { { mVertexAttribs, mVertexData } });
	mDrawObj->replaceVboMesh(mPointCloud);
}

void WFTE_v2App::renderScene()
{
	//Color VBO
	mColorTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mColorTarget->getSize());
	gl::setMatrices(mMayaCam.getCamera());
	mTexRgb->bind();
	gl::pointSize(mDrawSize);
	mDrawObj->draw();
	mColorTarget->unbindFramebuffer();
	mTexRgb->unbind();

	//Luminance VBO
	mLumTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mColorTarget->getSize());
	gl::setMatrices(mMayaCam.getCamera());
	mTexRgb->bind();
	gl::pointSize(mBloomSize);
	mShaderRender->uniform("mBloomMin", mBloomMin);
	mShaderRender->uniform("mBloomMax", mBloomMax);
	mDrawObj->replaceGlslProg(mShaderRender);
	mDrawObj->draw();
	mLumTarget->unbindFramebuffer();
	mTexRgb->unbind();

	//Blur
	mFilterTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mFilterTarget->getSize());
	gl::setMatricesWindow(getWindowSize());
	mLumTarget->bindTexture();
	mShaderFilter->bind();
	gl::drawSolidRect(Rectf({ vec2(0), mLumTarget->getSize() }));
	mLumTarget->unbindTexture();
	mFilterTarget->unbindFramebuffer();

	//Resolve
	mFinalTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mFinalTarget->getSize());
	gl::setMatricesWindow(getWindowSize());
	mLumTarget->bindTexture(0);
	mFilterTarget->bindTexture(1);
	mShaderResolve->bind();
	mShaderResolve->uniform("mHdrTarget", 0);
	mShaderResolve->uniform("mBloomTarget", 1);
	mShaderResolve->uniform("mExposure", mExposure);
	mShaderResolve->uniform("mBloomFactor", mBloomFactor);
	mShaderResolve->uniform("mSceneFactor", mSceneFactor);
	gl::drawSolidRect(Rectf({ vec2(0), mLumTarget->getSize() }));
	mLumTarget->unbindTexture();
	mFilterTarget->unbindTexture();
	mFinalTarget->unbindFramebuffer();
	
}

void WFTE_v2App::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::color(Color::white());

	gl::setMatrices(mMayaCam.getCamera());
	mTexRgb->bind();
	mDrawObj->replaceGlslProg(mShaderColor);
	gl::pointSize(1.0);
	mDrawObj->draw();
	mTexRgb->unbind();
	gl::setMatricesWindow(getWindowSize());
	gl::enableAdditiveBlending();
	gl::draw(mFinalTarget->getColorTexture(), vec2(0));

	gl::setMatricesWindow(getWindowSize());
	gl::draw(mFinalTarget->getColorTexture(), vec2(0));

	mGUI->draw();
}

void WFTE_v2App::exit()
{
	mCinderDS->stop();
}

CINDER_APP_NATIVE( WFTE_v2App, RendererGl )
