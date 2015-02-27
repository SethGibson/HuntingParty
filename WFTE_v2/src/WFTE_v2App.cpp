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
#include "cinder/TriMesh.h"
#include "CiDSAPI.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

static ivec2 S_DIMS(480, 360);
static ivec2 S_STEP(60, 40);

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

	gl::BatchRef		mCloudDrawObj;
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
	
	gl::GlslProgRef		mShaderHBlur;
	gl::GlslProgRef		mShaderVBlur;
	gl::FboRef			mHBlurTarget;
	gl::FboRef			mVBlurTarget;

	//Cube Mesh
	gl::BatchRef		mMeshDrawObj;
	gl::GlslProgRef		mMeshShader;
	geom::Cube			mMeshCube;
	gl::VboMeshRef		mMeshVbo;
	gl::VboRef			mInstanceData;
	geom::BufferLayout	mInstanceAttribs;
	vector<vec3>		mMeshPoints;

	gl::VboRef			mMeshVertexData;
	gl::VboMeshRef		mMeshVertex;
	geom::BufferLayout	mMeshVertexAttribs;
	gl::BatchRef		mMeshVertexDraw;
	gl::GlslProgRef		mMeshVertexShader;
	vector<vec3>		mVertexPoints;
	TriMeshRef			mVertexTriMesh;

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
						mBloomSize,
						mBlurSize;

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
	mBlurSize = 1.0f / 512.0f;

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
	mGUI->addParam("Blur Amount", &mBlurSize);
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
		console() << "Error loading resolve shader: " << endl;
		console() << e.what() << endl;
	}
	try
	{
		mShaderHBlur = gl::GlslProg::create(loadAsset("WFTE_vert_fbo.glsl"), loadAsset("WFTE_04_hblur_frag.glsl"));
		mShaderVBlur = gl::GlslProg::create(loadAsset("WFTE_vert_fbo.glsl"), loadAsset("WFTE_04_vblur_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading blur shaders: " << endl;
		console() << e.what() << endl;
	}
	try
	{
		mMeshShader = gl::GlslProg::create(loadAsset("WFTE_inst_vert.glsl"), loadAsset("WFTE_inst_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading instance shaders: " << endl;
		console() << e.what() << endl;
	}
	try
	{
		mMeshVertexShader = gl::GlslProg::create(loadAsset("WFTE_mesh_vert.glsl"), loadAsset("WFTE_mesh_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading instance shaders: " << endl;
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
	mMeshPoints.clear();
	mVertexPoints.clear();

	mVertexTriMesh = TriMesh::create();
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
			if (dx % S_STEP.x == 0 && dy % S_STEP.y == 0)
			{
				mMeshPoints.push_back(vec3(cx, cy, cz));
				mVertexPoints.push_back(vec3(cx, cy, cz));
				mVertexTriMesh->appendVertex(vec3(cx, cy, cz));
			}
		}
	}

	mVertexData = gl::Vbo::create(GL_ARRAY_BUFFER, mPoints, GL_DYNAMIC_DRAW);
	mVertexAttribs.append(geom::POSITION, 3, sizeof(CloudPoint), offsetof(CloudPoint, PPosition));
	mVertexAttribs.append(geom::COLOR, 4, sizeof(CloudPoint), offsetof(CloudPoint, PColor));
	mVertexAttribs.append(geom::TEX_COORD_0, 2, sizeof(CloudPoint), offsetof(CloudPoint, PTexCoord));

	mPointCloud = gl::VboMesh::create(mPoints.size(), GL_POINTS, { { mVertexAttribs, mVertexData } });
	mCloudDrawObj = gl::Batch::create(mPointCloud, mShaderColor);

	mMeshCube = geom::Cube().size(vec3(0.025));
	mInstanceData = gl::Vbo::create(GL_ARRAY_BUFFER, mMeshPoints, GL_DYNAMIC_DRAW);
	mInstanceAttribs.append(geom::CUSTOM_0, 3, 0, 0, 1);

	mMeshVbo = gl::VboMesh::create(mMeshCube);
	mMeshVbo->appendVbo(mInstanceAttribs, mInstanceData);
	mMeshDrawObj = gl::Batch::create(mMeshVbo, mMeshShader, { { geom::CUSTOM_0, "iPosition" } });

	mMeshVertexData = gl::Vbo::create(GL_ARRAY_BUFFER, mVertexPoints, GL_DYNAMIC_DRAW);
	mMeshVertexAttribs.append(geom::POSITION, 3, 0, 0, 0);
	mMeshVertex = gl::VboMesh::create(mVertexPoints.size(), GL_TRIANGLE_STRIP, { { mMeshVertexAttribs, mMeshVertexData } });
	mMeshVertexDraw = gl::Batch::create(mMeshVertex, mMeshVertexShader);
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

	gl::Fbo::Format cBlurFormat;
	cBlurFormat.colorTexture(gl::Texture2d::Format().internalFormat(GL_RGBA16F));
	mHBlurTarget = gl::Fbo::create(1280, 720, cBlurFormat);
	mVBlurTarget = gl::Fbo::create(1280, 720, cBlurFormat);

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
	mMeshPoints.clear();
	mVertexPoints.clear();
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
				if (dx % S_STEP.x == 0 && dy % S_STEP.y == 0)
				{
					mMeshPoints.push_back(vec3(cx, cy, cz));
					mVertexPoints.push_back(vec3(cx, cy, cz));
				}
			}
			id++;
		}
	}

	mVertexData->bufferData(mPoints.size()*sizeof(CloudPoint), mPoints.data(), GL_DYNAMIC_DRAW);
	mPointCloud = gl::VboMesh::create(mPoints.size(), GL_POINTS, { { mVertexAttribs, mVertexData } });
	mCloudDrawObj->replaceVboMesh(mPointCloud);

	mInstanceData->bufferData(mMeshPoints.size()*sizeof(vec3), mMeshPoints.data(), GL_DYNAMIC_DRAW);
	mMeshVbo = gl::VboMesh::create(mMeshCube);
	mMeshVbo->appendVbo(mInstanceAttribs, mInstanceData);
	mMeshDrawObj->replaceVboMesh(mMeshVbo);

	mMeshVertexData->bufferData(mVertexPoints.size()*sizeof(vec3), mVertexPoints.data(), GL_DYNAMIC_DRAW);
	mMeshVertex = gl::VboMesh::create(mVertexPoints.size(), GL_TRIANGLE_STRIP, { { mMeshVertexAttribs, mMeshVertexData } });
	mMeshVertexDraw->replaceVboMesh(mMeshVertex);
}

void WFTE_v2App::renderScene()
{
	gl::enableAdditiveBlending();
	//Color VBO
	mColorTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mColorTarget->getSize());
	gl::setMatrices(mMayaCam.getCamera());
	mTexRgb->bind();
	gl::pointSize(mDrawSize);
	mCloudDrawObj->draw();
	mColorTarget->unbindFramebuffer();
	mTexRgb->unbind();

	//Luminance VBO
	mLumTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mLumTarget->getSize());
	gl::setMatrices(mMayaCam.getCamera());
	mTexRgb->bind();
	gl::pointSize(mBloomSize);
	mShaderRender->uniform("mBloomMin", mBloomMin);
	mShaderRender->uniform("mBloomMax", mBloomMax);
	mCloudDrawObj->replaceGlslProg(mShaderRender);
	mCloudDrawObj->draw();
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
	mShaderResolve->uniform("mHdrTex", 0);
	mShaderResolve->uniform("mBloomTex", 1);
	mShaderResolve->uniform("mExposure", mExposure);
	mShaderResolve->uniform("mBloomFactor", mBloomFactor);
	mShaderResolve->uniform("mSceneFactor", mSceneFactor);
	gl::drawSolidRect(Rectf({ vec2(0), mLumTarget->getSize() }));
	mLumTarget->unbindTexture();
	mFilterTarget->unbindTexture();
	mFinalTarget->unbindFramebuffer();

	//Blur H Stage
	mHBlurTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mHBlurTarget->getSize());
	gl::setMatricesWindow(getWindowSize());
	mFinalTarget->bindTexture();
	mShaderHBlur->bind();
	mShaderHBlur->uniform("mBlurSize", mBlurSize);
	gl::drawSolidRect(Rectf({ vec2(0), mFinalTarget->getSize() }));
	mFilterTarget->unbindTexture();
	mHBlurTarget->unbindFramebuffer();

	//Blur V Stage
	mVBlurTarget->bindFramebuffer();
	gl::clear(Color::black());
	gl::viewport(ivec2(0), mVBlurTarget->getSize());
	gl::setMatricesWindow(getWindowSize());
	mHBlurTarget->bindTexture();
	mShaderVBlur->bind();
	mShaderVBlur->uniform("mBlurSize", mBlurSize);
	gl::drawSolidRect(Rectf({ vec2(0), mHBlurTarget->getSize() }));
	mHBlurTarget->unbindTexture();
	mVBlurTarget->unbindFramebuffer();
	gl::disableAlphaBlending();
}

void WFTE_v2App::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
	gl::color(Color::white());
	gl::enableAdditiveBlending();
	gl::setMatrices(mMayaCam.getCamera());
	mTexRgb->bind();
	mCloudDrawObj->replaceGlslProg(mShaderColor);
	gl::pointSize(mDrawSize);
	mCloudDrawObj->draw();
	mTexRgb->unbind();

	//gl::enableWireframe();
	//mMeshVertexDraw->draw();
	gl::disableWireframe();
	mMeshDrawObj->drawInstanced(mMeshPoints.size());

	gl::setMatricesWindow(getWindowSize());
	gl::draw(mFinalTarget->getColorTexture(), vec2(0));
	gl::draw(mVBlurTarget->getColorTexture(), vec2(0));
	
	mGUI->draw();
}

void WFTE_v2App::exit()
{
	mCinderDS->stop();
}

CINDER_APP_NATIVE( WFTE_v2App, RendererGl )
