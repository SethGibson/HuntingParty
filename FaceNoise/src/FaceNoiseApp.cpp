#ifdef _DEBUG
#pragma comment(lib, "DSAPI.dbg.lib")
#else
#pragma comment(lib, "DSAPI.lib")
#endif

#include <iostream>
#include <algorithm>
#include <array>
#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/audio/Context.h"
#include "cinder/audio/MonitorNode.h"
#include "cinder/audio/Utilities.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/Camera.h"
#include "cinder/MayaCamUI.h"
#include "cinder/params/Params.h"
#include "CiDSAPI.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

static ivec2 S_DIMS(320, 240);

class FaceNoiseApp : public App
{
public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void mouseDrag(MouseEvent event) override;
	void update() override;
	void draw() override;
	void exit();

	struct FacePoint
	{
		vec3 PPosition;
		vec2 PTexCoord;
		FacePoint(vec3 pPos, vec2 pUv) :PPosition(pPos), PTexCoord(pUv){}
	};
private:
	void setupDS();
	void setupMesh();
	void setupAudio();
	void normalizeAudioValues();

	gl::VboRef mDataObj;
	geom::BufferLayout mAttribObj;
	gl::VboMeshRef mMeshObj;
	gl::BatchRef mDrawObj;
	gl::GlslProgRef mShaderObj;
	vector<FacePoint> mPoints;

	CinderDSRef mCinderDS;
	gl::Texture2dRef mTexRgb;

	CameraPersp mCamera;
	MayaCamUI mMayaCam;

	audio::InputDeviceNodeRef		mInputDeviceNode;
	audio::MonitorSpectralNodeRef	mMonitorSpectralNode;
	vector<float>					mMagSpectrum;
	vector<float>					mMagNormalized;
	
	int								mAudioStart, mAudioDir;
	float							mAudioScaleTop, mAudioScaleBottom, mLowThresh, mHighThresh;
	params::InterfaceGlRef			mGUI;
};

void FaceNoiseApp::setup()
{
	setupDS();
	setupMesh();
	setupAudio();

	getWindow()->setSize(1280, 720);
	setFrameRate(60);

	mCamera.setPerspective(45.0f, getWindowAspectRatio(), 0.1f, 100.0f);
	mCamera.lookAt(vec3(0), vec3(0, 0, -1), vec3(0, -1, 0));
	mCamera.setCenterOfInterestPoint(vec3(0, 0, -3));
	mMayaCam.setCurrentCam(mCamera);
	mCamera.setFovHorizontal(57.0f);
	mTexRgb = gl::Texture::create(640, 480);

	gl::enableDepthRead();
	gl::enableDepthWrite();

	mAudioScaleTop = 5.0f;
	mAudioScaleBottom = 3.0f;
	mLowThresh = 0.25f;
	mHighThresh = 0.75f;

	mGUI = params::InterfaceGl::create("Audio Params", vec2(300));
	mGUI->addParam("Audio Scale Top", &mAudioScaleTop);
	mGUI->addParam("Audio Scale Btm", &mAudioScaleBottom);
	mGUI->addParam("Low Thresh (0-1)", &mLowThresh);
	mGUI->addParam("High Thresh (0-1)", &mHighThresh);
	mAudioStart = 0;
	mAudioDir = 1;
}

void FaceNoiseApp::setupDS()
{
	mCinderDS = CinderDSAPI::create();
	mCinderDS->init();
	mCinderDS->initDepth(FrameSize::DEPTHQVGA, 60);
	mCinderDS->initRgb(FrameSize::RGBVGA, 60);
	mCinderDS->start();

	getSignalCleanup().connect(std::bind(&FaceNoiseApp::exit, this));
}

void FaceNoiseApp::setupMesh()
{
	try
	{
		mShaderObj = gl::GlslProg::create(loadAsset("facenoise_vert.glsl"), loadAsset("facenoise_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading shaders: " << endl;
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
			float cu = dx / (float)S_DIMS.x;
			float cv = dy / (float)S_DIMS.y;
			mPoints.push_back(FacePoint(vec3(cx, cy, cz), vec2(cu, cv)));
		}
	}

	mDataObj = gl::Vbo::create(GL_ARRAY_BUFFER, mPoints, GL_DYNAMIC_DRAW);
	mAttribObj.append(geom::POSITION, 3, sizeof(FacePoint), offsetof(FacePoint, PPosition));
	mAttribObj.append(geom::TEX_COORD_0, 2, sizeof(FacePoint), offsetof(FacePoint, PTexCoord));


	mMeshObj = gl::VboMesh::create(mPoints.size(), GL_POINTS, { { mAttribObj, mDataObj } });
	mDrawObj = gl::Batch::create(mMeshObj, mShaderObj);
}

void FaceNoiseApp::setupAudio()
{
	auto ctx = audio::Context::master();
	mInputDeviceNode = ctx->createInputDeviceNode();

	auto monitorFormat = audio::MonitorSpectralNode::Format().fftSize(256).windowSize(128);
	mMonitorSpectralNode = ctx->makeNode(new audio::MonitorSpectralNode(monitorFormat));
	
	mInputDeviceNode >> mMonitorSpectralNode;
	mInputDeviceNode->enable();
	
	ctx->enable();
}

void FaceNoiseApp::mouseDown( MouseEvent event )
{
	mMayaCam.mouseDown(event.getPos());
}

void FaceNoiseApp::mouseDrag(MouseEvent event)
{
	mMayaCam.mouseDrag(event.getPos(), event.isLeftDown(), false, event.isRightDown());
}

void FaceNoiseApp::normalizeAudioValues()
{
	mMagNormalized.clear();

	float cMean = (mHighThresh - mLowThresh)*0.5f;
	auto mMagBounds = std::minmax_element(mMagSpectrum.begin(), mMagSpectrum.end());

	int cMaxAudioId = mMagSpectrum.size();

	for (int aid = 0; aid < cMaxAudioId; ++aid)
	//for (auto mit = mMagSpectrum.begin(); mit != mMagSpectrum.end(); ++mit)
	{
		int cLookupID = (aid + mAudioStart) % cMaxAudioId;
		//int cLookupID = aid;
		float cVal = mMagSpectrum[cLookupID];
		float nVal = lmap<float>(cVal, *mMagBounds.first, *mMagBounds.second, 0.0f, 1.0f);

		if (nVal < mLowThresh)
			nVal += cMean;
		else if (nVal > mHighThresh)
			nVal -= cMean;

		mMagNormalized.push_back(nVal);
	}

	if (getElapsedFrames()%2==0)
		mAudioStart = (mAudioStart + 1) % cMaxAudioId;

}

void FaceNoiseApp::update()
{
	mCinderDS->update();
	mTexRgb->update(mCinderDS->getRgbFrame());
	const uint16_t* cDepth = mCinderDS->getDepthFrame().getData();
	mPoints.clear();
	
	mMagSpectrum = mMonitorSpectralNode->getMagSpectrum();
	normalizeAudioValues();

	int id = 0;
	for (int dy = 0; dy < S_DIMS.y; ++dy)
	{
		for (int dx = 0; dx < S_DIMS.x; ++dx)
		{
			float cVal = (float)cDepth[id];
			if (cVal>100 && cVal < 1000 && dy%2==0)
			{

				int aid = (int)lmap<float>(dy, 0, S_DIMS.y, 0, mMagSpectrum.size()-1);
				float cx = lmap<float>(dx, 0, S_DIMS.x, -1.3333f, 1.3333f);
				float cy = lmap<float>(dy, 0, S_DIMS.y, -1.0f, 1.0f);
				float cz = lmap<float>(cVal, 100, 1000, -1, -5);

				float cAudioScale = lmap<float>(cy, -1.0f, 1.0f, mAudioScaleTop, mAudioScaleBottom);
				float cAudio = mMagNormalized[aid] * cAudioScale;

				vec2 cUV = mCinderDS->mapColorToDepth((float)dx, (float)dy, cVal);
				cUV.y = 1.0 - cUV.y;
				mPoints.push_back(FacePoint(vec3(cx*cAudio, cy, cz), cUV));
			}
			id++;
		}
	}

	//normals
	mDataObj->bufferData(mPoints.size()*sizeof(FacePoint), mPoints.data(), GL_DYNAMIC_DRAW);
	mMeshObj = gl::VboMesh::create(mPoints.size(), GL_POINTS, { { mAttribObj, mDataObj } });
	mDrawObj->replaceVboMesh(mMeshObj);
}

void FaceNoiseApp::draw()
{
	gl::clear( Color( 0.25f, 0.15f, 0.1f ) ); 
	gl::color(Color::white());
	gl::setMatrices(mMayaCam.getCamera());
	//gl::enableAdditiveBlending();
	gl::pointSize(3.0f);
	gl::ScopedTextureBind cRgb(mTexRgb);
	mDrawObj->draw();

	gl::enableAdditiveBlending();
	gl::pointSize(5.0f);
	mDrawObj->draw();
	gl::disableAlphaBlending();

	gl::setMatricesWindow(getWindowSize());
	mGUI->draw();
}

void FaceNoiseApp::exit()
{
	mCinderDS->stop();
}

CINDER_APP( FaceNoiseApp, RendererGl )
