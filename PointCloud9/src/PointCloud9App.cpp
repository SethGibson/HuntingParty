#include <algorithm>
#include <memory>
#include "cinder/app/AppNative.h"
#include "cinder/Arcball.h"
#include "cinder/audio/Context.h"
#include "cinder/audio/MonitorNode.h"
#include "cinder/audio/Utilities.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Rand.h"
#include "cinder/Thread.h"
#include "ciTri.h"
#include "DSAPI.h"
#include "DSAPIUtil.h"

#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif

using namespace ci;
using namespace ci::app;
using namespace std;

static Vec2i S_DEPTH_SIZE(320, 240);
static Vec2i S_COLOR_SIZE(640, 480);
static Vec2i S_P9_STEP(32, 24);
static int S_SWITCH_TIME = 60;
static float S_CUBE_OFFSET = 30;

typedef pair<Vec3f, Vec2f> vertex_pair;

class PointCloud9App : public AppNative
{
public:
	void prepareSettings(Settings *pSettings);
	void setup();
	void mouseDown( MouseEvent pEvent );	
	void mouseDrag(MouseEvent pEvent);
	void update();
	void updateAudio();
	void draw();
	void shutdown();
	void getSampleSet();
	bool SamplesLoaded;
	int MaxSamples;

	struct PC9P
	{
		bool Active;
		Vec2i IPosition;
		Vec3f WPosition;
		Color PColor;
		float Size;

		PC9P(int pX, int pY)
		{
			IPosition = Vec2i(pX, pY);
			WPosition = Vec3f::zero();
			Active = false;
			PColor = Color::white();
			Size = 10.0f;
		}
	};

	struct PC9Tri
	{
		Vec3f A;
		Vec3f B;
		Vec3f C;
		Vec3f Centroid;
	};

	shared_ptr<thread>		mThread;

private:
	bool setupDSAPI();
	void setupCamera();
	void setupAudio();
	
	int roundTo(int pNum, int pMult);
	void getTriangles();

	DSAPI *mDSAPI;
	DSThird *mDSRgb;
	DSHardware *mDSHW;
	DSCalibIntrinsicsRectified mZIntrins;
	DSCalibIntrinsicsRectified mRgbIntrins;

	double *mRgbTrans;
	uint16_t *mDepthBuffer;
	uint8_t *mRgbBuffer;

	gl::VboMeshRef mVboMesh;
	CameraPersp mCamera;
	MayaCamUI mMayaCam;
	gl::Texture mRgbTexture;
	gl::Texture::Format mFormat;

	vector<PC9P> mCubes;

	vector<Vec2f> mSampleSet;
	vector<Triangle> mTriangles;
	vector<PC9Tri> mTrianglesCamera;
	float mTriRes;
	int mRedId, mAudioId;

	audio::InputDeviceNodeRef mInputDeviceNode;
	audio::MonitorSpectralNodeRef mMonitorSpectralNode;
	vector<float> mMagSpectrum;
	float mMagMean, mMagMin,mMagMax;

};

void PointCloud9App::prepareSettings(Settings *pSettings)
{
	pSettings->setWindowSize(1280, 720);
	pSettings->setFrameRate(60);
}

void PointCloud9App::setup()
{
	setupCamera();
	setupDSAPI();
	setupAudio();

	
	gl::VboMesh::Layout cLayout;
	cLayout.setStaticIndices();
	//cLayout.setDynamicTexCoords2d();
	cLayout.setDynamicColorsRGB();
	cLayout.setDynamicPositions();

	size_t cCount = S_DEPTH_SIZE.x*S_DEPTH_SIZE.y;
	vector<uint32_t> cIndices;
	uint32_t cId = 0;
	mVboMesh = gl::VboMesh::create(cCount, cCount, cLayout, GL_POINTS);
	
	for (int dy = 0; dy < S_DEPTH_SIZE.y; ++dy)
	{
		for (int dx = 0; dx < S_DEPTH_SIZE.x; ++dx)
		{
			cIndices.push_back(cId);
			++cId;
			if (dx%S_P9_STEP.x == 0 && dy%S_P9_STEP.y == 0)
			{
				mCubes.push_back(PC9P(dx, dy));
			}
		}
	}
	mFormat.setInternalFormat(GL_BGRA_EXT);
	mFormat.setMinFilter(GL_LINEAR);
	mFormat.setMagFilter(GL_LINEAR);
	mFormat.setWrapS(GL_CLAMP);
	mFormat.setWrapT(GL_CLAMP);

	mVboMesh->bufferIndices(cIndices);
	
	mRedId = Rand::randInt(0, mCubes.size());
	mAudioId = 2;
	mTriRes = 40.0f;
	//generate sample set
	MaxSamples = mCubes.size() * 0.75f;
	SamplesLoaded = false;
	mThread = shared_ptr<thread>(new thread( bind(&PointCloud9App::getSampleSet, this) ) );
	
}

void PointCloud9App::getSampleSet()
{
	int cMaxSamples = MaxSamples;
	int cSId = 0;
	vector<Vec2i> cFound;

	while (cSId < cMaxSamples)
	{
		int cx = roundTo(Rand::randInt(0, S_DEPTH_SIZE.x), S_P9_STEP.x);
		int cy = roundTo(Rand::randInt(0, S_DEPTH_SIZE.y), S_P9_STEP.y);
		Vec2i cPt(cx, cy);
		if (std::find(cFound.begin(), cFound.end(), cPt) == cFound.end())
		{
			mSampleSet.push_back(Vec2f(cPt));
			cFound.push_back(Vec2i(cPt));
			cSId += 1;
		}
	}
	SamplesLoaded = true;
}

void PointCloud9App::setupCamera()
{
	mCamera.setPerspective(49.04f, getWindowAspectRatio(), 100, 4000);
	mCamera.lookAt(Vec3f::zero(), Vec3f(0,0,1000), Vec3f(0, -1, 0));
	mCamera.setCenterOfInterestPoint(Vec3f(0, 0, 750));
	mMayaCam.setCurrentCam(mCamera);
}

void PointCloud9App::setupAudio()
{
	auto cAudioCtx = audio::Context::master();
	mInputDeviceNode = cAudioCtx->createInputDeviceNode();

	auto cMonitorFormat = audio::MonitorSpectralNode::Format().fftSize(2048).windowSize(1024);
	mMonitorSpectralNode = cAudioCtx->makeNode(new audio::MonitorSpectralNode(cMonitorFormat));

	mInputDeviceNode >> mMonitorSpectralNode;
	mInputDeviceNode->enable();
	cAudioCtx->enable();

	mMagMean = 0;
}

bool PointCloud9App::setupDSAPI()
{
	bool retVal = true;
	mDSAPI = DSCreate(DS_DS4_PLATFORM);
	mRgbTrans = new double[3]{0};

	if (!mDSAPI->probeConfiguration())
	{
		retVal = false;
		console() << "Unable to get DS hardware config" << endl;
	}
	if (!mDSAPI->isCalibrationValid())
	{
		retVal = false;
		console() << "Calibration is invalid" << endl;
	}
	if (!mDSAPI->enableLeft(true))
	{
		retVal = false;
		console() << "Unable to start left stream" << endl;
	}
	if (!mDSAPI->enableRight(true))
	{
		retVal = false;
		console() << "Unable to start right stream" << endl;
	}
	if (!mDSAPI->enableZ(true))
	{
		retVal = false;
		console() << "Unable to start depth stream" << endl;
	}
	if (!mDSAPI->setLRZResolutionMode(true, S_DEPTH_SIZE.x, S_DEPTH_SIZE.y, 60, DS_LUMINANCE8))
	{
		retVal = false;
		console() << "Unable to set requested depth resolution" << endl;
	}
	if (!mDSAPI->getCalibIntrinsicsZ(mZIntrins))
	{
		retVal = false;
		console() << "Unable to get depth intrinsics" << endl;
	}
	//Hardware
	mDSHW = mDSAPI->accessHardware();
	if (!mDSHW)
	{
		retVal = false;
		console() << "Unable to access hardware" << endl;
	}

	// RGB Stuff
	mDSRgb = mDSAPI->accessThird();
	if (!mDSRgb)
	{
		retVal = false;
		console() << "Unable to open rgb camera" << endl;
	}

	if (!mDSRgb->enableThird(true))
	{
		retVal = false;
		console() << "Unable to enable Rgb camera" << endl;
	}
	if (!mDSRgb->setThirdResolutionMode(true, S_COLOR_SIZE.x, S_COLOR_SIZE.y, 60, DS_RGB8))
	{
		retVal = false;
		console() << "Unable to set Rgb resolution" << endl;
	}
	if (!mDSRgb->getCalibIntrinsicsRectThird(mRgbIntrins))
	{
		retVal = false;
		console() << "Unable to get Rgb intrinsics" << endl;
	}
	if (!mDSRgb->getCalibIntrinsicsRectThird(mRgbIntrins))
	{
		retVal = false;
		console() << "Unable to get Rgb intrinsics" << endl;
	}
	if (!mDSRgb->getCalibExtrinsicsZToRectThird(mRgbTrans))
	{
		retVal = false;
		console() << "Unable to get Rgb offset" << endl;
	}


	if (!mDSHW->setAutoExposure(DS_BOTH_IMAGERS, false))
	{
		retVal = false;
		console() << "Unable to set exposure" << endl;
	}
	// Go
	if (!mDSAPI->startCapture())
	{
		retVal = false;
		console() << "Unable to start ds4" << endl;
	}

	return retVal;
}

void PointCloud9App::mouseDown(MouseEvent pEvent)
{
	mMayaCam.mouseDown(pEvent.getPos());
}

void PointCloud9App::mouseDrag(MouseEvent pEvent)
{
	mMayaCam.mouseDrag(pEvent.getPos(), pEvent.isLeftDown(), false, pEvent.isRightDown());
}


void PointCloud9App::updateAudio()
{
	mMagSpectrum = mMonitorSpectralNode->getMagSpectrum();
	auto cFFTBounds = std::minmax(mMagSpectrum.begin(), mMagSpectrum.end());

	float cSum = 0.0f;
	float cMax = -1.0f;
	float cMin = 10000.0f;
	mMagMean = 0.0f;
	for (auto cV : mMagSpectrum)
	{
		if (cV > cMax)
			cMax = cV;
		else if (cV < cMin)
			cMin = cV;
		cSum += cV;
	}

	if (mMagSpectrum.size() > 0)
	{
		cSum /= (float)mMagSpectrum.size();
		mMagMean = math<float>::clamp(cSum * 10000, 0, 1);
	}

	mMagMin = cMin;
	mMagMax = cMax;

	std::reverse(mMagSpectrum.begin(), mMagSpectrum.end());
}

void PointCloud9App::update()
{
	updateAudio();

	mDSAPI->grab();
	mDepthBuffer = mDSAPI->getZImage();
	mRgbBuffer = (uint8_t *)mDSRgb->getThirdImage();

	if (SamplesLoaded)
		getTriangles();

	gl::VboMesh::VertexIter cIter = mVboMesh->mapVertexBuffer();

	int cCubeId = 0;
	for (int dy = 0; dy < S_DEPTH_SIZE.y; dy++)
	{
		for (int dx = 0; dx < S_DEPTH_SIZE.x; dx++)
		{
			float cDepth = (float)mDepthBuffer[dy*S_DEPTH_SIZE.x + dx];
			if (dy % 2 == 0 && dx % 2 == 0)
			{
				cIter.setPosition(Vec3f::zero());
				cIter.setColorRGB(Color::black());

				if (cDepth > 0 && cDepth < 2000)
				{
					float cZCamera[3]{0};

					float cZImage[3] = { static_cast<float>(dx), static_cast<float>(dy), cDepth };
					float cRgbCamera[3]{0};
					float cRgbImage[2]{0};

					DSTransformFromZImageToZCamera(mZIntrins, cZImage, cZCamera);
					DSTransformFromZCameraToRectThirdCamera(mRgbTrans, cZCamera, cRgbCamera);
					DSTransformFromThirdCameraToRectThirdImage(mRgbIntrins, cRgbCamera, cRgbImage);

					cIter.setPosition(cZCamera[0], cZCamera[1], cZCamera[2]);
					if (cRgbImage[0]>=0&&cRgbImage[1]>=0)
					{
						int cId = ((int)cRgbImage[1] * S_COLOR_SIZE.x + (int)cRgbImage[0]) * 3;
						uint8_t cR = mRgbBuffer[cId];
						uint8_t cG = mRgbBuffer[cId + 1];
						uint8_t cB = mRgbBuffer[cId + 2];
						cIter.setColorRGB(ColorT<uint8_t>(cR, cG, cB));
					}
					else
						cIter.setColorRGB(Color::white());
				}
			}

			if (dx%S_P9_STEP.x == 0 && dy%S_P9_STEP.y==0)
			{
				int cMagId = (int)lmap<float>((float)cCubeId, 0, mCubes.size(), 0.f, mMagSpectrum.size()-1);
				float cFFTVal = mMagSpectrum[cMagId];
				cFFTVal = lmap<float>(cFFTVal, mMagMin, mMagMax, 5, 60);

				float cZCubeImg[3]{mCubes.at(cCubeId).IPosition.x, mCubes.at(cCubeId).IPosition.y, cDepth-S_CUBE_OFFSET};
				float cZCubeCam[3]{0};
				DSTransformFromZImageToZCamera(mZIntrins, cZCubeImg, cZCubeCam);
				mCubes.at(cCubeId).WPosition.x = cZCubeCam[0];
				mCubes.at(cCubeId).WPosition.y = cZCubeCam[1];
				mCubes.at(cCubeId).WPosition.z = cZCubeCam[2];
				mCubes.at(cCubeId).Active = (cDepth > 0 && cDepth < 2000);
				mCubes.at(cCubeId).Size = cFFTVal;

				if (cCubeId == mRedId)
					mCubes.at(cCubeId).PColor = Color(1, 0, 0);
				else
					mCubes.at(cCubeId).PColor = Color::white();
				cCubeId++;
			}
			++cIter;
		}
	}

	//mAudioId = (mAudioId + 1) % 5;
	if (getElapsedFrames() % S_SWITCH_TIME == 0)
		mRedId = Rand::randInt(0, mCubes.size());
	if (getElapsedFrames() % 10 == 0)
		mTriRes = (float)roundTo(Rand::randInt(40, 75), 5);
}

void PointCloud9App::draw()
{
	// clear out the window with black
	gl::clear( Color( 0.1f, 0.1f, 0.1f ) ); 
	gl::color(Color::white());
	gl::setMatrices(mMayaCam.getCamera());

	glPointSize(5.5f);
	//gl::enable(GL_TEXTURE_2D);
	gl::enable(GL_DEPTH_TEST);
	gl::enableAdditiveBlending();
	//mRgbTexture.enableAndBind();

	gl::draw(mVboMesh);
	//mRgbTexture.unbind();

	//gl::disable(GL_TEXTURE_2D);
	gl::lineWidth(3.0f);
	if (SamplesLoaded)
	{
		gl::color(ColorA(1, 1, 1, 0.2f));

		int cTID = 0;
		for (auto cT : mTrianglesCamera)
		{
			gl::drawLine(cT.A, cT.B);
			gl::drawLine(cT.B, cT.C);
			gl::drawLine(cT.C, cT.A);
			cTID++;
		}
	}

	for (PC9P cP : mCubes)
	{
		if (cP.Active)
		{
			gl::color(cP.PColor);
			gl::drawCube(cP.WPosition, Vec3f(cP.Size, cP.Size, cP.Size));
		}
	}
	gl::disableAlphaBlending();
	/*
	gl::setMatricesWindow(getWindowSize());
	if (mRgbTexture)
		gl::draw(mRgbTexture, Rectf(0, 0, 320, 240));
		*/
}

void PointCloud9App::shutdown()
{
	mThread->join();
	delete[] mRgbTrans;
	mDSAPI->stopCapture();
	DSDestroy(mDSAPI);
}


int PointCloud9App::roundTo(int pNum, int pMult)
{
	if(pMult == 0)
		return pNum;

	int cMod = pNum % pMult;
	if (cMod == 0)
		return pNum;
	return pNum + pMult - cMod;
}

void PointCloud9App::getTriangles()
{
	mTriangles.clear();
	mTrianglesCamera.clear();

	mTriangles = ciTri::triangulate(mSampleSet, mTriRes);

	for (auto t : mTriangles)
	{
		float cZImage[3]{0};
		float cZCamera[3]{0};
		float cDepth = 0;
		Vec2f cA, cB, cC;
		PC9Tri cTri3d;

		//get camera a
		cA = t.a;
		cDepth = (float)mDepthBuffer[(int)cA.y*S_DEPTH_SIZE.x + (int)cA.x];
		if (cDepth < 100 || cDepth>2000)
			continue;
		cZImage[0] = cA.x;
		cZImage[1] = cA.y;
		cZImage[2] = cDepth-S_CUBE_OFFSET;
		DSTransformFromZImageToZCamera(mZIntrins, cZImage, cZCamera);
		cTri3d.A = Vec3f(cZCamera[0], cZCamera[1], cZCamera[2]);

		//get camera b
		cB = t.b;
		cDepth = (float)mDepthBuffer[(int)cB.y*S_DEPTH_SIZE.x + (int)cB.x];
		if (cDepth < 100 || cDepth>2000)
			continue;
		cZImage[0] = cB.x;
		cZImage[1] = cB.y;
		cZImage[2] = cDepth - S_CUBE_OFFSET;
		DSTransformFromZImageToZCamera(mZIntrins, cZImage, cZCamera);
		cTri3d.B = Vec3f(cZCamera[0], cZCamera[1], cZCamera[2]);

		//get camera c
		cC = t.c;
		cDepth = (float)mDepthBuffer[(int)cC.y*S_DEPTH_SIZE.x + (int)cC.x];
		if (cDepth < 100 || cDepth>2000)
			continue;
		cZImage[0] = cC.x;
		cZImage[1] = cC.y;
		cZImage[2] = cDepth - S_CUBE_OFFSET;
		DSTransformFromZImageToZCamera(mZIntrins, cZImage, cZCamera);
		cTri3d.C = Vec3f(cZCamera[0], cZCamera[1], cZCamera[2]);

		mTrianglesCamera.push_back(cTri3d);
	}
}

CINDER_APP_NATIVE( PointCloud9App, RendererGl )
