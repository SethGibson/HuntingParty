#include "CiDSAPI.h"

using namespace std;

namespace CinderDS
{
	vector<camera_type> GetCameraList()
	{
		vector<camera_type> cCameraList;
		int cNumCameras = DSGetNumberOfCameras(true);
		for (int i = 0; i < cNumCameras; ++i)
		{
			cCameraList.push_back(camera_type(i, DSGetCameraSerialNumber(i)));
		}

		return cCameraList;
	}

	CinderDSRef CinderDSAPI::create()
	{
		return CinderDSRef(new CinderDSAPI());
	}

	CinderDSAPI::CinderDSAPI() : mHasValidConfig(false), mHasValidCalib(false),
									mHasRgb(false), mHasDepth(false),
									mHasLeft(false), mHasRight(false),
									mIsInit(false), mUpdated(false),
									mLRZWidth(0), mLRZHeight(0), mRgbWidth(0), mRgbHeight(0), mDSAPI(nullptr){}

	CinderDSAPI::~CinderDSAPI(){}

	bool CinderDSAPI::init()
	{
		if (mDSAPI == nullptr)
			mDSAPI = DSAPIRef(DSCreate(DS_DS4_PLATFORM), DSDestroy);

		return open();
	}

	bool CinderDSAPI::init(uint32_t pSerialNo)
	{
		if (mDSAPI == nullptr)
			mDSAPI = DSAPIRef(DSCreate(DS_DS4_PLATFORM, pSerialNo), DSDestroy);

		if (mDSAPI)
		{
			if (!mHasValidConfig)
				mHasValidConfig = mDSAPI->probeConfiguration();
			if (!mHasValidCalib)
				mHasValidCalib = mDSAPI->isCalibrationValid();

			mIsInit = mHasValidConfig&&mHasValidCalib;
		}
		return mIsInit;
	}

	bool CinderDSAPI::initForAlignment()
	{
		mDSAPI->accessThird()->enableThird(true);


		mDSAPI->accessEmitter()->enableEmitter(true);


		mDSAPI->enableLRCrop(true);
		mDSAPI->enableLeft(false);
		mDSAPI->enableRight(false);
		mDSAPI->enableZ(true);


		if (!mDSAPI->setLRZResolutionMode(true, 480, 360, 30, DS_LUMINANCE8)) return false;
		if (!mDSAPI->accessThird()->setThirdResolutionMode(true, 640, 480, 30, DS_RGB8)) return false;

		return true;
	}

	bool CinderDSAPI::initRgb(const FrameSize &pRes, const int &pFPS)
	{
		ivec2 cSize;
		if(setupStream(pRes, cSize))
		{
			mDSRGB = mDSAPI->accessThird();
			if (mDSRGB)
			{
				if(mDSRGB->enableThird(true))
				{
					mHasRgb = mDSRGB->setThirdResolutionMode(true, cSize.x, cSize.y, pFPS, DS_RGB8);
					if (mHasRgb)
					{
						mRgbWidth = cSize.x;
						mRgbHeight = cSize.y;
						mRgbFrame = Surface8u(mRgbWidth, mRgbHeight, false, SurfaceChannelOrder::RGB);
						mDSRGB->getCalibIntrinsicsRectThird(mRgbIntrinsics);
						mDSRGB->getCalibExtrinsicsZToRectThird(mZToRgb);
					}
				}
			}
		}
		return mHasRgb;
	}

	bool CinderDSAPI::initDepth(const FrameSize &pRes, const int &pFPS)
	{
		ivec2 cSize;
		if (setupStream(pRes, cSize))
		{
			if(mDSAPI->enableZ(true))
			{
				mHasDepth = mDSAPI->setLRZResolutionMode(true, cSize.x, cSize.y, pFPS, DS_LUMINANCE8);
				if (mHasDepth)
				{
					mLRZWidth = cSize.x;
					mLRZHeight = cSize.y;
					mDepthFrame = Channel16u(mLRZWidth, mLRZHeight);
					mDSAPI->getCalibIntrinsicsZ(mZIntrinsics);
				}
			}
		}
		return mHasDepth;
	}

	bool CinderDSAPI::initStereo(const FrameSize &pRes, const int &pFPS, const StereoCam &pWhich)
	{
		ivec2 cSize;
		if (setupStream(pRes, cSize))
		{
			if ((mLRZWidth > 0 && cSize.x != mLRZWidth) ||
				(mLRZHeight > 0 && cSize.y != mLRZHeight))
				return false;

			bool cModeSet = false;
			if (pWhich == DS_LEFT || pWhich == DS_BOTH)
			{
				mHasLeft = mDSAPI->enableLeft(true);
				cModeSet = mDSAPI->setLRZResolutionMode(true, cSize.x, cSize.y, pFPS, DS_LUMINANCE8);
			}
			if (pWhich == DS_RIGHT || pWhich == DS_BOTH)
			{
				mHasRight = mDSAPI->enableRight(true);
				cModeSet = mDSAPI->setLRZResolutionMode(true, cSize.x, cSize.y, pFPS, DS_LUMINANCE8);
			}

			switch (pWhich)
			{
			case DS_LEFT:
				return mHasLeft&&cModeSet;

			case DS_RIGHT:
				return mHasRight&&cModeSet;

			case DS_BOTH:
				return (mHasLeft && mHasRight) && cModeSet;
			}
		}
		return false;
	}

	bool CinderDSAPI::start()
	{
		return mDSAPI->startCapture();
	}

	bool CinderDSAPI::update()
	{
		bool retVal = mDSAPI->grab();
		if (retVal)
		{
			if (mHasRgb)
				mRgbFrame = Surface8u((uint8_t *)mDSRGB->getThirdImage(), mRgbWidth, mRgbHeight, mRgbWidth * 3, SurfaceChannelOrder::RGB);
			if (mHasDepth)
				mDepthFrame = Channel16u(mLRZWidth, mLRZHeight, int32_t(mLRZWidth*sizeof(uint16_t)), 1, mDSAPI->getZImage());
			if (mHasLeft)
				mLeftFrame = Channel8u(mLRZWidth, mLRZHeight, mLRZWidth, 1, (uint8_t *)mDSAPI->getLImage());
			if (mHasRight)
				mRightFrame = Channel8u(mLRZWidth, mLRZHeight, mLRZWidth, 1, (uint8_t *)mDSAPI->getRImage());
		}
		return retVal;;
	}

	bool CinderDSAPI::stop()
	{
		if (mDSAPI)
			return mDSAPI->stopCapture();
		return false;
	}

	const Surface8u& CinderDSAPI::getRgbFrame()
	{
		return mRgbFrame;
	}

	const Channel8u& CinderDSAPI::getLeftFrame()
	{
		return mLeftFrame;
	}

	const Channel8u& CinderDSAPI::getRightFrame()
	{
		return mRightFrame;
	}

	const Channel16u& CinderDSAPI::getDepthFrame()
	{
		return mDepthFrame;
	}

	const vector<ivec2>& CinderDSAPI::mapDepthToColorFrame()
	{
		mDepthToColor.clear();
		Channel16u::Iter cIter = mDepthFrame.getIter();
		while (cIter.line())
		{
			while (cIter.pixel())
			{
				float cZ[3]{static_cast<float>(cIter.x()),
					static_cast<float>(cIter.y()),
					static_cast<float>(cIter.v())
				};

				float cRgb[2]{0};
				DSTransformFromZImageToRectThirdImage(mZIntrinsics, mZToRgb, mRgbIntrinsics, cZ, cRgb);
				mDepthToColor.push_back(ivec2((int)cRgb[0], (int)cRgb[1]));
			}
		}

		return mDepthToColor;
	}

	/*
	const Surface32f& CinderDSAPI::mapDepthToCameraTable()
	{
		mDepthToCameraTable = Surface32f(mLRZWidth, mLRZHeight, false, SurfaceChannelOrder::RGB);
		Surface32f::Iter cSurfIter = mDepthToCameraTable.getIter();
		Channel16u::Iter cDepthIter = mDepthFrame.getIter();

		while (cDepthIter.line())
		{
			while (cDepthIter.pixel())
			{
				int cx = cDepthIter.x();
				int cy = cDepthIter.y();

				float cZ[3]{static_cast<float>(cx),
							static_cast<float>(cy),
							static_cast<float>(cDepthIter.v())};
				float cZCam[3]{0};

				DSTransformFromZImageToZCamera(mZIntrinsics, cZ, cZCam);
				mDepthToCameraTable.setPixel(ivec2(cx, cy), Colorf(cZCam[0], cZCam[1], 0.0f));
			}
		}
	}*/

	const vec3 CinderDSAPI::getZCameraSpacePoint(float pX, float pY, float pZ)
	{
		float cZImage[3]{ pX, pY, pZ };
		float cZCamera[3]{0};

		DSTransformFromZImageToZCamera(mZIntrinsics, cZImage, cZCamera);
		return vec3(cZCamera[0], cZCamera[1], cZCamera[2]);
	}

	const vec3 CinderDSAPI::getZCameraSpacePoint(int pX, int pY, uint16_t pZ)
	{
		return getZCameraSpacePoint(static_cast<float>(pX), static_cast<float>(pY), static_cast<float>(pZ));
	}

	const vec3 CinderDSAPI::getZCameraSpacePoint(vec3 pPoint)
	{
		return getZCameraSpacePoint(pPoint.x, pPoint.y, pPoint.z);
	}

	const Color CinderDSAPI::getDepthSpaceColor(float pX, float pY, float pZ)
	{
		vec3 cZCamera = getZCameraSpacePoint(pX, pY, pZ);
		return getColorFromZCamera(cZCamera.x, cZCamera.y, cZCamera.z);
	}

	const vec2 CinderDSAPI::mapColorToDepth(float pX, float pY, float pZ)
	{
		float cZImage[3]{ pX, pY, pZ };
		float cZCamera[3]{0};
		float cRgbCamera[3]{0};
		float cRgbImage[2]{0};
		DSTransformFromZImageToZCamera(mZIntrinsics, cZImage, cZCamera);
		DSTransformFromZCameraToRectThirdCamera(mZToRgb, cZCamera, cRgbCamera);
		DSTransformFromThirdCameraToRectThirdImage(mRgbIntrinsics, cRgbCamera, cRgbImage);

		float cu = static_cast<int>(cRgbImage[0]) / (float)mRgbWidth;
		float cv = static_cast<int>(cRgbImage[1]) / (float)mRgbHeight;

		return vec2(cu, cv);
	}
	const Color CinderDSAPI::getColorFromZCamera(float pX, float pY, float pZ)
	{
		float cZCamera[3]{pX, pY, pZ};
		float cRgbCamera[3]{0};
		float cRgbImage[2]{0};
		DSTransformFromZCameraToRectThirdCamera(mZToRgb, cZCamera, cRgbCamera);
		DSTransformFromThirdCameraToRectThirdImage(mRgbIntrinsics, cRgbCamera, cRgbImage);

		/*
		mRgbIter = mRgbFrame.getIter();
		float cR = mRgbIter.r((int)cRgbImage[0], (int)cRgbImage[1]) / 255.0f;
		float cG = mRgbIter.g((int)cRgbImage[0], (int)cRgbImage[1]) / 255.0f;
		float cB = mRgbIter.b((int)cRgbImage[0], (int)cRgbImage[1]) / 255.0f;
		return Color(cR, cG, cB);
		*/

		ivec2 cPos(static_cast<int>(cRgbImage[0]), static_cast<int>(cRgbImage[1]));
		ColorA cColor = mRgbFrame.getPixel(cPos);
		return Color(cColor.r, cColor.g, cColor.b);
	}

	const Color CinderDSAPI::getColorFromZCamera(vec3 pPoint)
	{
		return getColorFromZCamera(pPoint.x, pPoint.y, pPoint.z);
	}

	const vec2 CinderDSAPI::getDepthFOVs()
	{
		float cFovX, cFovY;
		DSFieldOfViewsFromIntrinsicsRect(mZIntrinsics, cFovX, cFovY);
		return vec2(cFovX, cFovY);
	}

	const vec2 CinderDSAPI::getRgbFOVs()
	{
		float cFovX, cFovY;
		DSFieldOfViewsFromIntrinsicsRect(mRgbIntrinsics, cFovX, cFovY);
		return vec2(cFovX, cFovY);
	}

	const DSAPIRef CinderDSAPI::getDSAPI()
	{
		return mDSAPI;
	}

	DSThird* CinderDSAPI::getDSThird()
	{
		return mDSRGB;
	}

	const DSCalibIntrinsicsRectified CinderDSAPI::getZIntrinsics()
	{
		return mZIntrinsics;
	}
	const DSCalibIntrinsicsRectified CinderDSAPI::getRgbIntrinsics()
	{
		return mRgbIntrinsics;
	}

	bool CinderDSAPI::open()
	{
		if (mDSAPI)
		{
			if (!mHasValidConfig)
				mHasValidConfig = mDSAPI->probeConfiguration();
			if (!mHasValidCalib)
				mHasValidCalib = mDSAPI->isCalibrationValid();

			mIsInit = mHasValidConfig&&mHasValidCalib;
		}
		return mIsInit;
	}

	bool CinderDSAPI::setupStream(const FrameSize &pRes, ivec2 &pOutSize)
	{
		if (!mIsInit)
		{
			if (!open())
				return false;
		}

		if (mIsInit)
		{
			switch (pRes)
			{
			case RGBVGA:
				pOutSize = ivec2(640, 480);
				break;
			case RGBHD:
				pOutSize = ivec2(1920, 1080);
				break;
			case DEPTHSD:
				pOutSize = ivec2(480, 360);
				break;
			case DEPTHVGA:
				pOutSize = ivec2(628, 468);
				break;
			case DEPTHQVGA:
				pOutSize = ivec2(320, 240);
				break;
			}
			return true;
		}
		return false;
	}
};