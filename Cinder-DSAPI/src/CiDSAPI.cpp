#include "CiDSAPI.h"

using namespace std;

namespace CinderDS
{
	vector<camera_type> GetCameraList()
	{
		vector<camera_type> cCameraList;
		for (int i = 0; i < DSGetNumberOfCameras(true); ++i)
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
									mLRZWidth(0), mLRZHeight(0), mRgbWidth(0), mRgbHeight(0){}

	CinderDSAPI::~CinderDSAPI(){}

	bool CinderDSAPI::init()
	{
		if (mDSAPI==nullptr)
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

	bool CinderDSAPI::initRgb(const FrameSize &pRes, const float &pFPS)
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
					}
				}
			}
		}
		return mHasRgb;
	}

	bool CinderDSAPI::initDepth(const FrameSize &pRes, const float &pFPS)
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
				}
			}
		}
		return mHasDepth;
	}

	bool CinderDSAPI::initStereo(const FrameSize &pRes, const float &pFPS, const StereoCam &pWhich)
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
				pOutSize = ivec2(628, 468);
				break;
			case DEPTHVGA:
				pOutSize = ivec2(640, 480);
				break;
			}
			return true;
		}
		return false;
	}
};