#include "CiDSAPI.h"

using namespace std;

namespace CinderDS
{
	CinderDSRef create()
	{

	}

	CinderDSAPI::CinderDSAPI()
	{

	}

	CinderDSAPI::~CinderDSAPI()
	{

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
					mHasColor = mDSRGB->setThirdResolutionMode(true, cSize.x, cSize.y, pFPS, DS_RGB8);
					if (mHasColor)
					{
						mRGBWidth = cSize.x;
						mRGBHeight = cSize.y;
					}
				}
			}
		}
		return mHasColor;
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
			if (pWhich == DS_LEFT || pWhich == DS_BOTH)
			{

			}
			if (pWhich == DS_RIGHT || pWhich == DS_BOTH)
			{

			}

			switch (pWhich)
			{
			case DS_LEFT:
				return mHasLeft;
			case DS_RIGHT:
				return mHasRight;
			case DS_BOTH:
				return mHasLeft && mHasRight;
			}
		}
		return false;
	}

	bool CinderDSAPI::start()
	{

	}

	bool CinderDSAPI::update()
	{

	}

	bool CinderDSAPI::stop()
	{
		if (mDSAPI&&mIsRunning)
		{
			mIsRunning = false;
			mDSAPI->stopCapture();
		}
	}

	Surface8u CinderDSAPI::getColorFrame()
	{
		if (mUpdated&&mHasColor)
			return mColorFrame;
	}

	Channel8u CinderDSAPI::getLeftFrame()
	{
		if (mUpdated&&mHasLeft)
			return mLeftFrame;
	}

	Channel8u CinderDSAPI::getRightFrame()
	{
		if (mUpdated&&mHasRight)
			return mRightFrame;
	}

	Channel16u CinderDSAPI::getDepthFrame()
	{
		if (mUpdated&&mHasDepth)
			return mDepthFrame;
	}

	bool CinderDSAPI::getConfigAndCalib()
	{
		if (mDSAPI)
		{
			mHasValidConfig = mDSAPI->probeConfiguration();
			mHasValidCalib = mDSAPI->isCalibrationValid();

			return mHasValidConfig&&mHasValidCalib;
		}

		return false;
	}

	bool CinderDSAPI::setupStream(const FrameSize &pRes, ivec2 &pOutSize)
	{
		if (!mHasValidConfig||!mHasValidCalib)
		{
			if (getConfigAndCalib())
			{
				switch (pRes)
				{
				case COLORVGA:
					pOutSize = ivec2(640, 480);
					break;
				case COLORHD:
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
		return false;
	}
};