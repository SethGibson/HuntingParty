#ifndef __CI_DSAPI__
#define __CI_DSAPI__
#include <memory>
#include "DSAPI.h"
#include "DSAPIUtil.h"
#include "cinder/CinderGlm.h"
#include "cinder/gl/Texture.h"

using namespace ci;
using namespace std;
using namespace glm;

namespace CinderDS
{
	enum FrameSize
	{
		DEPTHSD,	// 480x360
		DEPTHVGA,	// 640x480 (628x468)
		COLORVGA,	// 640x480
		COLORHD,	// 1920x1080
	};

	enum StereoCam
	{
		DS_LEFT,
		DS_RIGHT,
		DS_BOTH
	};

	class CinderDSAPI;
	typedef std::shared_ptr<DSAPI> DSAPIRef;
	typedef std::shared_ptr<CinderDSAPI> CinderDSRef;

	class CinderDSAPI
	{
	public:
		static CinderDSRef create();
		~CinderDSAPI();

		bool initRgb(const FrameSize &pRes, const float &pFPS);
		bool initDepth(const FrameSize &pRes, const float &pFPS);
		bool initStereo(const FrameSize &pRes, const float &pFPS, const StereoCam &pWhich);
		bool start();
		bool update();
		bool stop();

		int getDepthWidth(){ return mLRZWidth; }
		int getDepthHeight(){ return mLRZHeight; }
		int getRgbWidth(){ return mRGBWidth; }
		int getRgbHeight(){ return mRGBHeight; }
		ivec2 getDepthSize(){ return ivec2(mLRZWidth, mLRZHeight);  }
		ivec2 getRgbSize(){ return ivec2(mRGBWidth, mRGBHeight); }

		Surface8u getColorFrame();
		Channel8u getLeftFrame();
		Channel8u getRightFrame();
		Channel16u getDepthFrame();

	protected:
		CinderDSAPI();

	private:
		bool	getConfigAndCalib();
		bool	setupStream(const FrameSize &pRes, ivec2 &pOutSize);

		bool	mHasValidConfig,
				mHasValidCalib,
				mHasColor, mHasColorTex,
				mHasDepth, mHasDepthTex,
				mHasLeft, mHasLeftTex,
				mHasRight, mHasRightTex,
				mIsRunning,
				mUpdated;

		int		mLRZWidth,
				mLRZHeight,
				mRGBWidth,
				mRGBHeight;

		DSAPIRef	mDSAPI;
		DSThird		*mDSRGB;

		Surface8u mColorFrame;
		Channel8u mLeftFrame;
		Channel8u mRightFrame;
		Channel16u mDepthFrame;
	};
};
#endif