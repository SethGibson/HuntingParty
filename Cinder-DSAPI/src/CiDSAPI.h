#ifndef __CI_DSAPI__
#define __CI_DSAPI__
#include <memory>
#include "DSAPI.h"
#include "DSAPIUtil.h"
#include "cinder/Channel.h"
#include "cinder/CinderGlm.h"
#include "cinder/gl/Texture.h"
#include "cinder/Surface.h"

using namespace ci;
using namespace std;

namespace CinderDS
{
	enum FrameSize
	{
		DEPTHSD,	// 480x360
		DEPTHVGA,	// 640x480 (628x468)
		RGBVGA,	// 640x480
		RGBHD,	// 1920x1080
	};

	enum StereoCam
	{
		DS_LEFT,
		DS_RIGHT,
		DS_BOTH
	};

	typedef pair<int, uint32_t> camera_type;
	static vector<camera_type> GetCameraList();

	class CinderDSAPI;
	typedef std::shared_ptr<DSAPI> DSAPIRef;
	typedef std::shared_ptr<CinderDSAPI> CinderDSRef;

	class CinderDSAPI
	{
	protected:
		CinderDSAPI();
	public:
		static CinderDSRef create();
		~CinderDSAPI();

		bool init();
		bool init(uint32_t pSerialNo);
		bool initRgb(const FrameSize &pRes, const float &pFPS);
		bool initDepth(const FrameSize &pRes, const float &pFPS);
		bool initStereo(const FrameSize &pRes, const float &pFPS, const StereoCam &pWhich);
		bool start();
		bool update();
		bool stop();

		int getDepthWidth(){ return mLRZWidth; }
		int getDepthHeight(){ return mLRZHeight; }
		int getRgbWidth(){ return mRgbWidth; }
		int getRgbHeight(){ return mRgbHeight; }
		const ivec2 getDepthSize(){ return ivec2(mLRZWidth, mLRZHeight); }
		const ivec2 getRgbSize(){ return ivec2(mRgbWidth, mRgbHeight); }

		const Surface8u& getRgbFrame();
		const Channel8u& getLeftFrame();
		const Channel8u& getRightFrame();
		const Channel16u& getDepthFrame();

	private:
		bool	open();
		bool	setupStream(const FrameSize &pRes, ivec2 &pOutSize);

		bool	mHasValidConfig,
				mHasValidCalib,
				mHasRgb,
				mHasDepth,
				mHasLeft,
				mHasRight,
				mIsInit,
				mUpdated;

		int32_t	mLRZWidth,
				mLRZHeight,
				mRgbWidth,
				mRgbHeight;

		DSAPIRef	mDSAPI;
		DSThird		*mDSRGB;

		Surface8u mRgbFrame;
		Channel8u mLeftFrame;
		Channel8u mRightFrame;
		Channel16u mDepthFrame;

		Surface8uRef mRgbFrameRef;
		Channel8uRef mLeftFrameRef;
		Channel8uRef mRightFrameRef;
		Channel16uRef mDepthFrameRef;

	};
};
#endif