#ifndef __CI_DSAPI__
#define __CI_DSAPI__
#include <memory>
#include "DSAPI.h"
#include "DSAPIUtil.h"

namespace CinderDS
{
	enum VideoResolution
	{
		DEPTHSD,	// 480x360
		DEPTHVGA,	// 640x480
		COLORVGA,	// 640x480
		COLORHD,	// 1920x1080
	};

	class CinderDSAPI;
	typedef std::shared_ptr<DSAPI> DSAPIRef;
	typedef std::shared_ptr<CinderDSAPI> CinderDSRef;

	class CinderDSAPI
	{
	public:
		static CinderDSRef create();
		~CinderDSAPI();

		bool initColor(VideoResolution pRes, float pFPS);
		bool initDepth(VideoResolution pRes, float pFPS);
		bool initStereo(VideoResolution pRes, float pFPS, bool pLeft, bool pRight);
		void start();
		void update();
		void stop();

		uint8_t* getColorFrame();
		uint8_t* getLeftFrame();
		uint8_t* getRightFrame();
		uint16_t* getDepthFrame();

	private:
		bool	mHasValidConfig,
				mHasValidCalib;

		bool	mHasColor,
				mHasDepth,
				mHasLeft,
				mHasRight;

		DSAPIRef	mDSAPI;
		DSThird		*mDSRGB;

	};
};
#endif