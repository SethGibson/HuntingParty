#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/Shader.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/ObjLoader.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

#include "CiDSAPI.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Rand.h"
#include <glm/gtc/matrix_transform.hpp>
#include "CinderOpenCV.h"
#include "opencv2\gpu\gpu.hpp"
#include "opencv2\videostab\optical_flow.hpp"
#include <deque>
#include "RGBD.h"
#include "nanoflann.hpp"
#include <random>
#include <tuple>
#include "cinder/gl/gl.h"


using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;
using namespace cv;

struct ParticleInstance
{
public:
	vec3 InitialPosition;
	vec2 DepthPixelCoord;

	ParticleInstance(vec3 startPos, vec2 depthPixelCoord, float currentTime)
	{
		InitialPosition = startPos;
		DepthPixelCoord = depthPixelCoord;
	}
};

const static int MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE = 200000; //THERE'S A LIMIT TO PARTICLES DEPENDING ON THE GPU, I DON'T KNOW WHY -- somwhere aroud 75k is when it starts glitching
const static float DEATH_LIFETIME_IN_SECONDS = 3.5; //total time until dying particle instances expire
const static int SECONDS_BETWEEN_NEW_SPAWNS = 0.5;
const static int DEPTH_SUBSAMPLE_WINDOW_SIZE = 4; //X by X window size to subsample the depth buffer for spawning instanced meshes. 1 = full depth buffer, 2 = 2x2 so half the number of instances.
const static int VARIABLE_CAP = 50; //max diff between front and back pixels per window to qualify as valid depth subsample.
const static int DESIRED_EDGE_NUM_CAP = 150; //desired cap (i.e., number to decimate down to) to the number of edges we keep for kd-tree indexing.

int width;
int height;
int numberToDraw = 0;
int backNumberToDraw = 0;
int numberTrianglesToDraw = 0;
float GravityForceMultiplier = 9.8;
float ForwardForceMultiplier = 1;
uint32_t previousTotalDepth = 0;

double previousElapsedTime = 0;

const float MaxDeathTimeSeconds = 5;

std::mt19937 engine;

//std::uniform_real_distribution<float> dist(0.3f, 1.2f);

float getRandomFloat(float a, float b) { return std::uniform_real_distribution<float>(a, b)(engine); }

class DyingParticlesManager
{
	enum ParticleManagerAppendSetting
	{
		AddIfNotFull,
		DeleteOldestToAdd
	};

private:

	void KillFrontParticleBatch()
	{
		CurrentTotalParticles -= mDyingParticleBatches.front().size();

		mDyingParticleBatches.pop_front();
		BatchCreationTime.pop_front();
	}

public:
	std::deque<std::vector<ParticleInstance>> mDyingParticleBatches;
	std::deque<float> BatchCreationTime;

	int CurrentTotalParticles = 0;
	ParticleManagerAppendSetting appendSetting = ParticleManagerAppendSetting::DeleteOldestToAdd;

	bool AddDyingParticleBatch(std::vector<ParticleInstance> particles, float currentTime)
	{
		if (particles.size() > 0 && CurrentTotalParticles + particles.size() > MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE)
		{ //no space... time to clear out enough space for the batch
			if (appendSetting == ParticleManagerAppendSetting::DeleteOldestToAdd)
			{ //clear enough space at the end to add this full particle batch
				int numParticlesToRemove = ((CurrentTotalParticles + particles.size()) - MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE);
				while (numParticlesToRemove > 0)
				{
					if (numParticlesToRemove < mDyingParticleBatches.front().size())
					{ //front batch is bigger than the space we need to clear
						mDyingParticleBatches.front().erase(mDyingParticleBatches.front().begin(), //first
							mDyingParticleBatches.front().begin() + numParticlesToRemove); //remove all below this index
						CurrentTotalParticles -= numParticlesToRemove;
						numParticlesToRemove = 0;
					}
					else
					{ //need to remove more than one batch.
						numParticlesToRemove -= mDyingParticleBatches.front().size();
						KillFrontParticleBatch();
					}

				}
			}
		}

		if (CurrentTotalParticles + particles.size() <= MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE)
		{
			CurrentTotalParticles += particles.size();

			mDyingParticleBatches.push_back(particles);
			BatchCreationTime.push_back(currentTime);
			return true;
		}

		return false;
	}

	void CleanupUpdate(float currentTime)
	{
		while (BatchCreationTime.empty() == false && BatchCreationTime.front() + DEATH_LIFETIME_IN_SECONDS < currentTime)
		{
			KillFrontParticleBatch();
		}
	}

	void SetupBatchDraw(std::vector<bool> &dyingExistsArray, vec3* mappedPositionsVBO, float* totalLivingTimesVBO, float* dyingScalesVBO, float currentTime, float dyingScale, int currentDyingExistsArrayWidth, int currentDyingExistsArrayHeight)
	{
		std::fill(dyingExistsArray.begin(), dyingExistsArray.end(), false);

		for (int i = 0; i < mDyingParticleBatches.size(); i++)
		{
			for (int j = 0; j < mDyingParticleBatches[i].size(); j++)
			{
				//we should never end up rendering more than the buffer can hold because the "Add" function checks for that.
				*mappedPositionsVBO++ = mDyingParticleBatches[i][j].InitialPosition;
				*totalLivingTimesVBO++ = (currentTime - BatchCreationTime[i]);
				*dyingScalesVBO++ = dyingScale; //getRandomFloat(0.1, 10.0);

				dyingExistsArray[mDyingParticleBatches[i][j].DepthPixelCoord.x + (mDyingParticleBatches[i][j].DepthPixelCoord.y * currentDyingExistsArrayWidth)] = true;
			}
		}
	}
};

class HP_WaitingRTApp : public App {
public:
	void setup();
	void cleanup() override;
	void resize();
	void update();
	void draw();
	void keyDown(KeyEvent event);
	void mouseDown(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;

	void SetupParticles(vec3 *positions, vec3 *backPositions, float *scales, vec4 *colors, float *dyingScales, float *fillerScales, vec3 *deathPositions, float *totalLivingTimes, vec3 *trianglePositions, uint16_t *triangleIndices, vec4 *triangleColors, float elapsed);
	std::vector<uint16_t> GetSuperPixelAverageDepths(uint16_t* depthMap, int width, int height, int subsampleWindowSize);

	void SetupBackParticlesDraw(std::vector<float> backParticlePositions, std::vector<float> backParticleScales, vec3 *backPositionsVBO, float *backScalesVBO, int particleArrayWidth, int particleArrayHeight);

	std::vector<vec4> GetRGBSubsampledColors(std::vector<uint16_t> subsampledDepths, int subsampledWidth, int subsampledHeight);
	void SetupTriangles(std::vector<uint16_t> &subsampledDepths, std::vector<vec4> &subsampledColors, int subsampleWidth, int subsampledHeight, vec3* &trianglePositions, vec4* &triangleColors, uint16_t* &triangleIndices);
	void GLDrawTriangles();

	void UpdateOpticalFlow(std::vector<uint16_t> &depthFrame, int depthWidth, int depthHeight);

	CameraPersp			mCam;
	gl::BatchRef		mBatch;
	gl::BatchRef		mBatchDyingCubes;
	gl::BatchRef		mBatchBackubes;
	gl::BatchRef		mBatchTriangles;

	gl::TextureRef		mTexture;

	gl::GlslProgRef		mGlsl;
	gl::GlslProgRef		mGlslDyingCubes;
	gl::GlslProgRef		mGlslBackCubes;
	gl::GlslProgRef		mGlslTriangles;

	gl::VboRef			mInstancePositionVbo;
	gl::VboRef			mInstanceScaleVbo;
	gl::VboRef			mVertColorsVbo;

	gl::VboRef			mInstanceDyingDataVbo;
	gl::VboRef			mInstanceDyingScaleVbo;
	gl::VboRef			mInstanceDyingTotalLifetimesVbo;

	gl::VboRef			mInstanceBackPositionVbo;
	gl::VboRef			mInstanceFillerScaleVbo;

	gl::VboRef			mTrianglePositionsVbo;
	gl::VboRef			mTriangleIndicesVbo;
	gl::VboRef			mTriangleColorsVbo;
	gl::VboMeshRef		mTrianglesVboMesh;

	gl::TextureRef		mLogoTexture;

	//DSAPI
	CinderDSRef mDSAPI;
	MayaCamUI mMayaCam;

	Surface8u mRgbBuffer;
	Channel16u mDepthBuffer;

	DSCalibIntrinsicsRectified zIntrin, thirdIntrin;
	double zToThirdTrans[3];

	std::vector<bool> mPreviouslyHadDepth;
	std::vector<bool> mCurrentlyHasDyingInstance;
	std::vector<vec3> mPreviousValidDepthCoord;

	DyingParticlesManager mDyingParticleManager;

	std::vector<int> mHogCounts;
	std::vector<long> mHogTotals;
	const float HogStartDepth = 500;
	const int HogBinStep = 30;
	const int NumberOfDepthSteps = 50; //only go (30 * 50) = 1500 pixels in from StartDepth
	float mBackDepthPlaneZ = 0;

	float TimeOfLastInstanceBatchSpawn = -10;

	int mDepthSubsampleSize;
	bool DrawBackParticles = true;
	bool DrawDyingParticles = true;
	bool DrawFrontParticles = true;
	bool DrawTriangles = true;
	bool DrawBatchTriangles = false;
	bool MaskFrontParticles = false;
	bool ModuloTriangleDirections = false;

	Mat previousDepthImage;
	Mat currentFlow;
	bool firstFrame = true;

	std::vector<vec4> subsampledColors;
	vector<vec3> rgbPoints;

	bool PauseRGBCapture = false;
};