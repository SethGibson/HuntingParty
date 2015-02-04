#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif

#include "cinder/app/AppNative.h"
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
#include <deque>
#include "depth_filter.h"
#include "RGBD.h"
#include "nanoflann.hpp"
#include <random>
#include <tuple>


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

const static int MaxNumberDyingParticleBufferSize = 50000; //THERE'S A LIMIT TO PARTICLES DEPENDING ON THE GPU, I DON'T KNOW WHY -- somwhere aroud 75k is when it starts glitching
const static float DeathLifetimeInSeconds = 1.5; //total time until dying particle instances expire
const static int SecondsBetweenNewSpawns = 0.5;
const static int DepthSubsampleWindowSize = 4; //X by X window size to subsample the depth buffer for spawning instanced meshes. 1 = full depth buffer, 2 = 2x2 so half the number of instances.
const static int VariableCap = 50; //max diff between front and back pixels per window to qualify as valid depth subsample.
const static int DesiredEdgeNumCap = 150; //desired cap (i.e., number to decimate down to) to the number of edges we keep for kd-tree indexing.

class DyingParticlesManager
{

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

	void AddDyingParticleBatch(std::vector<ParticleInstance> particles, float currentTime)
	{
		if (CurrentTotalParticles + particles.size() <= MaxNumberDyingParticleBufferSize)
		{
			CurrentTotalParticles += particles.size();

			mDyingParticleBatches.push_back(particles);
			BatchCreationTime.push_back(currentTime);
		}
	}

	void CleanupUpdate(float currentTime)
	{
		while (BatchCreationTime.empty() == false && BatchCreationTime.front() + DeathLifetimeInSeconds < currentTime)
		{
			KillFrontParticleBatch();
		}
	}

	void SetupBatchDraw(vec3* mappedPositionsVBO, float* totalLivingTimesVBO, float currentTime)
	{
		for (int i = 0; i < mDyingParticleBatches.size(); i++)
		{
			for (int j = 0; j < mDyingParticleBatches[i].size(); j++)
			{
				//we should never end up rendering more than the buffer can hold because the "Add" function checks for that.
				*mappedPositionsVBO++ = mDyingParticleBatches[i][j].InitialPosition;
				*totalLivingTimesVBO++ = (currentTime - BatchCreationTime[i]);
			}
		}
	}
};

template <typename T>
struct EdgeCloud
{
	std::vector<ivec2>  edges;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return edges.size(); }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T *p1, const size_t idx_p2, size_t /*size*/) const
	{
		const T d0 = p1[0] - edges[idx_p2].x;
		const T d1 = p1[1] - edges[idx_p2].y;
		return d0*d0 + d1*d1;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim == 0) return edges[idx].x;
		else return edges[idx].y;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};

typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<int, EdgeCloud<int>>, EdgeCloud<int>, 2> my_kd_tree_t;

class HP_WaitingRTApp : public AppNative {
public:
	void setup();
	void shutdown() override;
	void resize();
	void update();
	void draw();
	void keyDown(KeyEvent event);
	void mouseDown(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;

	void SetupParticles(vec3 *positions, vec3 *backPositions, float *scales, float *fillerScales, vec3 *deathPositions, float *totalLivingTimes, float elapsed);
	std::vector<uint16_t> GetSuperPixelAverageDepths(uint16_t* depthMap, int width, int height, int subsampleWindowSize);
	vec3 GetSuperPixelAveragePosition(std::vector<uint16_t> averageDepths, int width, int subsampleWindowSize, int x, int y);

	void GetKDEdgeIndex(uint16_t* depthFrame, int depthWidth, int depthHeight, int lowThresh, int highThresh, Mat &cannyEdgePixels, EdgeCloud<int> &edgeCloud);

	CameraPersp			mCam;
	gl::BatchRef		mBatch;
	gl::BatchRef		mBatchDyingCubes;
	gl::TextureRef		mTexture;
	gl::GlslProgRef		mGlsl;
	gl::GlslProgRef		mGlslDyingCubes;
	gl::VboRef			mInstancePositionVbo;
	gl::VboRef			mInstanceScaleVbo;
	gl::VboRef			mInstanceDyingDataVbo;
	gl::VboRef			mInstanceDyingTotalLifetimesVbo;
	gl::VboRef			mInstanceFillerScaleVbo;

	gl::BatchRef		mBatchBackubes;
	gl::GlslProgRef		mGlslBackCubes;
	gl::VboRef			mInstanceBackPositionVbo;

	//DSAPI
	CinderDSRef mDSAPI;
	MayaCamUI mMayaCam;

	Surface8u mRgbBuffer;
	Channel16u mDepthBuffer;

	std::vector<bool> mPreviouslyHadDepth;
	//std::vector<bool> mCurrentlyHasDeadInstance;
	std::vector<vec3> mPreviousValidDepthCoord;

	DyingParticlesManager mDyingParticleManager;
	//std::ofstream out;

	depthf_Handle mDepthFilter;
	//uint16_t* zAligned;

	int lowThreshold = 4;	// low = 4; high = 6 gives proper outlines
	int highThreshold = 6;	// low = 4; high = 5 gives a sweet interior outliney effect, and fixes the back-estimation

	std::vector<int> mHogCounts;
	std::vector<long> mHogTotals;
	const float HogStartDepth = 500;
	const int HogBinStep = 30;
	const int NumberOfDepthSteps = 50; //only go (30 * 50) = 1500 pixels in from StartDepth
	float mBackDepthPlaneZ = 0;

	float TimeOfLastInstanceBatchSpawn = -10;

	int mDepthSubsampleSize;
};

int width;
int height;
int numberToDraw = 0;
int backNumberToDraw = 0;

double previousElapsedTime = 0;

const float MaxDeathTimeSeconds = 5;

std::mt19937 engine;

//std::uniform_real_distribution<float> dist(0.3f, 1.2f);

float getRandomFloat(float a, float b) { return std::uniform_real_distribution<float>(a, b)(engine); }


void HP_WaitingRTApp::setup()
{
	mDepthSubsampleSize = DepthSubsampleWindowSize;

	//out = std::ofstream("output.txt");


	mDSAPI = CinderDSAPI::create();
	mDSAPI->init();
	mDSAPI->initDepth(CinderDS::FrameSize::DEPTHSD, 60);
	mDSAPI->initRgb(CinderDS::FrameSize::RGBVGA, 30);
	//mDSAPI->initForAlignment();
	mDSAPI->start();

	width = mDSAPI->getDepthWidth();
	height = mDSAPI->getDepthHeight();
	/*
	//width = 480;
	//height = 360;

	//mDepthFilter = depthf_Create(DEPTH_FILTER_KNN, width, height);
	//int rgbW = mDSAPI->getDSAPI()->accessThird()->thirdWidth();
	//int rgbH = mDSAPI->getDSAPI()->accessThird()->thirdHeight();
	//zAligned = new uint16_t[rgbW * rgbH];
	*/

	//mCam.lookAt(vec3(0, CAMERA_Y_RANGE.first, 0), vec3(0));
	vec2 cFOVs = mDSAPI->getDepthFOVs();
	mCam.setPerspective(cFOVs.y, getWindowAspectRatio(), 100, 2500);
	mCam.lookAt(vec3(0, 0, 0), vec3(0, 0, 1000), vec3(0, -1, 0));

	mMayaCam = MayaCamUI(mCam);

	mTexture = gl::Texture::create(loadImage(loadAsset("texture.jpg")), gl::Texture::Format().mipmap());
#if ! defined( CINDER_GL_ES )
	mGlsl = gl::GlslProg::create(loadAsset("shader.vert"), loadAsset("shader.frag"));
	mGlslDyingCubes = gl::GlslProg::create(loadAsset("shaderDying.vert"), loadAsset("shaderDying.frag"));
	mGlslBackCubes = gl::GlslProg::create(loadAsset("shaderBack.vert"), loadAsset("shaderBack.frag"));
#else
	mGlsl = gl::GlslProg::create(loadAsset("shader_es2.vert"), loadAsset("shader_es2.frag"));
#endif

	gl::VboMeshRef mesh = gl::VboMesh::create(geom::Cube());
	gl::VboMeshRef meshDying = gl::VboMesh::create(geom::Cone().subdivisionsAxis(4));
	gl::VboMeshRef meshBack = gl::VboMesh::create(geom::Sphere().subdivisions(4));

	// create an array of initial per-instance positions laid out in a 2D grid
	std::vector<vec3> positions;
	std::vector<vec3> backPositions;
	std::vector<float> cubeScales;
	std::vector<float> fillerScales;
	std::vector<vec3> deathPositions = std::vector<vec3>(MaxNumberDyingParticleBufferSize);
	std::vector<float> deathTotalLifetimes = std::vector<float>(MaxNumberDyingParticleBufferSize);
	for (size_t potX = 0; potX < width; ++potX) {
		for (size_t potY = 0; potY < height; ++potY) {
			float instanceX = potX / (float)width - 0.5f;
			float instanceY = potY / (float)height - 0.5f;
			vec3 pos = vec3(instanceX * vec3(1, 0, 0) + instanceY * vec3(0, 0, 1));
			positions.push_back(pos);
			backPositions.push_back(pos);

			cubeScales.push_back(randFloat(0.2, 5));
			fillerScales.push_back(randFloat(0.2, 5));

			mPreviouslyHadDepth.push_back(false);
			//mCurrentlyHasDeadInstance.push_back(false);
			mPreviousValidDepthCoord.push_back(pos);
		}
	}

	for (int i = 0; i < NumberOfDepthSteps; i++)
	{
		mHogCounts.push_back(0);
		mHogTotals.push_back(0);
	}

	// create the VBO which will contain per-instance (rather than per-vertex) data
	mInstancePositionVbo = gl::Vbo::create(GL_ARRAY_BUFFER, positions.size() * sizeof(vec3), positions.data(), GL_DYNAMIC_DRAW);
	mInstanceScaleVbo = gl::Vbo::create(GL_ARRAY_BUFFER, cubeScales.size() * sizeof(float), cubeScales.data(), GL_DYNAMIC_DRAW);
	mInstanceDyingDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER, (MaxNumberDyingParticleBufferSize)* sizeof(vec3), deathPositions.data(), GL_DYNAMIC_DRAW);
	mInstanceDyingTotalLifetimesVbo = gl::Vbo::create(GL_ARRAY_BUFFER, (MaxNumberDyingParticleBufferSize)* sizeof(float), deathTotalLifetimes.data(), GL_DYNAMIC_DRAW);
	mInstanceBackPositionVbo = gl::Vbo::create(GL_ARRAY_BUFFER, backPositions.size() * sizeof(vec3), backPositions.data(), GL_DYNAMIC_DRAW);
	mInstanceFillerScaleVbo = gl::Vbo::create(GL_ARRAY_BUFFER, fillerScales.size() * sizeof(float), fillerScales.data(), GL_DYNAMIC_DRAW);

	// we need a geom::BufferLayout to describe this data as mapping to the CUSTOM_0 semantic, and the 1 (rather than 0) as the last param indicates per-instance (rather than per-vertex)
	geom::BufferLayout instanceDataLayout;
	instanceDataLayout.append(geom::Attrib::CUSTOM_0, 3, 0, 0, 1 /* per instance */);

	geom::BufferLayout instanceScaleLayout;
	instanceScaleLayout.append(geom::Attrib::CUSTOM_1, 1, 0, 0, 1 /* per instance */);

	// now add it to the VboMesh we already made of the Teapot
	mesh->appendVbo(instanceDataLayout, mInstancePositionVbo);
	mesh->appendVbo(instanceScaleLayout, mInstanceScaleVbo);

	// and finally, build our batch, mapping our CUSTOM_0 attribute to the "vInstancePosition" GLSL vertex attribute
	mBatch = gl::Batch::create(mesh, mGlsl, { { geom::Attrib::CUSTOM_0, "vInstancePosition" }, { geom::Attrib::CUSTOM_1, "fCubeScale" } });


	geom::BufferLayout instanceDyingDataLayout;
	instanceDyingDataLayout.append(geom::Attrib::CUSTOM_2, 3, 0, 0, 1 /* per instance */);

	geom::BufferLayout instanceDeathTotalLifetimesLayout;
	instanceDeathTotalLifetimesLayout.append(geom::Attrib::CUSTOM_3, 1, 0, 0, 1 /* per instance */);

	meshDying->appendVbo(instanceScaleLayout, mInstanceScaleVbo);
	meshDying->appendVbo(instanceDyingDataLayout, mInstanceDyingDataVbo);
	meshDying->appendVbo(instanceDeathTotalLifetimesLayout, mInstanceDyingTotalLifetimesVbo);

	mBatchDyingCubes = gl::Batch::create(meshDying, mGlslDyingCubes, { { geom::Attrib::CUSTOM_1, "fCubeScale" }, { geom::Attrib::CUSTOM_2, "vDeathInitialPosition" }, { geom::Attrib::CUSTOM_3, "fTotalLifetime" } });


	geom::BufferLayout instanceBackDataLayout;
	instanceBackDataLayout.append(geom::Attrib::CUSTOM_0, 3, 0, 0, 1 /* per instance */);

	geom::BufferLayout instanceFillerScaleLayout;
	instanceFillerScaleLayout.append(geom::Attrib::CUSTOM_1, 1, 0, 0, 1 /* per instance */);

	meshBack->appendVbo(instanceBackDataLayout, mInstanceBackPositionVbo);
	meshBack->appendVbo(instanceFillerScaleLayout, mInstanceFillerScaleVbo);

	mBatchBackubes = gl::Batch::create(meshBack, mGlslBackCubes, { { geom::Attrib::CUSTOM_0, "vInstancePosition" }, { geom::Attrib::CUSTOM_1, "fCubeScale" } });


	gl::enableDepthWrite();
	gl::enableDepthRead();
	gl::enableAlphaBlending();

	mTexture->bind();

	previousElapsedTime = getElapsedSeconds();
}

void HP_WaitingRTApp::shutdown()
{
	mDSAPI->stop();
	//out.close();
}

void HP_WaitingRTApp::resize()
{
	// now tell our Camera that the window aspect ratio has changed
	mCam.setPerspective(60, getWindowAspectRatio(), 100, 2500);
	mMayaCam.setCurrentCam(mCam);

	// and in turn, let OpenGL know we have a new camera
	gl::setMatrices(mCam);
}

void HP_WaitingRTApp::update()
{

	float elapsed = getElapsedSeconds();
	float frameTime = elapsed - previousElapsedTime;
	previousElapsedTime = elapsed;

	//cleanup any old particles before setting up the draw.
	mDyingParticleManager.CleanupUpdate(elapsed);

	mDSAPI->update();

	mDepthBuffer = mDSAPI->getDepthFrame();
	mRgbBuffer = mDSAPI->getRgbFrame();

	// update our instance positions; map our instance data VBO, write new positions, unmap
	vec3 *positions = (vec3*)mInstancePositionVbo->mapWriteOnly(true);
	vec3 *backPositions = (vec3*)mInstanceBackPositionVbo->mapWriteOnly(true);
	float *scales = (float*)mInstanceScaleVbo->mapWriteOnly(true);
	float *fillerScales = (float*)mInstanceFillerScaleVbo->mapWriteOnly(true);
	vec3 *deathPositions = (vec3*)mInstanceDyingDataVbo->mapWriteOnly(true);
	float *totalLivingTimes = (float*)mInstanceDyingTotalLifetimesVbo->mapWriteOnly(true);

	numberToDraw = 0;
	backNumberToDraw = 0;


	/*
	just in place to test different things right now.
	*/
	if (false)
	{
		//Given mapped buffers, set up all the particles for drawing.
		SetupParticles(positions, backPositions, scales, fillerScales, deathPositions, totalLivingTimes, elapsed);
	}
	else
	{
		//subsampled depth particles:::

		std::vector<uint16_t> subsampledDepths = GetSuperPixelAverageDepths(mDepthBuffer.getDataStore().get(), width, height, mDepthSubsampleSize);
		int subsampledWidth = width / mDepthSubsampleSize;
		int subsampledHeight = height / mDepthSubsampleSize;



		/* CANNY EDGE BUILDING */
		EdgeCloud<int> edgeCloud; //pass as reference to fill the "edges" array
		Mat cannyEdgePixels; //pass as reference to set the mat of canny edge pixels
		GetKDEdgeIndex(subsampledDepths.data(), subsampledWidth, subsampledHeight, 9, 11, cannyEdgePixels, edgeCloud);

		my_kd_tree_t index(2, edgeCloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
		index.buildIndex();
		bool kdIndexReady = (index.size() > 0);
		/* END OF CANNY EDGE BUILDING */


		for (int y = 0; y < subsampledHeight; y++)
		{
			for (int x = 0; x < subsampledWidth; x++)
			{
				float subsampledAverageDepth = subsampledDepths[(y * subsampledWidth) + x];
				bool cannyEdgePixel = (cannyEdgePixels.data[x + (subsampledWidth * y)] != 0);

				if (subsampledAverageDepth != 0 && subsampledAverageDepth < mCam.getFarClip())
				{
					vec3 subsampledDepthPoint = vec3(
						((float)x * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
						((float)y * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
						subsampledAverageDepth);

					vec3 worldPos = mDSAPI->getDepthSpacePoint(subsampledDepthPoint);

					*positions++ = worldPos;
					*scales++ = 1.0f;// (float)mDepthSubsampleSize / 2;// 1.0f;

					numberToDraw++;

					//--------------
					if (kdIndexReady)
					{
						const float mirrorDepthLerpPastPercentage = 0.8; //percentage of Z to add PAST the axis depth, based on how far away this depth pixel was from it's axis. (0% to 100%)
						const float silhouetteZOffset = 1500; //additional z offset past the edge this pixel is bound to, so pixels near the edge still add at least *some* z.

						const int query_pt[2] = { x, y };
						const size_t num_results = 1;
						std::vector<size_t>   ret_index(num_results);
						std::vector<int> out_dist_sqr(num_results);
						index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
						float axisZ = subsampledDepths[edgeCloud.edges[ret_index[0]].x + edgeCloud.edges[ret_index[0]].y * subsampledWidth];
						float zDiff = math<float>::abs(axisZ - subsampledAverageDepth);


						float lerpPercent = (math<float>::clamp(lmap<float>(math<float>::sqrt(out_dist_sqr[0]), 0, 100, 0, 1.5), 0, 1.5));
						float s = math<float>::sin(lerpPercent);
						float additionalZ = silhouetteZOffset * (s * s);

						additionalZ = zDiff;  /* UNCOMMENT THIS LINE TO GET FLAT BACKS, MULTIPLY zDiff by 2 TO GET MIRROR */

						*backPositions++ = vec3(worldPos.x, worldPos.y, subsampledAverageDepth + (additionalZ * 1));

						*fillerScales++ = 2;

						backNumberToDraw++;
					}

					//--------
				}
			}
		}
	}


	mInstancePositionVbo->unmap();
	mInstanceScaleVbo->unmap();
	mInstanceDyingDataVbo->unmap();
	mInstanceDyingTotalLifetimesVbo->unmap();
	mInstanceBackPositionVbo->unmap();
	mInstanceFillerScaleVbo->unmap();
}

//Given mapped buffers, set up all the particles for drawing.
void HP_WaitingRTApp::SetupParticles(vec3 *positions, vec3 *backPositions, float *scales, float *fillerScales, vec3 *deathPositions, float *totalLivingTimes, float elapsed)
{
	std::vector<ParticleInstance> dyingParticlesToAdd = std::vector<ParticleInstance>();


	uint16_t* originalDepthBuffer = mDepthBuffer.getDataStore().get();

	/* CANNY EDGE BUILDING */
	EdgeCloud<int> edgeCloud; //pass as reference to fill the "edges" array
	Mat cannyEdgePixels; //pass as reference to set the mat of canny edge pixels
	GetKDEdgeIndex(originalDepthBuffer, width, height, lowThreshold, highThreshold, cannyEdgePixels, edgeCloud);

	my_kd_tree_t index(2, edgeCloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();
	bool kdIndexReady = (index.size() > 0);
	/* END OF CANNY EDGE BUILDING */


	for (int potX = 0; potX < width; ++potX)
	{
		for (int potY = 0; potY < height; ++potY)
		{
			float v = originalDepthBuffer[potX + (width * potY)];
			bool cannyEdgePixel = (cannyEdgePixels.data[potX + (width * potY)] != 0);
			//	v = lmap<float>(v, 1, 255, 400, 5000); // (float)depth[potX + (width * potY)];//depth.data[potX + (width * potY)];

			if (v != 0 && mCam.getFarClip() > v)
			{
				vec3 worldPos = mDSAPI->getDepthSpacePoint(vec3((float)potX, (float)potY, v));

				//*positions++ = vec3(potX / width - 0.5f, potY / height - 0.5f, 0);// depth[potX + (potY * width)]
				*positions++ = worldPos;

				if (cannyEdgePixel)
					*scales++ = 1.5f;
				else
					*scales++ = 0.5f;//0.3f;

				numberToDraw++;

				mPreviousValidDepthCoord[potX + (potY * width)] = worldPos;
				mPreviouslyHadDepth[potX + (potY * width)] = true;

				if (kdIndexReady)
				{
					const float mirrorDepthLerpPastPercentage = 0.8; //percentage of Z to add PAST the axis depth, based on how far away this depth pixel was from it's axis. (0% to 100%)
					const float silhouetteZOffset = 150; //additional z offset past the edge this pixel is bound to, so pixels near the edge still add at least *some* z.

					const int query_pt[2] = { potX, potY };
					const size_t num_results = 1;
					std::vector<size_t>   ret_index(num_results);
					std::vector<int> out_dist_sqr(num_results);
					index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
					float axisZ = originalDepthBuffer[edgeCloud.edges[ret_index[0]].x + edgeCloud.edges[ret_index[0]].y * width];
					float zDiff = math<float>::abs(axisZ - v);
					//float additionalZ = zDiff + (zDiff * mirrorDepthLerpPastPercentage); //original depth + diff on axis + percentage of the original depth difference

					//float additionalZ = silhouetteZOffset + ((zDiff + (zDiff * mirrorDepthLerpPastPercentage)) * (math<float>::clamp(lmap<float>(out_dist_sqr[0], 0, 1000, 0.3, 1), 0.3, 1)));
					/* ^ ^ ^																^ ^ ^ ^ <- this lmap&clamp simply make depth pixels close to their edge use less of the z addition (smoothing out where it meets the silhouette)
					Variables at your disposal:
					- z-value of nearest edge pixel
					- distance of this pixel from nearest edge pixel
					With this you can:
					- lerp in how much of the additional depth to use based on distance from the edge
					- check if the edge for this pixel is actually behind. If it isn't, you could skip the math::abs() and simply not add the instance.

					Next steps???:
					- somehow normalize the different edge z's closer to each other, i.e. if sorted and within a threshold to the ones next to it, kind of lerp them toward each other...
					- without doing this, we have the shattered glass back effect, which may be fine.
					NOTE: ANOTHER WAY TO SOLVE THIS: simply use low = 4 and high = 5 for the canny threshold to fix the back, and don't render the edges anymore.
					*/
					float lerpPercent = (math<float>::clamp(lmap<float>(math<float>::sqrt(out_dist_sqr[0]), 0, 100, 0, 1.5), 0, 1.5));
					float s = math<float>::sin(lerpPercent);
					float additionalZ = silhouetteZOffset * (s * s);/* + (zDiff * mirrorDepthLerpPastPercentage) * lerpPercent*/;

					additionalZ = zDiff;

					*backPositions++ = vec3(worldPos.x, worldPos.y, v + (additionalZ * 1));

					*fillerScales++ = 2;

					backNumberToDraw++;
				}
			}
			else //no depth here now, if there was depth, start dying.
			{
				if (cannyEdgePixel &&
					mDyingParticleManager.CurrentTotalParticles + dyingParticlesToAdd.size() < MaxNumberDyingParticleBufferSize &&
					mPreviouslyHadDepth[potX + (potY * width)]) //there was depth and it just died
				{
					dyingParticlesToAdd.push_back(ParticleInstance(mPreviousValidDepthCoord[potX + (potY * width)], vec2(potX, potY), elapsed));
				}

				mPreviouslyHadDepth[potX + (potY * width)] = false;
			}
		}
	}

	if (dyingParticlesToAdd.size() > 0)
		mDyingParticleManager.AddDyingParticleBatch(dyingParticlesToAdd, elapsed);

	//setup the gpu memory positions
	mDyingParticleManager.SetupBatchDraw(deathPositions, totalLivingTimes, elapsed);
}

void HP_WaitingRTApp::draw()
{
	gl::clear(Color::black());


	gl::setMatrices(mMayaCam.getCamera());

	// glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));

	//glm::mat4 blah = glm::rotate((float)(abs(cos(getElapsedSeconds() / 8)) * 180), glm::vec3(0, 0, 1));
	glm::mat4 blah;

	mGlsl->bind();
	mGlsl->uniform("rotationMatrix", blah);

	mGlslDyingCubes->bind();
	mGlslDyingCubes->uniform("rotationMatrix", blah);

	mGlslBackCubes->bind();
	mGlslBackCubes->uniform("rotationMatrix", blah);

	mGlslDyingCubes->bind();
	mGlslDyingCubes->uniform("MaxLifetime", DeathLifetimeInSeconds);

	mBatchBackubes->drawInstanced(backNumberToDraw);
	//mBatchDyingCubes->drawInstanced(mDyingParticleManager.CurrentTotalParticles);
	mBatch->drawInstanced(numberToDraw);

	//console() << numberToDraw << endl;
}

void HP_WaitingRTApp::keyDown(KeyEvent event)
{
	if (event.getChar() == 'q'){
		lowThreshold -= 1;
	}
	else if (event.getChar() == '1'){
		lowThreshold += 1;
	}
	else if (event.getChar() == '2'){
		highThreshold += 1;
	}
	else if (event.getChar() == 'w'){
		highThreshold -= 1;
	}
	else if (event.getChar() == '3'){
		do {
			mDepthSubsampleSize = math<int>::min(8, mDepthSubsampleSize + 1); //MAX OUT AT 12x12 window size
		} while (width % mDepthSubsampleSize != 0 || height % mDepthSubsampleSize != 0);
	}
	else if (event.getChar() == 'e'){
		do {
			mDepthSubsampleSize = math<int>::max(1, mDepthSubsampleSize - 1);
		} while (width % mDepthSubsampleSize != 0 || height % mDepthSubsampleSize != 0);
	}

}

void HP_WaitingRTApp::mouseDown(MouseEvent event)
{
	mMayaCam.mouseDown(event.getPos());
}

void HP_WaitingRTApp::mouseDrag(MouseEvent event)
{
	vec2 pos = vec2(event.getPos());
	if (event.isLeftDown() == false)
	{
		pos *= 20;
	}
	mMayaCam.mouseDrag(pos, event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

std::vector<uint16_t> HP_WaitingRTApp::GetSuperPixelAverageDepths(uint16_t* depthMap, int width, int height, int subsampleWindowSize)
{
	const int orthant_width = subsampleWindowSize;
	const int orthant_height = subsampleWindowSize;

	std::vector<uint16_t> averages(width / subsampleWindowSize * height / subsampleWindowSize);

	for (int row = 0; row < height; row += orthant_height)
	{
		for (int col = 0; col < width; col += orthant_width)
		{
			int topLeftX = col - col % orthant_width;
			int topLeftY = row - row % orthant_height;

			//int total = 0, totalSq = 0;

			//int numDepthPointsUsed = 0;
			std::list<uint16_t> windowPixels;
			uint32_t sum = 0;
			uint16_t lowestDepth;
			uint16_t highestDepth;
			for (int y = topLeftY; y < topLeftY + orthant_height; y++)
			{ //every row in orthant
				for (int x = topLeftX; x < topLeftX + orthant_width; x++)
				{ //every column in row

					/*
					DO WHATEVER YOU WANT HERE FOR EVERY PIXEL IN THE ORTHANT --------
					For the sake of example, we're just adding every pixel to the "averages" array, and then at the
					end of the x/y loop (when the orthant is fully looped thru), we divide the sum by the total # of pixels.

					You could even create an array analogous to the averages array, where you assign each pixel in the image
					a "window" to perform operations on if you so desire. i.e. pixel 1,1 (from the top left) could have a 3x3
					window of:
					0,1,2,...
					0,1,2,... (all multiplied by 1 * image width, since it's a 1D depth array)
					0,1,2,... (all multiplied by 2 * image width, since it's a 1D depth array)

					by making the orthant sizes 3x3 (i.e. NUM_COLUMNS = 160; NUM_ROWS = 120;)
					*/
					if (depthMap[x + (y * width)] != 0 && depthMap[x + (y * width)] < mCam.getFarClip())
					{
						//averages[((topLeftY / orthant_height) * (width / orthant_width)) + (topLeftX / orthant_width)] += depthMap[x + (y * width)];
						//numDepthPointsUsed++;
						uint16_t d = depthMap[x + (y * width)];
						sum += d;
						windowPixels.push_back(d);

						if (windowPixels.size() == 1)
						{ //initialize lowest/highest
							lowestDepth = d;
							highestDepth = d;
						}
						else
						{ //keep track of lowest and highest
							if (lowestDepth > d)
								lowestDepth = d;

							if (highestDepth < d)
								highestDepth = d;
						}

						//total += depthMap[x + (y * width)];
						//totalSq += depthMap[x + (y * width)] * depthMap[x + (y * width)];
					}
				}
			}

			if (windowPixels.size() > 0)
			{
				if (highestDepth - lowestDepth > VariableCap)
				{
					averages[((topLeftY / orthant_height) * (width / orthant_width)) + (topLeftX / orthant_width)] = 0;
				}
				else
				{
					averages[((topLeftY / orthant_height) * (width / orthant_width)) + (topLeftX / orthant_width)] = (uint16_t)(sum / windowPixels.size());// numDepthPointsUsed;
				}
				//float varianceOfDepth = totalSq / numDepthPointsUsed - (total / numDepthPointsUsed) * (total / numDepthPointsUsed);
				//if (math<float>::sqrt(varianceOfDepth) > 50)
				//{
				//	averages[((topLeftY / orthant_height) * (width / orthant_width)) + (topLeftX / orthant_width)] = 0;
				//}
			}
		}
	}

	return averages;
}

void HP_WaitingRTApp::GetKDEdgeIndex(uint16_t* depthFrame, int depthWidth, int depthHeight, int lowThresh, int highThresh, Mat &cannyEdgePixels, EdgeCloud<int> &edgeCloud)
{
	Mat depth = Mat(Size(depthWidth, depthHeight), CV_16UC1, depthFrame);//.getIter();
	Mat depthf(Size(depthWidth, depthHeight), CV_8UC1);
	depth.convertTo(depthf, CV_8UC1, 255.0 / 4096.0);

	//uint16_t* depth = mDepthBuffer.getDataStore().get();
	//uint8_t* rgb = mRgbBuffer.getDataStore().get();
	InputArray input = InputArray(depthf);
	OutputArray output = OutputArray(depthf);
	output.create(Size(depthWidth, depthHeight), CV_8UC1);
	cv::Canny(input, output, lowThresh, highThresh);


	cannyEdgePixels = output.getMatRef();

	//kd-tree of canny edges to proper depth values for all the valid depth pixels later

	std::vector<ivec2> validEdges;
	for (int y = 0; y < depthHeight; y++)
	{
		for (int x = 0; x < depthWidth; x++)
		{
			uint16_t originalDepth = depthFrame[x + (y * depthWidth)];

			if (output.getMat().data[x + y * depthWidth] != 0 && //isn't zero (therefore 255, aka an edge)
				originalDepth != 0 && //is valid depth point
				originalDepth < mCam.getFarClip()) //isn't clipped by clip plane
			{
				////valid edge & depth, now check if we're being redundant

				//int rem = originalDepth % HogBinStep;
				////round up to farther bin
				//originalDepth += HogBinStep - rem;


				validEdges.push_back(ivec2(x, y));
			}
		}
	}


	if (validEdges.size() <= DesiredEdgeNumCap)
	{
		edgeCloud.edges = validEdges;
	}
	else //decimate to desired number of edges
	{
		for (int i = 0; i < validEdges.size(); i += validEdges.size() / DesiredEdgeNumCap)
		{
			edgeCloud.edges.push_back(validEdges[i]);
		}
	}

	/*
	Mat depth = Mat(Size(width, height), CV_16U, mDepthBuffer.getDataStore().get());//.getIter();

	//uint16_t* depth = mDepthBuffer.getDataStore().get();
	//uint8_t* rgb = mRgbBuffer.getDataStore().get();
	InputArray input = InputArray(depth);
	OutputArray output = OutputArray(depth);
	output.create(Size(width, height), CV_16U);
	cv::Canny(input, output, 1, 3);
	*/

	/* //LEO DEPTH FILTER (not working, because stuff is setup wrong?)
	uint16_t* depth = mDepthBuffer.getDataStore().get();
	uint8_t* rgb = mRgbBuffer.getDataStore().get();
	RGBD remapper(mDSAPI->getDSAPI().get());
	remapper.getRGBAlignedZ(mDSAPI->getDSAPI()->getZImage(), zAligned);

	depthf_Filter(mDepthFilter, zAligned, rgb, depth);

	//depthf_Filter(mDepthFilter, depth, rgb, remapZ);	// comment this block out
	//depth = remapZ;										//
	*/
}


#if defined( CINDER_MSW ) && ! defined( CINDER_GL_ANGLE )
auto options = RendererGl::Options().version(3, 3); // instancing functions are technically only in GL 3.3
#else
auto options = RendererGl::Options(); // implemented as extensions in Mac OS 10.7+
#endif
CINDER_APP_NATIVE(HP_WaitingRTApp, RendererGl(options))