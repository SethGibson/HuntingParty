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

enum BackParticleSetting
{
	ShatteredGlass,
	AveragedShatteredGlass,
	Mountainous
};

const static int MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE = 75000; //THERE'S A LIMIT TO PARTICLES DEPENDING ON THE GPU, I DON'T KNOW WHY -- somwhere aroud 75k is when it starts glitching
const static float DEATH_LIFETIME_IN_SECONDS = 3.5; //total time until dying particle instances expire
const static int SECONDS_BETWEEN_NEW_SPAWNS = 0.5;
const static int DEPTH_SUBSAMPLE_WINDOW_SIZE = 8; //X by X window size to subsample the depth buffer for spawning instanced meshes. 1 = full depth buffer, 2 = 2x2 so half the number of instances.
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

	bool AddDyingParticleBatch(std::vector<ParticleInstance> particles, float currentTime)
	{
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

	void SetupParticles(vec3 *positions, vec3 *backPositions, float *scales, float *dyingScales, float *fillerScales, vec3 *deathPositions, float *totalLivingTimes, vec3 *trianglePositions, uint16_t *triangleIndices, vec4 *triangleColors, float elapsed, BackParticleSetting backSetting);
	std::vector<uint16_t> GetSuperPixelAverageDepths(uint16_t* depthMap, int width, int height, int subsampleWindowSize);

	void SetupBackParticlesDraw(std::vector<float> backParticlePositions, std::vector<float> backParticleScales, vec3 *backPositionsVBO, float *backScalesVBO, BackParticleSetting particlesSetting, int particleArrayWidth, int particleArrayHeight);

	void GetKDEdgeIndex(uint16_t* depthFrame, int depthWidth, int depthHeight, int lowThresh, int highThresh, Mat &cannyEdgePixels, EdgeCloud<int> &edgeCloud);
	std::vector<vec4> GetRGBSubsampledColors(std::vector<uint16_t> subsampledDepths, int subsampledWidth, int subsampledHeight);
	void SetupTriangles(std::vector<uint16_t> &subsampledDepths, std::vector<vec4> &subsampledColors, int subsampleWidth, int subsampledHeight, vec3* &trianglePositions, vec4* &triangleColors, uint16_t* &triangleIndices, int x, int y);
	void GLDrawTriangles();

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
	BackParticleSetting currentParticleSetting = BackParticleSetting::ShatteredGlass;
	bool DrawBackParticles = true;
	bool DrawDyingParticles = true;
	bool DrawFrontParticles = true;
	bool DrawTriangles = false;
	bool DrawBatchTriangles = false;
	bool MaskFrontParticles = false;
	bool ModuloTriangleDirections = true;
};


void HP_WaitingRTApp::setup()
{
	getWindow()->setSize(1280, 720);
	mDepthSubsampleSize = DEPTH_SUBSAMPLE_WINDOW_SIZE;

	//out = std::ofstream("output.txt");


	mDSAPI = CinderDSAPI::create();
	mDSAPI->init();
	mDSAPI->initDepth(CinderDS::FrameSize::DEPTHSD, 60);
	mDSAPI->initRgb(CinderDS::FrameSize::RGBVGA, 60);
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

	mDSAPI->getDSAPI()->getCalibIntrinsicsZ(zIntrin);
	mDSAPI->getDSThird()->getCalibIntrinsicsRectThird(thirdIntrin);
	mDSAPI->getDSThird()->getCalibExtrinsicsZToRectThird(zToThirdTrans);


	mCam.setPerspective(cFOVs.y, getWindowAspectRatio(), 100, 2500);
	mCam.lookAt(vec3(0, 0, 0), vec3(0, 0, 1000), vec3(0, -1, 0));
	mCam.setCenterOfInterestPoint(vec3(0, 0, 750));
	mMayaCam = MayaCamUI(mCam);

	mTexture = gl::Texture::create(loadImage(loadAsset("blank_texture.png")), gl::Texture::Format().mipmap());
	mLogoTexture = gl::Texture::create(loadImage(loadAsset("linkin_park_logo_flipped.png")));
#if ! defined( CINDER_GL_ES )
	mGlsl = gl::GlslProg::create(loadAsset("shader.vert"), loadAsset("shader.frag"));
	mGlslDyingCubes = gl::GlslProg::create(loadAsset("shaderDying.vert"), loadAsset("shaderDying.frag"));
	mGlslBackCubes = gl::GlslProg::create(loadAsset("shaderBack.vert"), loadAsset("shaderBack.frag"));
	mGlslTriangles = gl::GlslProg::create(loadAsset("shaderTriangles.vert"), loadAsset("shaderTriangles.frag"));
#else
	mGlsl = gl::GlslProg::create(loadAsset("shader_es2.vert"), loadAsset("shader_es2.frag"));
#endif

	gl::VboMeshRef mesh = gl::VboMesh::create(geom::Sphere().subdivisions(4));
	gl::VboMeshRef meshDying = gl::VboMesh::create(geom::Cone().subdivisionsAxis(4).apex(1).base(5).height(4));
	gl::VboMeshRef meshBack = gl::VboMesh::create(geom::Cube().size(vec3(2)));

	// create an array of initial per-instance positions laid out in a 2D grid
	std::vector<vec3> positions;
	std::vector<vec3> backPositions;
	std::vector<float> cubeScales;
	std::vector<float> fillerScales;
	std::vector<vec3> deathPositions = std::vector<vec3>(MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE);
	std::vector<float> deathTotalLifetimes = std::vector<float>(MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE);
	std::vector<float> dyingScales = std::vector<float>(MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE);
	std::vector<vec3> trianglePositions;
	std::vector<vec4> triangleVertColors;
	std::vector<uint16_t> triangleIndices;
	for (size_t potX = 0; potX < width; ++potX) {
		for (size_t potY = 0; potY < height; ++potY) {
			float instanceX = potX / (float)width - 0.5f;
			float instanceY = potY / (float)height - 0.5f;
			vec3 pos = vec3(instanceX * vec3(1, 0, 0) + instanceY * vec3(0, 0, 1));
			positions.push_back(pos);
			backPositions.push_back(pos);

			trianglePositions.push_back(pos);
			trianglePositions.push_back(pos);
			triangleIndices.resize(triangleIndices.size() + 6, 0); //+6 because 3 indices per triangle, 2 triangles per pixel.
			triangleVertColors.push_back(vec4(1, 1, 1, 1)); //the RGB color for upper left triangle
			triangleVertColors.push_back(vec4(1, 1, 1, 1)); //the RGB color for bottom right triangle

			cubeScales.push_back(randFloat(0.2, 5));
			fillerScales.push_back(randFloat(0.2, 5));

			mPreviouslyHadDepth.push_back(false);
			mCurrentlyHasDyingInstance.push_back(false);
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
	mInstanceDyingDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER, (MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE)* sizeof(vec3), deathPositions.data(), GL_DYNAMIC_DRAW);
	mInstanceDyingScaleVbo = gl::Vbo::create(GL_ARRAY_BUFFER, (MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE)* sizeof(float), dyingScales.data(), GL_DYNAMIC_DRAW);
	mInstanceDyingTotalLifetimesVbo = gl::Vbo::create(GL_ARRAY_BUFFER, (MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE)* sizeof(float), deathTotalLifetimes.data(), GL_DYNAMIC_DRAW);
	mInstanceBackPositionVbo = gl::Vbo::create(GL_ARRAY_BUFFER, backPositions.size() * sizeof(vec3), backPositions.data(), GL_DYNAMIC_DRAW);
	mInstanceFillerScaleVbo = gl::Vbo::create(GL_ARRAY_BUFFER, fillerScales.size() * sizeof(float), fillerScales.data(), GL_DYNAMIC_DRAW);
	mTrianglePositionsVbo = gl::Vbo::create(GL_ARRAY_BUFFER, trianglePositions.size() * sizeof(vec3), trianglePositions.data(), GL_DYNAMIC_DRAW);
	mTriangleColorsVbo = gl::Vbo::create(GL_ARRAY_BUFFER, triangleVertColors.size() * sizeof(vec4), triangleVertColors.data(), GL_DYNAMIC_DRAW);

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

	geom::BufferLayout instanceDyingScaleLayout;
	instanceDyingScaleLayout.append(geom::Attrib::CUSTOM_1, 1, 0, 0, 1 /* per instance */);

	geom::BufferLayout instanceDyingDataLayout;
	instanceDyingDataLayout.append(geom::Attrib::CUSTOM_2, 3, 0, 0, 1 /* per instance */);

	geom::BufferLayout instanceDeathTotalLifetimesLayout;
	instanceDeathTotalLifetimesLayout.append(geom::Attrib::CUSTOM_3, 1, 0, 0, 1 /* per instance */);

	meshDying->appendVbo(instanceDyingScaleLayout, mInstanceDyingScaleVbo);
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




		//3 indices for each triangle, 2 triangles per position
	mTriangleIndicesVbo = gl::Vbo::create(GL_ELEMENT_ARRAY_BUFFER, trianglePositions.size() * 3 * sizeof(uint16_t), triangleIndices.data(), GL_DYNAMIC_DRAW);

	geom::BufferLayout trianglePositionsLayout;
	trianglePositionsLayout.append(geom::Attrib::POSITION, 3, 0, 0, 0);

	geom::BufferLayout triangleColorsLayout;
	triangleColorsLayout.append(geom::Attrib::COLOR, 4, 0, 0, 0);

	//mTriangleColorsVbo ....? use vbo of color in layout???
	
	mTrianglesVboMesh = gl::VboMesh::create(
		(uint32_t)trianglePositions.size(), GL_TRIANGLES, { { trianglePositionsLayout, mTrianglePositionsVbo }, { triangleColorsLayout, mTriangleColorsVbo } }, (uint32_t)trianglePositions.size() * 3, GL_UNSIGNED_SHORT, mTriangleIndicesVbo);
	mBatchTriangles = gl::Batch::create(mTrianglesVboMesh, mGlslTriangles);


	gl::enableDepthWrite();
	gl::enableDepthRead();
	gl::enableAlphaBlending();

	mTexture->bind(0);
	mLogoTexture->bind(1);

	previousElapsedTime = getElapsedSeconds();
}

void HP_WaitingRTApp::cleanup()
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
//first is the depth value, second is the index (only set if depth value is valid)
template <typename T>
std::vector<pair<bool, uint16_t>> getSquareFromTopLeft(std::vector<T> depthData, int x, int y, int w, int h, float farClipZ)
{
	std::vector<pair<bool, uint16_t>> window = { { false, 0 }, { false, 0 }, { false, 0 }, { false, 0 } };

	window[0].first = depthData[x + (y * w)] != 0 && depthData[x + (y * w)] < farClipZ; //set to the top left pixel
	window[0].second = x + (y * w);

	if (x == w - 1 && y == h - 1) //bottom right
	{
		window[1].first = false;
		window[2].first = false;
		window[3].first = false;
	}
	else if (x == w - 1 || y == h - 1) //bottom right)
	{
		if (x == w - 1) //right
		{
			window[1].first = false;
			window[2].first = depthData[x + ((y + 1) * w)] != 0 && depthData[x + ((y + 1) * w)] < farClipZ;
			window[2].second = x + ((y + 1) * w);
			window[3].first = false;
		}

		if (y == h - 1) //bottom
		{
			window[1].first = depthData[(x + 1) + (y * w)] != 0 && depthData[(x + 1) + (y * w)] < farClipZ;
			window[1].second = (x + 1) + (y * w);
			window[2].first = false;
			window[3].first = false;
		}
	}
	else
	{
		window[1].first = depthData[(x + 1) + (y * w)] != 0 && depthData[(x + 1) + (y * w)] < farClipZ;
		window[1].second = (x + 1) + (y * w);
		window[2].first = depthData[x + ((y + 1) * w)] != 0 && depthData[x + ((y + 1) * w)] < farClipZ;
		window[2].second = x + ((y + 1) * w);
		window[3].first = depthData[(x + 1) + ((y + 1) * w)] != 0 && depthData[(x + 1) + ((y + 1) * w)] < farClipZ;
		window[3].second = (x + 1) + ((y + 1) * w);
	}


	return window;
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
	float *dyingScales = (float*)mInstanceDyingScaleVbo->mapWriteOnly(true);
	float *fillerScales = (float*)mInstanceFillerScaleVbo->mapWriteOnly(true);
	vec3 *deathPositions = (vec3*)mInstanceDyingDataVbo->mapWriteOnly(true);
	float *totalLivingTimes = (float*)mInstanceDyingTotalLifetimesVbo->mapWriteOnly(true);

	vec3 *trianglePositions = (vec3*)mTrianglePositionsVbo->mapWriteOnly(true);
	uint16_t *trangleIndices = (uint16_t*)mTriangleIndicesVbo->mapWriteOnly(true);
	vec4 *triangleColors = (vec4*)mTriangleColorsVbo->mapWriteOnly(true);

	SetupParticles(positions, backPositions, scales, dyingScales, fillerScales, deathPositions, totalLivingTimes, trianglePositions, trangleIndices, triangleColors, elapsed, currentParticleSetting);


	mInstancePositionVbo->unmap();
	mInstanceScaleVbo->unmap();
	mInstanceDyingDataVbo->unmap();
	mInstanceDyingTotalLifetimesVbo->unmap();
	mInstanceBackPositionVbo->unmap();
	mInstanceDyingScaleVbo->unmap();
	mInstanceFillerScaleVbo->unmap();

	mTrianglePositionsVbo->unmap();
	mTriangleIndicesVbo->unmap();
	mTriangleColorsVbo->unmap();
}

//Given mapped buffers, set up all the particles for drawing.
void HP_WaitingRTApp::SetupParticles(vec3 *positions, vec3 *backPositions, float *scales, float *dyingScales, float *fillerScales, vec3 *deathPositions, float *totalLivingTimes, vec3 *trianglePositions, uint16_t *triangleIndices, vec4 *triangleColors, float elapsed, BackParticleSetting backSetting)
{
	numberToDraw = 0;
	backNumberToDraw = 0;
	numberTrianglesToDraw = 0;

	std::vector<ParticleInstance> dyingParticlesToAdd = std::vector<ParticleInstance>();
	std::unordered_map<int, std::vector<ParticleInstance>> dyingGlassBins = std::unordered_map<int, std::vector<ParticleInstance>>(); //Z-value with all x/y integer coords (unmapped) of dying particles associated.

	std::vector<float> backParticleDepthsToAdd = std::vector<float>();
	std::vector<float> backParticleScalesToAdd = std::vector<float>();


	uint16_t* originalDepthBuffer = mDepthBuffer.getDataStore().get();

	//subsampled depth particles:::
	std::vector<uint16_t> subsampledDepths = GetSuperPixelAverageDepths(mDepthBuffer.getDataStore().get(), width, height, mDepthSubsampleSize);
	int subsampledWidth = width / mDepthSubsampleSize;
	int subsampledHeight = height / mDepthSubsampleSize;

	/* CANNY EDGE BUILDING */
	EdgeCloud<int> edgeCloud; //pass as reference to fill the "edges" array
	Mat cannyEdgePixels; //pass as reference to set the mat of canny edge pixels
	if (mDepthSubsampleSize == 1)
		GetKDEdgeIndex(subsampledDepths.data(), subsampledWidth, subsampledHeight, lowThreshold, highThreshold, cannyEdgePixels, edgeCloud);
	else
		GetKDEdgeIndex(subsampledDepths.data(), subsampledWidth, subsampledHeight, 9, 11, cannyEdgePixels, edgeCloud);

	my_kd_tree_t index(2, edgeCloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();
	bool kdIndexReady = (index.size() > 0);
	/* END OF CANNY EDGE BUILDING */

	std::vector<vec4> subsampledColors = GetRGBSubsampledColors(subsampledDepths, subsampledWidth, subsampledHeight);

	uint32_t currentTotalDepth = 0;

	for (int y = 0; y < subsampledHeight; y++)
	{
		for (int x = 0; x < subsampledWidth; x++)
		{			
			SetupTriangles(subsampledDepths, subsampledColors, subsampledWidth, subsampledHeight, trianglePositions, triangleColors, triangleIndices, x, y);
			

			float subsampledAverageDepth = subsampledDepths[(y * subsampledWidth) + x];
			bool cannyEdgePixel = (cannyEdgePixels.data[x + (subsampledWidth * y)] != 0);

			if (subsampledAverageDepth != 0 && subsampledAverageDepth < mCam.getFarClip())
			{
				currentTotalDepth += subsampledAverageDepth;

				vec3 subsampledDepthPoint = vec3(
					((float)x * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
					((float)y * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
					subsampledAverageDepth);

				vec3 worldPos = mDSAPI->getZCameraSpacePoint(subsampledDepthPoint);

				*positions++ = worldPos;

				if (cannyEdgePixel)
					*scales++ = 1.5f;
				else
					*scales++ = 0.5f;//0.3f;

				numberToDraw++;

				mPreviousValidDepthCoord[x + (y * subsampledWidth)] = worldPos;
				mPreviouslyHadDepth[x + (y * subsampledWidth)] = true;

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
					float additionalZ;
					float zAddMultiplyFactor = 1;
					float finalZ;
					float spawnBackParticle = true;

					switch (backSetting)
					{
					case BackParticleSetting::ShatteredGlass:
						additionalZ = zDiff * 2;  /* MULTIPLY zDiff by 2 TO GET MIRROR */
						finalZ = subsampledAverageDepth + (additionalZ * zAddMultiplyFactor); //calculate this early

						finalZ = axisZ;

						if (mCurrentlyHasDyingInstance[x + (y * subsampledWidth)] == false)
						{
							//store the subsampledDepthMap INDEX of the z position being using (i.e. mapping an initial position to which edge pixel it used)
							dyingGlassBins[edgeCloud.edges[ret_index[0]].x + edgeCloud.edges[ret_index[0]].y * subsampledWidth].push_back(
								ParticleInstance(vec3(worldPos.x, worldPos.y, finalZ), vec2(x, y), elapsed));
							spawnBackParticle = false;
						}

						break;
					case BackParticleSetting::AveragedShatteredGlass:
						additionalZ = zDiff;  /* MULTIPLY zDiff by 2 TO GET MIRROR */
						finalZ = subsampledAverageDepth + (additionalZ * zAddMultiplyFactor);
						break;
					case BackParticleSetting::Mountainous:
						float lerpPercent;
						lerpPercent = (math<float>::clamp(lmap<float>(math<float>::sqrt(out_dist_sqr[0]), 0, 100, 0, 1.5), 0, 1.5));
						float s = math<float>::sin(lerpPercent);
						additionalZ = silhouetteZOffset * (s * s);
						finalZ = subsampledAverageDepth + (additionalZ * zAddMultiplyFactor);
						break;
					}

					if (spawnBackParticle)
					{
						float backScale = 0.75;
						backParticleDepthsToAdd.push_back(finalZ);
						backParticleScalesToAdd.push_back(backScale);
					}
					else
					{
						//no depth, but still add something to keep the width/height's proper for the back particle array:
						backParticleDepthsToAdd.push_back(0);
						backParticleScalesToAdd.push_back(0);
					}
				}
			}
			else //no depth here now, if there was depth, start dying.
			{
				//no depth, but still add something to keep the width/height's proper for the back particle array:
				backParticleDepthsToAdd.push_back(0);
				backParticleScalesToAdd.push_back(0);

				if (dyingGlassBins.empty() &&
					currentParticleSetting != BackParticleSetting::ShatteredGlass && //If this isn't empty, then it should trump normal dying particles.
					cannyEdgePixel &&
					mDyingParticleManager.CurrentTotalParticles + dyingParticlesToAdd.size() < MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE &&
					mPreviouslyHadDepth[x + (y * subsampledWidth)]) //there was depth and it just died
				{
					dyingParticlesToAdd.push_back(ParticleInstance(mPreviousValidDepthCoord[x + (y * subsampledWidth)], vec2(x, y), elapsed));
				}

				mPreviouslyHadDepth[x + (y * subsampledWidth)] = false;
			}
		}
	}
	
	//Setup the back particles for drawing, based on the info we passed into the pair array.
	SetupBackParticlesDraw(backParticleDepthsToAdd, backParticleScalesToAdd, backPositions, fillerScales, backSetting, subsampledWidth, subsampledHeight);

	if (dyingParticlesToAdd.size() > 0)
	{
		mDyingParticleManager.AddDyingParticleBatch(dyingParticlesToAdd, elapsed);
	}
	if (dyingGlassBins.size() > 0)
	{
		for (auto it = dyingGlassBins.begin(); it != dyingGlassBins.end(); ++it)
		{
			if (it->second.size() > (8.0f / max(1.0f, min(3.0f, lmap<float>(it->second[0].InitialPosition.z, 1.0f, 3.0f, 1000, 3000)))) * 8 / mDepthSubsampleSize * 8 / mDepthSubsampleSize && randFloat(0.0, 1.0) < 0.1) //&& 1/10 the time.
				mDyingParticleManager.AddDyingParticleBatch(it->second, elapsed);
		}
	}

	//setup the gpu memory positions
	mDyingParticleManager.SetupBatchDraw(mCurrentlyHasDyingInstance, deathPositions, totalLivingTimes, dyingScales, elapsed, 1.0f, subsampledWidth, subsampledHeight);


	ForwardForceMultiplier = (currentTotalDepth / (subsampledWidth * subsampledHeight)) - (previousTotalDepth / (subsampledWidth * subsampledHeight));
	previousTotalDepth = currentTotalDepth;
}

void HP_WaitingRTApp::draw()
{
	gl::clear(Color(0.05f, 0.1f, 0.15f));
	gl::enableAdditiveBlending();
	GLDrawTriangles();

	gl::setMatrices(mMayaCam.getCamera());

	// glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));

	//glm::mat4 blah = glm::rotate((float)(abs(cos(getElapsedSeconds() / 8)) * 180), glm::vec3(0, 0, 1));
	glm::mat4 blah;

	if (DrawBackParticles)
	{
		mGlslBackCubes->bind();
		mGlslBackCubes->uniform("rotationMatrix", blah);

		mBatchBackubes->drawInstanced(backNumberToDraw);
	}
	if (DrawDyingParticles)
	{
		mGlslDyingCubes->bind();
		mGlslDyingCubes->uniform("rotationMatrix", blah);

		mGlslDyingCubes->bind();
		mGlslDyingCubes->uniform("MaxLifetime", DEATH_LIFETIME_IN_SECONDS);

		mGlslDyingCubes->bind();
		mGlslDyingCubes->uniform("Gravity", GravityForceMultiplier);

		mGlslDyingCubes->bind();
		mGlslDyingCubes->uniform("ForwardForceMult", 1); // ForwardForceMultiplier); /* COMMENTED OUT FOR NOW, NEEDS LERP! */

		mBatchDyingCubes->drawInstanced(mDyingParticleManager.CurrentTotalParticles);
	}
	if (DrawFrontParticles)
	{
		mGlsl->bind();
		mGlsl->uniform("rotationMatrix", blah);

		mGlsl->bind();
		mGlsl->uniform("UseMaskTexture", MaskFrontParticles);

		mGlsl->bind();

		glActiveTexture(GL_TEXTURE0);
		gl::ScopedTextureBind cubeTex(mTexture);
		mGlsl->uniform("uTex0", 0);

		glActiveTexture(GL_TEXTURE1);
		gl::ScopedTextureBind logoTex(mLogoTexture);
		mGlsl->uniform("uLogoTex", 1);

		glActiveTexture(GL_TEXTURE0);

		mBatch->drawInstanced(numberToDraw);
	}


	//if (DrawTriangles)
	//{
	//	mGlslTriangles->bind();
	//	mGlslTriangles->uniform("rotationMatrix", blah);

	//	//mBatchTriangles->drawInstanced(numberTrianglesToDraw);
	//	mBatchTriangles->draw();
	//}



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

			mPreviousValidDepthCoord.clear();
			mPreviousValidDepthCoord = std::vector<vec3>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			mPreviouslyHadDepth.clear();
			mPreviouslyHadDepth = std::vector<bool>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			std::fill(mPreviouslyHadDepth.begin(), mPreviouslyHadDepth.end(), false);
		} while (width % mDepthSubsampleSize != 0 || height % mDepthSubsampleSize != 0);
	}
	else if (event.getChar() == 'e'){
		do {
			mDepthSubsampleSize = math<int>::max(1, mDepthSubsampleSize - 1);

			mPreviousValidDepthCoord.clear();
			mPreviousValidDepthCoord = std::vector<vec3>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			mPreviouslyHadDepth.clear();
			mPreviouslyHadDepth = std::vector<bool>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			std::fill(mPreviouslyHadDepth.begin(), mPreviouslyHadDepth.end(), false);
		} while (width % mDepthSubsampleSize != 0 || height % mDepthSubsampleSize != 0);
	}
	else if (event.getChar() == 'z'){
		currentParticleSetting = BackParticleSetting::ShatteredGlass;
	}
	else if (event.getChar() == 'x'){
		currentParticleSetting = BackParticleSetting::AveragedShatteredGlass;
	}
	else if (event.getChar() == 'c'){
		currentParticleSetting = BackParticleSetting::Mountainous;
	}
	else if (event.getChar() == 'v'){
		GravityForceMultiplier = -GravityForceMultiplier;
	}
	else if (event.getChar() == 'a'){
		DrawBackParticles = !DrawBackParticles;
	}
	else if (event.getChar() == 's'){
		DrawDyingParticles = !DrawDyingParticles;
	}
	else if (event.getChar() == 'd'){
		DrawFrontParticles = !DrawFrontParticles;
	}
	else if (event.getChar() == 'f'){
		MaskFrontParticles = !MaskFrontParticles;
	}
	else if (event.getChar() == 'g'){
		ModuloTriangleDirections = !ModuloTriangleDirections;
	}
	else if (event.getChar() == 'h'){
		DrawTriangles = !DrawTriangles;
	}
	else if (event.getChar() == 'j'){
		DrawBatchTriangles = !DrawBatchTriangles;
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

	if (subsampleWindowSize == 1) //if we shouldn't be subsampling because the window size is 1x1, just pass the depthMap as a vector<uint16_t>
	{
		for (int i = 0; i < averages.size(); i++)
		{
			averages[i] = depthMap[i];
		}
		return averages;
	}

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

			if (windowPixels.size() == 0 || highestDepth - lowestDepth > VARIABLE_CAP)
			{
				averages[((topLeftY / orthant_height) * (width / orthant_width)) + (topLeftX / orthant_width)] = 0;
			}
			else
			{
				averages[((topLeftY / orthant_height) * (width / orthant_width)) + (topLeftX / orthant_width)] = (uint16_t)(sum / windowPixels.size());// numDepthPointsUsed;
			}
		}
	}

	return averages;
}

template <typename T>
std::vector<T> TransposeWindow(std::vector<T> window)
{
	//0, 1, 2      8, 7, 6
	//3, 4, 5  ->  5, 4, 3
	//6, 7, 8      2, 1, 0
	std::vector<T> new_window(window.size());

	new_window[0] = window[8];
	new_window[1] = window[7];
	new_window[2] = window[6];
	new_window[3] = window[5];
	new_window[4] = window[4];
	new_window[5] = window[3];
	new_window[6] = window[2];
	new_window[7] = window[1];
	new_window[8] = window[0];

	return new_window;
}

/// <summary>
/// Pulling a window of pixels out of an image. All arrays involved are 1-dimensional.
/// </summary>
/// <param name="pixels">pixel array, each byte should have a grayscale pixel color, so the size of the array should be (width * height)</param>
/// <param name="x">middle x position of window</param>
/// <param name="y">middle y position of window</param>
/// <param name="width">width of the image</param>
/// <param name="height">height of the image</param>
/// <param name="window_size">dimensions of the window</param>
/// <returns></returns>
template <typename T>
std::vector<T> getWindowFromMiddle(std::vector<T> &PreviouslyComputedWindow, std::vector<T> depthData, int x, int y, int w, int h, int window_size = 3)
{
	//window size is only 3x3 for now, this is a 9-byte array of depthData
	//0, 1, 2
	//3, 4, 5
	//6, 7, 8
	std::vector<T> window(window_size * window_size);

	//Corner Cases
	if (x == 0 && y == 0) //top left is middle
	{
		window[0] = depthData[x + 1 + ((y + 1) * w)]; //mirror top left with bottom right
		window[1] = depthData[x + ((y + 1) * w)]; //mirror top middle with bottom middle
		window[2] = depthData[x + 1 + ((y + 1) * w)]; //mirror top right with bottom right

		window[3] = depthData[x + 1 + (y * w)]; //mirror middle-left with middle-right
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x + 1 + (y * w)];

		window[6] = depthData[x + 1 + ((y + 1) * w)]; //mirror bottom-left with bottom-right
		window[7] = depthData[x + ((y + 1) * w)];
		window[8] = depthData[x + 1 + ((y + 1) * w)];
	}
	else if (x == w - 1 && y == 0) //top right is middle
	{
		window[0] = depthData[x - 1 + ((y + 1) * w)]; //0 <- 6
		window[1] = depthData[x + ((y + 1) * w)]; //1 <- 7
		window[2] = depthData[x - 1 + ((y + 1) * w)]; //2 <- 6

		window[3] = depthData[x - 1 + (y * w)];
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x - 1 + (y * w)]; //5 <- 3

		window[6] = depthData[x - 1 + ((y + 1) * w)];
		window[7] = depthData[x + ((y + 1) * w)];
		window[8] = depthData[x - 1 + ((y + 1) * w)]; //8 <- 6
	}
	else if (x == 0 && y == h - 1) //bottom left is middle
	{
		window[0] = depthData[x + 1 + ((y - 1) * w)]; //0 <- 2
		window[1] = depthData[x + ((y - 1) * w)];
		window[2] = depthData[x + 1 + ((y - 1) * w)];

		window[3] = depthData[x + 1 + (y * w)]; //3 <- 5
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x + 1 + (y * w)];

		window[6] = depthData[x + 1 + ((y - 1) * w)]; //6 <- 2
		window[7] = depthData[x + ((y - 1) * w)]; //7 <- 1
		window[8] = depthData[x + 1 + ((y - 1) * w)]; //8 <- 2
	}
	else if (x == w - 1 && y == h - 1) //bottom right is middle
	{
		window[0] = depthData[x - 1 + ((y - 1) * w)];
		window[1] = depthData[x + ((y - 1) * w)];
		window[2] = depthData[x - 1 + ((y - 1) * w)]; //2 <- 0

		window[3] = depthData[x - 1 + (y * w)];
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x - 1 + (y * w)]; //5 <- 3

		window[6] = depthData[x - 1 + ((y - 1) * w)]; //6 <- 0
		window[7] = depthData[x + ((y - 1) * w)]; //7 <- 1
		window[8] = depthData[x - 1 + ((y - 1) * w)]; //8 <- 0
	}

	//0, 1, 2
	//3, 4, 5
	//6, 7, 8

	//Side Cases
	else if (y == 0) // middle is in top row
	{
		window[0] = depthData[x - 1 + ((y + 1) * w)]; //0 <- 6
		window[1] = depthData[x + ((y + 1) * w)]; //1 <- 7
		window[2] = depthData[x + 1 + ((y + 1) * w)]; //2 <- 8

		window[3] = depthData[x - 1 + (y * w)];
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x + 1 + (y * w)];

		window[6] = depthData[x - 1 + ((y + 1) * w)];
		window[7] = depthData[x + ((y + 1) * w)];
		window[8] = depthData[x + 1 + ((y + 1) * w)];
	}
	else if (x == 0) // middle is in left-most column
	{
		window[0] = depthData[x + 1 + ((y - 1) * w)]; //0 <- 2
		window[1] = depthData[x + ((y - 1) * w)];
		window[2] = depthData[x + 1 + ((y - 1) * w)];

		window[3] = depthData[x + 1 + (y * w)]; //3 <- 5
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x + 1 + (y * w)];

		window[6] = depthData[x + 1 + ((y + 1) * w)]; //6 <- 8
		window[7] = depthData[x + ((y + 1) * w)];
		window[8] = depthData[x + 1 + ((y + 1) * w)];
	}
	else if (y == h - 1) //middle is in bottom row
	{
		window[0] = depthData[x - 1 + ((y - 1) * w)];
		window[1] = depthData[x + ((y - 1) * w)];
		window[2] = depthData[x + 1 + ((y - 1) * w)];

		window[3] = depthData[x - 1 + (y * w)];
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x + 1 + (y * w)];

		window[6] = depthData[x - 1 + ((y - 1) * w)]; //6 <- 0
		window[7] = depthData[x + ((y - 1) * w)]; //7 <- 1
		window[8] = depthData[x + 1 + ((y - 1) * w)]; //8 <- 2
	}
	else if (x == w - 1) //middle is in right-most column
	{
		window[0] = depthData[x - 1 + ((y - 1) * w)];
		window[1] = depthData[x + ((y - 1) * w)];
		window[2] = depthData[x - 1 + ((y - 1) * w)]; //2 <- 0

		window[3] = depthData[x - 1 + (y * w)];
		window[4] = depthData[x + (y * w)];
		window[5] = depthData[x - 1 + (y * w)]; //5 <- 3

		window[6] = depthData[x - 1 + ((y + 1) * w)];
		window[7] = depthData[x + ((y + 1) * w)];
		window[8] = depthData[x - 1 + ((y + 1) * w)]; //8 <- 6
	}

	//Middle Not a Border
	else
	{    //old
		//X, 1, 2 -- move 1, 2 -> 0, 1
		//X, 4, 5 -- move 4, 5 -> 3, 4
		//X, 7, 8 -- move 7, 8 -> 6, 7
		if (PreviouslyComputedWindow.size() == (w * h) && x > 1 && y > 1 && x < w - 2 && y < h - 2)
		{ //x just incremented by 1, and we're not next to a border, so we should be able to use the previously computed window to avoid re-computation

			//All this saves is a little bit of time calculating indeces as shown in the else statement here... it doesn't save on the computations for calculating pixel values
			window[0] = PreviouslyComputedWindow[1];
			window[1] = PreviouslyComputedWindow[2];
			window[2] = depthData[x + 1 + ((y - 1) * w)];

			window[3] = PreviouslyComputedWindow[4];
			window[4] = PreviouslyComputedWindow[5];
			window[5] = depthData[x + 1 + (y * w)];

			window[6] = PreviouslyComputedWindow[7];
			window[7] = PreviouslyComputedWindow[8];
			window[8] = depthData[x + 1 + ((y + 1) * w)];
		}
		else
		{ //compute the window from scratch
			window[0] = depthData[x - 1 + ((y - 1) * w)];
			window[1] = depthData[x + ((y - 1) * w)];
			window[2] = depthData[x + 1 + ((y - 1) * w)];

			window[3] = depthData[x - 1 + (y * w)];
			window[4] = depthData[x + (y * w)];
			window[5] = depthData[x + 1 + (y * w)];

			window[6] = depthData[x - 1 + ((y + 1) * w)];
			window[7] = depthData[x + ((y + 1) * w)];
			window[8] = depthData[x + 1 + ((y + 1) * w)];
		}
	}

	PreviouslyComputedWindow = window;

	window = TransposeWindow(window);

	return window;
}

//imgDepths is the subsampled back Z's and scales
std::vector<float> RunBlurKernel(std::vector<float> imgDepths, int w, int h)
{
	std::vector<float> blurredImage;
	std::vector<float> PreviouslyComputedWindow; //used to speed up window computation

	//average out the array data first.
	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			float averageZ = 0;
			if (imgDepths[x + (y * w)] != 0)
			{
				std::vector<float> currentWindow = getWindowFromMiddle<float>(PreviouslyComputedWindow, imgDepths, x, y, w, h, 3); //3x3 window size
				int numUsedInWindow = 0;
				for (int i = 0; i < currentWindow.size(); i++)
				{
					if (currentWindow[i] != 0)
					{
						averageZ += currentWindow[i];
						numUsedInWindow++;
					}
				}

				averageZ /= (float)numUsedInWindow;
			}
			else
			{
				PreviouslyComputedWindow.clear();
			}

			blurredImage.push_back(averageZ);
		}
	}

	return blurredImage;
}

void HP_WaitingRTApp::SetupBackParticlesDraw(std::vector<float> backParticleDepths, std::vector<float> backParticleScales, vec3 *backPositionsVBO, float *backScalesVBO, BackParticleSetting particlesSetting, int particleArrayWidth, int particleArrayHeight)
{
	if (particlesSetting == BackParticleSetting::AveragedShatteredGlass)
	{
		backParticleDepths = RunBlurKernel(backParticleDepths, particleArrayWidth, particleArrayHeight);
	}

	for (int y = 0; y < particleArrayHeight; y++)
	{
		for (int x = 0; x < particleArrayWidth; x++)
		{
			if (backParticleDepths[x + (y * particleArrayWidth)] != 0) //the array still has all the invaild depth values to keep the stride.
			{
				vec3 subsampledDepthPoint = vec3(
					((float)x * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
					((float)y * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
					backParticleDepths[x + (y * particleArrayWidth)]);

				vec3 worldPos = mDSAPI->getZCameraSpacePoint(subsampledDepthPoint);

				*backPositionsVBO++ = worldPos;
				*backScalesVBO++ = backParticleScales[x + (y * particleArrayWidth)];
				backNumberToDraw++;
			}
		}
	}
}

void HP_WaitingRTApp::GetKDEdgeIndex(uint16_t* depthFrame, int depthWidth, int depthHeight, int lowThresh, int highThresh, Mat &cannyEdgePixels, EdgeCloud<int> &edgeCloud)
{
	Mat depth = Mat(Size(depthWidth, depthHeight), CV_16U, depthFrame);//.getIter();
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


	if (validEdges.size() <= DESIRED_EDGE_NUM_CAP)
	{
		edgeCloud.edges = validEdges;
	}
	else //decimate to desired number of edges
	{
		for (int i = 0; i < validEdges.size(); i += validEdges.size() / DESIRED_EDGE_NUM_CAP)
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

std::vector<vec4> HP_WaitingRTApp::GetRGBSubsampledColors(std::vector<uint16_t> subsampledDepths, int subsampledWidth, int subsampledHeight)
{
	std::vector<vec4> subsampledColors;

	for (int y = 0; y < subsampledHeight; y++)
	{
		for (int x = 0; x < subsampledWidth; x++)
		{
			float zImage[] = { static_cast<float>(x * mDepthSubsampleSize), static_cast<float>(y * mDepthSubsampleSize), static_cast<float>(subsampledDepths[x + (y * subsampledWidth)]) }, zCamera[3], thirdCamera[3], thirdImage[2];
			DSTransformFromZImageToZCamera(zIntrin, zImage, zCamera);
			DSTransformFromZCameraToRectThirdCamera(zToThirdTrans, zCamera, thirdCamera);
			DSTransformFromThirdCameraToRectThirdImage(thirdIntrin, thirdCamera, thirdImage);

			if (zImage[2] != 0) //depth is valid
			{
				if (thirdImage[0]>0 && thirdImage[0] < mDSAPI->getRgbWidth() && thirdImage[1]>0 && thirdImage[0] < mDSAPI->getRgbHeight())
				{
					uint8_t r = mRgbBuffer.getData()[(((int)thirdImage[0] + ((int)thirdImage[1] * thirdIntrin.rw)) * 3) + 0];
					uint8_t g = mRgbBuffer.getData()[(((int)thirdImage[0] + ((int)thirdImage[1] * thirdIntrin.rw)) * 3) + 1];
					uint8_t b = mRgbBuffer.getData()[(((int)thirdImage[0] + ((int)thirdImage[1] * thirdIntrin.rw)) * 3) + 2];
					subsampledColors.push_back(vec4((float)r / (float)255, (float)g / (float)255, (float)b / (float)255, 1.0));
				}
			}
			else
			{
				subsampledColors.push_back(vec4(0, 0, 0, 0));
			}
		}
	}

	return subsampledColors;
}

void HP_WaitingRTApp::SetupTriangles(std::vector<uint16_t> &subsampledDepths, std::vector<vec4> &subsampledColors, int subsampledWidth, int subsampledHeight, vec3* &trianglePositions, vec4* &triangleColors, uint16_t* &triangleIndices, int x, int y)
{
	if (DrawTriangles == false)
	{
		return;
	}

	vec3 trianglePoint = vec3(
		((float)x * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
		((float)y * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
		subsampledDepths[x + (y * subsampledWidth)]);
	vec3 p = mDSAPI->getZCameraSpacePoint(trianglePoint);

	*trianglePositions++ = p;
	*trianglePositions++ = p;
	vec4 triColor1(0, 0, 0, 0);
	vec4 triColor2(0, 0, 0, 0);

	std::vector<pair<bool, uint16_t>> windowFromTopLeft = getSquareFromTopLeft<uint16_t>(subsampledDepths, x, y, subsampledWidth, subsampledHeight, mCam.getFarClip());

	if ((x + y) % 2 && ModuloTriangleDirections)
	{ //if it's an odd index, setup triangles to be Top Right and/or Bottom Left
		if (windowFromTopLeft[0].first &&
			windowFromTopLeft[3].first &&
			windowFromTopLeft[1].first)
		{

			*triangleIndices++ = windowFromTopLeft[0].second * 2;
			*triangleIndices++ = windowFromTopLeft[3].second * 2;
			*triangleIndices++ = windowFromTopLeft[1].second * 2;

			vec4 c1 = subsampledColors[windowFromTopLeft[0].second];
			vec4 c2 = subsampledColors[windowFromTopLeft[3].second];
			vec4 c3 = subsampledColors[windowFromTopLeft[1].second];

			vec4 cAvg = vec4(c1 + c2 + c3);
			cAvg /= 3;
			triColor1 = cAvg;

			numberTrianglesToDraw++;
		}
		if (windowFromTopLeft[0].first &&
			windowFromTopLeft[3].first &&
			windowFromTopLeft[2].first)
		{
			*triangleIndices++ = windowFromTopLeft[0].second * 2 + 1;
			*triangleIndices++ = windowFromTopLeft[3].second * 2 + 1;
			*triangleIndices++ = windowFromTopLeft[2].second * 2 + 1;

			vec4 c1 = subsampledColors[windowFromTopLeft[0].second];
			vec4 c2 = subsampledColors[windowFromTopLeft[3].second];
			vec4 c3 = subsampledColors[windowFromTopLeft[2].second];

			vec4 cAvg = vec4(c1 + c2 + c3);
			cAvg /= 3;
			triColor2 = cAvg;

			numberTrianglesToDraw++;
		}
	}
	else
	{ //if it's an even index, setup triangles to be Top Left and/or Bottom Right
		if (windowFromTopLeft[2].first &&
			windowFromTopLeft[1].first &&
			windowFromTopLeft[0].first)
		{

			*triangleIndices++ = windowFromTopLeft[2].second * 2;
			*triangleIndices++ = windowFromTopLeft[1].second * 2;
			*triangleIndices++ = windowFromTopLeft[0].second * 2;

			vec4 c1 = subsampledColors[windowFromTopLeft[2].second];
			vec4 c2 = subsampledColors[windowFromTopLeft[1].second];
			vec4 c3 = subsampledColors[windowFromTopLeft[0].second];

			vec4 cAvg = vec4(c1 + c2 + c3);
			cAvg /= 3;
			triColor1 = cAvg;

			numberTrianglesToDraw++;
		}
		if (windowFromTopLeft[2].first &&
			windowFromTopLeft[3].first &&
			windowFromTopLeft[1].first)
		{
			*triangleIndices++ = windowFromTopLeft[2].second * 2 + 1;
			*triangleIndices++ = windowFromTopLeft[3].second * 2 + 1;
			*triangleIndices++ = windowFromTopLeft[1].second * 2 + 1;

			vec4 c1 = subsampledColors[windowFromTopLeft[2].second];
			vec4 c2 = subsampledColors[windowFromTopLeft[3].second];
			vec4 c3 = subsampledColors[windowFromTopLeft[1].second];

			vec4 cAvg = vec4(c1 + c2 + c3);
			cAvg /= 3;
			triColor2 = cAvg;

			numberTrianglesToDraw++;
		}
	}

	*triangleColors++ = triColor1;
	*triangleColors++ = triColor2;
}

void HP_WaitingRTApp::GLDrawTriangles()
{
	if (DrawTriangles)
	{
		if (DrawBatchTriangles)
		{
			mGlslTriangles->bind();
			mGlslTriangles->uniform("rotationMatrix", glm::mat4());

			//mBatchTriangles->drawInstanced(numberTrianglesToDraw);
			mBatchTriangles->draw();
		}
		else
		{
		vec3 *trianglePositions = (vec3*)mTrianglePositionsVbo->map(GL_READ_ONLY);
		uint16_t *triangleIndices = (uint16_t*)mTriangleIndicesVbo->map(GL_READ_ONLY);
		vec4 *triangleColors = (vec4*)mTriangleColorsVbo->map(GL_READ_ONLY);

		//GLuint positions;
		//glGenBuffers(1, &positions);
		//glBindBuffer(GL_ARRAY_BUFFER, positions);
		//glBufferData(GL_ARRAY_BUFFER, numberTrianglesToDraw * sizeof(vec3), trianglePositions, GL_DYNAMIC_DRAW);

		//GLuint indices;
		//glGenBuffers(1, &indices);
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices);
		//glBufferData(GL_ELEMENT_ARRAY_BUFFER, numberTrianglesToDraw * 3 * sizeof(uint16_t), trangleIndices, GL_DYNAMIC_DRAW);

		//
		//glUseProgram(mGlsl->getHandle());
		//

		//glBindBuffer(GL_ARRAY_BUFFER, positions);
		//glVertexAttribPointer(positions, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), trianglePositions);
		//glEnableVertexAttribArray(positions);

		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices);
		//glVertexAttribPointer(indices, 3, GL_FLOAT, GL_FALSE, sizeof(uint16_t), trangleIndices);
		//glVertexAttribDivisor(indices, 3);
		//glEnableVertexAttribArray(indices);

		//glDrawElements(GL_TRIANGLES, numberTrianglesToDraw, GL_UNSIGNED_SHORT, trangleIndices);
		//glDeleteBuffers(1, &indices);
		//glDeleteBuffers(1, &positions);


		////mTriangleIndicesVbo->bufferData(numberTrianglesToDraw * 3 * 2 * sizeof(uint16_t), trangleIndices, GL_DYNAMIC_DRAW);

		gl::begin(GL_TRIANGLES);
		for (int i = 0; i < numberTrianglesToDraw; i++)
		{
			vec3 ab(trianglePositions[triangleIndices[(i * 3) + 1]] - trianglePositions[triangleIndices[(i * 3) + 0]]);
			vec3 ac(trianglePositions[triangleIndices[(i * 3) + 2]] - trianglePositions[triangleIndices[(i * 3) + 0]]);
			vec3 n = vec3(ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z, ab.x * ac.y - ab.y * ac.x);

			//vec3 a = trianglePositions[trangleIndices[(i * 3) + 0]];
			//vec3 b = trianglePositions[trangleIndices[(i * 3) + 1]];
			//vec3 c = trianglePositions[trangleIndices[(i * 3) + 2]];
			//float averageDepth = (a.z + b.z + c.z) / 3;
			//averageDepth = lmap<float>(averageDepth, 500, 2000, 0, 1);
			//gl::color(averageDepth, averageDepth, averageDepth, 0.75);

			vec4 c1 = triangleColors[triangleIndices[(i * 3) + 0]];
			vec4 c2 = triangleColors[triangleIndices[(i * 3) + 1]];
			vec4 c3 = triangleColors[triangleIndices[(i * 3) + 2]];

			vec4 cAvg = vec4(c1 + c2 + c3);
			cAvg /= 3;

			gl::color(cAvg.r, cAvg.g, cAvg.b, cAvg.a);
			gl::color(cAvg.r, cAvg.g, cAvg.b, cAvg.a);
			gl::color(cAvg.r, cAvg.g, cAvg.b, cAvg.a);

			gl::vertex(trianglePositions[triangleIndices[(i * 3) + 0]]);
			gl::vertex(trianglePositions[triangleIndices[(i * 3) + 1]]);
			gl::vertex(trianglePositions[triangleIndices[(i * 3) + 2]]);
		}
		gl::end();

		mTrianglePositionsVbo->unmap();
		mTriangleIndicesVbo->unmap();
		mTriangleColorsVbo->unmap();

		}
	}
}


#if defined( CINDER_MSW ) && ! defined( CINDER_GL_ANGLE )
auto options = RendererGl::Options().version(3, 3); // instancing functions are technically only in GL 3.3
#else
auto options = RendererGl::Options(); // implemented as extensions in Mac OS 10.7+
#endif
CINDER_APP(HP_WaitingRTApp, RendererGl(options))