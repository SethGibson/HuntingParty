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
		if (CurrentTotalParticles + particles.size() > MAX_NUMBER_DYING_PARTICLE_BUFFER_SIZE)
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

	void SetupParticles(vec3 *positions, vec3 *backPositions, float *scales, float *dyingScales, float *fillerScales, vec3 *deathPositions, float *totalLivingTimes, float elapsed);
	std::vector<uint16_t> GetSuperPixelAverageDepths(uint16_t* depthMap, int width, int height, int subsampleWindowSize);

	void SetupBackParticlesDraw(std::vector<float> backParticlePositions, std::vector<float> backParticleScales, vec3 *backPositionsVBO, float *backScalesVBO, int particleArrayWidth, int particleArrayHeight);

	std::vector<vec4> GetRGBSubsampledColors(std::vector<uint16_t> subsampledDepths, int subsampledWidth, int subsampledHeight);

	CameraPersp			mCam;
	gl::BatchRef		mBatch;
	gl::BatchRef		mBatchDyingCubes;
	gl::BatchRef		mBatchBackubes;

	gl::TextureRef		mTexture;

	gl::GlslProgRef		mGlsl;
	gl::GlslProgRef		mGlslDyingCubes;
	gl::GlslProgRef		mGlslBackCubes;

	gl::VboRef			mInstancePositionVbo;
	gl::VboRef			mInstanceScaleVbo;

	gl::VboRef			mInstanceDyingDataVbo;
	gl::VboRef			mInstanceDyingScaleVbo;
	gl::VboRef			mInstanceDyingTotalLifetimesVbo;

	gl::VboRef			mInstanceBackPositionVbo;
	gl::VboRef			mInstanceFillerScaleVbo;

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
	bool DrawBackParticles = true;
	bool DrawDyingParticles = true;
	bool DrawFrontParticles = true;
	bool DrawTriangles = false;
	bool MaskFrontParticles = false;
	bool ModuloTriangleDirections = true;
};


void HP_WaitingRTApp::setup()
{
	mDepthSubsampleSize = DEPTH_SUBSAMPLE_WINDOW_SIZE;

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

	mDSAPI->getDSAPI()->getCalibIntrinsicsZ(zIntrin);
	mDSAPI->getDSThird()->getCalibIntrinsicsRectThird(thirdIntrin);
	mDSAPI->getDSThird()->getCalibExtrinsicsZToRectThird(zToThirdTrans);


	mCam.setPerspective(cFOVs.y, getWindowAspectRatio(), 100, 2500);
	mCam.lookAt(vec3(0, 0, 0), vec3(0, 0, 1000), vec3(0, -1, 0));

	mMayaCam = MayaCamUI(mCam);

	mTexture = gl::Texture::create(loadImage(loadAsset("blank_texture.png")), gl::Texture::Format().mipmap());
	mLogoTexture = gl::Texture::create(loadImage(loadAsset("linkin_park_logo_flipped.png")));
#if ! defined( CINDER_GL_ES )
	mGlsl = gl::GlslProg::create(loadAsset("shader.vert"), loadAsset("shader.frag"));
	mGlslDyingCubes = gl::GlslProg::create(loadAsset("shaderDying.vert"), loadAsset("shaderDying.frag"));
	mGlslBackCubes = gl::GlslProg::create(loadAsset("shaderBack.vert"), loadAsset("shaderBack.frag"));
#else
	mGlsl = gl::GlslProg::create(loadAsset("shader_es2.vert"), loadAsset("shader_es2.frag"));
#endif

	gl::VboMeshRef mesh = gl::VboMesh::create(geom::Sphere().subdivisions(4));
	gl::VboMeshRef meshDying = gl::VboMesh::create(geom::Cone().subdivisionsAxis(4));
	gl::VboMeshRef meshBack = gl::VboMesh::create(geom::Cube());

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

	gl::enableDepthWrite();
	gl::enableDepthRead();
	gl::enableAlphaBlending();

	mTexture->bind(0);
	mLogoTexture->bind(1);

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
	float *dyingScales = (float*)mInstanceDyingScaleVbo->mapWriteOnly(true);
	float *fillerScales = (float*)mInstanceFillerScaleVbo->mapWriteOnly(true);
	vec3 *deathPositions = (vec3*)mInstanceDyingDataVbo->mapWriteOnly(true);
	float *totalLivingTimes = (float*)mInstanceDyingTotalLifetimesVbo->mapWriteOnly(true);

	SetupParticles(positions, backPositions, scales, dyingScales, fillerScales, deathPositions, totalLivingTimes, elapsed);


	mInstancePositionVbo->unmap();
	mInstanceScaleVbo->unmap();
	mInstanceDyingDataVbo->unmap();
	mInstanceDyingTotalLifetimesVbo->unmap();
	mInstanceBackPositionVbo->unmap();
	mInstanceDyingScaleVbo->unmap();
	mInstanceFillerScaleVbo->unmap();
}

//Given mapped buffers, set up all the particles for drawing.
void HP_WaitingRTApp::SetupParticles(vec3 *positions, vec3 *backPositions, float *scales, float *dyingScales, float *fillerScales, vec3 *deathPositions, float *totalLivingTimes, float elapsed)
{
	numberToDraw = 0;
	backNumberToDraw = 0;
	numberTrianglesToDraw = 0;

	std::vector<ParticleInstance> dyingParticlesToAdd = std::vector<ParticleInstance>();

	uint16_t* originalDepthBuffer = mDepthBuffer.getDataStore().get();

	//subsampled depth particles:::
	std::vector<uint16_t> subsampledDepths = GetSuperPixelAverageDepths(mDepthBuffer.getDataStore().get(), width, height, mDepthSubsampleSize);
	int subsampledWidth = width / mDepthSubsampleSize;
	int subsampledHeight = height / mDepthSubsampleSize;

	//std::vector<vec4> subsampledColors = GetRGBSubsampledColors(subsampledDepths, subsampledWidth, subsampledHeight);

	uint32_t currentTotalDepth = 0;

	for (int y = 0; y < subsampledHeight; y++)
	{
		for (int x = 0; x < subsampledWidth; x++)
		{
			float subsampledAverageDepth = subsampledDepths[(y * subsampledWidth) + x];

			if (subsampledAverageDepth != 0 && subsampledAverageDepth < mCam.getFarClip())
			{
				currentTotalDepth += subsampledAverageDepth;

				vec3 subsampledDepthPoint = vec3(
					((float)x * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
					((float)y * (float)mDepthSubsampleSize) + ((float)mDepthSubsampleSize / (float)2),
					subsampledAverageDepth);

				vec3 worldPos = mDSAPI->getDepthSpacePoint(subsampledDepthPoint);

				*positions++ = worldPos;
				*scales++ = 0.5f;//0.3f;

				numberToDraw++;

				dyingParticlesToAdd.push_back(ParticleInstance(worldPos, vec2(x, y), elapsed)); //add to list of particles to add, manager will take care of making sure they fit.
			}
		}
	}
	

	mDyingParticleManager.AddDyingParticleBatch(dyingParticlesToAdd, elapsed);

	//setup the gpu memory positions
	mDyingParticleManager.SetupBatchDraw(mCurrentlyHasDyingInstance, deathPositions, totalLivingTimes, dyingScales, elapsed, 1.0f, subsampledWidth, subsampledHeight);

	ForwardForceMultiplier = (currentTotalDepth / (subsampledWidth * subsampledHeight)) - (previousTotalDepth / (subsampledWidth * subsampledHeight));
	previousTotalDepth = currentTotalDepth;
}

void HP_WaitingRTApp::draw()
{
	gl::clear(Color::black());

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
}

void HP_WaitingRTApp::keyDown(KeyEvent event)
{
	if (event.getChar() == 'v'){
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
	else if (event.getChar() == 'v'){
		GravityForceMultiplier = -GravityForceMultiplier;
	}
	else if (event.getChar() == '1'){
		do {
			mDepthSubsampleSize = math<int>::min(8, mDepthSubsampleSize + 1); //MAX OUT AT 12x12 window size

			mPreviousValidDepthCoord.clear();
			mPreviousValidDepthCoord = std::vector<vec3>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			mPreviouslyHadDepth.clear();
			mPreviouslyHadDepth = std::vector<bool>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			std::fill(mPreviouslyHadDepth.begin(), mPreviouslyHadDepth.end(), false);
		} while (width % mDepthSubsampleSize != 0 || height % mDepthSubsampleSize != 0);
	}
	else if (event.getChar() == '2'){
		do {
			mDepthSubsampleSize = math<int>::max(1, mDepthSubsampleSize - 1);

			mPreviousValidDepthCoord.clear();
			mPreviousValidDepthCoord = std::vector<vec3>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			mPreviouslyHadDepth.clear();
			mPreviouslyHadDepth = std::vector<bool>(width / mDepthSubsampleSize * height / mDepthSubsampleSize);
			std::fill(mPreviouslyHadDepth.begin(), mPreviouslyHadDepth.end(), false);
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

void HP_WaitingRTApp::SetupBackParticlesDraw(std::vector<float> backParticleDepths, std::vector<float> backParticleScales, vec3 *backPositionsVBO, float *backScalesVBO, int particleArrayWidth, int particleArrayHeight)
{
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

				vec3 worldPos = mDSAPI->getDepthSpacePoint(subsampledDepthPoint);

				*backPositionsVBO++ = worldPos;
				*backScalesVBO++ = backParticleScales[x + (y * particleArrayWidth)];
				backNumberToDraw++;
			}
		}
	}
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
				uint8_t r = mRgbBuffer.getData()[(((int)thirdImage[0] + ((int)thirdImage[1] * thirdIntrin.rw)) * 3) + 0];
				uint8_t g = mRgbBuffer.getData()[(((int)thirdImage[0] + ((int)thirdImage[1] * thirdIntrin.rw)) * 3) + 1];
				uint8_t b = mRgbBuffer.getData()[(((int)thirdImage[0] + ((int)thirdImage[1] * thirdIntrin.rw)) * 3) + 2];
				subsampledColors.push_back(vec4((float)r / (float)255, (float)g / (float)255, (float)b / (float)255, 1.0));
			}
			else
			{
				subsampledColors.push_back(vec4(0, 0, 0, 0));
			}
		}
	}

	return subsampledColors;
}


#if defined( CINDER_MSW ) && ! defined( CINDER_GL_ANGLE )
auto options = RendererGl::Options().version(3, 3); // instancing functions are technically only in GL 3.3
#else
auto options = RendererGl::Options(); // implemented as extensions in Mac OS 10.7+
#endif
CINDER_APP_NATIVE(HP_WaitingRTApp, RendererGl(options))