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

const static float Gravity = -9.8;
const static int MaxNumberDyingParticleBufferSize = 75000; //THERE'S A LIMIT TO PARTICLES DEPENDING ON THE GPU, I DON'T KNOW WHY -- somwhere aroud 75k is when it starts glitching
const static float DeathLifetimeInSeconds = 1.5;

class DyingParticlesManager
{

private:

	void KillFrontParticleBatch()
	{
		CurrentTotalParticles -= mDyingParticleBatches.front().size();

		mDyingParticleBatches.pop_front();
		BatchCreationTime.pop_front();
	}

	vec3 ParticlePositionWithGravityOffset(ParticleInstance particle, float totalLivingTime, float gravity = Gravity)
	{
		return vec3(particle.InitialPosition.x, particle.InitialPosition.y + (gravity * totalLivingTime) * 10, particle.InitialPosition.z);
	}

	float ParticleOpacity(ParticleInstance particle, float totalLivingTime)
	{
		return 1.0 - (totalLivingTime / DeathLifetimeInSeconds);
	}

public:
	std::deque<std::vector<ParticleInstance>> mDyingParticleBatches;
	std::deque<float> BatchCreationTime;

	int CurrentTotalParticles = 0;

	DyingParticlesManager()
	{

	}

	int ManualCount()
	{
		int count = 0;
		for (int i = 0; i < mDyingParticleBatches.size(); i++)
		{
			count += mDyingParticleBatches[i].size();
		}

		return (count);
	}

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
	int highThreshold = 6;	// low = 4; high = 5 gives a sweet interior outliney effect

	std::vector<int> mHogCounts;
	std::vector<long> mHogTotals;
	const float HogStartDepth = 500;
	const int HogBinStep = 30;
	const int NumberOfDepthSteps = 50; //only go (30 * 50) = 1500 pixels in from StartDepth
	float mBackDepthPlaneZ = 0;
};

int width;
int height;
int numberToDraw = 0;
int backNumberToDraw = 0;

double previousElapsedTime = 0;

const float MaxDeathTimeSeconds = 5;


void HP_WaitingRTApp::setup()
{
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
	mCam.setPerspective(cFOVs.y, getWindowAspectRatio(), 100, 5000);
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
	mInstanceDyingDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER, (MaxNumberDyingParticleBufferSize) * sizeof(vec3), deathPositions.data(), GL_DYNAMIC_DRAW);
	mInstanceDyingTotalLifetimesVbo = gl::Vbo::create(GL_ARRAY_BUFFER, (MaxNumberDyingParticleBufferSize)* sizeof(float), deathTotalLifetimes.data(), GL_DYNAMIC_DRAW);
	mInstanceBackPositionVbo = gl::Vbo::create(GL_ARRAY_BUFFER, backPositions.size() * sizeof(vec3), backPositions.data(), GL_DYNAMIC_DRAW);

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
	meshBack->appendVbo(instanceBackDataLayout, mInstanceBackPositionVbo);

	mBatchBackubes = gl::Batch::create(meshBack, mGlslBackCubes, { { geom::Attrib::CUSTOM_0, "vInstancePosition" } });


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
	mCam.setPerspective(60, getWindowAspectRatio(), 1, 1000);
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
	vec3 *deathPositions = (vec3*)mInstanceDyingDataVbo->mapWriteOnly(true);
	float *totalLivingTimes = (float*)mInstanceDyingTotalLifetimesVbo->mapWriteOnly(true);

	std::vector<ParticleInstance> dyingParticlesToAdd = std::vector<ParticleInstance>();

	numberToDraw = 0;
	backNumberToDraw = 0;
	

	
	uint16_t* originalDepthBuffer = mDepthBuffer.getDataStore().get();
	Mat depth = Mat(Size(width, height), CV_16UC1, mDepthBuffer.getDataStore().get());//.getIter();
	Mat depthf(Size(width, height), CV_8UC1);
	depth.convertTo(depthf, CV_8UC1, 255.0 / 4096.0);

	//uint16_t* depth = mDepthBuffer.getDataStore().get();
	//uint8_t* rgb = mRgbBuffer.getDataStore().get();
	InputArray input = InputArray(depthf);
	OutputArray output = OutputArray(depthf);
	output.create(Size(width, height), CV_8UC1);
	cv::Canny(input, output, lowThreshold, highThreshold);


	//Histogram of gradients?
	std::fill(mHogCounts.begin(), mHogCounts.end(), 0); //reset all counts to zero
	std::fill(mHogTotals.begin(), mHogTotals.end(), 0); //reset all totals to zero

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (output.getMat().data[x + (y * width)] != 0)
			{
				int originalDepth = originalDepthBuffer[x + (y * width)];
				if (originalDepth != 0)
				{
					int rem = originalDepth % HogBinStep;
					//round up to farther bin
					originalDepth += HogBinStep - rem;

					//normalized to be in the 10's, now turn into an index
					originalDepth -= HogStartDepth; //start depth should be first index
					originalDepth /= HogBinStep; //normalized to an index

					if (originalDepth >= 0 && originalDepth && originalDepth < mHogCounts.size())
					{
						mHogTotals[originalDepth] += originalDepthBuffer[x + (y * width)];
						mHogCounts[originalDepth]++;
					}
				}
			}
		}
	}

	int highestCount = 0;
	int highestCountIndex = 0;
	for (int i = 0; i < mHogCounts.size(); i++)
	{
		if (mHogCounts[i] > highestCount)
		{
			highestCount = mHogCounts[i];
			highestCountIndex = i;
		}
	}

	if (highestCount < 75) //min threshold for counting as needing a back plane
	{
		mBackDepthPlaneZ = 0;
	}
	else
	{
		float desiredBackPlaneZ = ((float)mHogTotals[highestCountIndex] / (float)highestCount); //get the average actual Z value of all those edges in the best bin.

		if (mBackDepthPlaneZ != 0)
		{ //lerp into place
			float lerpRate = 0.3;
			if (mBackDepthPlaneZ < desiredBackPlaneZ)
				lerpRate = 0.6; //if it's trying to go farther back, lerp in faster

			mBackDepthPlaneZ += (desiredBackPlaneZ - mBackDepthPlaneZ) * lerpRate;
		}
		else //snap into place
		{
			mBackDepthPlaneZ = desiredBackPlaneZ;
		}
		console() << mBackDepthPlaneZ << endl;
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

	for (int potX = 0; potX < width; ++potX)
	{
		for (int potY = 0; potY < height; ++potY)
		{
			float v = mDepthBuffer.getDataStore().get()[potX + (width * potY)];
			bool cannyEdgePixel = (output.getMat().data[potX + (width * potY)] != 0);
			//	v = lmap<float>(v, 1, 255, 400, 5000); // (float)depth[potX + (width * potY)];//depth.data[potX + (width * potY)];

			if (v != 0 && mCam.getFarClip() > v)
			{
				vec3 worldPos = mDSAPI->getDepthSpacePoint(vec3((float)potX, (float)potY, v));

				//*positions++ = vec3(potX / width - 0.5f, potY / height - 0.5f, 0);// depth[potX + (potY * width)]
				*positions++ = worldPos;

				if (cannyEdgePixel)
					*scales++ = 1.5f;
				else
					*scales++ = 0.3f;

				numberToDraw++;

				mPreviousValidDepthCoord[potX + (potY * width)] = worldPos;
				mPreviouslyHadDepth[potX + (potY * width)] = true;

				if (mBackDepthPlaneZ > 0 && mCam.getFarClip() > v)
				{ //BACK FILLERS
					*backPositions++ = vec3(worldPos.x, worldPos.y, mBackDepthPlaneZ);
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

	//console() << mDyingParticleManager.CurrentTotalParticles << endl;
	
	mInstancePositionVbo->unmap();
	mInstanceScaleVbo->unmap();
	mInstanceDyingDataVbo->unmap();
	mInstanceDyingTotalLifetimesVbo->unmap();
	mInstanceBackPositionVbo->unmap();
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
	mBatchDyingCubes->drawInstanced(mDyingParticleManager.CurrentTotalParticles);
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

#if defined( CINDER_MSW ) && ! defined( CINDER_GL_ANGLE )
auto options = RendererGl::Options().version(3, 3); // instancing functions are technically only in GL 3.3
#else
auto options = RendererGl::Options(); // implemented as extensions in Mac OS 10.7+
#endif
CINDER_APP_NATIVE(HP_WaitingRTApp, RendererGl(options))