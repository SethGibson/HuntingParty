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


using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;
using namespace cv;

class HP_WaitingRTApp : public AppNative {
public:
	void setup();
	void shutdown() override;
	void resize();
	void update();
	void draw();
	void mouseDown(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;

	CameraPersp			mCam;
	gl::BatchRef		mBatch;
	gl::BatchRef		mBatchDyingCubes;
	gl::TextureRef		mTexture;
	gl::GlslProgRef		mGlsl;
	gl::GlslProgRef		mGlslDyingCubes;
	gl::VboRef			mInstanceDataVbo;
	gl::VboRef			mInstanceScaleVbo;
	gl::VboRef			mInstanceDyingDataVbo;
	gl::VboRef			mInstanceDyingTotalTimeVbo;

	//DSAPI
	CinderDSRef mDSAPI;
	MayaCamUI mMayaCam;

	Surface8u mRgbBuffer;
	Channel16u mDepthBuffer;

	std::vector<bool> mPreviouslyHadDepth;
	std::vector<bool> mCurrentlyHasDeadInstance;
	std::vector<float> mTimeOfDeath;
	std::vector<vec3> mDeadParticleStartingPositions;
};

int width;
int height;
int numberToDraw = 0;
int numberOfDyingToDraw = 0;

double previousElapsedTime = 0;

const float MaxDeathTimeSeconds = 5;


void HP_WaitingRTApp::setup()
{
	mDSAPI = CinderDSAPI::create();
	mDSAPI->init();
	mDSAPI->initDepth(CinderDS::FrameSize::DEPTHSD, 60);
	mDSAPI->initRgb(CinderDS::FrameSize::RGBVGA, 30);
	mDSAPI->start();

	width = mDSAPI->getDepthWidth();
	height = mDSAPI->getDepthHeight();

	//mCam.lookAt(vec3(0, CAMERA_Y_RANGE.first, 0), vec3(0));
	vec2 cFOVs = mDSAPI->getDepthFOVs();
	mCam.setPerspective(cFOVs.y, getWindowAspectRatio(), 100, 5000);
	mCam.lookAt(vec3(0, 0, 0), vec3(0, 0, 1000), vec3(0, -1, 0));

	mMayaCam = MayaCamUI(mCam);

	mTexture = gl::Texture::create(loadImage(loadAsset("texture.jpg")), gl::Texture::Format().mipmap());
#if ! defined( CINDER_GL_ES )
	mGlsl = gl::GlslProg::create(loadAsset("shader.vert"), loadAsset("shader.frag"));
	mGlslDyingCubes = gl::GlslProg::create(loadAsset("shaderDying.vert"), loadAsset("shaderDying.frag"));
#else
	mGlsl = gl::GlslProg::create(loadAsset("shader_es2.vert"), loadAsset("shader_es2.frag"));
#endif

	gl::VboMeshRef mesh = gl::VboMesh::create(geom::Cube());
	gl::VboMeshRef meshDying = gl::VboMesh::create(geom::Cone().subdivisionsAxis(4));

	// create an array of initial per-instance positions laid out in a 2D grid
	std::vector<vec3> positions;
	std::vector<float> cubeScales;
	std::vector<vec3> deathPositions;
	std::vector<float> totalTimesSinceDeath;
	for (size_t potX = 0; potX < width; ++potX) {
		for (size_t potY = 0; potY < height; ++potY) {
			float instanceX = potX / (float)width - 0.5f;
			float instanceY = potY / (float)height - 0.5f;
			vec3 pos = vec3(instanceX * vec3(1, 0, 0) + instanceY * vec3(0, 0, 1));
			positions.push_back(pos);

			cubeScales.push_back(randFloat(0.2, 5));

			deathPositions.push_back(pos);
			totalTimesSinceDeath.push_back(0.0);

			mPreviouslyHadDepth.push_back(false);
			mCurrentlyHasDeadInstance.push_back(false);
			mTimeOfDeath.push_back(0.0);
			mDeadParticleStartingPositions.push_back(pos);
		}
	}

	// create the VBO which will contain per-instance (rather than per-vertex) data
	mInstanceDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER, positions.size() * sizeof(vec3), positions.data(), GL_DYNAMIC_DRAW);
	mInstanceScaleVbo = gl::Vbo::create(GL_ARRAY_BUFFER, cubeScales.size() * sizeof(float), cubeScales.data(), GL_DYNAMIC_DRAW);
	mInstanceDyingDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER, deathPositions.size() * sizeof(vec3), deathPositions.data(), GL_DYNAMIC_DRAW);
	mInstanceDyingTotalTimeVbo = gl::Vbo::create(GL_ARRAY_BUFFER, totalTimesSinceDeath.size() * sizeof(float), totalTimesSinceDeath.data(), GL_DYNAMIC_DRAW);

	// we need a geom::BufferLayout to describe this data as mapping to the CUSTOM_0 semantic, and the 1 (rather than 0) as the last param indicates per-instance (rather than per-vertex)
	geom::BufferLayout instanceDataLayout;
	instanceDataLayout.append(geom::Attrib::CUSTOM_0, 3, 0, 0, 1 /* per instance */);

	geom::BufferLayout instanceScaleLayout;
	instanceScaleLayout.append(geom::Attrib::CUSTOM_1, 1, 0, 0, 1 /* per instance */);

	// now add it to the VboMesh we already made of the Teapot
	mesh->appendVbo(instanceDataLayout, mInstanceDataVbo);
	mesh->appendVbo(instanceScaleLayout, mInstanceScaleVbo);

	// and finally, build our batch, mapping our CUSTOM_0 attribute to the "vInstancePosition" GLSL vertex attribute
	mBatch = gl::Batch::create(mesh, mGlsl, { { geom::Attrib::CUSTOM_0, "vInstancePosition" }, { geom::Attrib::CUSTOM_1, "fCubeScale" } });

	
	geom::BufferLayout instanceDyingDataLayout;
	instanceDyingDataLayout.append(geom::Attrib::CUSTOM_2, 3, 0, 0, 1 /* per instance */);

	geom::BufferLayout instanceDeathTimeLayout;
	instanceDeathTimeLayout.append(geom::Attrib::CUSTOM_3, 1, 0, 0, 1 /* per instance */);

	meshDying->appendVbo(instanceScaleLayout, mInstanceScaleVbo);
	meshDying->appendVbo(instanceDyingDataLayout, mInstanceDyingDataVbo);
	meshDying->appendVbo(instanceDeathTimeLayout, mInstanceDyingTotalTimeVbo);

	mBatchDyingCubes = gl::Batch::create(meshDying, mGlslDyingCubes, { { geom::Attrib::CUSTOM_1, "fCubeScale" }, { geom::Attrib::CUSTOM_2, "vDeathPosition" }, { geom::Attrib::CUSTOM_3, "fSecondsSinceDeath" } });

	gl::enableDepthWrite();
	gl::enableDepthRead();

	mTexture->bind();

	previousElapsedTime = getElapsedSeconds();
}

void HP_WaitingRTApp::shutdown()
{
	mDSAPI->stop();
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

	mDSAPI->update();

	mDepthBuffer = mDSAPI->getDepthFrame();
	mRgbBuffer = mDSAPI->getRgbFrame();

	// update our instance positions; map our instance data VBO, write new positions, unmap
	vec3 *positions = (vec3*)mInstanceDataVbo->mapWriteOnly(true);
	float *scales = (float*)mInstanceScaleVbo->mapWriteOnly(true);
	vec3 *deathPositions = (vec3*)mInstanceDyingDataVbo->mapWriteOnly(true);
	float *dyingTimes = (float*)mInstanceDyingTotalTimeVbo->mapWriteOnly(true);

	numberToDraw = 0;
	numberOfDyingToDraw = 0;

	//Mat depth = Mat(Size(width, height), CV_16UC1, mDepthBuffer.getDataStore().get());//.getIter();

	uint16_t* depth = mDepthBuffer.getDataStore().get();

	for (int potX = 0; potX < width; ++potX)
	{
		for (int potY = 0; potY < height; ++potY)
		{
			float v = (float)depth[potX + (width * potY)];//depth.data[potX + (width * potY)];

			if (v != 0)
			{
				vec3 worldPos = mDSAPI->getDepthSpacePoint(vec3((float)potX, (float)potY, v));

				//*positions++ = vec3(potX / width - 0.5f, potY / height - 0.5f, 0);// depth[potX + (potY * width)]
				*positions++ = worldPos;

				*scales++ = 0.5f;

				numberToDraw++;

				if (mCurrentlyHasDeadInstance[potX + (potY * width)] == false)
				{
					mDeadParticleStartingPositions[potX + (potY * width)] = worldPos;
				}

				mPreviouslyHadDepth[potX + (potY * width)] = true;
			}
			else //no depth here now, if there was depth, start dying.
			{
				if (mPreviouslyHadDepth[potX + (potY * width)]) //there was depth and it just died
				{
					if (mCurrentlyHasDeadInstance[potX + (potY * width)])
					{
						*dyingTimes = elapsed - mTimeOfDeath[potX + (potY * width)];
					}
					else //new death
					{
						*dyingTimes = 0;
						mTimeOfDeath[potX + (potY * width)] = elapsed;
					}

					if (*dyingTimes < MaxDeathTimeSeconds) //there's an old death here, and it's not too old, continue to add to draw batch.
					{
						*deathPositions++ = mDeadParticleStartingPositions[potX + (potY * width)];

						*dyingTimes++;
						numberOfDyingToDraw++;
						mCurrentlyHasDeadInstance[potX + (potY * width)] = true;
					}
					else //too old
					{
						mCurrentlyHasDeadInstance[potX + (potY * width)] = false;
					}

				}

				mPreviouslyHadDepth[potX + (potY * width)] = false;
			}
		}
	}

	console() << numberOfDyingToDraw << endl;
	
	mInstanceDataVbo->unmap();
	mInstanceScaleVbo->unmap();
	mInstanceDyingDataVbo->unmap();
	mInstanceDyingTotalTimeVbo->unmap();
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

	mBatchDyingCubes->drawInstanced(numberOfDyingToDraw);
	mBatch->drawInstanced(numberToDraw);

	//console() << numberToDraw << endl;
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