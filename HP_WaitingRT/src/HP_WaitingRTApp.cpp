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

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

class HP_WaitingRTApp : public AppNative {
public:
	void setup();
	void shutdown() override;
	void resize();
	void update();
	void draw();

	CameraPersp			mCam;
	gl::BatchRef		mBatch;
	gl::TextureRef		mTexture;
	gl::GlslProgRef		mGlsl;
	gl::VboRef			mInstanceDataVbo;

	//DSAPI
	CinderDSRef mDSAPI;

	Surface8u mRgbBuffer;
	Channel16u mDepthBuffer;
};

const float DRAW_SCALE = 200;
const pair<float, float> CAMERA_Y_RANGE(32, 80);

int width;
int height;
int numberToDraw = 0;


void HP_WaitingRTApp::setup()
{
	mDSAPI = CinderDSAPI::create();
	mDSAPI->init();
	mDSAPI->initDepth(CinderDS::FrameSize::DEPTHSD, 60);
	mDSAPI->initRgb(CinderDS::FrameSize::RGBVGA, 30);
	mDSAPI->start();

	width = mDSAPI->getDepthWidth();
	height = mDSAPI->getDepthHeight();

	mCam.lookAt(vec3(0, CAMERA_Y_RANGE.first, 0), vec3(0));

	mTexture = gl::Texture::create(loadImage(loadAsset("texture.jpg")), gl::Texture::Format().mipmap());
#if ! defined( CINDER_GL_ES )
	mGlsl = gl::GlslProg::create(loadAsset("shader.vert"), loadAsset("shader.frag"));
#else
	mGlsl = gl::GlslProg::create(loadAsset("shader_es2.vert"), loadAsset("shader_es2.frag"));
#endif

	gl::VboMeshRef mesh = gl::VboMesh::create(geom::Cube());

	// create an array of initial per-instance positions laid out in a 2D grid
	std::vector<vec3> positions;
	for (size_t potX = 0; potX < width; ++potX) {
		for (size_t potY = 0; potY < height; ++potY) {
			float instanceX = potX / (float)width - 0.5f;
			float instanceY = potY / (float)height - 0.5f;
			positions.push_back(vec3(instanceX * vec3(DRAW_SCALE, 0, 0) + instanceY * vec3(0, 0, DRAW_SCALE)));
		}
	}

	// create the VBO which will contain per-instance (rather than per-vertex) data
	mInstanceDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER, positions.size() * sizeof(vec3), positions.data(), GL_DYNAMIC_DRAW);

	// we need a geom::BufferLayout to describe this data as mapping to the CUSTOM_0 semantic, and the 1 (rather than 0) as the last param indicates per-instance (rather than per-vertex)
	geom::BufferLayout instanceDataLayout;
	instanceDataLayout.append(geom::Attrib::CUSTOM_0, 3, 0, 0, 1 /* per instance */);

	// now add it to the VboMesh we already made of the Teapot
	mesh->appendVbo(instanceDataLayout, mInstanceDataVbo);

	// and finally, build our batch, mapping our CUSTOM_0 attribute to the "vInstancePosition" GLSL vertex attribute
	mBatch = gl::Batch::create(mesh, mGlsl, { { geom::Attrib::CUSTOM_0, "vInstancePosition" } });

	gl::enableDepthWrite();
	gl::enableDepthRead();

	mTexture->bind();
}

void HP_WaitingRTApp::shutdown()
{
	mDSAPI->stop();
}

void HP_WaitingRTApp::resize()
{
	// now tell our Camera that the window aspect ratio has changed
	mCam.setPerspective(60, getWindowAspectRatio(), 1, 1000);

	// and in turn, let OpenGL know we have a new camera
	gl::setMatrices(mCam);
}

void HP_WaitingRTApp::update()
{
	mDSAPI->update();

	mDepthBuffer = mDSAPI->getDepthFrame();
	mRgbBuffer = mDSAPI->getRgbFrame();

	// move the camera up and down on Y
	mCam.lookAt(vec3(0, CAMERA_Y_RANGE.first + abs(sin(getElapsedSeconds() / 4)) * (CAMERA_Y_RANGE.second - CAMERA_Y_RANGE.first), 0), vec3(0));

	// update our instance positions; map our instance data VBO, write new positions, unmap
	vec3 *positions = (vec3*)mInstanceDataVbo->mapWriteOnly(true);
	numberToDraw = 0;
	uint16_t* depth = mDepthBuffer.getDataStore().get();//.getIter();

	for (int potX = 0; potX < width; ++potX)
	{
		for (int potY = 0; potY < height; ++potY)
		{
			if (depth[potX + (potY * width)] != 0)
			{
				*positions++ = vec3(potX / width - 0.5f, potY / height - 0.5f, 0);// depth[potX + (potY * width)]

				numberToDraw++;
			}
		}
	}
	mInstanceDataVbo->unmap();
}

void HP_WaitingRTApp::draw()
{
	gl::clear(Color::black());
	gl::setMatrices(mCam);

	mBatch->drawInstanced(numberToDraw);
	console() << numberToDraw << endl;
}

#if defined( CINDER_MSW ) && ! defined( CINDER_GL_ANGLE )
auto options = RendererGl::Options().version(3, 3); // instancing functions are technically only in GL 3.3
#else
auto options = RendererGl::Options(); // implemented as extensions in Mac OS 10.7+
#endif
CINDER_APP_NATIVE(HP_WaitingRTApp, RendererGl(options))