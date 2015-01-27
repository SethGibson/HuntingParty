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
	void mouseDown(MouseEvent event) override;
	void mouseDrag(MouseEvent event) override;

	CameraPersp			mCam;
	gl::BatchRef		mBatch;
	gl::TextureRef		mTexture;
	gl::GlslProgRef		mGlsl;
	gl::VboRef			mInstanceDataVbo;
	gl::VboRef			mInstanceScaleVbo;

	//DSAPI
	CinderDSRef mDSAPI;
	MayaCamUI mMayaCam;

	Surface8u mRgbBuffer;
	Channel16u mDepthBuffer;
};

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

	//mCam.lookAt(vec3(0, CAMERA_Y_RANGE.first, 0), vec3(0));
	vec2 cFOVs = mDSAPI->getDepthFOVs();
	mCam.setPerspective(cFOVs.y, getWindowAspectRatio(), 100, 5000);
	mCam.lookAt(vec3(0, 0, 500), vec3(0, 0, 1000), vec3(0, -1, 0));

	mMayaCam = MayaCamUI(mCam);

	mTexture = gl::Texture::create(loadImage(loadAsset("texture.jpg")), gl::Texture::Format().mipmap());
#if ! defined( CINDER_GL_ES )
	mGlsl = gl::GlslProg::create(loadAsset("shader.vert"), loadAsset("shader.frag"));
#else
	mGlsl = gl::GlslProg::create(loadAsset("shader_es2.vert"), loadAsset("shader_es2.frag"));
#endif

	gl::VboMeshRef mesh = gl::VboMesh::create(geom::Cube());

	// create an array of initial per-instance positions laid out in a 2D grid
	std::vector<vec3> positions;
	std::vector<float> cubeScales;
	for (size_t potX = 0; potX < width; ++potX) {
		for (size_t potY = 0; potY < height; ++potY) {
			float instanceX = potX / (float)width - 0.5f;
			float instanceY = potY / (float)height - 0.5f;
			positions.push_back(vec3(instanceX * vec3(1, 0, 0) + instanceY * vec3(0, 0, 1)));

			cubeScales.push_back(randFloat(0.2, 5));
		}
	}

	// create the VBO which will contain per-instance (rather than per-vertex) data
	mInstanceDataVbo = gl::Vbo::create(GL_ARRAY_BUFFER, positions.size() * sizeof(vec3), positions.data(), GL_DYNAMIC_DRAW);
	mInstanceScaleVbo = gl::Vbo::create(GL_ARRAY_BUFFER, cubeScales.size() * sizeof(float), cubeScales.data(), GL_DYNAMIC_DRAW);

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
	mMayaCam.setCurrentCam(mCam);

	// and in turn, let OpenGL know we have a new camera
	gl::setMatrices(mCam);
}

void HP_WaitingRTApp::update()
{
	mDSAPI->update();

	mDepthBuffer = mDSAPI->getDepthFrame();
	mRgbBuffer = mDSAPI->getRgbFrame();

	// update our instance positions; map our instance data VBO, write new positions, unmap
	vec3 *positions = (vec3*)mInstanceDataVbo->mapWriteOnly(true);
	float *scales = (float*)mInstanceScaleVbo->mapWriteOnly(true);
	numberToDraw = 0;
	uint16_t* depth = mDepthBuffer.getDataStore().get();//.getIter();



	float time = getElapsedSeconds();

	for (int potX = 0; potX < width; ++potX)
	{
		for (int potY = 0; potY < height; ++potY)
		{
			float v = (float)depth[potX + (potY * width)];
			if (v != 0)
			{
				vec3 worldPos = mDSAPI->getDepthSpacePoint(vec3((float)potX, (float)potY, v));
				//*positions++ = vec3(potX / width - 0.5f, potY / height - 0.5f, 0);// depth[potX + (potY * width)]
				*positions++ = worldPos;

				*scales++ = 0.5f;

				numberToDraw++;
			}
		}
	}
	mInstanceDataVbo->unmap();
	mInstanceScaleVbo->unmap();
}

#include <glm/gtc/matrix_transform.hpp>

void HP_WaitingRTApp::draw()
{
	gl::clear(Color::black());
	gl::setMatrices(mMayaCam.getCamera());

	// glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
	
	glm::mat4 blah = glm::rotate((float)(abs(cos(getElapsedSeconds() / 8)) * 180), glm::vec3(0, 0, 1));

	mGlsl->bind();
	mGlsl->uniform("rotationMatrix", blah);

	mBatch->drawInstanced(numberToDraw);

	gl::drawLine(vec3(300, 300, 500), vec3(200, 200, 1300));
	console() << numberToDraw << endl;
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