#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif
#include <memory>
#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

#include "cinder/gl/Shader.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/VboMesh.h"
#include "cinder/ObjLoader.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"

#include "CiDSAPI.h"
//#include "DSAPI.h"
//#include "DSAPIUtil.h"
//#include "CinderOpenCV.h

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

class HP_WaitingRTApp : public AppNative
{
public:
	void setup() override;
	void shutdown() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;

private:
	//DSAPI
	CinderDSRef mDSAPI;
	
	//point cloud
	vector<vec3> mCloudPoints;

	Surface8u mRgbBuffer;
	Channel16u mDepthBuffer;

	CameraPersp			mCam;
	gl::BatchRef		mBatch;
	gl::TextureRef		mTexture;
	gl::GlslProgRef		mGlsl;
	gl::VboRef			mInstanceDataVbo;
};

const float DRAW_SCALE = 1;
const pair<float, float> CAMERA_Y_RANGE(0, 50);

int numberToDraw = 0;

double gameTime = 0;

void HP_WaitingRTApp::setup()
{
	//DSAPI INIT
	mDSAPI = CinderDSAPI::create();
	mDSAPI->init();
	mDSAPI->initDepth(CinderDS::FrameSize::DEPTHSD, 60);
	mDSAPI->initRgb(CinderDS::FrameSize::RGBVGA, 60);
	mDSAPI->start();
	
	//END DSAPI INIT

	//INSTANCED TEAPOTS INIT
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
	for (size_t potX = 0; potX < mDSAPI->getDepthWidth(); ++potX) {
		for (size_t potY = 0; potY < mDSAPI->getDepthHeight(); ++potY) {
			float instanceX = potX / (float)mDSAPI->getDepthWidth() - 0.5f;
			float instanceY = potY / (float)mDSAPI->getDepthHeight() - 0.5f;
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

void HP_WaitingRTApp::mouseDown( MouseEvent event )
{
}

void HP_WaitingRTApp::update()
{
	console() << "next frame time: " << (getElapsedSeconds() - gameTime) << endl;

	double time = getElapsedSeconds();
	mDSAPI->update();

	mDepthBuffer = mDSAPI->getDepthFrame();
	mRgbBuffer = mDSAPI->getRgbFrame();

	console() << "DSAPI time: " << (getElapsedSeconds() - time) << endl;

	//INSTANCED TEAPOT UPDATE
	// move the camera up and down on Y
	mCam.lookAt(vec3(0, CAMERA_Y_RANGE.first + abs(sin(getElapsedSeconds() / 4)) * (CAMERA_Y_RANGE.second - CAMERA_Y_RANGE.first), 0), vec3(0));

	time = getElapsedSeconds();

	// update our instance positions; map our instance data VBO, write new positions, unmap
	vec3 *positions = (vec3*)mInstanceDataVbo->mapWriteOnly(true);
	numberToDraw = 0;

	console() << "Update time: " << (getElapsedSeconds() - time) << endl;

	uint16_t* depth = mDepthBuffer.getDataStore().get();//.getIter();


	int width = mDSAPI->getDepthWidth();
	int height = mDSAPI->getDepthHeight();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			//float instanceX = ;
			//float instanceY = ;
			if (depth[x + (y * width)] != 0)
			{
				*positions++ = vec3(x / width - 0.5f, y / height - 0.5f, 0);// lmap<uint16_t>(iter.v(), 490, 5000, 0, 500));

				numberToDraw++;
			}
		}
	}

	//while (iter.line())
	//{
	//	while (iter.pixel())
	//	{
	//		if (iter.v() != 0)
	//		{
	//			float instanceX = iter.x() / (float)mDSAPI->getDepthWidth() - 0.5f;
	//			float instanceY = iter.y() / (float)mDSAPI->getDepthHeight() - 0.5f;

	//			*positions++ = vec3(instanceX, instanceY, 0);// lmap<uint16_t>(iter.v(), 490, 5000, 0, 500));
	//			
	//			numberToDraw++;
	//		}
	//	}
	//}
	mInstanceDataVbo->unmap();

	gameTime = getElapsedSeconds();
}

void HP_WaitingRTApp::draw()
{
	console() << "between update and draw time: " << (getElapsedSeconds() - gameTime) << endl;

	gl::clear( Color( 0, 0, 0 ) ); 

	////gl::draw(gl::Texture::create(mRgbBuffer));
	////gl::drawSolidCircle(vec2(getWindowWidth() / 2, getWindowHeight() / 2), 20);

	gl::setMatrices(mCam);

	double time = getElapsedSeconds();

	mBatch->drawInstanced(numberToDraw);//mDSAPI->getDepthWidth() * mDSAPI->getDepthHeight());

	console() << "draw time: " << (getElapsedSeconds() - time) << endl;

	//console() << numberToDraw << endl;
	gameTime = getElapsedSeconds();
}

#if defined( CINDER_MSW ) && ! defined( CINDER_GL_ANGLE )
auto options = RendererGl::Options().version(3, 3); // instancing functions are technically only in GL 3.3
#else
auto options = RendererGl::Options(); // implemented as extensions in Mac OS 10.7+
#endif
CINDER_APP_NATIVE(HP_WaitingRTApp, RendererGl(options))
