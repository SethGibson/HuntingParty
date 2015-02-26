#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif
#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Batch.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/Camera.h"
#include "cinder/MayaCamUI.h"
#include "CiDSAPI.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

static ivec2 S_DIMS(320, 240);
static int S_STEP = 1;

class InstanceCloudApp : public AppNative
{
public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void mouseDrag(MouseEvent event) override;
	void update() override;
	void draw() override;

	void exit();
	struct InstData
	{
		vec3 IPosition;
		vec4 IColor;
		float ISize;
		InstData(vec3 pPos, vec4 pCol, float pSize) :IPosition(pPos), IColor(pCol), ISize(pSize){}
	};

private:
	void setupDSAPI();
	void setupMesh();

	gl::VboRef mInstanceData;
	gl::VboMeshRef mVboMesh;
	geom::BufferLayout mInstanceAttribs;
	geom::Cube mCube;
	gl::GlslProgRef mShader;
	gl::BatchRef mDrawObj;

	vector<InstData> mInst;
	CinderDSRef mCinderDS;

	CameraPersp mCamera;
	MayaCamUI mMaya;
};

void InstanceCloudApp::setup()
{
	getWindow()->setSize(1280, 720);
	setupDSAPI();
	setupMesh();
}

void InstanceCloudApp::setupDSAPI()
{
	mCinderDS = CinderDSAPI::create();

	mCinderDS->init();
	mCinderDS->initDepth(FrameSize::DEPTHQVGA, 60);
	mCinderDS->start();
	getSignalShutdown().connect(std::bind(&InstanceCloudApp::exit, this));
}

void InstanceCloudApp::setupMesh()
{
	try
	{
		mShader = gl::GlslProg::create(loadAsset("instcloud_vert.glsl"), loadAsset("instcloud_frag.glsl"));
	}
	catch (const gl::GlslProgExc &e)
	{
		console() << "Error loading shaders: " << endl;
		console() << e.what() << endl;
	}

	mCube = geom::Cube();
	mVboMesh = gl::VboMesh::create(mCube);

	for (int vy = 0; vy < S_DIMS.y; vy += S_STEP)
	{
		for (int vx = 0; vx < S_DIMS.x; vx += S_STEP)
		{
			mInst.push_back(InstData(vec3(0), vec4(0), 1.0f));
		}
	}
	mInstanceData = gl::Vbo::create(GL_ARRAY_BUFFER, mInst.size()*sizeof(InstData), mInst.data(), GL_DYNAMIC_DRAW);
	mInstanceAttribs.append(geom::CUSTOM_0, 3, sizeof(InstData), offsetof(InstData, IPosition), 1);
	mInstanceAttribs.append(geom::CUSTOM_1, 4, sizeof(InstData), offsetof(InstData, IColor), 1);
	mInstanceAttribs.append(geom::CUSTOM_2, 1, sizeof(InstData), offsetof(InstData, ISize), 1);

	mVboMesh->appendVbo(mInstanceAttribs, mInstanceData);
	mDrawObj = gl::Batch::create(mVboMesh, mShader, { { geom::CUSTOM_0, "iPosition" }, { geom::CUSTOM_1, "iColor" }, { geom::CUSTOM_1, "iSize" } });
	vec2 cFOV = mCinderDS->getDepthFOVs();

	mCamera.setPerspective(45.0f, getWindowAspectRatio(), 100.0f, 4000.0f);
	mCamera.lookAt(vec3(0), vec3(0, 0, 1), vec3(0, -1, 0));
	mCamera.setCenterOfInterestPoint(vec3(0, 0, 250));
	
	mMaya.setCurrentCam(mCamera);
}

void InstanceCloudApp::mouseDown( MouseEvent event )
{
	mMaya.mouseDown(event.getPos());
}

void InstanceCloudApp::mouseDrag(MouseEvent event)
{
	mMaya.mouseDrag(event.getPos(), event.isLeftDown(), false, event.isRightDown());
}

void InstanceCloudApp::update()
{
	mCinderDS->update();
	mInst.clear();
	const uint16_t *cDepth = mCinderDS->getDepthFrame().getData();

	int id = 0;
	for (int dy = 0; dy < S_DIMS.y; ++dy)
	{
		for (int dx = 0; dx < S_DIMS.x; ++dx)
		{
			float cVal = (float)cDepth[id];
			if (cVal>100 && cVal < 1000)
			{
				vec3 cWorld = mCinderDS->getZCameraSpacePoint(dx, dy, cDepth[id]);
				float cR = lmap<float>(dx, 0, S_DIMS.x, 0.1f, 1.0f);
				float cG = lmap<float>(dy, 0, S_DIMS.y, 0.1f, 1.0f);
				mInst.push_back(InstData(cWorld, vec4(cR, cG, 1, 1), math<float>::sin(dx)));
			}
			id++;
		}
	}

	mInstanceData->bufferData(mInst.size()*sizeof(InstData), mInst.data(), GL_DYNAMIC_DRAW);
	mVboMesh->gl::VboMesh::create(mCube);
	mVboMesh->appendVbo(mInstanceAttribs, mInstanceData);
	mDrawObj->replaceVboMesh(mVboMesh);
}

void InstanceCloudApp::draw()
{
	gl::clear( Color( 0.1f, 0.25f, 0 ) ); 
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::setMatrices(mMaya.getCamera());
	mDrawObj->drawInstanced(mInst.size());
}

void InstanceCloudApp::exit()
{
	mCinderDS->stop();
}

CINDER_APP_NATIVE( InstanceCloudApp, RendererGl )
