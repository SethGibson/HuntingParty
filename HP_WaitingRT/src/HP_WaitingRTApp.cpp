#ifdef _DEBUG
#pragma comment(lib, "DSAPI32.dbg.lib")
#else
#pragma comment(lib, "DSAPI32.lib")
#endif
#include <memory>
#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

#include "cinder/Rand.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Context.h"
#include "cinder/gl/Shader.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/Vao.h"
#include "cinder/gl/GlslProg.h"

#include "CiDSAPI.h"
//#include "DSAPI.h"
//#include "DSAPIUtil.h"
//#include "CinderOpenCV.h

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace CinderDS;

/**
Particle type holds information for rendering and simulation.
Used to buffer initial simulation values.
*/
struct Particle
{
	vec3	pos;
	vec3	ppos;
	vec3	home;
	ColorA  color;
	float	damping;
};

int NUM_PARTICLES = 300e3;

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

	gl::GlslProgRef mRenderProg;
	gl::GlslProgRef mUpdateProg;

	// Descriptions of particle data layout.
	gl::VaoRef		mAttributes[2];
	// Buffers holding raw particle data on GPU.
	gl::VboRef		mParticleBuffer[2];

	// Current source and destination buffers for transform feedback.
	// Source and destination are swapped each frame after update.
	std::uint32_t	mSourceIndex = 0;
	std::uint32_t	mDestinationIndex = 1;
};


const float DRAW_SCALE = 1;
const pair<float, float> CAMERA_Y_RANGE(-100, 600);

void HP_WaitingRTApp::setup()
{
	//DSAPI INIT
	mDSAPI = CinderDSAPI::create();
	mDSAPI->init();
	mDSAPI->initDepth(CinderDS::FrameSize::DEPTHSD, 60);
	mDSAPI->initRgb(CinderDS::FrameSize::RGBVGA, 60);
	mDSAPI->start();

	NUM_PARTICLES = mDSAPI->getDepthWidth() * mDSAPI->getDepthHeight();
	
	//END DSAPI INIT

	//GPU PARTICLES INIT
	// Create initial particle layout.
	vector<Particle> particles;
	particles.assign(NUM_PARTICLES, Particle());
	const float azimuth = 256.0f * M_PI / particles.size();
	const float inclination = M_PI / particles.size();
	const float radius = 180.0f;
	for (int y = 0; y < mDSAPI->getDepthHeight(); y++)
	{
		for (int x = 0; x < mDSAPI->getDepthWidth(); x++)
		{
			int i = x + (y * mDSAPI->getDepthWidth());
			auto &p = particles.at(i);
			p.pos = vec3(x, y, 0);
			p.home = p.pos;
			p.ppos = p.home + Rand::randVec3f() * 10.0f; // random initial velocity
			p.damping = Rand::randFloat(0.965f, 0.985f);
			p.color = Color(CM_HSV, lmap<float>(i, 0.0f, particles.size(), 0.0f, 0.66f), 1.0f, 1.0f);
		}
	}

	// Create particle buffers on GPU and copy data into the first buffer.
	// Mark as static since we only write from the CPU once.
	mParticleBuffer[mSourceIndex] = gl::Vbo::create(GL_ARRAY_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_STATIC_DRAW);
	mParticleBuffer[mDestinationIndex] = gl::Vbo::create(GL_ARRAY_BUFFER, particles.size() * sizeof(Particle), nullptr, GL_STATIC_DRAW);

	// Create a default color shader.
	mRenderProg = gl::getStockShader(gl::ShaderDef().color());

	for (int i = 0; i < 2; ++i)
	{	// Describe the particle layout for OpenGL.
		mAttributes[i] = gl::Vao::create();
		gl::ScopedVao vao(mAttributes[i]);

		// Define attributes as offsets into the bound particle buffer
		gl::ScopedBuffer buffer(mParticleBuffer[i]);
		gl::enableVertexAttribArray(0);
		gl::enableVertexAttribArray(1);
		gl::enableVertexAttribArray(2);
		gl::enableVertexAttribArray(3);
		gl::enableVertexAttribArray(4);
		gl::vertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (const GLvoid*)offsetof(Particle, pos));
		gl::vertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Particle), (const GLvoid*)offsetof(Particle, color));
		gl::vertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (const GLvoid*)offsetof(Particle, ppos));
		gl::vertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (const GLvoid*)offsetof(Particle, home));
		gl::vertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(Particle), (const GLvoid*)offsetof(Particle, damping));
	}

	// Load our update program.
	// Match up our attribute locations with the description we gave.

#if defined( CINDER_GL_ES_3 )
	mUpdateProg = gl::GlslProg::create(gl::GlslProg::Format().vertex(loadAsset("particleUpdate_es3.vs"))
		.fragment(loadAsset("no_op_es3.fs"))
#else
	mUpdateProg = gl::GlslProg::create(gl::GlslProg::Format().vertex(loadAsset("particleUpdate.vs"))
#endif
		.feedbackFormat(GL_INTERLEAVED_ATTRIBS)
		.feedbackVaryings({ "position", "pposition", "home", "color", "damping" })
		.attribLocation("iPosition", 0)
		.attribLocation("iColor", 1)
		.attribLocation("iPPosition", 2)
		.attribLocation("iHome", 3)
		.attribLocation("iDamping", 4)
		);
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
	mDSAPI->update();

	mDepthBuffer = mDSAPI->getDepthFrame();
	mRgbBuffer = mDSAPI->getRgbFrame();

	console() << getFrameRate() << endl;

	//GPU PARTICLES UPDATE
	// Update particles on the GPU
	gl::ScopedGlslProg prog(mUpdateProg);
	gl::ScopedState rasterizer(GL_RASTERIZER_DISCARD, true);	// turn off fragment stage

	// Bind the source data (Attributes refer to specific buffers).
	gl::ScopedVao source(mAttributes[mSourceIndex]);
	// Bind destination as buffer base.
	gl::bindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, mParticleBuffer[mDestinationIndex]);
	gl::beginTransformFeedback(GL_POINTS);

	// Draw source into destination, performing our vertex transformations.
	gl::drawArrays(GL_POINTS, 0, NUM_PARTICLES);

	gl::endTransformFeedback();

	// Swap source and destination for next loop
	std::swap(mSourceIndex, mDestinationIndex);
}

void HP_WaitingRTApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 

	////gl::draw(gl::Texture::create(mRgbBuffer));
	////gl::drawSolidCircle(vec2(getWindowWidth() / 2, getWindowHeight() / 2), 20);

	//Channel16u::Iter iter = mDepthBuffer.getIter();// mDSAPI->getDepthWidth(), mDSAPI->getDepthHeight()));

	//int x = 0;
	//int y = 0;
	//int v = 0;
	//while (iter.line())
	//{
	//	while (iter.pixel())
	//	{
	//		x = iter.x();
	//		y = iter.y();
	//		v = iter.v();
	//		//if (iter.x() > 300 && iter.x() < 302 && iter.y() > 300 && iter.y() < 302)
	//		//gl::drawSolidCircle(vec2(iter.x() + 0.5, iter.y() + 0.5), lmap<uint16_t>(iter.v(), 490, 5000, 0, 1));
	//	}
	//}

	gl::setMatricesWindowPersp(getWindowSize());
	gl::enableDepthRead();
	gl::enableDepthWrite();

	gl::ScopedGlslProg render(mRenderProg);
	gl::ScopedVao vao(mAttributes[mSourceIndex]);
	gl::context()->setDefaultShaderVars();
	gl::drawArrays(GL_POINTS, 0, NUM_PARTICLES);
}

CINDER_APP_NATIVE(HP_WaitingRTApp, RendererGl)