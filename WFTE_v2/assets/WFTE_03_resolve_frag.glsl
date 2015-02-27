#version 150

uniform sampler2D mHdrTex;
uniform sampler2D mBloomTex;

uniform float mExposure = 0.9;
uniform float mBloomFactor = 1.0;
uniform float mSceneFactor = 1.0;

out vec4 oColor;

void main()
{
	vec4 c = vec4(0);

	c+=texelFetch(mHdrTex, ivec2(gl_FragCoord.xy), 0) * mSceneFactor;
	c+=texelFetch(mBloomTex, ivec2(gl_FragCoord.xy), 0) * mBloomFactor;

	c.rgb = vec3(1.0)-exp(-c.rgb*mExposure);
	oColor = c;
}