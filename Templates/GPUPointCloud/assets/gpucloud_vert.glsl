#version 150

uniform mat4 ciModelViewProjection;
uniform mat4 ciModelViewMatrix;
uniform mat4 ciNormalMatrix;
uniform sampler2D mTexDepth;
uniform float ciElapsedSeconds;

in vec4 ciPosition;
in vec3 ciNormal;
in vec2 ciTexCoord0;

out vec4 ViewNormal;
out vec4 Color;

void main()
{
	float cDepth = texture2D(mTexDepth, ciTexCoord0).x;
	
	vec3 cPos = ciPosition.xyz;
	cPos.z += (cDepth/255.0)*900.0;
	Color = vec4(cDepth,cDepth,cDepth,1.0);
	gl_Position = ciModelViewProjection * vec4(cPos,1);
}