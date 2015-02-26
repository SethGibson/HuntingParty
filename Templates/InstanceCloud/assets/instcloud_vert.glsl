#version 150

uniform mat4 ciModelViewProjection;
in vec4 ciPosition;

in vec3 iPosition;
in vec4 iColor;
in float iSize;

out vec4 Color;
void main()
{
	vec4 mPosition = ciPosition;
	mPosition.x*=iSize;
	mPosition.y*=iSize;
	mPosition.z*=iSize;

	gl_Position = ciModelViewProjection * (4.0*(mPosition)+vec4(iPosition,0));
	Color = iColor;
}