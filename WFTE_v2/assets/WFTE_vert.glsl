#version 150

uniform mat4 ciModelViewProjection;
uniform sampler2D mTexDepth;

in vec4 ciPosition;
in vec4 ciColor;
in vec2 ciTexCoord0;

out vec4 Color;
void main()
{
	Color = ciColor;
	gl_Position = ciModelViewProjection * ciPosition;
}