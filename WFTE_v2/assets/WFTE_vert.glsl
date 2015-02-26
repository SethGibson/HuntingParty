#version 150

uniform mat4 ciModelViewProjection;

in vec4 ciPosition;
in vec4 ciColor;
in vec2 ciTexCoord0;

out vec4 Color;
out vec2 UV;
void main()
{
	Color = ciColor;
	UV = ciTexCoord0;
	gl_Position = ciModelViewProjection * ciPosition;
}