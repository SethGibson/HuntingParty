#version 150

uniform sampler2D mRgbTex;
in vec4 Color;
in vec2 UV;
out vec4 oColor;

void main()
{
	oColor = texture2D(mRgbTex, UV)*Color;
}