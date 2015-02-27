#version 150
uniform sampler2D mRgbTex;
in vec4 Color;
in vec2 UV;
out vec4 oColor;

float l = 1.0;
float m = 0.75;
float t = 0.5;

vec2 s = vec2(640,480);
//int sam = ;
//float q = 5;

void main()
{
	vec4 cColor = texture2D(mRgbTex, UV)*Color;
	vec4 sum = cColor;
	sum *= (m/l);
	sum *= 1.0 + (sum / (t*t));
	sum -= 0.5;
	
	sum = sum / (1.0+sum);
	oColor = sum*2;
}