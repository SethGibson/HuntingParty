#version 150
uniform sampler2D mRgbTex;
in vec4 Color;
in vec2 UV;
out vec4 oColor;

float l = 1.0;
float m = 0.5;
float t = 0.25;
void main()
{
	vec4 cColor = texture2D(mRgbTex, UV);
	cColor *= (m/l);
	cColor *= 1.0 + (cColor / (t*t));
	cColor -= 0.5;
	
	oColor = cColor / (1.0 + cColor);
}