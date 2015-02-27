#version 150
uniform float mBloomMin;
uniform float mBloomMax;
uniform sampler2D mRgbTex;

in vec2 UV;
in vec4 Color;
out vec4 oColor;

void main()
{
	vec3 cColor = texture2D(mRgbTex, UV).rgb;
	float Y = dot(cColor,vec3(0.299,0.587,0.144));

	oColor = Color*vec4(cColor * 5.0 * smoothstep(mBloomMin,mBloomMax,Y),1.0);
}

//858 775 4985