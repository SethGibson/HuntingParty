#version 150
uniform float mBloomMin = 0.4;
uniform float mBloomMax = 1.0;
uniform sampler2D mRgbTex;
in vec2 UV;
out vec4 oColor;

void main()
{
	vec3 cColor = texture2D(mRgbTex, UV).rgb;
	float Y = dot(cColor,vec3(0.299,0.587,0.144));

	oColor = vec4(cColor * 6.0 * smoothstep(mBloomMin,mBloomMax,Y),1.0);
}

//858 775 4985