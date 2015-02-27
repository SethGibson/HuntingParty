#version 150
uniform sampler2D mTexColor;
uniform sampler2D mTexFinal;
uniform sampler2D mTexBlur;

uniform float mColorAmt;
uniform float mFinalAmt;
uniform float mBlurAmt;

in vec2 UV;
out vec4 oColor;
void main()
{
	vec4 cColor = texture2D(mTexColor, UV)*mColorAmt;
	vec4 cFinal = texture2D(mTexFinal, UV)*mFinalAmt;
	vec4 cBlur = texture2D(mTexBlur, UV)*mBlurAmt;

	oColor = cColor + cFinal + cBlur;
}