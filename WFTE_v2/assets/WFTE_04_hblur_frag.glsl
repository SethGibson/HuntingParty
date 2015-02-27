#version 150

uniform sampler2D mRgbTex;
uniform float mBlurSize;

in vec2 UV;
out vec4 oColor;

void main()
{
   vec4 sum = vec4(0.0);
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y - 4.0*mBlurSize)) * 0.05;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y - 3.0*mBlurSize)) * 0.09;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y - 2.0*mBlurSize)) * 0.12;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y - mBlurSize)) * 0.15;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y)) * 0.16;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y + mBlurSize)) * 0.15;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y + 2.0*mBlurSize)) * 0.12;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y + 3.0*mBlurSize)) * 0.09;
   sum += texture2D(mRgbTex, vec2(UV.x, UV.y + 4.0*mBlurSize)) * 0.05;
 
   oColor = vec4(sum.rgb,sum.b);
}