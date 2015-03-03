#version 150

uniform mat4	ciModelViewProjection;
uniform mat3	ciNormalMatrix;
uniform mat4	rotationMatrix;

in vec4		ciPosition;
in vec2		ciTexCoord0;
in vec3		ciNormal;
in vec4		ciColor;

in vec4		vColor0;
in vec4		vColor1;
in vec4		vColor2;

out vec3 Position; // In world space

out vec2		TexCoord;
out lowp vec4	Color;
out vec3		Normal;

float HueToRGB(float f1, float f2, float hue)
{
	if (hue < 0.0)
		hue += 1.0;
	else if (hue > 1.0)
		hue -= 1.0;

	float res;
	
	if ((6.0 * hue) < 1.0)
		res = f1 + (f2 - f1) * 6.0 * hue;
	else if ((2.0 * hue) < 1.0)
		res = f2;
	else if ((3.0 * hue) < 2.0)
		res = f1 + (f2 - f1) * ((2.0 / 3.0) - hue) * 6.0;
	else
		res = f1;

	return res;
}

vec3 HSLToRGB(vec3 hsl)
{
	vec3 rgb;

	if (hsl.y == 0.0)
		rgb = vec3(hsl.z); // Luminance
	else
	{
		float f2;

		if (hsl.z < 0.5)
			f2 = hsl.z * (1.0 + hsl.y);
		else
			f2 = (hsl.z + hsl.y) - (hsl.y * hsl.z);

		float f1 = 2.0 * hsl.z - f2;

		rgb.r = HueToRGB(f1, f2, hsl.x + (1.0/3.0));
		rgb.g = HueToRGB(f1, f2, hsl.x);
		rgb.b = HueToRGB(f1, f2, hsl.x - (1.0/3.0));
	}

	return rgb;
}

void main( void )
{
	gl_Position	= ciModelViewProjection * ( vec4((rotationMatrix * (ciPosition)).xyz, ciPosition.w) );
	float normalizedMappedZ = 0 + (1 - 0) * ((gl_Position.z - 100) / (2500 - 100));
	Color 		= (vColor0 + vColor1 + vColor2).rgba / 3;
	TexCoord	= ciTexCoord0;
	Normal		= ciNormalMatrix * (rotationMatrix * vec4(ciNormal,0)).xyz;
}