#version 150

uniform sampler2D uTex0;
uniform sampler2D uLogoTex;
uniform bool UseMaskTexture;

in vec4	Color;
in vec3	Normal;
in vec2 ScreenPos;
in vec2 InstanceXY;
in vec2	TexCoord;

out vec4 			oColor;

void main( void )
{
	vec3 normal = normalize( -Normal );
	float diffuse = max( dot( normal, vec3( 0, 0, -1 ) ), 0 );
	oColor = texture( uTex0, TexCoord.st ) * Color * diffuse;

	if (UseMaskTexture)
	{
		vec2 coords = 0 + (1 - 0) * ((InstanceXY.xy - -200) / (200 - -200));
		//coords.x = 0 + (1 - 0) * ((InstanceXY.x - -500) / (500 - -500));
		//coords.y = 0 + (1 - 0) * ((InstanceXY.y - -500) / (500 - -500));

		//user ScreenPos variable instead of coords to map it to the camera position.
		vec4 color = texture2D(uLogoTex, coords);
		if (color.b > 0.1)
		{
			oColor = color;
		}
	}
}