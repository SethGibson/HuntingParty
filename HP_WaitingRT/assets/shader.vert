#version 150

uniform mat4	ciModelViewProjection;
uniform mat3	ciNormalMatrix;
uniform mat4	rotationMatrix;

in vec4		ciPosition;
in vec2		ciTexCoord0;
in vec3		ciNormal;
in vec4		ciColor;
in vec3		vInstancePosition; // per-instance position variable
in float	fCubeScale;

out vec3 Position; // In world space

out vec2		TexCoord;
out lowp vec4	Color;
out vec3		Normal;

void main( void )
{
	gl_Position	= ciModelViewProjection * ( vec4((rotationMatrix * (fCubeScale * ciPosition)).xyz, ciPosition.w) + vec4( vInstancePosition, 0 ) );
	Color 		= ciColor;
	TexCoord	= ciTexCoord0;
	Normal		= ciNormalMatrix * (rotationMatrix * vec4(ciNormal,0)).xyz;
}
