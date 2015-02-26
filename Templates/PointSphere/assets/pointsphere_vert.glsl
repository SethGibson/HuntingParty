#version 150

uniform mat4 ciModelViewProjection;
uniform mat4 ciModelViewMatrix;
uniform mat4 ciNormalMatrix;
uniform sampler2D mTexDepth;
uniform float ciElapsedSeconds;

in vec4 ciPosition;
in vec3 ciNormal;
in vec2 ciTexCoord0;

out vec4 ViewNormal;
out vec4 Color;

void main()
{
	float cDepth = texture2D(mTexDepth, ciTexCoord0).x;
	
	vec3 cPos = ciPosition.xyz;
	//cPos.x += ciNormal.x*((cDepth/255.0)*100.0);
	//cPos.z += ciNormal.z*((cDepth/255.0)*100.0);
	float cNDepth = (cDepth/255.0)*900.0;
	cNDepth += 0.1*sin( (cPos.y+ciElapsedSeconds)*16.0 );
	//cNDepth += 0.01* ( (sin(cPos.x+ciElapsedSeconds)+cos(cPos.y+ciElapsedSeconds))*10.0 );
	cPos.z += cNDepth;
	gl_Position = ciModelViewProjection * vec4(cPos,1);
	Color = vec4(cDepth,cDepth,cDepth,1.0);


	//vec3 cPos = ciPosition.xyz;
	//cPos.x += ciNormal.x*abs(sin(ciElapsedSeconds));
	//cPos.z += ciNormal.z*abs(cos(ciElapsedSeconds));
	//gl_Position = ciModelViewProjection * vec4(cPos,1);
	
}