#version 150

in vec4 Color;
out vec4 oColor;

void main()
{
	//oColor = vec4(1,0,0,1);
	oColor = Color;
}