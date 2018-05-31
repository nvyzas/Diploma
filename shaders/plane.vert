#version 330 core

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec2 inTexCoord;
layout (location = 2) in vec3 inNormal;

out vec2 TexCoord;
out vec3 Normal;

uniform mat4 gMVP;
uniform mat4 gSpecific;

void main()
{
	vec4 posLocal = gSpecific * vec4(inPosition, 1.0);
	gl_Position = gMVP * posLocal;
	
	TexCoord = inTexCoord;
	Normal = inNormal;
}