#version 330 core

layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inColor;

out vec4 Color;

uniform mat4 gMVP;
uniform mat4 gSpecific;

void main()
{
	vec4 posLocal = gSpecific * vec4(inPosition, 1.0);
	gl_Position = gMVP * posLocal;
    Color = vec4(inColor, 1);
}