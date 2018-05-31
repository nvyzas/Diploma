#version 330 core

// Input from vertex shader
in VS_OUT
{
	vec3 color;
} fs_in;

// Output
layout (location = 0) out vec4 color;

void main(void)
{
	// Write incoming color to the framebuffer
	color = vec4(fs_in.color, 1.0);
}
