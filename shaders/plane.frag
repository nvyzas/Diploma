#version 330 core

in vec2 TexCoord;
in vec3 Normal;

out vec4 Color;

uniform sampler2D sampler;

void main()
{
    //Color = texelFetch(sampler, ivec2(gl_FragCoord.xy), 0);
	Color = texture(sampler, TexCoord);
}
